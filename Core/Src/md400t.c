/*
 * md400t.c
 *
 *  Created on: Sep 8, 2025
 *      Author: bongs
 */
#include "md400t.h"
#include "main.h"   // USART2_EN_Pin, Port 매크로 사용(있으면 DE/RE 토글)
#include <string.h>
#include <stdio.h>

/* UART 핸들러 포인터 */
static UART_HandleTypeDef *s_hu = NULL;

/* RS485 DE/RE 토글 */
static inline void rs485_tx_begin(void){
#ifdef USART2_EN_Pin
  HAL_GPIO_WritePin(USART2_EN_GPIO_Port, USART2_EN_Pin, GPIO_PIN_SET);
#endif
}
static inline void rs485_tx_end(void){
#ifdef USART2_EN_Pin
  if (s_hu){
    while(!__HAL_UART_GET_FLAG(s_hu, UART_FLAG_TC)) {}
  }
  HAL_GPIO_WritePin(USART2_EN_GPIO_Port, USART2_EN_Pin, GPIO_PIN_RESET);
#endif
}

void MD400T_Init(UART_HandleTypeDef *huart){ s_hu = huart; }

/* 출력 변수 정의 */
volatile int16_t  FR_M1_CurrRPM=0; volatile float FR_M1_Amp=0.f; volatile uint8_t FR_M1_Status=0; volatile int32_t FR_M1_Encoder=0;
volatile int16_t  RR_M2_CurrRPM=0; volatile float RR_M2_Amp=0.f; volatile uint8_t RR_M2_Status=0; volatile int32_t RR_M2_Encoder=0;
volatile int16_t  RL_M1_CurrRPM=0; volatile float RL_M1_Amp=0.f; volatile uint8_t RL_M1_Status=0; volatile int32_t RL_M1_Encoder=0;
volatile int16_t  FL_M2_CurrRPM=0; volatile float FL_M2_Amp=0.f; volatile uint8_t FL_M2_Status=0; volatile int32_t FL_M2_Encoder=0;

static volatile uint32_t s_last_ms[256]={0};

/* 2's complement checksum */
static uint8_t md_ck(const uint8_t *p, uint16_t n){
  uint32_t s=0; for(uint16_t i=0;i<n;i++) s+=p[i];
  return (uint8_t)(-((int32_t)(s & 0xFF)));
}
static inline int16_t u8_to_i16(uint8_t lo, uint8_t hi){ return (int16_t)((uint16_t)lo | ((uint16_t)hi<<8)); }
static inline int32_t u8_to_i32(uint8_t b0,uint8_t b1,uint8_t b2,uint8_t b3){
  return (int32_t)((uint32_t)b0 | ((uint32_t)b1<<8) | ((uint32_t)b2<<16) | ((uint32_t)b3<<24));
}

/* 수신 파서 (수신 헤더 순서: TMID(0xB8)=H1, 그 다음 0xB7=H0) */
enum { S_H0=0,S_H1,S_ID,S_PID,S_LEN,S_DAT,S_CK };
static uint8_t hdr0, hdr1, id, pid, len, di, buf[64];

static uint32_t rx_last_tick = 0;
#define RX_TIMEOUT_MS  10
#define MD_MAX_FRAME   (2+1+1+1+64+1)   /* H0,H1,ID,PID,LEN,DATA,CK */
#define MD_Q_SIZE      8                /* 2의 거듭제곱 */
static struct {
  uint8_t f[MD_Q_SIZE][MD_MAX_FRAME];
  uint8_t n[MD_Q_SIZE];
  volatile uint8_t head, tail;
} q;
static volatile uint32_t q_drop = 0;
static inline void q_push(const uint8_t *fr, uint8_t n){
  uint8_t nx=(q.head+1)&(MD_Q_SIZE-1);
  if(nx==q.tail){ q_drop++; return; } /* overflow → drop */
  memcpy(q.f[q.head], fr, n); q.n[q.head]=n; q.head=nx;
}
int MD400T_PopFrame(uint8_t *out, uint8_t *len_out){
  __disable_irq();
  if(q.tail==q.head){ __enable_irq(); return 0; }
  uint8_t i=q.tail; q.tail=(q.tail+1)&(MD_Q_SIZE-1);
  uint8_t n=q.n[i]; memcpy(out, q.f[i], n);
  __enable_irq(); *len_out=n; return 1;
}
void MD400T_DebugDump(void){
  uint8_t fr[MD_MAX_FRAME], n;
  while(MD400T_PopFrame(fr,&n)){
    /* 수신 프레임은 H1(0xB8), H0(0xB7) 순서로 저장됨 */
    if (n>=6 && fr[0]==MD400T_H1 && fr[1]==MD400T_H0 && fr[3]==MD400T_PID_PNT_MAIN_DATA){
      for(uint8_t i=0;i<n;i++) printf("%02X ", fr[i]);
      printf("\r\n");
    }
  }
  if (q_drop){ printf("QDROP=%lu\r\n", (unsigned long)q_drop); q_drop=0; }
}

void MD400T_RxByte(uint8_t b){
  static uint8_t st=S_H0;
  uint32_t now = HAL_GetTick();
  if (now - rx_last_tick > RX_TIMEOUT_MS) { st = S_H0; di = 0; }
  rx_last_tick = now;
  switch(st){
    case S_H0: if (b==MD400T_H1){ hdr0=b; st=S_H1; } else st=S_H0; break;     /* H1 먼저 */
    case S_H1: if (b==MD400T_H0){ hdr1=b; st=S_ID; } else st=S_H0; break;     /* 그 다음 H0 */
    case S_ID: id=b; st=S_PID; break;
    case S_PID: pid=b; st=S_LEN; break;
    case S_LEN: len=b; if(len>sizeof(buf)){ di=0; st=S_H0; break; } di=0; st=(len==0)?S_CK:S_DAT; break;
    case S_DAT: buf[di++]=b; if(di>=len) st=S_CK; break;
    case S_CK: {
      /* 헤더~데이터 복사해 체크섬 계산용 배열 구성 (저장도 수신 순서 그대로: H1,H0,ID...) */
      uint16_t nsum = 2 + 1 + 1 + 1 + len;            /* H1,H0,ID,PID,LEN,DATA */
      uint8_t  tmp[2 + 1 + 1 + 1 + 64];
      tmp[0]=hdr0; tmp[1]=hdr1; tmp[2]=id; tmp[3]=pid; tmp[4]=len;
      memcpy(&tmp[5], buf, len);

      uint8_t ok = (md_ck(tmp, nsum) == b);           /* b = 수신 체크섬 */
      if (ok) {
        tmp[nsum] = b;
        q_push(tmp, (uint8_t)(nsum + 1));
      }

      if (ok && pid==MD400T_PID_PNT_MAIN_DATA && len==MD400T_PNT_MAIN_LEN) {
        int16_t  m1_rpm = u8_to_i16(buf[0],buf[1]);
        uint16_t m1_a10 = (uint16_t)u8_to_i16(buf[2],buf[3]);
        uint8_t  m1_st  = buf[4];
        int32_t  m1_enc = u8_to_i32(buf[5],buf[6],buf[7],buf[8]);

        int16_t  m2_rpm = u8_to_i16(buf[9],buf[10]);
        uint16_t m2_a10 = (uint16_t)u8_to_i16(buf[11],buf[12]);
        uint8_t  m2_st  = buf[13];
        int32_t  m2_enc = u8_to_i32(buf[14],buf[15],buf[16],buf[17]);

        float m1_amp = (float)m1_a10 * 0.1f;
        float m2_amp = (float)m2_a10 * 0.1f;

        if (id==1){ /* ID1: RL(M1), FL(M2) */
          RL_M1_CurrRPM=m1_rpm; RL_M1_Amp=m1_amp; RL_M1_Status=m1_st; RL_M1_Encoder=m1_enc;
          FL_M2_CurrRPM=m2_rpm; FL_M2_Amp=m2_amp; FL_M2_Status=m2_st; FL_M2_Encoder=m2_enc;
        } else if (id==2){ /* ID2: FR(M1), RR(M2) */
          FR_M1_CurrRPM=m1_rpm; FR_M1_Amp=m1_amp; FR_M1_Status=m1_st; FR_M1_Encoder=m1_enc;
          RR_M2_CurrRPM=m2_rpm; RR_M2_Amp=m2_amp; RR_M2_Status=m2_st; RR_M2_Encoder=m2_enc;
        }
        if (id < 256) s_last_ms[id] = HAL_GetTick();
      }

      st = S_H0;
      break;
    }
  }
}

uint32_t MD400T_LastUpdateMs(uint8_t id_){ return s_last_ms[id_]; }

/* 송신: 단일 바이트 payload (송신 헤더는 표준 순서 H0(0xB7),H1(0xB8)) */
void MD400T_Send1(uint8_t id_, uint8_t pid_, uint8_t len_, uint8_t val){
  if(!s_hu) return;
  uint8_t f[7]={ MD400T_H0, MD400T_H1, id_, pid_, len_, val, 0 };
  f[6] = md_ck(f, 6);
  rs485_tx_begin();
  HAL_UART_Transmit(s_hu,f,sizeof(f),10);
  rs485_tx_end();
}

/* 간단 리트라이 유틸: PID10(CMD=61) 브로드캐스트 ON 안정화 */
static void send_cmd_with_retry(uint8_t id_, uint8_t pid, uint8_t val,
                                uint8_t tries, uint32_t gap_ms)
{
  for (uint8_t i=0; i<tries; i++){
    MD400T_Send1(id_, pid, 1, val);
    HAL_Delay(gap_ms);
    /* PID 210이 최근에 들어오면 성공 판단 */
    if ((HAL_GetTick() - MD400T_LastUpdateMs(id_)) <= 200) break;
  }
}

/* 개별 초기화 (ID별로 호출) */
void MD400T_ApplyInit(uint8_t id_)
{
  if (!id_ || id_==254) return;        // 브로드캐스트 금지

  /* 정지 시 브레이크 */
  MD400T_Send1(id_,  MD400T_PID_STOP_STATUS_MOT1, 1, 2);
  MD400T_Send1(id_,  MD400T_PID_STOP_STATUS_MOT2, 1, 2);

  /* 동기 타입 */
  MD400T_Send1(id_,  MD400T_PID_SYNC_TYPE,        1, 2);

  /* 피드백 부호 보정(필요한 드라이버에만) */
  if (id_ == 2) {
    MD400T_Send1(id_, MD400T_PID_INV_SIGN_OUT,  1, 1); // M1 feedback invert
    MD400T_Send1(id_, MD400T_PID_INV_SIGN_OUT2, 1, 1); // M2 feedback invert
  }

  /* 메인데이터 브로드캐스트 ON — 3회, 30ms 간격 */
  send_cmd_with_retry(id_, MD400T_PID_COMMAND, MD400T_CMD_PNT_MAIN_DATA_BC_ON, 3, 30);
}

/* 속도 명령 (PID=207=0xCF, LEN=7), data=[1,rpmA_L,rpmA_H, 1,rpmB_L,rpmB_H, 0] */
int MD400T_SetSpeedAB_RS485(uint8_t id_, int16_t rpmA, int16_t rpmB){
  if(!s_hu) return -1;

  /* ID2는 출력축 배선/기계 부호 보정 */
  if (id_ == 2) {rpmA = -rpmA; rpmB = -rpmB;}

  uint8_t f[13] = {
    MD400T_H0, MD400T_H1, id_, MD400T_PID_SET_SPEED_AB, 7,
    1, (uint8_t)(rpmA & 0xFF), (uint8_t)((rpmA>>8) & 0xFF),
    1, (uint8_t)(rpmB & 0xFF), (uint8_t)((rpmB>>8) & 0xFF),
    0, 0
  };
  f[sizeof(f)-1] = md_ck(f, sizeof(f)-1);
  rs485_tx_begin();
  HAL_StatusTypeDef st = HAL_UART_Transmit(s_hu, f, sizeof(f), 10);
  rs485_tx_end();
  return (st==HAL_OK)?0:-2;
}

/* 약한 심볼 대체용 strong 심볼 */
int MD400_SetSpeedAB_RS485(uint8_t id, int16_t rpmA, int16_t rpmB){
  return MD400T_SetSpeedAB_RS485(id, rpmA, rpmB);
}

/* (옵션) 워치독: 주기적으로 호출해 PID210 끊기면 BC_ON 재전송 */
void MD400T_Service(void){
  uint32_t now = HAL_GetTick();
  for (uint8_t id=1; id<=2; id++){
    if (now - MD400T_LastUpdateMs(id) > 500){
      MD400T_Send1(id, MD400T_PID_COMMAND, 1, MD400T_CMD_PNT_MAIN_DATA_BC_ON);
    }
  }
}

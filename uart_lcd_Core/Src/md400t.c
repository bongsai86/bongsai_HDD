/*
 * md400t.c  — RS232 듀얼 포트(논리ID) 버전
 *  UART2 ↔ 논리ID 1  (RL=M1, FL=M2)
 *  UART4 ↔ 논리ID 2  (FR=M1, RR=M2)
 *  on-wire 통신 ID는 RS232 전환 후 항상 1 사용
 */
#include "md400t.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ==== 전역 피드백 변수 ==== */
volatile int16_t  FR_M1_CurrRPM=0; volatile float FR_M1_Amp=0.f; volatile uint8_t FR_M1_Status=0; volatile int32_t FR_M1_Encoder=0;
volatile int16_t  RR_M2_CurrRPM=0; volatile float RR_M2_Amp=0.f; volatile uint8_t RR_M2_Status=0; volatile int32_t RR_M2_Encoder=0;
volatile int16_t  RL_M1_CurrRPM=0; volatile float RL_M1_Amp=0.f; volatile uint8_t RL_M1_Status=0; volatile int32_t RL_M1_Encoder=0;
volatile int16_t  FL_M2_CurrRPM=0; volatile float FL_M2_Amp=0.f; volatile uint8_t FL_M2_Status=0; volatile int32_t FL_M2_Encoder=0;

/* ==== 유틸 ==== */
static uint8_t md_ck(const uint8_t *p, uint16_t n){ uint32_t s=0; for(uint16_t i=0;i<n;i++) s+=p[i]; return (uint8_t)(-((int32_t)(s & 0xFF))); }
static inline int16_t u8_to_i16(uint8_t lo, uint8_t hi){ return (int16_t)((uint16_t)lo | ((uint16_t)hi<<8)); }
static inline int32_t u8_to_i32(uint8_t b0,uint8_t b1,uint8_t b2,uint8_t b3){
  return (int32_t)((uint32_t)b0 | ((uint32_t)b1<<8) | ((uint32_t)b2<<16) | ((uint32_t)b3<<24));
}

/* ==== 포트 컨텍스트 ==== */
#define MD_RX_TIMEOUT_MS  20
typedef struct {
  UART_HandleTypeDef *hu;     /* 바인딩된 UART 핸들 */
  /* 파서 상태(H1→H0→ID→PID→LEN→DATA→CK) */
  uint8_t st, hdr0, hdr1, id, pid, len, di;
  uint8_t buf[64];
  uint32_t rx_last_tick;
  uint32_t last_update_ms;    /* PID210 처리 시각 */
} md_port_t;

static md_port_t s_port[3];   /* [1]=UART2, [2]=UART4 만 사용 */

/* RS232 전환: RS485 DE/RE 토글 무효화 */
static inline void tx_begin(void){ (void)0; }
static inline void tx_end(void){   (void)0; }

/* ==== API 구현 ==== */
void MD400T_InitPort(uint8_t logical_id, UART_HandleTypeDef *huart){
  if (logical_id<1 || logical_id>2) return;
  memset(&s_port[logical_id], 0, sizeof(md_port_t));
  s_port[logical_id].hu = huart;
  s_port[logical_id].st = 0;            /* S_H0 */
  s_port[logical_id].di = 0;
  s_port[logical_id].rx_last_tick = 0;
  s_port[logical_id].last_update_ms = 0;
}

uint32_t MD400T_LastUpdateMs_Port(uint8_t logical_id){
  if (logical_id<1 || logical_id>2) return 0;
  return s_port[logical_id].last_update_ms;
}

/* 수신 바이트 주입(포트별) — 수신 헤더 순서는 H1(0xB8) 후 H0(0xB7) */
void MD400T_RxByte_Port(uint8_t logical_id, uint8_t b){
  if (logical_id<1 || logical_id>2) return;
  md_port_t *p = &s_port[logical_id];

  enum { S_H0=0,S_H1,S_ID,S_PID,S_LEN,S_DAT,S_CK };
  uint32_t now = HAL_GetTick();
  if (now - p->rx_last_tick > MD_RX_TIMEOUT_MS) { p->st = S_H0; p->di = 0; }
  p->rx_last_tick = now;

  switch(p->st){
    case S_H0: if (b==MD400T_H1){ p->hdr0=b; p->st=S_H1; } else p->st=S_H0; break;
    case S_H1: if (b==MD400T_H0){ p->hdr1=b; p->st=S_ID; } else p->st=S_H0; break;
    case S_ID: p->id=b; p->st=S_PID; break;
    case S_PID: p->pid=b; p->st=S_LEN; break;
    case S_LEN:
      p->len=b;
      if (p->len > sizeof(p->buf)){ p->di=0; p->st=S_H0; break; }
      p->di=0; p->st=(p->len==0)?S_CK:S_DAT;
      break;
    case S_DAT:
      p->buf[p->di++] = b;
      if (p->di >= p->len) p->st = S_CK;
      break;
    case S_CK: {
      uint16_t nsum = 2 + 1 + 1 + 1 + p->len;
      uint8_t  tmp[2 + 1 + 1 + 1 + 64];
      tmp[0]=p->hdr0; tmp[1]=p->hdr1; tmp[2]=p->id; tmp[3]=p->pid; tmp[4]=p->len;
      if (p->len) memcpy(&tmp[5], p->buf, p->len);

      uint8_t ok = (md_ck(tmp, nsum) == b);

      if (ok && p->pid==MD400T_PID_PNT_MAIN_DATA && p->len==MD400T_PNT_MAIN_LEN){
        int16_t  m1_rpm = u8_to_i16(p->buf[0],p->buf[1]);
        uint16_t m1_a10 = (uint16_t)u8_to_i16(p->buf[2],p->buf[3]);
        uint8_t  m1_st  = p->buf[4];
        int32_t  m1_enc = u8_to_i32(p->buf[5],p->buf[6],p->buf[7],p->buf[8]);

        int16_t  m2_rpm = u8_to_i16(p->buf[9],p->buf[10]);
        uint16_t m2_a10 = (uint16_t)u8_to_i16(p->buf[11],p->buf[12]);
        uint8_t  m2_st  = p->buf[13];
        int32_t  m2_enc = u8_to_i32(p->buf[14],p->buf[15],p->buf[16],p->buf[17]);

        float m1_amp = (float)m1_a10 * 0.1f;
        float m2_amp = (float)m2_a10 * 0.1f;

        if (logical_id==1){ /* UART2: RL(M1), FL(M2) */
          RL_M1_CurrRPM=m1_rpm; RL_M1_Amp=m1_amp; RL_M1_Status=m1_st; RL_M1_Encoder=m1_enc;
          FL_M2_CurrRPM=m2_rpm; FL_M2_Amp=m2_amp; FL_M2_Status=m2_st; FL_M2_Encoder=m2_enc;
        } else {            /* UART4: FR(M1), RR(M2) */
          FR_M1_CurrRPM=m1_rpm; FR_M1_Amp=m1_amp; FR_M1_Status=m1_st; FR_M1_Encoder=m1_enc;
          RR_M2_CurrRPM=m2_rpm; RR_M2_Amp=m2_amp; RR_M2_Status=m2_st; RR_M2_Encoder=m2_enc;
        }
        p->last_update_ms = now;
      }
      p->st = S_H0;
    } break;
  }
}

/* 단순 세팅 송신(예: BC_ON). on-wire ID=1 고정 */
void MD400T_Send1_Port(uint8_t logical_id, uint8_t pid_, uint8_t len_, uint8_t val){
  if (logical_id<1 || logical_id>2) return;
  md_port_t *p = &s_port[logical_id];
  if (!p->hu) return;

  uint8_t f[7] = { MD400T_H0, MD400T_H1, 1 /*on-wire ID*/, pid_, len_, val, 0 };
  f[6] = md_ck(f, 6);
  tx_begin();
  HAL_UART_Transmit(p->hu, f, sizeof(f), 10);
  tx_end();
}

/* 2바이트(INT, LSB first) 세팅 송신 — Slow Start/Down 등 */
void MD400T_Send2_Port(uint8_t logical_id, uint8_t pid_, uint16_t val){
  if (logical_id<1 || logical_id>2) return;
  md_port_t *p = &s_port[logical_id];
  if (!p->hu) return;

  uint8_t f[9] = { MD400T_H0, MD400T_H1, 1, pid_, 2,
                   (uint8_t)(val & 0xFF), (uint8_t)((val>>8) & 0xFF), 0 };
  f[8] = md_ck(f, 8);
  tx_begin();
  HAL_UART_Transmit(p->hu, f, sizeof(f), 10);
  tx_end();
}

/* 개별 초기화(논리ID로 포트 선택, on-wire ID=1)
 * 출력 순서(각 프레임 사이 100ms 지연, 1회씩만):
 *  B7 B8 01 18 01 02 75
 *  B7 B8 01 1C 01 02 71
 *  (lid2 전용) 22/23 = 1
 *  (옵션) Slow Start/Down 108/109/111/112
 *  B7 B8 01 0A 01 3D 48
 */
void MD400T_ApplyInit(uint8_t logical_id){
  if (logical_id < 1 || logical_id > 2) return;

  /* 1) MOT1/2 정지 시 브레이크 */
  MD400T_Send1_Port(logical_id, MD400T_PID_STOP_STATUS_MOT1, 1, 2);
  HAL_Delay(100);
  MD400T_Send1_Port(logical_id, MD400T_PID_STOP_STATUS_MOT2, 1, 2);
  HAL_Delay(100);

  /* 2) 논리ID2: 피드백 부호 반전(22,23) */
  if (logical_id == 2){
    MD400T_Send1_Port(2, MD400T_PID_INV_SIGN_OUT,  1, 1);  /* M1 */
    HAL_Delay(100);
    MD400T_Send1_Port(2, MD400T_PID_INV_SIGN_OUT2, 1, 1);  /* M2 */
    HAL_Delay(100);
  }

  /* 3) Slow Start / Slow Down (필요시 값은 md400t.h의 *_DEFAULT로 조정) */
  MD400T_Send2_Port(logical_id, MD400T_PID_SLOW_START1, MD400T_SS1_DEFAULT);
  HAL_Delay(100);
  MD400T_Send2_Port(logical_id, MD400T_PID_SLOW_START2, MD400T_SS2_DEFAULT);
  HAL_Delay(100);
  MD400T_Send2_Port(logical_id, MD400T_PID_SLOW_DOWN1,  MD400T_SD1_DEFAULT);
  HAL_Delay(100);
  MD400T_Send2_Port(logical_id, MD400T_PID_SLOW_DOWN2,  MD400T_SD2_DEFAULT);
  HAL_Delay(100);

  /* 4) 메인데이터 브로드캐스트 ON — 1회 */
  MD400T_Send1_Port(logical_id, MD400T_PID_COMMAND, 1, MD400T_CMD_PNT_MAIN_DATA_BC_ON);
  HAL_Delay(100);
}

/* 속도 명령 — 논리ID로 포트를 선택. on-wire ID=1 고정.
   데이터: [1,rpmA_L,rpmA_H, 1,rpmB_L,rpmB_H, 0] */
int MD400T_SetSpeedAB(uint8_t logical_id, int16_t rpmA, int16_t rpmB){
  if (logical_id<1 || logical_id>2) return -1;
  md_port_t *p = &s_port[logical_id];
  if (!p->hu) return -1;

  /* 논리ID 2는 배선상 CW/CCW 반대 → 명령 부호 반전 */
  if (logical_id == 2) { rpmA = -rpmA; rpmB = -rpmB; }

  uint8_t f[13] = {
    MD400T_H0, MD400T_H1, 1, MD400T_PID_SET_SPEED_AB, 7,
    1, (uint8_t)(rpmA & 0xFF), (uint8_t)((rpmA>>8) & 0xFF),
    1, (uint8_t)(rpmB & 0xFF), (uint8_t)((rpmB>>8) & 0xFF),
    0, 0
  };
  f[sizeof(f)-1] = md_ck(f, sizeof(f)-1);

  tx_begin();
  HAL_StatusTypeDef st = HAL_UART_Transmit(p->hu, f, sizeof(f), 10);
  tx_end();

  return (st==HAL_OK)?0:-2;
}

/* 호환 strong 심볼: 기존 RS485 함수명 유지 */
int MD400_SetSpeedAB_RS485(uint8_t logical_id, int16_t rpmA, int16_t rpmB){
  return MD400T_SetSpeedAB(logical_id, rpmA, rpmB);
}

/* 포트 생존 확인: 최근 PID210 수신 나이 검사 */
uint8_t MD400T_PortAlive(uint8_t logical_id, uint32_t max_age_ms){
  if (logical_id<1 || logical_id>2) return 0;
  uint32_t last = MD400T_LastUpdateMs_Port(logical_id);
  if (last == 0) return 0;
  return (HAL_GetTick() - last) <= max_age_ms;
}

/* 드라이버 전원 지연 대비: 필요 시 재초기화 루프(현재는 사용 안 함) */
void MD400T_WaitAliveAndInit(uint8_t logical_id, uint32_t timeout_ms){
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeout_ms){
    MD400T_ApplyInit(logical_id);

    uint32_t t1 = HAL_GetTick();
    while ((HAL_GetTick() - t1) < 1000){
      if (MD400T_PortAlive(logical_id, 200)) return;   /* 피드백 활성화됨 */
      HAL_Delay(100);
    }
  }
}

/* 주기적 BC_ON 재전송 비활성화(중복 방지) */
void MD400T_Service(void){ /* no-op */ }

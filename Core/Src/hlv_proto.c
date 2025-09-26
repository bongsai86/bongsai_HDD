/*
 * hlv_proto.c
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */
#include "hlv_proto.h"
#include <string.h>
#include <math.h>

/* md400t.c 전역 RPM (PID 210 결과) */
extern volatile int16_t FR_M1_CurrRPM, RR_M2_CurrRPM;
extern volatile int16_t RL_M1_CurrRPM, FL_M2_CurrRPM;

static UART_HandleTypeDef *s_hu = NULL;

void HLV_Init(UART_HandleTypeDef *huart){ s_hu = huart; }

/* STX 'S''T''X', AorM, ESTOP, MODE, SPEED(be i16,m/s*100), STEER(be i16,deg),
   ALIVE, CR 0x0D, LF 0x0A */
enum {S0,S1,S2,B_AORM,B_ESTOP,B_MODE,B_VH,B_VL,B_TH,B_TL,B_ALV,B_CR,B_LF};
static uint8_t st=S0, buf[8];

static inline int16_t be_i16(uint8_t h, uint8_t l){ return (int16_t)((h<<8)|l); }
static volatile HLV_Cmd s_last = {0};
void HLV_RxByte(uint8_t b){
  switch(st){
    case S0: st=(b=='S')?S1:S0; break;
    case S1: st=(b=='T')?S2:S0; break;
    case S2: st=(b=='X')?B_AORM:S0; break;
    case B_AORM: buf[0]=b; st=B_ESTOP; break;
    case B_ESTOP: buf[1]=b; st=B_MODE; break;
    case B_MODE: buf[2]=b; st=B_VH; break;
    case B_VH: buf[3]=b; st=B_VL; break;
    case B_VL: buf[4]=b; st=B_TH; break;
    case B_TH: buf[5]=b; st=B_TL; break;
    case B_TL: buf[6]=b; st=B_ALV; break;
    case B_ALV: buf[7]=b; st=B_CR; break;
    case B_CR:  st=(b==0x0D)?B_LF:S0; break;
    case B_LF:
      if (b==0x0A){
        int16_t v100=be_i16(buf[3],buf[4]);   /* m/s*100 */
        int16_t th=be_i16(buf[5],buf[6]);   /* deg  */
        double v = ((double)v100)/100.0;
        double t = ((double)th);

        HLV_Cmd c;
        c.aorm = buf[0]?1:0;
        c.estop= buf[1]?1:0;
        c.mode = (buf[2]<=2)?(steer_mode_t)buf[2]:DM_2WIS;
        if (v> HLV_SPD_MAX_MPS) v= HLV_SPD_MAX_MPS;
        if (v<-HLV_SPD_MAX_MPS) v=-HLV_SPD_MAX_MPS;
        double lim = (c.mode==DM_4WIS)?18.0:30.0;
        if (t >  lim) { t =  lim; }
        if (t < -lim) { t = -lim; }
        c.speed_mps = v;
        c.steer_deg = t;
        c.alive=buf[7];
        c.tick_ms=HAL_GetTick();
        s_last=c;
      }
      st=S0; break;
    default: st=S0; break;
  }
}

bool HLV_GetLatest(HLV_Cmd *out, uint32_t *age_ms){
  HLV_Cmd snap = s_last;
  uint32_t age = HAL_GetTick() - snap.tick_ms;
  if (out) *out = snap;
  if (age_ms) *age_ms = age;
  return (age <= HLV_TMO_MS);
}

/* PCU->HLV: 25B, little-endian
   STX 'S''T''X', AorM, ESTOP, MODE,
   vFL/vFR/vRL/vRR (i16 le, =m/s*100),
   aFL/aFR/aRL/aRR (i16 le, =deg*10),
   ALIVE, CR, LF */
static uint8_t pcualive=0;
static inline void le_i16(uint8_t *p, int16_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)((v>>8)&0xFF); }

void HLV_SendFeedback(const wheel_cmd_t *w, steer_mode_t mode,
                      uint8_t aorm, uint8_t estop){
  if (!s_hu) return;
  uint8_t p[25]; memset(p,0,sizeof(p));
  p[0]='S'; p[1]='T'; p[2]='X';
  p[3]=aorm?1:0; p[4]=estop?1:0; p[5]=(uint8_t)mode;

  /* RPM을 m/s로 변환해서 전송 */
  int16_t rpmFL = FL_M2_CurrRPM;  /* ID1 M2 = FL */
  int16_t rpmFR = FR_M1_CurrRPM;  /* ID2 M1 = FR */
  int16_t rpmRL = RL_M1_CurrRPM;  /* ID1 M1 = RL */
  int16_t rpmRR = RR_M2_CurrRPM;  /* ID2 M2 = RR */

  /* ② RPM -> m/s: y = x * 0.01 * 20.6 * pi / 60 */
  const double K = 0.01 * 20.6 * 3.141592653589793 / 60.0;
  double vFL_mps = (double)rpmFL * K;
  double vFR_mps = (double)rpmFR * K;
  double vRL_mps = (double)rpmRL * K;
  double vRR_mps = (double)rpmRR * K;

  /* ③ m/s*100로 패킹 */
  int16_t sFL = (int16_t)lround(vFL_mps * 100.0);
  int16_t sFR = (int16_t)lround(vFR_mps * 100.0);
  int16_t sRL = (int16_t)lround(vRL_mps * 100.0);
  int16_t sRR = (int16_t)lround(vRR_mps * 100.0);

  int16_t tFL = (int16_t)lround(w->ang_fl * 10.0);
  int16_t tFR = (int16_t)lround(w->ang_fr * 10.0);
  int16_t tRL = (int16_t)lround(w->ang_rl * 10.0);
  int16_t tRR = (int16_t)lround(w->ang_rr * 10.0);

  le_i16(&p[ 6], sFL); le_i16(&p[ 8], sFR);
  le_i16(&p[10], sRL); le_i16(&p[12], sRR);
  le_i16(&p[14], tFL); le_i16(&p[16], tFR);
  le_i16(&p[18], tRL); le_i16(&p[20], tRR);

  p[22]=pcualive++; p[23]=0x0D; p[24]=0x0A;
  HAL_UART_Transmit(s_hu, p, sizeof(p), 5);
}



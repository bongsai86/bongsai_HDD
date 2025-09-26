#ifndef INC_MD400T_H_
#define INC_MD400T_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==== 프로토콜 상수 ==== */
#define MD400T_H0                    0xB7
#define MD400T_H1                    0xB8
#define MD400T_BROADCAST_ID          254  /* 사용 지양(개별 초기화 권장) */

/* ==== PID / COMMAND ==== */
#define MD400T_PID_COMMAND             10
#define MD400T_CMD_PNT_MAIN_DATA_BC_ON 61

// 명령 반전(참고)
// #define MD400T_PID_INV_SIGN_CMD   16   // (기존) BIT0=M1, BIT1=M2
// #define MD400T_PID_INV_SIGN_CMD1  18   // (기존) M2 개별

// ▼ (추가) 피드백(OUT) 부호 반전 PID — 매뉴얼의 PID 값으로 설정할 것
#define MD400T_PID_INV_SIGN_OUT        22  // M1 feedback invert (DATA: 0/1)
#define MD400T_PID_INV_SIGN_OUT2       23  // M2 feedback invert (DATA: 0/1)


#define MD400T_PID_STOP_STATUS_MOT1    24
#define MD400T_PID_STOP_STATUS_MOT2    28
#define MD400T_PID_SYNC_TYPE           82

/* 수신 Point Data */
#define MD400T_PID_PNT_MAIN_DATA       210  /* 0xD2 */
#define MD400T_PNT_MAIN_LEN            18

/* 속도 명령 */
#define MD400T_PID_SET_SPEED_AB        207  /* 0xCF, LEN=7 */

/* ==== 파싱 결과 (배선 주의)
 * 배선: ID1 = RL(M1) / FL(M2),  ID2 = FR(M1) / RR(M2)
 */
extern volatile int16_t  FR_M1_CurrRPM;
extern volatile float    FR_M1_Amp;
extern volatile uint8_t  FR_M1_Status;
extern volatile int32_t  FR_M1_Encoder;

extern volatile int16_t  RR_M2_CurrRPM;
extern volatile float    RR_M2_Amp;
extern volatile uint8_t  RR_M2_Status;
extern volatile int32_t  RR_M2_Encoder;

extern volatile int16_t  RL_M1_CurrRPM;
extern volatile float    RL_M1_Amp;
extern volatile uint8_t  RL_M1_Status;
extern volatile int32_t  RL_M1_Encoder;

extern volatile int16_t  FL_M2_CurrRPM;
extern volatile float    FL_M2_Amp;
extern volatile uint8_t  FL_M2_Status;
extern volatile int32_t  FL_M2_Encoder;

/* ==== API ==== */
void     MD400T_Init(UART_HandleTypeDef *huart_rs485);
void     MD400T_RxByte(uint8_t b);
uint32_t MD400T_LastUpdateMs(uint8_t id);

/* 단순 세팅 송신 */
void     MD400T_Send1(uint8_t id, uint8_t pid, uint8_t len, uint8_t val);

/* 개별 초기화(권장) */
void     MD400T_ApplyInit(uint8_t id);   /* ID=1,2 각각 호출 */

/* 속도 명령 (M1=A, M2=B) */
int      MD400T_SetSpeedAB_RS485(uint8_t id, int16_t rpmA, int16_t rpmB);

/* 약한 심볼 대체용 strong 심볼(필요 시) */
int      MD400_SetSpeedAB_RS485(uint8_t id, int16_t rpmA, int16_t rpmB);

/* 디버그(옵션) */
int      MD400T_PopFrame(uint8_t *out, uint8_t *len_out);
void     MD400T_DebugDump(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_MD400T_H_ */

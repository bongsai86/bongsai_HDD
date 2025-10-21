#ifndef INC_MD400T_H_
#define INC_MD400T_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==== 프로토콜 상수 ==== */
#define MD400T_H0                       0xB7
#define MD400T_H1                       0xB8
#define MD400T_BROADCAST_ID             254  /* 사용 지양(개별 초기화 권장) */

/* ==== PID / COMMAND ==== */
#define MD400T_PID_COMMAND              10
#define MD400T_CMD_PNT_MAIN_DATA_BC_ON  61

/* 피드백(OUT) 부호 반전 PID */
#define MD400T_PID_INV_SIGN_OUT         22  /* M1 feedback invert (DATA: 0/1) */
#define MD400T_PID_INV_SIGN_OUT2        23  /* M2 feedback invert (DATA: 0/1) */

#define MD400T_PID_STOP_STATUS_MOT1     24
#define MD400T_PID_STOP_STATUS_MOT2     28
#define MD400T_PID_SYNC_TYPE            82

/* Slow Start / Slow Down (INT, LEN=2, 0~1023 ≈ 0~15s) */
#define MD400T_PID_SLOW_START1          108
#define MD400T_PID_SLOW_START2          109
#define MD400T_PID_SLOW_DOWN1           111
#define MD400T_PID_SLOW_DOWN2           112

/* 기본값(원하면 빌드시 변경) */
#ifndef MD400T_SS1_DEFAULT
#define MD400T_SS1_DEFAULT              512
#endif
#ifndef MD400T_SS2_DEFAULT
#define MD400T_SS2_DEFAULT              512
#endif
#ifndef MD400T_SD1_DEFAULT
#define MD400T_SD1_DEFAULT              256
#endif
#ifndef MD400T_SD2_DEFAULT
#define MD400T_SD2_DEFAULT              256
#endif

/* 수신 Point Data */
#define MD400T_PID_PNT_MAIN_DATA        210  /* 0xD2 */
#define MD400T_PNT_MAIN_LEN             18

/* 속도 명령 */
#define MD400T_PID_SET_SPEED_AB         207  /* 0xCF, LEN=7 */

/* ==== 파싱 결과 (배선 주의)
 * 배선: 논리ID1(UART2) = RL(M1) / FL(M2)
 *       논리ID2(UART4) = FR(M1) / RR(M2)
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

/* ==== RS232 이중 포트 API (논리ID 기준) ====
 * 논리ID: 1 → UART2,  2 → UART4
 * on-wire 통신 ID는 RS232 전환 후 항상 1 사용
 */

/* 포트 바인딩 */
void     MD400T_InitPort(uint8_t logical_id, UART_HandleTypeDef *huart);

/* 수신 바이트(포트별 파서) */
void     MD400T_RxByte_Port(uint8_t logical_id, uint8_t b);

/* 최근 업데이트 시각(ms) */
uint32_t MD400T_LastUpdateMs_Port(uint8_t logical_id);

/* 단순 세팅 송신(예: BC_ON 등) */
void     MD400T_Send1_Port(uint8_t logical_id, uint8_t pid, uint8_t len, uint8_t val);

/* 2바이트(INT) 세팅 송신(예: Slow Start/Down) */
void     MD400T_Send2_Port(uint8_t logical_id, uint8_t pid, uint16_t val);

/* 개별 초기화(권장): 논리ID별 호출 */
void     MD400T_ApplyInit(uint8_t logical_id);

/* 속도 명령 (M1=A, M2=B) — 논리ID로 포트 선택 */
int      MD400T_SetSpeedAB(uint8_t logical_id, int16_t rpmA, int16_t rpmB);

/* 상태 확인/대기 유틸 */
uint8_t  MD400T_PortAlive(uint8_t logical_id, uint32_t max_age_ms);
void     MD400T_WaitAliveAndInit(uint8_t logical_id, uint32_t timeout_ms);

/* 주기적 유지보수(브로드캐스트 재요청 등) */
void     MD400T_Service(void);

/* 호환 매크로/심볼: 기존 RS485 함수명을 RS232 포트 라우팅으로 매핑 */
#define  MD400T_SetSpeedAB_RS485(logical_id, rpmA, rpmB) MD400T_SetSpeedAB((logical_id), (rpmA), (rpmB))
int      MD400_SetSpeedAB_RS485(uint8_t logical_id, int16_t rpmA, int16_t rpmB);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_MD400T_H_ */

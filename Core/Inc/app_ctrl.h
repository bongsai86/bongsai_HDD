/*
 * app_ctrl.h
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */

#ifndef INC_APP_CTRL_H_
#define INC_APP_CTRL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 상태 */
typedef enum { ST_BOOT = 0, ST_ESTOP, ST_AUTO, ST_AUTO_FAIL, ST_MANUAL, ST_RC_VIEW } ctrl_state_t;
typedef enum { DRIVE_MANUAL = 0, DRIVE_AUTO, DRIVE_AUTO_FAIL, DRIVE_RC_VIEW } drive_mode_t;

/* 선택: 현재 드라이브 모드 조회 */
drive_mode_t CTRL_GetDriveMode(void);

/* Board silkscreen PWMx -> TIM3_CHy mapping (확정) */
#define PWM1_TO_CH   2   /* PWM1 = TIM3_CH2 (PC7) */
#define PWM2_TO_CH   3   /* PWM2 = TIM3_CH3 (PB0) */
#define PWM3_TO_CH   4   /* PWM3 = TIM3_CH4 (PB1) */
#define PWM4_TO_CH   1   /* PWM4 = TIM3_CH1 (PC6) */

/* 로봇 축 ↔ 보드 PWM 라벨 매핑 (당신이 확인한 ‘원하는 동작’ 기준) */
#define SERVO_CH_FR  PWM1_TO_CH  /* FR = PWM1 */
#define SERVO_CH_FL  PWM2_TO_CH  /* FL = PWM2 */
#define SERVO_CH_RR  PWM3_TO_CH  /* RR = PWM3 */
#define SERVO_CH_RL  PWM4_TO_CH  /* RL = PWM4 */

/* 서보 극성(+1/-1)과 트림[deg]
#define SERVO_SIGN_FL (-1)
#define SERVO_SIGN_FR (+1)
#define SERVO_SIGN_RL (+1)
#define SERVO_SIGN_RR (-1)

#define SERVO_TRIM_FL (0.0)
#define SERVO_TRIM_FR (0.0)
#define SERVO_TRIM_RL (0.0)
#define SERVO_TRIM_RR (0.0)
*/

/* 타임아웃·램프 */
#define RC_TIMEOUT_MS      60
#define HLC_TIMEOUT_MS     300
#define RPM_MAX            150
#define RPM_RAMP_PER_TICK  15   /* 50ms */

/* API */
void CTRL_Init(void);
void CTRL_Tick50ms(void);

/* 약한 심볼 */
bool CTRL_ReadEStop(void) __attribute__((weak));
int  MD400_SetSpeedAB_RS485(uint8_t drv_id, int16_t rpmA, int16_t rpmB) __attribute__((weak));

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* INC_APP_CTRL_H_ */

/*
 * servo_pwm.h
 *
 *  Created on: Aug 20, 2025
 *      Author: bongs
 */

#ifndef INC_SERVO_PWM_H_
#define INC_SERVO_PWM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Super400 servo config
#define SERVO_CENTER_US   1500
#define SERVO_MIN_US       800
#define SERVO_MAX_US      2200
//#define SERVO_DEG2US    7.7778f   // (2200-800)/180

void SERVO_Init(TIM_HandleTypeDef *htim3);
void SERVO_SetUS(uint8_t ch, uint16_t us);  // ch: 1~4, us: 800~2200 권장

#ifdef __cplusplus
}
#endif
#endif /* SERVO_PWM_H */

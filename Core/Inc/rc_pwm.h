/*
 * rc_pwm.h
 *
 *  Created on: Aug 19, 2025
 *      Author: bongs
 */

#ifndef INC_RC_PWM_H_
#define INC_RC_PWM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void RC_PWM_Init(TIM_HandleTypeDef *htim2,
                 GPIO_TypeDef *port,
                 uint16_t pin_ch1, uint16_t pin_ch2,
                 uint16_t pin_ch3, uint16_t pin_ch4);

void RC_PWM_OnEdge(uint16_t GPIO_Pin);      // HAL_GPIO_EXTI_Callback()에서 호출
void RC_PWM_GetMicros(uint32_t out_us[4]);  // us 스냅샷 복사

#ifdef __cplusplus
}
#endif
#endif /* RC_PWM_H */

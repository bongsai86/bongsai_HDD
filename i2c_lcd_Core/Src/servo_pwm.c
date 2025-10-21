/*
 * servo_pwm.c
 *
 *  Created on: Aug 20, 2025
 *      Author: bongs
 */


#include "servo_pwm.h"

static TIM_HandleTypeDef *s_tim3;

void SERVO_Init(TIM_HandleTypeDef *htim3){
  s_tim3 = htim3;
  HAL_TIM_PWM_Start(s_tim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(s_tim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(s_tim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(s_tim3, TIM_CHANNEL_4);
  // 중립(1500us)로 초기화
  SERVO_SetUS(1,1500); SERVO_SetUS(2,1500);
  SERVO_SetUS(3,1500); SERVO_SetUS(4,1500);
}


// 중립 유지 (1500us 그대로 OK)
void SERVO_SetUS(uint8_t ch, uint16_t us){
  if (us < 800)  us = 800;     // Super400: 800–2200 us
  if (us > 2200) us = 2200;
  uint32_t chx =
    (ch==1)?TIM_CHANNEL_1:(ch==2)?TIM_CHANNEL_2:(ch==3)?TIM_CHANNEL_3:TIM_CHANNEL_4;
  __HAL_TIM_SET_COMPARE(s_tim3, chx, us);  // TIM3=1MHz → CCR=µs
}

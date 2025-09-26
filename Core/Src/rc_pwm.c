/*
 * rc_pwm.c
 *
 *  Created on: Aug 19, 2025
 *      Author: bongs
 */

#include "rc_pwm.h"

#define RC_MIN_US 700u
#define RC_MAX_US 2300u

static TIM_HandleTypeDef *s_tim2;
static GPIO_TypeDef *s_port;
static uint16_t s_pins[4];

static volatile uint32_t s_rise_us[4]  = {0,0,0,0};
static volatile uint32_t s_width_us[4] = {0,0,0,0};

static inline uint32_t tnow_us(void){
  return __HAL_TIM_GET_COUNTER(s_tim2);   // TIM2 1us tick (PSC=83)
}

static inline int pin_to_idx(uint16_t pin){
  for (int i=0;i<4;i++) if (s_pins[i]==pin) return i;
  return -1;
}

void RC_PWM_Init(TIM_HandleTypeDef *htim2,
                 GPIO_TypeDef *port,
                 uint16_t pin_ch1, uint16_t pin_ch2,
                 uint16_t pin_ch3, uint16_t pin_ch4)
{
  s_tim2 = htim2;
  s_port = port;
  s_pins[0]=pin_ch1; s_pins[1]=pin_ch2; s_pins[2]=pin_ch3; s_pins[3]=pin_ch4;

  // TIM2는 외부에서 MX_TIM2_Init 후 HAL_TIM_Base_Start() 되어 있어야 함.
}

void RC_PWM_OnEdge(uint16_t GPIO_Pin)
{
  int idx = pin_to_idx(GPIO_Pin);
  if (idx < 0) return;

  GPIO_PinState s = HAL_GPIO_ReadPin(s_port, GPIO_Pin);
  uint32_t t = tnow_us();

  if (s == GPIO_PIN_SET) {
    s_rise_us[idx] = t;                          // rising
  } else {
    uint32_t dt = t - s_rise_us[idx];            // falling
    if (dt >= RC_MIN_US && dt <= RC_MAX_US) {    // 글리치 필터
      s_width_us[idx] = dt;
    }
  }
}

void RC_PWM_GetMicros(uint32_t out_us[4])
{
  __disable_irq();                 // 짧은 크리티컬 섹션
  out_us[0]=s_width_us[0];
  out_us[1]=s_width_us[1];
  out_us[2]=s_width_us[2];
  out_us[3]=s_width_us[3];
  __enable_irq();
}


/*
 * hlv_proto.h
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */

#ifndef INC_HLV_PROTO_H_
#define INC_HLV_PROTO_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HLV_SPD_MAX_MPS 1.5
#define HLV_TMO_MS      300

typedef struct {
  uint8_t aorm;            /* 0=Manual, 1=Auto */
  uint8_t estop;           /* 0/1 */
  steer_mode_t mode;       /* 0:2WIS,1:4WIS,2:PIVOT */
  double speed_mps;        /* [-1.5..1.5] */
  double steer_deg;        /* [-30..30] */
  uint8_t alive;
  uint32_t tick_ms;
} HLV_Cmd;

void HLV_Init(UART_HandleTypeDef *huart);        /* ex) &huart3 */
void HLV_RxByte(uint8_t b);                      /* USART Rx IT에서 1바이트씩 호출 */
bool HLV_GetLatest(HLV_Cmd *out, uint32_t *age_ms);
void HLV_SendFeedback(const wheel_cmd_t *w, steer_mode_t mode,
                      uint8_t aorm, uint8_t estop);

#ifdef __cplusplus
}
#endif

#endif /* INC_HLV_PROTO_H_ */

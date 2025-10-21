/*
 * lcd_kclr2.h
 *
 *  Created on: Sep 12, 2025
 *      Author: bongs
 */

#ifndef INC_LCD_KCLR2_H_
#define INC_LCD_KCLR2_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* LC-KLCD-R2 DISPLAY COMMAND */
#define CHAR_LCD_RESET        0xCA
#define CHAR_LCD_LINE         0xCB
#define CHAR_LCD_SHOW         0xCC
#define CHAR_LCD_INSTRUCTION  0xCD
#define CHAR_LCD_CGRAM        0xCE

#ifdef __cplusplus
extern "C" {
#endif

void LCD_Init(UART_HandleTypeDef *hu);
void LCD_Clear(void);
void LCD_Position(uint8_t col_1to16, uint8_t row_1to2);
void LCD_PutChar(uint8_t ch);                 /* 0xCC 후 1char */
void LCD_PrintAt(uint8_t row, uint8_t col, const char *s);
void LCD_PrintPad(uint8_t row, uint8_t col, const char *s, uint8_t width);

/* ROMO-B 상태 4영역 표시(16x2 고정 자리) */
#include "app_ctrl.h"
#include "kinematics.h"
void LCD_UpdateStatus(uint8_t hlv_ok,
                      ctrl_state_t st,
                      steer_mode_t smode,
                      uint8_t estop_remote, /* Re_E-ST */
                      uint8_t estop_button, /* Bu_E-ST */
                      uint8_t estop_hlv);   /* Hi_E-ST */

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_KCLR2_H_ */

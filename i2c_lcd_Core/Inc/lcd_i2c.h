/*
 * lcd_i2c.h
 *
 *  Created on: Oct 13, 2025
 *      Author: bongs
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include <stdint.h>
#include "app_ctrl.h"   /* ctrl_state_t, steer_mode_t */

/* P0=RS, P1=RW(미사용), P2=EN, P3=BL, P4..P7=D4..D7 */
#define LCD_PIN_RS   0x01  /* P0 */
#define LCD_PIN_EN   0x04  /* P2 */
#define LCD_PIN_BL   0x08  /* P3 : 백라이트 ON(활성 High) */
#define LCD_DATA_MASK 0xF0  /* P4..P7 -> D4..D7 */

void LCD_Init(void *hi2c, uint8_t i2c_addr7bit);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *s);
void LCD_PrintAt(uint8_t row, uint8_t col, const char *s);

void LCD_UpdateStatus(uint8_t hlv_ok,
                      ctrl_state_t st,
                      steer_mode_t smode,
                      uint8_t estop_remote,
                      uint8_t estop_button,
                      uint8_t estop_hlv);

#endif /* INC_LCD_I2C_H_ */

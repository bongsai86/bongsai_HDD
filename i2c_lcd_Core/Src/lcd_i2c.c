/*
 * lcd_i2c.c
 * I2C HD44780 16x2
 */
#include "lcd_i2c.h"
#include "stm32f4xx_hal.h"   // I2C_HandleTypeDef, HAL_*
#include "app_ctrl.h"        // ctrl_state_t, steer_mode_t
#include <string.h>
#include <stdio.h>

/* HD44780 commands */
#define LCD_FUNSET_4BIT_2LINE  0x28
#define LCD_DISP_ON            0x0C
#define LCD_ENTRY_INC          0x06
#define LCD_CLR                0x01
#define LCD_HOME               0x02
#define LCD_SET_DDRAM          0x80

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t s_addr = 0x27;       /* 7-bit I2C addr */
static uint8_t s_bl   = LCD_PIN_BL; /* backlight bit always on */

/* --- low level --- */
static HAL_StatusTypeDef expander_write(uint8_t v){
  if (!s_hi2c) return HAL_ERROR;
  return HAL_I2C_Master_Transmit(s_hi2c, (uint16_t)(s_addr<<1), &v, 1, 10);
}
static HAL_StatusTypeDef pulse_en(uint8_t v){
  HAL_StatusTypeDef r = expander_write((uint8_t)(v | LCD_PIN_EN));
  if (r != HAL_OK) return r;
  r = expander_write((uint8_t)(v & (uint8_t)~LCD_PIN_EN));
  HAL_Delay(1);
  return r;
}
static HAL_StatusTypeDef write4(uint8_t nibble, uint8_t rs){
  uint8_t v = (uint8_t)((nibble & 0xF0) | s_bl | (rs ? LCD_PIN_RS : 0));
  HAL_StatusTypeDef r = expander_write(v);
  if (r != HAL_OK) return r;
  return pulse_en(v);
}
static HAL_StatusTypeDef write8(uint8_t data, uint8_t rs){
  HAL_StatusTypeDef r = write4((uint8_t)(data & 0xF0), rs);
  if (r != HAL_OK) return r;
  return write4((uint8_t)((data<<4) & 0xF0), rs);
}
static void cmd(uint8_t c){ (void)write8(c, 0); }
static void data(uint8_t d){ (void)write8(d, 1); }

/* --- init --- */
static void init_hd44780(void){
  HAL_Delay(50);
  (void)write4(0x30, 0); HAL_Delay(5);
  (void)write4(0x30, 0); HAL_Delay(5);
  (void)write4(0x30, 0); HAL_Delay(5);
  (void)write4(0x20, 0); HAL_Delay(5);

  cmd(LCD_FUNSET_4BIT_2LINE);
  cmd(LCD_DISP_ON);
  cmd(LCD_CLR);
  HAL_Delay(2);
  cmd(LCD_ENTRY_INC);
}

static uint8_t try_pick_addr(uint8_t hint){
  if (hint){
    if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(hint<<1), 1, 20) == HAL_OK) return hint;
  }
  if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(0x27<<1), 1, 20) == HAL_OK) return 0x27;
  if (HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)(0x3F<<1), 1, 20) == HAL_OK) return 0x3F;
  return hint ? hint : 0x27;
}

/* --- public API --- */
void LCD_Init(void *hi2c /* I2C_HandleTypeDef* */, uint8_t i2c_addr7bit){
  s_hi2c = (I2C_HandleTypeDef*)hi2c;
  s_addr = try_pick_addr(i2c_addr7bit);
  s_bl   = LCD_PIN_BL;
  (void)expander_write(s_bl);
  init_hd44780();
}

void LCD_Clear(void){ cmd(LCD_CLR); HAL_Delay(2); }
void LCD_Home(void){ cmd(LCD_HOME); HAL_Delay(2); }

void LCD_SetCursor(uint8_t row, uint8_t col){
  if (row < 1) row = 1;
  if (row > 2) row = 2;
  if (col < 1) col = 1;
  if (col > 16) col = 16;
  uint8_t base = (row==1) ? 0x00 : 0x40;
  cmd((uint8_t)(LCD_SET_DDRAM | (uint8_t)(base + (col-1))));
}

void LCD_Print(const char *s){ while (*s) data((uint8_t)*s++); }

void LCD_PrintAt(uint8_t row, uint8_t col, const char *s){
  LCD_SetCursor(row, col);
  LCD_Print(s);
}

/* pad to 16 columns */
static void pad16(char *s){
  size_t n = strlen(s);
  if (n < 16) { memset(s+n, ' ', 16-n); s[16] = '\0'; }
  else         { s[16] = '\0'; }
}

void LCD_UpdateStatus(uint8_t hlv_ok,
                      ctrl_state_t st,
                      steer_mode_t smode,
                      uint8_t estop_remote,
                      uint8_t estop_button,
                      uint8_t estop_hlv)
{
  if (!s_hi2c) return;

  char l1[17], l2[17];

  /* 1행: 링크(8) + E-STOP 원인(8) */
  const char *link = hlv_ok ? "HLV CON " : "DIS-CON ";
  char est_seg[9];

  if (estop_remote || estop_button || estop_hlv) {
    if (st == ST_RC_VIEW) {
      snprintf(est_seg, sizeof(est_seg), "%-8s", "E-STOP");
    } else {
      uint8_t sum = (estop_remote?1:0) + (estop_button?1:0) + (estop_hlv?1:0);
      const char *cause =
        (sum >= 2)       ? "E-STOP" :
        (estop_remote)   ? "Re_E-ST" :
        (estop_button)   ? "Bu_E-ST" : "Hi_E-ST";
      snprintf(est_seg, sizeof(est_seg), "%-8s", cause);
    }
  } else {
    memset(est_seg, ' ', 8); est_seg[8] = '\0';
  }

  snprintf(l1, sizeof(l1), "%.8s%.8s", link, est_seg);
  pad16(l1);

  /* 2행: 상태/조향모드 */
  const char *state =
      (st==ST_AUTO)      ? "AUTO" :
      (st==ST_AUTO_FAIL) ? "AUTO_FAIL" :
      (st==ST_MANUAL)    ? "MANUAL" :
      (st==ST_RC_VIEW)   ? "RC_VIEW" :
      (st==ST_ESTOP)     ? "E-STOP" : "BOOT";

  const char *mode =
      (smode==DM_4WIS) ? "4WIS" :
      (smode==DM_PIVOT)? "PIVOT" : "2WIS";

  if (st == ST_ESTOP) {
    snprintf(l2, sizeof(l2), "%-16s", "E-STOP");
  } else if (st == ST_RC_VIEW) {
    snprintf(l2, sizeof(l2), "%-16s", "RC VIEW MODE");
  } else {
    snprintf(l2, sizeof(l2), "%-8s %-6s", state, mode);
  }
  pad16(l2);

  LCD_PrintAt(1,1,l1);
  LCD_PrintAt(2,1,l2);
}

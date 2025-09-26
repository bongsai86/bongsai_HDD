/*
 * lcd_kclr2.c
 *
 *  Created on: Sep 12, 2025
 *      Author: bongs
 */
#include "lcd_kclr2.h"
#include "app_ctrl.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *s_hu = NULL;

/* ---- 저수준 TX ---- */
static inline void tx1(uint8_t b){
  if (!s_hu) return;
  HAL_UART_Transmit(s_hu, &b, 1, 5);
}

static inline void LCD_Instruction(uint8_t cmd){
  tx1(CHAR_LCD_INSTRUCTION);
  tx1(cmd);
}

/* 공용 API --------------------------------------------------- */
void LCD_Init(UART_HandleTypeDef *hu){
  s_hu = hu;
  tx1(CHAR_LCD_RESET);          // 0xCA
  HAL_Delay(20);

  LCD_Instruction(0x01);        // Clear
  HAL_Delay(2);
  LCD_Instruction(0x0C);        // Display ON, cursor off
  LCD_Instruction(0x06);        // Entry mode: inc, no shift
  LCD_Position(1,1);
}

void LCD_Clear(void){
  LCD_Instruction(0x01);
  HAL_Delay(2);
}

void LCD_Position(uint8_t col_1to16, uint8_t row_1to2){
  if (col_1to16 < 1)  col_1to16 = 1;
  if (col_1to16 > 16) col_1to16 = 16;
  if (row_1to2  < 1)  row_1to2  = 1;
  if (row_1to2  > 2)  row_1to2  = 2;

  tx1(CHAR_LCD_LINE);     // 0xCB
  tx1(col_1to16);         // COL
  tx1(row_1to2);          // ROW
  HAL_Delay(2);
}

// 기존: tx1(CHAR_LCD_SHOW); tx1(ch);
void LCD_PutChar(uint8_t ch){
  tx1(ch);
}

void LCD_PrintAt(uint8_t row, uint8_t col, const char *s){
  LCD_Position(col, row);
  tx1(CHAR_LCD_SHOW);               // 0xCC
  while (*s) LCD_PutChar((uint8_t)*s++);
  tx1(0x00);                        // EOT
}

void LCD_PrintPad(uint8_t row, uint8_t col, const char *s, uint8_t width){
  LCD_Position(col, row);
  tx1(CHAR_LCD_SHOW);               // 0xCC
  uint8_t n=0;
  while (*s && n<width){ LCD_PutChar((uint8_t)*s++); n++; }
  for (; n<width; n++) LCD_PutChar(' ');
  tx1(0x00);                        // EOT
}

/* 내부 유틸 --------------------------------------------------- */
static void pad16(char *s){
  size_t n = strlen(s);
  if (n < 16) { memset(s + n, ' ', 16 - n); s[16] = '\0'; }
  else         { s[16] = '\0'; }
}

/* 항상 1·2행을 전부 덮어써서 출력 (부분 갱신 의존 제거) */
void LCD_UpdateStatus(uint8_t hlv_ok, ctrl_state_t st, steer_mode_t smode,
                      uint8_t est_r, uint8_t est_b, uint8_t est_h)
{
  if(!s_hu) return;

  char l1[17], l2[17];

  /* 1행: 링크(8) + ESTOP/원인(8) */
  const char *link = hlv_ok ? "HLV CON " : "DIS-CON ";
  char est_seg[9];
  if (est_r || est_b || est_h) {
    if (st == ST_RC_VIEW) {
      snprintf(est_seg, sizeof(est_seg), "%-8s", "E-STOP");   // RC_VIEW은 원인 무시
    } else {
      uint8_t sum = (est_r?1:0) + (est_b?1:0) + (est_h?1:0);
      const char *cause =
          (sum >= 2) ? "E-STOP" :
          (est_r)    ? "Re_E-ST" :
          (est_b)    ? "Bu_E-ST" : "Hi_E-ST";
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
      (st==ST_AUTO_FAIL) ? "AUTOFAIL" :
      (st==ST_MANUAL)    ? "MANUAL" :
      (st==ST_RC_VIEW)   ? "RC_VIEW" :
      (st==ST_ESTOP)     ? "E-STOP" : "BOOT";

  const char *mode =
      (smode==DM_4WIS) ? "4WIS" :
      (smode==DM_PIVOT)? "PIVOT" : "2WIS";

  if (st==ST_ESTOP) {
    snprintf(l2, sizeof(l2), "%-16s", state);            // E-STOP은 전체 고정
  } else if (st==ST_RC_VIEW) {
    snprintf(l2, sizeof(l2), "%-16s", "RC VIEW MODE");   // RC_VIEW 전용 문구
  } else {
    snprintf(l2, sizeof(l2), "%-8s %-6s", state, mode);  // AUTO/ MANUAL/ AUTOFAIL 모두 모드 표시
  }
  pad16(l2);

  LCD_PrintAt(1,1,l1);
  LCD_PrintAt(2,1,l2);
}


/*
 * app_ctrl.c
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */
#include "app_ctrl.h"
#include "rc_pwm.h"
#include "servo_pwm.h"
#include "rc_map.h"
#include "hlv_proto.h"
//#include "lcd_kclr2.h"
#include "lcd_i2c.h"
#include "md400t.h"          /* RS232 듀얼포트 제어 함수 사용 */
#include "main.h"            // HAL_GPIO_ReadPin, GPIO 포트/핀 심볼
#include <math.h>
#include <stdio.h>

//extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

/* === E-STOP 정의 === */
#define ESTOP_RC_THRESH_US   1101u          /* Re_E-ST: RC_ch1 ≤ 1101us */
#define ESTOP_BTN_PORT       GPIOE          /* Bu_E-ST: PE14 ↔ GND (NO 스위치) */
#define ESTOP_BTN_PIN        GPIO_PIN_14    /* Close=On → 핀=LOW */

static inline uint8_t read_re_estop_us(uint32_t ch1_us){
  return (ch1_us > 0u && ch1_us <= ESTOP_RC_THRESH_US) ? 1u : 0u;
}

/* NC 스위치: 평상시 GND로 LOW, 누르면 개방 → Pull-up으로 HIGH, HIGH면 E-STOP 활성(1) */
static inline uint8_t read_bu_estop(void){
  //return (HAL_GPIO_ReadPin(ESTOP_BTN_PORT, ESTOP_BTN_PIN) == GPIO_PIN_RESET) ? 1u : 0u; // E-Stop 스위치가 NO 일 때
  return (HAL_GPIO_ReadPin(ESTOP_BTN_PORT, ESTOP_BTN_PIN) == GPIO_PIN_SET) ? 1u : 0u; // E-Stop 스위치가 NC 일 때
}

/* 외부에서 호출 가능하도록 E-STOP OR 결과 반환 */
bool CTRL_ReadEStop(void){
  uint32_t rc_us[4]; RC_PWM_GetMicros(rc_us);      /* ch1=rc_us[0] */
  uint8_t re = read_re_estop_us(rc_us[0]);
  uint8_t bu = read_bu_estop();
  return (re | bu) ? true : false;
}

/* 모터드라이버 송신 분주: 50ms tick × 2 = 100ms */
static uint8_t md_tx_div = 0;

/* 내부 상태 */
static ctrl_state_t st;
static steer_mode_t dmode;             /* 2WIS/4WIS/PIVOT */
static volatile drive_mode_t drive_mode = DRIVE_MANUAL;

drive_mode_t CTRL_GetDriveMode(void){ return drive_mode; }

static int16_t rpmFR_cur, rpmFL_cur, rpmRR_cur, rpmRL_cur;

/* ===== RC_View 표시 ===== */
static void LCD_ShowRCView(const uint32_t rc_us[4]){
  char l1[17], l2[17];
  /* 16x2 기준 폭 16에 맞춤 */
  snprintf(l1, sizeof(l1), "C1:%4lu C2:%4lu", (unsigned long)rc_us[0], (unsigned long)rc_us[1]);
  snprintf(l2, sizeof(l2), "C3:%4lu C4:%4lu", (unsigned long)rc_us[2], (unsigned long)rc_us[3]);

  /* 깜빡임 방지를 위해 Clear 호출 제거 */
  // LCD_Clear();
  LCD_PrintAt(1,1,l1);
  LCD_PrintAt(2,1,l2);
}


static void outputs_safe(void){
  rpmFR_cur=rpmFL_cur=rpmRR_cur=rpmRL_cur=0;
  SERVO_SetUS(SERVO_CH_FL,1500);
  SERVO_SetUS(SERVO_CH_FR,1500);
  SERVO_SetUS(SERVO_CH_RL,1500);
  SERVO_SetUS(SERVO_CH_RR,1500);
}

static inline int16_t ramp(int16_t prev, int16_t target){
  int16_t d = target - prev;
  if (d >  RPM_RAMP_PER_TICK) d =  RPM_RAMP_PER_TICK;
  if (d < -RPM_RAMP_PER_TICK) d = -RPM_RAMP_PER_TICK;
  return prev + d;
}

/* === 바퀴별 조향각[deg] → 서보펄스[us] 변환 (샤시 한계 반영) ===
 * 우회전=양(+), 좌회전=음(-), 직진=0deg 가정
 * FR:  us = 7.69*x + 1530   * FR:  us = 7.69*x + 1500
 * FL:  us = 8.57*x + 1500   * FL:  us = 8.57*x + 1500
 * RR:  us = -7.89*x + 1475  * RR:  us = -7.89*x + 1500
 * RL:  us = -8.45*x + 1515  * RL:  us = -8.45*x + 1500
 * Clamp: 1150 ~ 1850 us
 */

#define SERVO_OUT_MIN_US   1150
#define SERVO_OUT_MAX_US   1850

static inline uint16_t deg_to_us_ch(uint8_t ch, double x_deg)
{
  double us;
  switch (ch) {
    case SERVO_CH_FR: /* FR (TIM3_CH1 / PC6) */
      us = 7.69 * x_deg + 1500.0;
      break;
    case SERVO_CH_FL: /* FL (TIM3_CH2 / PC7) */
      us = 8.57 * x_deg + 1500.0;
      break;
    case SERVO_CH_RR: /* RR (TIM3_CH4 / PB1) */
      us = -7.89 * x_deg + 1500.0;
      break;
    default:          /* RL (SERVO_CH_RL, TIM3_CH3 / PB0) */
      us = -8.45 * x_deg + 1500.0;
      break;
  }
  if (us < SERVO_OUT_MIN_US) us = SERVO_OUT_MIN_US;
  if (us > SERVO_OUT_MAX_US) us = SERVO_OUT_MAX_US;
  return (uint16_t)(us + 0.5); /* 반올림 */
}

/* RC_ch4 존 판정: 0=Auto, 1=Manual, 2=RC_View */
static inline uint8_t rc_zone_ch4(int us){
  if (us <= 1485) return 0u;        /* 1000~1485 = Auto */
  if (us <= 1550) return 1u;        /* 1486~1550 = Manual */
  return 2u;                         /* 1551~2000 = RC_View */
}

/* RC_ch3로 모드 선택 */
static steer_mode_t pick_mode_from_rc_us(int us){
  if      (us>=1516) return DM_PIVOT;
  else if (us>=1486) return DM_4WIS;
  else               return DM_2WIS;
}

void CTRL_Init(void){
  st=ST_BOOT; dmode=DM_2WIS; drive_mode=DRIVE_MANUAL;
  rpmFR_cur=rpmFL_cur=rpmRR_cur=rpmRL_cur=0;
  outputs_safe();
//  LCD_Init(&huart1);
  LCD_Init(&hi2c1, 0x27);   /* 모듈에 따라 0x27/0x3F */
  LCD_Clear();

  /* LCD BOOTING Message */
  LCD_Clear();
  LCD_PrintAt(1,1,"GO! ROMO-B^^ ");
  LCD_PrintAt(2,1,"by RMMLab.");

  HAL_Delay(2000);
  LCD_Clear();            // 부팅 메시지 지우기 (잔상 방지)
}

/* 50 ms 제어 본체 */
void CTRL_Tick50ms(void)
{
  /* RC 스냅샷 */
  uint32_t rc_us[4]; RC_PWM_GetMicros(rc_us);
  uint8_t ch4_zone = rc_zone_ch4((int)rc_us[3]);     // 0=Auto,1=Manual,2=RC_View
  wheel_cmd_t w = (wheel_cmd_t){0};

  /* HLV 최신 */
  HLV_Cmd hc; uint32_t age=0;
  bool have_hlv = HLV_GetLatest(&hc, &age);
  uint8_t hlv_auto = (have_hlv && hc.aorm) ? 1u : 0u;

  /* LCD 모드표시 전용 상태(ESTOP 무시) */
  ctrl_state_t st_disp;
  if (ch4_zone==0u) {
    st_disp = (hlv_auto && (age <= HLC_TIMEOUT_MS)) ? ST_AUTO : ST_AUTO_FAIL;
  } else if (ch4_zone==1u) {
    st_disp = ST_MANUAL;
  } else {
    st_disp = ST_RC_VIEW;
  }

  /* 외부 참조용 드라이브 모드 */
  drive_mode =
      (st_disp==ST_AUTO)      ? DRIVE_AUTO :
      (st_disp==ST_AUTO_FAIL) ? DRIVE_AUTO_FAIL :
      (st_disp==ST_RC_VIEW)   ? DRIVE_RC_VIEW :
                                DRIVE_MANUAL;

  /* 표시용 조향모드: ESTOP과 무관하게 '현재 모드 기준'으로 항상 갱신 */
  steer_mode_t dmode_disp = dmode; // 기본 기존값
  if (st_disp == ST_AUTO) {
    dmode_disp = (steer_mode_t)hc.mode;
  } else { // MANUAL, AUTO_FAIL, RC_VIEW → RC_ch3 선택 반영
    dmode_disp = pick_mode_from_rc_us((int)rc_us[2]);
  }

  /* E-STOP 신호 */
  uint8_t estop_rem = read_re_estop_us(rc_us[0]);
  uint8_t estop_btn = read_bu_estop();
  uint8_t estop_hlv = (have_hlv && hc.estop) ? 1u : 0u;

  /* 최종 운전 상태 */
  if ((estop_rem | estop_btn | estop_hlv) != 0u) {
    st = ST_ESTOP;
  } else if (ch4_zone==0u) {
    st = (hlv_auto && (age <= HLC_TIMEOUT_MS)) ? ST_AUTO : ST_AUTO_FAIL;
  } else if (ch4_zone==1u) {
    st = ST_MANUAL;
  } else {
    st = ST_RC_VIEW;
  }

  /* ESTOP 즉시 처리 */
  if (st == ST_ESTOP){
    outputs_safe();
    rpmFR_cur=rpmFL_cur=rpmRR_cur=rpmRL_cur=0;
    MD400_SetSpeedAB_RS485(1,0,0);  /* 논리ID1(UART2) */
    MD400_SetSpeedAB_RS485(2,0,0);  /* 논리ID2(UART4) */
    uint8_t hlv_ok = (have_hlv && (age <= HLC_TIMEOUT_MS)) ? 1u : 0u;
    /* ESTOP 중에도 표시용 조향모드는 dmode_disp로 계속 갱신되어 넘어옴 */
    LCD_UpdateStatus(hlv_ok, st_disp, dmode_disp, estop_rem, estop_btn, estop_hlv);
    /* return 없이 아래 피드백 루틴 진행 */
  }

  /* ESTOP 아닐 때 구동계 계산/출력 */
  if (st != ST_ESTOP) {
    /* 제어용 조향모드 선택 (AUTOFAIL도 RC 기준으로 갱신) */
    if (st == ST_AUTO) {
      dmode = (steer_mode_t)hc.mode;
    } else if (st == ST_MANUAL) {
      dmode = pick_mode_from_rc_us((int)rc_us[2]);
    } else if (st == ST_AUTO_FAIL) {
      dmode = dmode_disp; // RC_ch3 기준 모드 유지/표시와 일치
    } /* ST_RC_VIEW는 steer=0이므로 영향 없음 */

    /* 속도/조향 명령 */
    double spd = 0.0, steer_deg = 0.0;
    if (st == ST_MANUAL){
      spd = rc_speed_cmd((int)rc_us[0]);
      steer_deg = (dmode==DM_4WIS) ? rc_steer_deg_4WIS((int)rc_us[1])
                                   : rc_steer_deg_2WIS((int)rc_us[1]);
    } else if (st == ST_AUTO){
      spd = fmax(-1.0, fmin(1.0, hc.speed_mps / HLV_SPD_MAX_MPS));
      steer_deg = hc.steer_deg;
    } else if (st == ST_AUTO_FAIL){
      spd = 0.0; steer_deg = 0.0;  // 정지, 단 표시용 모드는 위에서 갱신됨
    } else { /* ST_RC_VIEW */
      spd = 0.0; steer_deg = 0.0;
    }

    /* 기하/서보/모터 출력 */
    if      (dmode==DM_2WIS)  kin_compute_2WIS(steer_deg, spd, &w);
    else if (dmode==DM_4WIS)  kin_compute_4WIS(steer_deg, spd, &w);
    else                      kin_compute_PIVOT(spd, &w);

    if (dmode == DM_4WIS) {
      const double LIM = 18.0;
      if (w.ang_fl >  LIM) { w.ang_fl =  LIM; }
      if (w.ang_fl < -LIM) { w.ang_fl = -LIM; }

      if (w.ang_fr >  LIM) { w.ang_fr =  LIM; }
      if (w.ang_fr < -LIM) { w.ang_fr = -LIM; }

      if (w.ang_rl >  LIM) { w.ang_rl =  LIM; }
      if (w.ang_rl < -LIM) { w.ang_rl = -LIM; }

      if (w.ang_rr >  LIM) { w.ang_rr =  LIM; }
      if (w.ang_rr < -LIM) { w.ang_rr = -LIM; }
    }

    SERVO_SetUS(SERVO_CH_FL, deg_to_us_ch(SERVO_CH_FL, w.ang_fl));
    SERVO_SetUS(SERVO_CH_FR, deg_to_us_ch(SERVO_CH_FR, w.ang_fr));
    SERVO_SetUS(SERVO_CH_RL, deg_to_us_ch(SERVO_CH_RL, w.ang_rl));
    SERVO_SetUS(SERVO_CH_RR, deg_to_us_ch(SERVO_CH_RR, w.ang_rr));

    int16_t cmd_fl = (int16_t)lrint(w.sp_fl * RPM_MAX);
    int16_t cmd_fr = (int16_t)lrint(w.sp_fr * RPM_MAX);
    int16_t cmd_rl = (int16_t)lrint(w.sp_rl * RPM_MAX);
    int16_t cmd_rr = (int16_t)lrint(w.sp_rr * RPM_MAX);

    cmd_fl = (cmd_fl >  RPM_MAX) ?  RPM_MAX : (cmd_fl < -RPM_MAX) ? -RPM_MAX : cmd_fl;
    cmd_fr = (cmd_fr >  RPM_MAX) ?  RPM_MAX : (cmd_fr < -RPM_MAX) ? -RPM_MAX : cmd_fr;
    cmd_rl = (cmd_rl >  RPM_MAX) ?  RPM_MAX : (cmd_rl < -RPM_MAX) ? -RPM_MAX : cmd_rl;
    cmd_rr = (cmd_rr >  RPM_MAX) ?  RPM_MAX : (cmd_rr < -RPM_MAX) ? -RPM_MAX : cmd_rr;

    rpmFR_cur = ramp(rpmFR_cur, cmd_fr);
    rpmRR_cur = ramp(rpmRR_cur, cmd_rr);
    rpmFL_cur = ramp(rpmFL_cur, cmd_fl);
    rpmRL_cur = ramp(rpmRL_cur, cmd_rl);

    /* 논리ID1(UART2): RL(M1), FL(M2) / 논리ID2(UART4): FR(M1), RR(M2) */
    /* 모터드라이버 속도 명령 송신 주기 = 100ms (50ms tick 2회에 한 번 송신) */
    if (++md_tx_div >= 2) {
      md_tx_div = 0;
      MD400_SetSpeedAB_RS485(1, rpmRL_cur, rpmFL_cur);  /* 논리ID1: UART2 */
      MD400_SetSpeedAB_RS485(2, rpmFR_cur, rpmRR_cur);  /* 논리ID2: UART4 */
    }
  }

  /* LCD 업데이트 (200ms) */
  uint8_t hlv_ok = (have_hlv && (age <= HLC_TIMEOUT_MS)) ? 1u : 0u;
  static uint8_t lcd_div = 0;
  if (++lcd_div >= 4) {
    lcd_div = 0;
    if (st == ST_ESTOP) {
      /* ESTOP 동안엔 RC_VIEW 화면을 덮어쓰지 않도록 상태화면만 */
      LCD_UpdateStatus(hlv_ok, st_disp, dmode_disp, estop_rem, estop_btn, estop_hlv);
    } else if (st_disp == ST_RC_VIEW) {
      LCD_ShowRCView(rc_us);
    } else {
      LCD_UpdateStatus(hlv_ok, st_disp, dmode_disp, estop_rem, estop_btn, estop_hlv);
    }
  }

  /* HLV 피드백 */
  HLV_SendFeedback(&w, dmode, (st==ST_AUTO)?1:0, (st==ST_ESTOP)?1:0);
}

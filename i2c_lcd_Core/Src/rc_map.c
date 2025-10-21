/*
 * rc_map.c
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */

#include "rc_map.h"

#define RC_MIN 1100
#define RC_MAX 1900

/* ===== Speed(Ch1) 선형화(전/후진 1차식) =====
 * 현재 조종기: 중립 1521us, deadband ±8us
 */
#define RC1_MID   1504 // 1521 DXs, 1504 Dxe
#define RC1_DB    8

#define RC1_F_SLOPE   ( 1.0 / (RC_MAX - (double)(RC1_MID + RC1_DB)) )
#define RC1_F_BIAS    (-(RC1_F_SLOPE) * (double)(RC1_MID + RC1_DB))
#define RC1_B_SLOPE   (-1.0 / (RC_MIN  - (double)(RC1_MID - RC1_DB)))
#define RC1_B_BIAS    (-(RC1_B_SLOPE) * (double)(RC1_MID - RC1_DB))

static inline int clamp_int(int v, int lo, int hi){
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline int in_db_speed(int x){
  return (x >= (RC1_MID - RC1_DB)) && (x <= (RC1_MID + RC1_DB));
}

double rc_speed_cmd(int x){
  x = clamp_int(x, RC_MIN, RC_MAX);             // 입력 클램프
  if (in_db_speed(x)) return 0.0;
  double y = (x > RC1_MID)
           ? (RC1_F_SLOPE * (double)x + RC1_F_BIAS)   /* 전진 */
           : (RC1_B_SLOPE * (double)x + RC1_B_BIAS);  /* 후진 */
  if (y >  1.0) y =  1.0;
  if (y < -1.0) y = -1.0;
  return y;
}

/* ===== Steer(Ch2) 선형화(좌/우 1차식) =====
 * 최대 조향각은 STEER*_MAX_DEG 로 결정
 */
#define RC2_MID        1502 // 1494 DXs, 1502 Dxe
#define RC2_DB         15

#define STEER2_MAX_DEG  22.0f
#define STEER4_MAX_DEG  18.0f

static inline int in_db_steer(int x){
  return (x >= (RC2_MID - RC2_DB)) && (x <= (RC2_MID + RC2_DB));
}

/* (좌) (MID-DB)→0°, RC_MIN→+MAX
 * (우) (MID+DB)→0°, RC_MAX→-MAX
 * 결과는 ±max_deg로 클램프
 */
static inline double steer_linear_deg(int x, double max_deg){
  x = clamp_int(x, RC_MIN, RC_MAX);             // 입력 클램프
  if (in_db_steer(x)) return 0.0;

  double d;
  if (x > (RC2_MID + RC2_DB)) {
    /* 우측: 음각 */
    double t = ( (double)x - (RC2_MID + RC2_DB) ) / (RC_MAX - (RC2_MID + RC2_DB));
    if (t > 1.0) t = 1.0;                       // 수치적 안전
    d = -max_deg * t;
  } else {
    /* 좌측: 양각 */
    double t = ( (double)x - (RC2_MID - RC2_DB) ) / (RC_MIN - (RC2_MID - RC2_DB));
    if (t > 1.0) t = 1.0;
    d =  max_deg * t;
  }
  if (d >  max_deg) d =  max_deg;
  if (d < -max_deg) d = -max_deg;
  return d;
}

double rc_steer_deg_2WIS(int x){
  return steer_linear_deg(x, STEER2_MAX_DEG);
}

double rc_steer_deg_4WIS(int x){
  return steer_linear_deg(x, STEER4_MAX_DEG);
}

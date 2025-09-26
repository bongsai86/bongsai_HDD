/*
 * kinematics.h
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { DM_2WIS=0, DM_4WIS=1, DM_PIVOT=2 } steer_mode_t;

typedef struct {
  double ang_fl, ang_fr, ang_rl, ang_rr;   /* [deg] */
  double sp_fl,  sp_fr,  sp_rl,  sp_rr;    /* [-1..1] */
} wheel_cmd_t;

void kin_compute_2WIS(double steer_deg, double speed_cmd, wheel_cmd_t* o);
void kin_compute_4WIS(double steer_deg, double speed_cmd, wheel_cmd_t* o);
void kin_compute_PIVOT(double speed_cmd, wheel_cmd_t* o);

#ifdef __cplusplus
}
#endif

#endif /* INC_KINEMATICS_H_ */

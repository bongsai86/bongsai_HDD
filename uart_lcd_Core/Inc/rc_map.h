/*
 * rc_map.h
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */

#ifndef INC_RC_MAP_H_
#define INC_RC_MAP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

double rc_speed_cmd(int x_us);          /* [-1..1] */
double rc_steer_deg_2WIS(int x_us);     /* [deg] */
double rc_steer_deg_4WIS(int x_us);     /* [deg] */

#ifdef __cplusplus
}
#endif
#endif /* INC_RC_MAP_H_ */

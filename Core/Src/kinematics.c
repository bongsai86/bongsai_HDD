/*
 * kinematics.c
 *
 *  Created on: Sep 5, 2025
 *      Author: bongs
 */
#include "kinematics.h"
#include <math.h>

static const double L=0.32, W=0.39;
static const double RAD=3.141592653589793/180.0, DEG=180.0/3.141592653589793;
static const double eps=1e-12;

static inline void norm4(double* a,double* b,double* c,double* d){
  double m=fmax(fmax(fabs(*a),fabs(*b)),fmax(fabs(*c),fabs(*d)));
  if(m>1.0){*a/=m;*b/=m;*c/=m;*d/=m;}
}

/* 2WIS: 앞 Ackermann 각, 뒤 0도. 속도는 업로드 수식. */
void kin_compute_2WIS(double steer_deg, double speed_cmd, wheel_cmd_t* o){
  double t=tan(steer_deg*RAD);
  if (fabs(t)<eps){
    o->ang_fl=o->ang_fr=o->ang_rl=o->ang_rr=0.0;
    o->sp_fl=o->sp_fr=o->sp_rl=o->sp_rr=speed_cmd; return;
  }
  double R=L/t, Rabs=fabs(R), Rin=R-0.5*W, Rout=R+0.5*W;

  double dfr=atan(L / (fabs(Rin)<eps?copysign(eps,Rin):Rin));
  double dfl=atan(L / (fabs(Rout)<eps?copysign(eps,Rout):Rout));
  o->ang_fr=dfr*DEG; o->ang_fl=dfl*DEG; o->ang_rr=0.0; o->ang_rl=0.0;

  double RFR=sqrt(L*L + Rin*Rin), RFL=sqrt(L*L + Rout*Rout);
  o->sp_fr = speed_cmd * (RFR / Rabs);
  o->sp_fl = speed_cmd * (RFL / Rabs);
  o->sp_rr = speed_cmd * (fabs(Rin)  / Rabs);
  o->sp_rl = speed_cmd * (fabs(Rout) / Rabs);

  norm4(&o->sp_fl,&o->sp_fr,&o->sp_rl,&o->sp_rr);
}

/* 4WIS: counter-phase Ackermann. 속도는 업로드 수식. */
void kin_compute_4WIS(double steer_deg, double speed_cmd, wheel_cmd_t* o){
  double t=tan(steer_deg*RAD);
  if (fabs(t)<eps){
    o->ang_fl=o->ang_fr=o->ang_rl=o->ang_rr=0.0;
    o->sp_fl=o->sp_fr=o->sp_rl=o->sp_rr=speed_cmd; return;
  }
  //double sgn=(steer_deg>=0.0)?1.0:-1.0, R=L/t, Rabs=fabs(R), n=0.5*L;
  double R=L/t, Rabs=fabs(R), n=0.5*L;
  double din=R-0.5*W; if(fabs(din)<eps)  din =copysign(eps,din);
  double dout=R+0.5*W; if(fabs(dout)<eps) dout=copysign(eps,dout);

  double ain=atan(n/din), aout=atan(n/dout);
  //o->ang_fr=+sgn*ain*DEG; o->ang_fl=+sgn*aout*DEG;
  //o->ang_rr=-sgn*ain*DEG; o->ang_rl=-sgn*aout*DEG;
  o->ang_fr= ain*DEG;     o->ang_fl= aout*DEG;
  o->ang_rr= ain*DEG;     o->ang_rl= aout*DEG;

  double L2=0.5*L, RFR=sqrt(L2*L2 + din*din), RFL=sqrt(L2*L2 + dout*dout);
  double k=speed_cmd / Rabs;
  o->sp_fr=k*RFR; o->sp_fl=k*RFL; o->sp_rr=k*RFR; o->sp_rl=k*RFL;

  norm4(&o->sp_fl,&o->sp_fr,&o->sp_rl,&o->sp_rr);
}

/* PIVOT: 각 고정, 좌우 반대 속도. */
void kin_compute_PIVOT(double speed_cmd, wheel_cmd_t* o){
  const double A = 30.0;
  // 각도: FL +30, FR -30, RL -30, RR +30
  o->ang_fl = +A; o->ang_fr = -A; o->ang_rl = +A; o->ang_rr = -A;

  // 속도: 좌/우 반대 회전(원위치 회전)
  o->sp_fl = +speed_cmd; o->sp_fr = -speed_cmd;
  o->sp_rl = +speed_cmd; o->sp_rr = -speed_cmd;

  norm4(&o->sp_fl,&o->sp_fr,&o->sp_rl,&o->sp_rr);
}


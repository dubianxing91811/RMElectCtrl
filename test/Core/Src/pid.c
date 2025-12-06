/*
 * pid.c
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */
#include "pid.h"
#include <math.h>
void pid_init(PID *p, float Kp, float Ki, float Kd){
p->Kp=Kp; p->Ki=Ki; p->Kd=Kd; p->ia=0; p->le=0;
p->out_min = -1e6f; p->out_max = 1e6f;
}
float pid_update(PID *p, float target, float measure, float dt){
float err = target - measure;
p->ia += err * dt;
float er = (err - p->le) / dt;
float out = p->Kp*err + p->Ki*p->ia + p->Kd*er;
if(out > p->out_max) out = p->out_max;
if(out < p->out_min) out = p->out_min;
p->le = err;
return out;
}


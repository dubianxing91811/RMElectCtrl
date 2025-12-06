/*
 * pid.h
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct { float Kp, Ki, Kd; float ia, le; float out_min, out_max; } PID;
void pid_init(PID *p, float Kp, float Ki, float Kd);
float pid_update(PID *p, float target, float measure, float dt);

#endif /* INC_PID_H_ */

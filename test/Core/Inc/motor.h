/*
 * motor.h
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f1xx_hal.h"
typedef enum {
	MOTOR_LEFT=0,
	MOTOR_RIGHT=1
} motor;
void motor_init(void);
void motor_set_pwm(motor x, uint16_t pwm);
void motor_set_dir(motor x, uint8_t dir);

#endif /* INC_MOTOR_H_ */

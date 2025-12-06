/*
 * encoder.h
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f1xx_hal.h"
void encoder_init(void);
int32_t encoder_get_left(void);
int32_t encoder_get_right(void);
void encoder_clear(void);

#endif /* INC_ENCODER_H_ */

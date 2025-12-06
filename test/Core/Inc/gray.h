/*
 * gray.h
 *
 *  Created on: Dec 6, 2025
 *      Author: rvvlt
 */

#ifndef INC_GRAY_H_
#define INC_GRAY_H_

#include "stm32f1xx_hal.h"

void Gray_Init(void);
void Gray_Read(uint8_t *gray_data);
float Gray_GetPosition(void);

#endif /* INC_GRAY_H_ */

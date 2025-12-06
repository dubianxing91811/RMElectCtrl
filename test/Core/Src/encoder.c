/*
 * encoder.c
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */
#include "encoder.h"
#include "main.h"



extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void encoder_init(void){
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

int32_t encoder_get_left(void){
return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
}
int32_t encoder_get_right(void){
return (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
}
void encoder_clear(void){
__HAL_TIM_SET_COUNTER(&htim3, 0);
__HAL_TIM_SET_COUNTER(&htim4, 0);
}


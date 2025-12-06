/*
 * motor.c
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */
#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
#define pwm_max (999)

void motor_init(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}


void motor_set_pwm(motor x, uint16_t pwm){
if(pwm > pwm_max) pwm = pwm_max;
if(x==MOTOR_LEFT){
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
} else {
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}
}

void motor_set_dir(motor x, uint8_t dir){
if(x==MOTOR_LEFT){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, dir?GPIO_PIN_SET:GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, dir?GPIO_PIN_RESET:GPIO_PIN_SET);
 } else {
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, dir?GPIO_PIN_SET:GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, dir?GPIO_PIN_RESET:GPIO_PIN_SET);
 }
}


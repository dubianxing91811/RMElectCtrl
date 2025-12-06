/*
 * gray.c
 *
 *  Created on: Nov 30, 2025
 *      Author: rvvlt
 */
#include "gray.h"

void Gray_Init(void)
{
    HAL_Delay(10);
}

void Gray_Read(uint8_t *gray_data)
{
	gray_data[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	gray_data[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	gray_data[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	gray_data[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	gray_data[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	gray_data[5] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	gray_data[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	gray_data[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}

static uint8_t gray_data_sign(const uint8_t *gray_data)
{
    uint8_t x = 0;
    for(int i = 0; i < 8; i++) {
        if(gray_data[i] != 0) {
            x = x + (1 << i);
        }
    }
    return x;
}

float Gray_GetPosition(void)
{
uint8_t gray_data[8];
Gray_Read(gray_data);
uint8_t x = gray_data_sign(gray_data);

const float weights[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};

float sum = 0.0f;
int cnt = 0;
for(int i=0;i<8;i++){
if(x & (1 << i)){
	sum += weights[i];
	cnt++;
   }
}

if(cnt == 0){
return 100.0f;
}
return sum / (float)cnt;
}


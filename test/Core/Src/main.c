/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <math.h>
#include "stdlib.h"
#include "gray.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define diameter 0.080f          //轮子直径
#define CPR 13200.0f             //转一圈的脉冲数
#define CONTROL_HZ 400.0f              //这两条用于设置25ms
#define CONTROL_DT (1.0f/CONTROL_HZ)

#define TIMER 1000               //设置的自动重装载值
#define TURN_PWM 250             //寻线时电机速度
#define GRAY_THRESHOLD 1.8f      //设置了灰度的突变阈值    暂时不知道放哪，先这样了
#define min_speed 0.50f          //最小速度限制

#define dec_speed 0.01f          //根据偏差程度的速度修正
#define inc_speed 0.05f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float base_target = 1.1f;
int lost_cnt = 0;
int   corner_state = 0;                      //这里的常量对应主循环开始的内容
int   corner_timer = 0;
float corner_threshold = 2.0f;
float corner_speed = 0.35f;

float last_line_bias = 0;
float turn = 0.0f;
int turn_dir = 0;

PID pid_left, pid_right;
int32_t last_left_cnt = 0, last_right_cnt = 0;
float left_target = 0.0f;
float right_target = 0.0f;
typedef enum {
    STATE_FOLLOW_LINE = 0,
    STATE_TURN_FIND_LINE
} CarState;
float spin_Kp = 0.35f;
CarState car_state = STATE_FOLLOW_LINE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static float per_sec(float mps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pid_reset(PID *pid){
    pid->ia = 0;
    pid->le = 0;
}
static float per_sec(float mps){
	   float per_s = mps * CPR/ (3.1415f * diameter) ;
	    return per_s;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  encoder_init();
  encoder_clear();
  Gray_Init();

 pid_init(&pid_left, 1.4f, 0.02f, 0.01f);
 pid_init(&pid_right, 1.4f, 0.02f, 0.01f);

  pid_left.out_min  = -per_sec(2.0f);
  pid_left.out_max  =  per_sec(2.5f);
  pid_right.out_min = -per_sec(2.0f);
  pid_right.out_max =  per_sec(2.5f);

  uint32_t last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t tick = HAL_GetTick();
	  if(tick - last_tick >= (uint32_t)(1000/CONTROL_HZ)){       //我设置了控制周期的时间间隔为2.5ms
	        last_tick = tick;
	        float line_bias = Gray_GetPosition();                 //计算了偏线程度

	        static float filtered_line_bias = 0.0f;
	        filtered_line_bias = 0.9f * filtered_line_bias + 0.1f * line_bias;           //滤波 → 响应慢→不过可能会影响速度*  以后可以考虑更好的解决方案

	  if(line_bias != 100.0f){
	      float change = fabs(filtered_line_bias - last_line_bias);
	      float abs_bias = fabs(line_bias);
	      if(abs_bias < corner_threshold){
	          if(change > GRAY_THRESHOLD){
	          base_target -= dec_speed;                                                   //突变偏线程度过大就先提前降速
	          if(base_target < min_speed) base_target = min_speed;
	      }
	      }else{
	      switch(corner_state){                                                   //进弯，出弯的状态机
	          case 0:
	              if(abs_bias > corner_threshold){
	                  corner_state = 1;
	                  corner_timer = 0;
	              }
	              break;

	          case 1:
	              base_target = corner_speed;                                     //这里有点问题，感觉出弯过大不能完全巡线，虽然可以调死参
	              if(abs_bias < 1.0f){
	                  corner_state = 2;
	              }
	              break;

	          case 2:
	              base_target += inc_speed;
	              if(base_target >= 1.1f){
	                  base_target = 1.1f;
	                  corner_state = 0;
	              }
	              break;
	                        }
	      }
	  }


      switch(car_state){
          case STATE_FOLLOW_LINE:
              if(line_bias == 100.0f){
                  lost_cnt++;
                  if(lost_cnt > 3){                                              //这里检查了3次 决定是否丢线
                      car_state = STATE_TURN_FIND_LINE;
                      turn_dir = (last_line_bias < 0) ? -1 : 1;
                      pid_reset(&pid_left);                                      //重置pid中的参数
                      pid_reset(&pid_right);
                      lost_cnt = 0;
                  }
              } else {
                  lost_cnt = 0;
                  last_line_bias = line_bias;
                  turn = spin_Kp * line_bias;
                  float max_turn = 0.3f;
                  if(turn > max_turn) turn = max_turn;
                  if(turn < -max_turn) turn = -max_turn;

                  left_target = base_target - turn;
                  right_target = base_target + turn;

                  if(left_target < 0.0f) left_target = 0.0f;
                  if(right_target < 0.0f) right_target = 0.0f;
              }
              break;

          case STATE_TURN_FIND_LINE:
              if(turn_dir == -1){                                               //按照上一次丢线的误差方向原地转向
                  motor_set_dir(MOTOR_LEFT, 1);
                  motor_set_dir(MOTOR_RIGHT, 0);
              } else {
                  motor_set_dir(MOTOR_LEFT, 0);
                  motor_set_dir(MOTOR_RIGHT, 1);
              }
              uint16_t pwm_turn = TURN_PWM;
              motor_set_pwm(MOTOR_LEFT, pwm_turn);
              motor_set_pwm(MOTOR_RIGHT, pwm_turn);
              if(line_bias != 100.0f){
                  car_state = STATE_FOLLOW_LINE;
                  last_line_bias = line_bias;
                  pid_reset(&pid_left);
                  pid_reset(&pid_right);
              }
              break;
      }
	  int32_t left_cnt = encoder_get_left();
	  int32_t right_cnt = encoder_get_right();
	  float left_dtcnt = (float)(left_cnt - last_left_cnt);
	  float right_dtcnt = (float)(right_cnt - last_right_cnt);
	  last_left_cnt = left_cnt; last_right_cnt = right_cnt;

	  float left_cps = left_dtcnt * CONTROL_HZ;                                  //计算出每秒钟编码器的脉冲数
	  float right_cps = right_dtcnt * CONTROL_HZ;

	  float left_target_cps = per_sec(left_target);               //  1.1m/s  除以轮子周长得到  r/s 每转增加13200个脉冲  即得到 目标值的：x个脉冲每秒
	  float right_target_cps = per_sec(right_target);
	  float left_out = pid_update(&pid_left, left_target_cps, left_cps, CONTROL_DT);
	  float right_out = pid_update(&pid_right, right_target_cps, right_cps, CONTROL_DT);                //将参数引入pid指针，并算出结果
	  float max_cps = per_sec(2.0f);

	  uint16_t pwm_left, pwm_right;

	  if(left_out >= 0){
	      motor_set_dir(MOTOR_LEFT, 0);
	      pwm_left = (uint16_t)( (left_out / max_cps) * TIMER );
	  } else {
	      motor_set_dir(MOTOR_LEFT, 1);
	      pwm_left = (uint16_t)( (-left_out / max_cps) * TIMER );
	  }

	  if(right_out >= 0){
	      motor_set_dir(MOTOR_RIGHT, 0);
	      pwm_right = (uint16_t)( (right_out / max_cps) * TIMER );
	  } else {
	      motor_set_dir(MOTOR_RIGHT, 1);
	      pwm_right = (uint16_t)( (-right_out / max_cps) * TIMER );
	  }

	  if(pwm_left > TIMER) pwm_left = TIMER;
	  if(pwm_right > TIMER) pwm_right = TIMER;

	  motor_set_pwm(MOTOR_LEFT, pwm_left);
	  motor_set_pwm(MOTOR_RIGHT, pwm_right);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 12;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 12;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

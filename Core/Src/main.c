/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hfile.h"

#define PID_P 500
#define PID_I 15
#define PID_D 0

#define Speed_Straight 20 // 目标速度，单位为编码器计数每周期
#define Speed_Diff 10      // 转向时左右轮速度差值
#define Speed_Diff_l 5
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_t pid_r, pid_l;
uint8_t pit_pid_flag;
int encoder_data_left = 0;
int encoder_data_right = 0;
int pwm_right = 0;
int pwm_left = 0;
uint8_t sensor_data[8]; // 用于存储解析后的传感器数据
uint8_t zhuanxiang_flag = 0; // 转向标志：0无转向，1左转，2右转
uint32_t delay = 0; // 转向延时计数器
uint8_t xunxian_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  OLED_Init();
  PID_Init(&pid_r, PID_P, PID_I, PID_D); // 初始化右轮 PID 参数
  PID_Init(&pid_l, PID_P, PID_I, PID_D); // 初始化左轮 PID 参数
  HAL_TIM_Base_Start_IT(&htim3);
  pid_l.target_val = Speed_Straight;
  pid_r.target_val = Speed_Straight;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart2, &Rx_Byte, 1);
  char start_cmd[] = "$0,0,1#";
  HAL_UART_Transmit(&huart2, (uint8_t *)start_cmd, strlen(start_cmd), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OLED_Printf(0, 0, OLED_6X8, "R:%d L:%d", encoder_data_right, encoder_data_left);
    OLED_Printf(0, 8, OLED_6X8, "pwm_R:%d L:%d    ", pwm_right, pwm_left);
    OLED_Update();
    //    Motor_Right_Run(50);
    //		Motor_Left_Run(50);
    Parse_Track_Data(Rx_Buffer, sensor_data);
    OLED_Printf(0, 16, OLED_6X8, "Sen: %d %d %d %d %d %d %d %d", sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6], sensor_data[7]);
		OLED_Printf(0, 32, OLED_6X8, "flag:%d    ",zhuanxiang_flag);
	OLED_Printf(0, 48, OLED_6X8, "state:%d    ",xunxian_state);
  xunxian_state = xunxian_scan(sensor_data);
    switch (xunxian_state)
    {
    case little_r: // 微微偏右
    {
      if (delay > 5000)
      {
        zhuanxiang_flag = 0;
        pid_l.target_val = Speed_Straight - Speed_Diff_l;
        pid_r.target_val = Speed_Straight + Speed_Diff_l;
        
      }

      break;
    }
    case little_l: // 微微偏左
    {
      if (delay > 5000)
      {
        zhuanxiang_flag = 0;
        pid_l.target_val = Speed_Straight - Speed_Diff_l;
        pid_r.target_val = Speed_Straight + Speed_Diff_l;
        
      }
      break;
    }

    case straight: // 直行
    {
      if (delay > 5000)
      {
        zhuanxiang_flag = 0;
        pid_l.target_val = Speed_Straight;
        pid_r.target_val = Speed_Straight;
        
      }
      break;
    }
    case left: // 左转
    {
      delay = 0;
      zhuanxiang_flag = 1;
      break;
    }
    case right: // 右转
    {
      delay = 0;
      zhuanxiang_flag = 2;
      break;
    }
    case zhuangxiang: // 转向标志
    {
      
    }
    default:
      break;
    }
		
		if (zhuanxiang_flag == 1)
      {
        Motor_Stop();
      }
      else if (zhuanxiang_flag == 2)
      {
        Motor_Stop();
      }

    if (pit_pid_flag && zhuanxiang_flag == 0) 
    {
      pit_pid_flag = 0;

      get_encoder_count_right(&encoder_data_right);
      get_encoder_count_left(&encoder_data_left);
      pwm_right = PID_Speed(&pid_r, encoder_data_right); // 计算右轮 PWM 输出
      pwm_left = PID_Speed(&pid_l, encoder_data_left);   // 计算左轮 PWM 输出
      // ---------- 动态控制右轮 ----------
      Motor_Right_Run(pwm_right);
      // ---------- 动态控制左轮 ----------
      Motor_Left_Run(pwm_left);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    pit_pid_flag = 1;
    delay += 10;
  }
}
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

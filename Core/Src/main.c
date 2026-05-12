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

#define ANGLE_P 50 // 角度环参数，这些要你自己调
#define ANGLE_I 0   // 角度环很多时候用不到 I，可设为 0
#define ANGLE_D 0





#define Speed_Straight 15 // 目标速度，单位为编码器计数每周期
#define Speed_Diff 10     // 转向时左右轮速度差值
#define Speed_Diff_l 5


#define AV_P 30 // 角度环参数，这些要你自己调
#define AV_I 0   // 角度环很多时候用不到 I，可设为 0
#define AV_D 10

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
PID_t pid_r, pid_l, pid_angle,pid_av;
volatile uint8_t pit_pid_flag = 0; // 【优化1】增加 volatile
int encoder_data_left = 0;
int encoder_data_right = 0;
int pwm_right = 0;
int pwm_left = 0;
uint8_t sensor_data[8];
uint8_t zhuanxiang_flag = 0;
volatile uint32_t delay = 0;         // 【优化1】增加 volatile
volatile uint16_t turn_cooldown = 0; // 【优化3】新增转向冷却倒计时
uint8_t xunxian_state = 0;
uint8_t xunxian_flag = 1;
uint8_t bizhang_is=0;
volatile uint32_t delay_buzz = 0;
int angle = 0;
int target_angle = 0;
uint8_t angle_flag = 0;
uint8_t delay_flag = 1;
uint8_t global_turn_dir = 0;           // 【新增】全局转向锁定：0未决定，1锁定左转，2锁定右转
extern volatile uint32_t tim3_ms_tick; // 【新增】声明 xunxian.c 里的时间戳变量
uint32_t distance = 0;
volatile uint32_t delay_distance = 0;
uint8_t distance_count = 0;
uint8_t bizhang_flag = 0;
volatile uint32_t delay_bizhang = 0;
volatile uint32_t delay_light = 0;
uint8_t distance_flag = 1;
uint8_t sudu_flag = 1;
uint8_t falg_bizhang = 1;
uint8_t falg_buzz = 1;
volatile uint32_t f_ms_tick;
uint8_t falg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TRUN_SPEED 0
void Ctrl_SAT(int v, int a, uint32_t time, uint8_t stop_on_line)
{
  PID_Init(&pid_av, AV_P, AV_I, AV_D);
  pid_av.target_val = a;
  
  // 清除左右轮PID的积分误差，重新初始化
  PID_Init(&pid_r, PID_P, PID_I, PID_D);
  PID_Init(&pid_l, PID_P, PID_I, PID_D);
  pid_l.target_val = v;
  pid_r.target_val = v;
  
  get_encoder_count_right(&encoder_data_right);
  get_encoder_count_left(&encoder_data_left);
  uint32_t ms_ = tim3_ms_tick;
  pit_pid_flag = 0; // 同步定时器节拍

  for (;;)
  {
    if (yawReady)
    {
      yawReady = 0;
      angle = currentYaw;
    }
    
    // 如果启用了寻线停止功能，在死循环中也需要解析串口数据
    if (stop_on_line)
    {
      Parse_Track_Data(Rx_Buffer, sensor_data);
      uint8_t state = xunxian_scan(sensor_data);
      // 当检测到并非全白(丢线)且并非全黑时，说明压到线了，立即跳出
      if (state != 6 && state != 0) // zhuangxiang = 6
      {
        Motor_Stop();
        falg_bizhang = 0;
        return;
      }
    }
    
    // 必须等待定时器中断标志位，才能保证速度PID结算周期正确
    if (pit_pid_flag)
    {
      pit_pid_flag = 0;
      get_encoder_count_right(&encoder_data_right);
      get_encoder_count_left(&encoder_data_left);
      
      // 【核心修复】：分离速度环与角度环的耦合，消除二次转向抽搐现象。
      // 计算底盘两侧轮子的“平均重心速度”，代表车辆实际的前进分量
      int avg_speed = (encoder_data_right + encoder_data_left) / 2;
      
      // 使用 pid_l 作为整个底盘统一的“基准推力”速度环，算出一个统一的基础 PWM
      int pwm_base = PID_Speed(&pid_l, avg_speed);
      
      int diff = PID_Angle(&pid_av, angle);
      
      // 如果是原地转向（基础目标速度为0），对角度环产生的差速输出进行限幅，避免转得太猛
      if (v == 0)
      {
        int max_diff = 30; // 你可以在这里调节原地转身的限幅大小，越小转得越慢越柔和
        if (diff > max_diff) diff = max_diff;
        if (diff < -max_diff) diff = -max_diff;
      }
      
      // 左右轮的瞬时输出绝对保持一致基础（避免了分别测算引发的瞬时受力不均），再叠加用于方向管控的 diff 差值
      pwm_right = pwm_base + diff;
      pwm_left = pwm_base - diff;
      
      Motor_Right_Run(pwm_right);
      Motor_Left_Run(pwm_left);
      
      OLED_Printf(0, 8, OLED_6X8, "pwm_R:%d L:%d    ", pwm_right, pwm_left); 
      OLED_Printf(0, 16, OLED_6X8, "diff: %d    ", angle); 
      OLED_Update();
    }

    if (time > 0 && (tim3_ms_tick - ms_ >= time))
    {
        Motor_Stop();
        falg_bizhang = 0;
        return;
    }
    else
    {
      falg_bizhang = 1;
    }
  }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  OLED_Init();
  PID_Init(&pid_r, PID_P, PID_I, PID_D);
  PID_Init(&pid_l, PID_P, PID_I, PID_D);
  PID_Init(&pid_angle, ANGLE_P, ANGLE_I, ANGLE_D);
  HAL_TIM_Base_Start_IT(&htim3);
  pid_l.target_val = Speed_Straight;
  pid_r.target_val = Speed_Straight;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart2, &Rx_Byte, 1);
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  char start_cmd[] = "$0,0,1#";
  HAL_UART_Transmit(&huart2, (uint8_t *)start_cmd, strlen(start_cmd), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
 
  while (1)
  {

    // 1. 最高频执行：解析数据和状态
    Parse_Track_Data(Rx_Buffer, sensor_data);
    xunxian_state = xunxian_scan(sensor_data);
    
    if (zhuanxiang_flag != 0) // 仅在转弯时调用 Led_Show
    {
      if (delay_light >= 100)
      {
        Led_Show();
        delay_light = 0;
      }
    }
    else
    {
      // 退出转弯时，重置灯光延时，并保证灯光熄灭
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // 熄灭 LED (PC13 一般高电平熄灭)
      delay_light = 0;
    }

    // 超声波测量周期必须大于 60ms (避免声波重叠及模块死机)
    if (delay_distance > 60)
    {
      HCSR04_Trigger();                // 发射超声波脉冲
      distance = HCSR04_GetDistance(); // 获取上一次测量的结果
      delay_distance = 0;              // 重置计数器
    }

    OLED_Printf(0, 0, OLED_6X8, "R:%d L:%d", encoder_data_right, encoder_data_left);
    OLED_Printf(0, 8, OLED_6X8, "pwm_R:%d L:%d    ", pwm_right, pwm_left);
    OLED_Printf(0, 16, OLED_6X8, "Sen: %d %d %d %d %d %d %d %d", sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6], sensor_data[7]);
    OLED_Printf(0, 24, OLED_6X8, "Dist: %d cm   ", distance);
    OLED_Printf(0, 32, OLED_6X8, "flag:%d    ", zhuanxiang_flag);
    OLED_Printf(0, 48, OLED_6X8, "state:%d    ", xunxian_state);
    OLED_Printf(0, 40, OLED_6X8, "Yaw: %d deg   ", angle);
    OLED_Update();

    if (yawReady)
    {
      yawReady = 0;
      angle = currentYaw;
    }

    // 注意将 uint32_t 转换为有符号整型检查，因为 HCSR04 可能会返回 -1 或 -2
    int32_t current_dist = (int32_t)distance;

    if (current_dist >= 0 && current_dist < 30)
    {
      if (distance_count < 200) distance_count++; // 增加上限防止 uint8_t 溢出

      if (distance_count >= 10)
      {
        if (bizhang_flag != 1) // 确保已经处于避障停车状态时，不再反复重置计时器
        {
          Motor_Stop();
          // 停车：速度目标清零，防止最下面的 PID 计算重新把车跑起来
          xunxian_flag = 0;
          bizhang_flag = 1;  // 首先进入阶段1：等待 15 秒
          distance_flag = 0; // 禁用距离检测新触发
          // 记录遇到障碍物【停车瞬间】的原始角度作为基准
          pid_angle.target_val = angle;
          
          // 重置蜂鸣器和避障计时以便响5秒
          delay_buzz = 0;
          delay_bizhang = 0;
        }
      }
    }
    else
    {
      distance_count = 0;
    }

    if (bizhang_flag == 1)
    {
      // 蜂鸣器非阻塞响5秒
      if (delay_buzz < 5000)
      {
				 falg = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // 蜂鸣器响 (假设低电平有效)15
      }
      else
      {
				falg = 2;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   // 蜂鸣器停止
      }
			
      if(delay_bizhang >= 15000)
      {
        bizhang_flag = 2;
        distance_count = 0;
        int f_a = angle;
        
        // 根据经过直角的次数（angle_flag）和顺时针/逆时针方向，来计算当前赛道的理论绝对平行角度
        int parallel_angle = 0;
        if (global_turn_dir == 2) // 锁定为右转赛道
        {
            switch(angle_flag) {
                case 0: parallel_angle = 0; break;
                case 1: parallel_angle = -90; break;
                case 2: parallel_angle = -180; break;
                case 3: parallel_angle = 90; break;
                default: parallel_angle = f_a; break;
            }
            
            // 右转赛道的避障动作：向左绕开
            // 1. 原地右转出弯角度（向右绕开）
            Ctrl_SAT(0, f_a - 90, 1500, 0); 
            // 2. 调用速度环走直线（保持转出的角度，耗时2秒）
            Ctrl_SAT(5, f_a - 90, 2000, 0); 
            // 3. 原地旋转回到平行于下个赛段黑线的绝对角度
            Ctrl_SAT(0, parallel_angle, 1500, 0);       
            // 4. 直行直到传感器碰到黑线
            Ctrl_SAT(5, parallel_angle, 0, 1);
            // 新增：寻到黑线后往前开0.5秒
            Ctrl_SAT(5, parallel_angle, 700, 0);
            // 5. 碰到黑线后，调用角度环原地右转90度，正对新的寻线方向
            Ctrl_SAT(0, parallel_angle - 90, 1500, 0); 
            
            // 【同步更新】：强制跳过了一个右直角
            global_turn_dir = 2; // 确保锁定右转
            angle_flag++;
        }
        else // global_turn_dir == 1 或 0 (默认左转赛道)
        {
            switch(angle_flag) {
                case 0: parallel_angle = 0; break;
                case 1: parallel_angle = 90; break;
                case 2: parallel_angle = -180; break;
                case 3: parallel_angle = -90; break;
                default: parallel_angle = f_a; break;
            }
            
            // 左转赛道的避障动作：向右绕开
            // 1. 原地右转出弯角度（基于当前实际车头 f_a 转 90 度以脱开障碍物）
            Ctrl_SAT(0, f_a - 90, 1500, 0); 
            // 2. 调用速度环走直线（速度5，保持转出的角度，耗时2秒）
            Ctrl_SAT(5, f_a - 90, 2000, 0); 
            // 3. 原地旋转回到平行于黑线的绝对角度
            Ctrl_SAT(0, parallel_angle, 1500, 0);       
            // 4. 不要预备切线，直接保持理论平行的方向直行，直到传感器碰到黑线
            Ctrl_SAT(5, parallel_angle, 0, 1);
            // 新增：寻到黑线后往前开0.5秒
            Ctrl_SAT(5, parallel_angle, 500, 0);
            // 5. 碰到黑线后，调用角度环原地左转90度，正对新的寻线方向，耗时1.5秒
            Ctrl_SAT(0, parallel_angle + 90, 1500, 0); 
            
            // 【同步更新】：强制跳过了一个左直角
            global_turn_dir = 1; // 确保锁定左转
            angle_flag++;
        }
        
        falg_bizhang = 1;
        xunxian_flag = 1;
        delay_bizhang = 0;
      }
      else
      {
        if (current_dist > 50)
        {
          delay_bizhang=0;
          bizhang_flag = 0;
          xunxian_flag = 1;
          distance_count = 0;
        }
      }
    }
    // 2. 寻线状态机
    if (xunxian_flag)
    {
      switch (xunxian_state)
      {
      case little_r: // 微微偏右
      {
        pid_l.target_val = Speed_Straight + Speed_Diff_l;
        pid_r.target_val = Speed_Straight - Speed_Diff_l;
        break;
      }
      case little_l: // 微微偏左
      {
        pid_l.target_val = Speed_Straight - Speed_Diff_l;
        pid_r.target_val = Speed_Straight + Speed_Diff_l;
        break;
      }
      case straight: // 直行
      {
        pid_l.target_val = Speed_Straight;
        pid_r.target_val = Speed_Straight;
        break;
      }
      case left: // 左转
      {
        // 【新增策略】如果全局已经锁定为右转，说明这是右转过冲导致的假信号，降级为普通微调
        if (global_turn_dir == 2)
        {
          pid_l.target_val = Speed_Straight - Speed_Diff_l;
          pid_r.target_val = Speed_Straight + Speed_Diff_l;
          break;
        }

        // 【优化3】冷却期内拦截
        if (turn_cooldown > 0)
        {
          pid_l.target_val = Speed_Straight - Speed_Diff_l;
          pid_r.target_val = Speed_Straight + Speed_Diff_l;
          break;
        }

        // 【新增策略】第一次遇到真正的左转，全局锁定
        if (global_turn_dir == 0)
        {
          global_turn_dir = 1;
        }

        delay = 0;
        zhuanxiang_flag = 1;
        angle_flag++;
        switch (angle_flag)
        {
        case 1:
          target_angle = 55;
          break;
        case 2:
          target_angle = 150;
          break;
        case 3:
          target_angle = -110;
          break;
        case 4:
          target_angle = -70;
          break;
        default:
          break;
        }
        break;
      }
      case right: // 右转
      {
        // 【新增策略】如果全局已经锁定为左转，说明这是左转过冲导致的假信号，降级为普通微调
        if (global_turn_dir == 1)
        {
          pid_l.target_val = Speed_Straight + Speed_Diff_l;
          pid_r.target_val = Speed_Straight - Speed_Diff_l;
          break;
        }

        // 【优化3】冷却期内拦截
        if (turn_cooldown > 0)
        {
          pid_l.target_val = Speed_Straight + Speed_Diff_l;
          pid_r.target_val = Speed_Straight - Speed_Diff_l;
          break;
        }

        // 【新增策略】第一次遇到真正的右转，全局锁定
        if (global_turn_dir == 0)
        {
          global_turn_dir = 2;
        }

        delay = 0;
        zhuanxiang_flag = 2;
        angle_flag++;
        switch (angle_flag)
        {
        case 1:
          target_angle = -55;
          break;
        case 2:
          target_angle = -150;
          break;
        case 3:
          target_angle = 110;
          break;
        case 4:
          target_angle = 150;
          break;
        default:
          break;
        }
        break;
      }
      default:
        break;
      }
    }

    // 3. 转向状态机
    if (zhuanxiang_flag == 1) // 左转
    {
      if (delay <= 2000)
      {
        Motor_Stop();
        xunxian_flag = 0;
        delay_flag = 0;
      }
      else
      {
        delay_flag = 1;

        // 计算当前角度到目标角度的“剩余航向角”
        // 这个公式可以自动处理 180 到 -180 的跳变问题
        int angle_error = target_angle - angle;
        if (angle_error > 180)
          angle_error -= 360;
        if (angle_error < -180)
          angle_error += 360;

        // 对于左转，angle_error 应该是正数（目标在当前的“左前方”）
        // 当这个差值减小到 0 甚至变负时，说明转过头了
        if (angle_error > 0) // 只要还没转到目标位置
        {
          pid_l.target_val = -10;
          pid_r.target_val = 20;
        }
        else
        {
          xunxian_flag = 1;
          zhuanxiang_flag = 0;
          turn_cooldown = 500;
          
          if (angle_flag >= 4)
          {
            // 转完第4个弯了，强行往前开0.5秒然后永久停车
            Ctrl_SAT(5, target_angle, 500, 0);
            Motor_Stop();
            while(1)
            {
               // 永久停止
               Motor_Stop();
            }
          }
        }
      }
    }
    else if (zhuanxiang_flag == 2) // 右转
    {
      if (delay <= 2000)
      {
        Motor_Stop();
        xunxian_flag = 0;
        delay_flag = 0;
      }
      else
      {
        delay_flag = 1;

        // 计算当前角度到目标角度的“剩余航向角”
        // 这个公式可以自动处理 180 到 -180 的跳变问题
        int angle_error = target_angle - angle;
        if (angle_error > 180)
          angle_error -= 360;
        if (angle_error < -180)
          angle_error += 360;

        // 对于右转，angle_error 应该是负数（目标在当前的“右前方”）
        // 当这个差值增加到 0 甚至变正时，说明转过头了
        if (angle_error < 0) // 只要还没转到目标位置
        {
          pid_l.target_val = 20;
          pid_r.target_val = -10;
        }
        else
        {
          xunxian_flag = 1;
          zhuanxiang_flag = 0;
          turn_cooldown = 500; // 【优化3】结束转弯，赋予冷却期
          
          if (angle_flag >= 4)
          {
            // 转完第4个弯了，强行往前开0.5秒然后永久停车
            Ctrl_SAT(5, target_angle, 500, 0);
            Motor_Stop();
            while(1)
            {
               // 永久停止
               Motor_Stop();
            }
          }
        }
      }
    }

    // 4. PID 运算与低频 OLED 刷新
    if (pit_pid_flag && delay_flag && falg_bizhang)
    {
      pit_pid_flag = 0;

      // 如果处于避障阶段 1 (死等15秒)，或者纯粹为了全停，直接封杀电机运转并跳过 PID
      if (bizhang_flag == 1)
      {
        Motor_Stop();
        pid_l.output_val = 0;
        pid_r.output_val = 0;
        pwm_left = 0;
        pwm_right = 0;
      }
      else
      {
        get_encoder_count_right(&encoder_data_right);
        get_encoder_count_left(&encoder_data_left);
        pwm_right = PID_Speed(&pid_r, encoder_data_right);
        pwm_left = PID_Speed(&pid_l, encoder_data_left);

        Motor_Right_Run(pwm_right);
        Motor_Left_Run(pwm_left);
      }
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    pit_pid_flag = 1;
    delay += 1;
    tim3_ms_tick++;
    f_ms_tick++;
    delay_distance++;
    delay_bizhang++;
    delay_light++;
		delay_buzz++;
    // 【优化3】在中断中递减冷却时间
    if (turn_cooldown > 0)
    {
      turn_cooldown--;
    }
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

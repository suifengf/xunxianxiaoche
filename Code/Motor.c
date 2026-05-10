#include "hfile.h"

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

void Motor_Stop(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
}

void Motor_Right_Run(int pwm_r)
{
    if(pwm_r >= 0)
    {
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
    }
    else
    {
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
      pwm_r = -pwm_r; // 设置好反转后，再提取绝对值给 PWM 寄存器
    }
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_r);
}

void Motor_Left_Run(int pwm_l)
{
    if(pwm_l >= 0)
    {
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);
    }
    else
    {
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);
        pwm_l = -pwm_l; // 设置好反转后，再提取绝对值
    }
}

void get_encoder_count_left(uint16_t* count_l)
{
	

}

void get_encoder_count_right(uint16_t* count_r)
{
	
}
#ifndef __MOTOR_H_
#define __MOTOR_H_

#define PWM_LEFT                 (PWMB_CH3_P76)
#define PWM_RIGHT                (PWMD_CH2_P51)
#include "hfile.h"

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Right_Run(int pwm_r);
void Motor_Left_Run(int pwm_l);
void get_encoder_count_left(uint16_t* count);
void get_encoder_count_right(uint16_t* count);
#endif

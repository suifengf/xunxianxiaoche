#include "hfile.h"


void PID_Init(PID_t *pid, int p, int i, int d) 
{
    pid->target_val = 0;
    pid->actual_val = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->err_prev = 0;
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->output_val = 0;
}

int PID_Speed(PID_t *pid, int current_speed) 
{
    /* C89 规定：变量声明必须在函数最开头 */
    int increment_val; /* 使用 32 位防止乘法过程溢出 */
    int term_p, term_i, term_d; 
    
    /* 1. 获取当前实际速度并计算当前误差 */
    pid->actual_val = current_speed;
    pid->err = pid->target_val - pid->actual_val;

    /* 
     * 2. 分步计算 P, I, D 三个项
     * 注意：先将 16 位的参数强制转换为 32 位 (int32)，
     * 否则在 STC/Keil 环境下两个 16 位整数相乘可能会在 16 位寄存器里溢出。
     */
    term_p = (int)pid->Kp * (pid->err - pid->err_last);
    term_i = (int)pid->Ki * pid->err;
    term_d = (int)pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);

    /* 3. 组合增量，并缩小之前放大的倍数 (假设放大了 100 倍) */
    increment_val = (term_p + term_i + term_d) / 100;

    /* 4. 累加增量，得到最终输出量 */
    pid->output_val += increment_val;

    /* 5. 更新误差状态，为下一次计算做准备 */
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;

    /* 6. 输出限幅 (假设 PWM 范围 0 - 10000) */
    if (pid->output_val > 60) 
    {
        pid->output_val = 60;
    } 
    else if (pid->output_val < -60) 
    { 
        pid->output_val = -60;
    }

    /* 截断返回 16 位结果 */
    return (int)pid->output_val;
}
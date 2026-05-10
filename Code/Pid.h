#ifndef __PID_H_
#define __PID_H_
typedef struct {
    int target_val;   /* 目标速度 */
    int actual_val;   /* 实际速度 */
    int err;          /* 当前误差 e(k) */
    int err_last;     /* 上一次误差 e(k-1) */
    int err_prev;     /* 上上次误差 e(k-2) */
    
    /* 
     * 放大后的 PID 参数。
     * 例如真实 Kp 为 2.5，则存储为 250 (放大100倍)
     */
    int Kp;
    int Ki;
    int Kd;
    
    int output_val;   /* 最终输出的控制量 (如 PWM 占空比) */
} PID_t;

void PID_Init(PID_t *pid, int p, int i, int d);
int PID_Speed(PID_t *pid, int current_speed);

#endif
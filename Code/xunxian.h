#ifndef __XUNXIAN_H_
#define __XUNXIAN_H_
#include "hfile.h"
#define Black 0
#define White 1
#define little_r 1
#define little_l 2
#define straight 3
#define left 4
#define right 5
#define zhuangxiang 6

// 将宏定义移到这里，供所有包含此头文件的 .c 文件使用
#define RX_MAX_LEN 100

// 使用 extern 声明变量，告诉编译器这些变量在别的地方分配了内存
extern uint8_t Rx_Byte;
extern char Rx_Buffer[RX_MAX_LEN];
extern uint8_t Rx_Index;
extern uint8_t Rx_Complete_Flag;

uint8_t xunxian_scan(uint8_t* color);
uint8_t run(uint8_t flag);
void buzzer_on(void);
uint8_t Parse_Track_Data(const char* rx_buffer, uint8_t* sensor_data);

#endif
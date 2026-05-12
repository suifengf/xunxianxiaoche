#ifndef __XUNXIAN_H_
#define __XUNXIAN_H_

#include "hfile.h"

// --- 宏定义 ---
#define Black 0
#define White 1
#define little_r 1
#define little_l 2
#define straight 3
#define left 4
#define right 5
#define zhuangxiang 6
#define NO_VALID_DATA 999.0f
#define RX_MAX_LEN 100

#define TRIG_PORT GPIOB
#define TRIG_PIN  GPIO_PIN_4
#define ECHO_PORT GPIOB
#define ECHO_PIN  GPIO_PIN_3
// --- 变量声明 (全部加上 extern，只声明不定义) ---
// JY61P 串口接收相关变量声明
extern uint8_t rxByte;          
extern uint8_t rxBuffer[11];    
extern uint8_t rxIndex;     
extern volatile int currentYaw;
extern volatile uint8_t yawReady;    

// 循迹模块 串口接收相关变量声明
extern uint8_t Rx_Byte;
extern char Rx_Buffer[RX_MAX_LEN];
extern uint8_t Rx_Index;
extern uint8_t Rx_Complete_Flag;

// --- 函数声明 ---
int GetYawAngle(uint8_t *packet);
uint8_t xunxian_scan(uint8_t* color);
uint8_t run(uint8_t flag);
void buzzer_on(void);
uint8_t Parse_Track_Data(const char* rx_buffer, uint8_t* sensor_data);
void HCSR04_Trigger(void);
int32_t HCSR04_GetDistance(void);

uint32_t Get_System_Us(void);
void delay_us(uint16_t us);
void Buzzer_Tone(uint16_t freq, uint16_t duration_ms);
void Play_Sprinkler_Music_Block(void);

#endif


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

void xunxian_init(void);
uint8_t xunxian_scan(uint8_t* color);
uint8_t run(uint8_t flag);
void buzzer_on(void);


#endif
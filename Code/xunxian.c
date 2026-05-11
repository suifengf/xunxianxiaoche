#include "xunxian.h"
#include <string.h>
#include <stdio.h>

// --- 真正的变量定义与初始化 (在这里分配内存，千万不要加 extern) ---
// 循迹模块接收变量定义
uint8_t Rx_Byte = 0;                
char Rx_Buffer[RX_MAX_LEN] = {0};     
uint8_t Rx_Index = 0;           
uint8_t Rx_Complete_Flag = 0;

// JY61P 接收变量定义
uint8_t rxByte = 0;          
uint8_t rxBuffer[11] = {0};    
uint8_t rxIndex = 0;     
volatile int currentYaw = 0;
volatile uint8_t yawReady = 0; 
volatile uint32_t tim3_ms_tick = 0;

/**
 * @brief 解析8路循迹模块的数字型数据
 * @param rx_buffer 接收到的串口字符串缓冲区
 * @param sensor_data 用于存放解析结果的数组首地址
 * @return 1: 解析成功, 0: 解析失败
 */
uint8_t Parse_Track_Data(const char* rx_buffer, uint8_t* sensor_data) {
    // 1. 查找数据头 "$D," 是否存在
    char *start = strstr(rx_buffer, "$D,");
    if (start == NULL) {
        return 0; // 未找到正确的数字型数据头
    }

    // 2. 准备一个临时数组存放提取的整型数据
    int temp_data[8];
    
    // 3. 使用 sscanf 按照协议格式提取数据
    int parsed_count = sscanf(start, "$D,x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d#",
                              &temp_data[0], &temp_data[1], &temp_data[2], &temp_data[3],
                              &temp_data[4], &temp_data[5], &temp_data[6], &temp_data[7]);

    // 4. 验证是否成功提取了全部8个数据
    if (parsed_count == 8) {
        // 将 int 类型数据存入 uint8_t 数组，索引 0-7 一一对应 x1-x8
        for (int i = 0; i < 8; i++) {
            sensor_data[i] = (uint8_t)temp_data[i];
        }
        return 1; // 解析成功
    }

    return 0; // 数据格式不匹配或不完整
}

uint8_t Get_Sensor_State(uint8_t *color) 
{
    uint8_t state = 0;
    for (int i = 0; i < 8; i++) 
    {
        if (color[i] == Black) 
        {
            state |= (1 << (7 - i)); // 把状态存入对应的二进制位
        }
    }
    return state;
}

uint8_t xunxian_scan(uint8_t* color)
{
    uint8_t state = Get_Sensor_State(color);

    // 【核心说明】根据 Get_Sensor_State：
    // bit7对应最左(1号)，bit0对应最右(8号)
    // 1 代表压到黑线，0 代表白底。

    // 【1】极端情况：全白（丢线）或全黑（十字路口/停止线）
    if (state == 0x00) return zhuangxiang; // 0b00000000 全白丢线
    if (state == 0xFF) return zhuangxiang; // 0b11111111 全黑

    // 【2】判断转直角/大弯 (左右两侧最边缘的探头大面积压线)
    // 左直角：必须包含1号探头，且2、3号也有压线 (如0xE0, 0xC0, 0xA0)
    uint8_t left_edge = state & 0xE0; // 提取1, 2, 3号探头 (0b11100000)
    if (left_edge == 0xE0 || left_edge == 0xC0 || left_edge == 0xA0) 
    {
        return left; 
    }
    
    // 右直角：必须包含8号探头，且6、7号也有压线 (如0x07, 0x03, 0x05)
    uint8_t right_edge = state & 0x07; // 提取6, 7, 8号探头 (0b00000111)
    if (right_edge == 0x07 || right_edge == 0x03 || right_edge == 0x05) 
    {
        return right;
    }

    // 【3】常规循迹与微调 (中间探头)
    switch (state) 
    {
        // === 完美的直线 (居中) ===
        case 0x18: // 0b00011000 (4和5号压线)
        case 0x10: // 0b00010000 (只有4号压线)
        case 0x08: // 0b00001000 (只有5号压线)
        case 0x38: // 0b00111000 (3,4,5号压线，线较粗时)
        case 0x1C: // 0b00011100 (4,5,6号压线，线较粗时)
            return straight;

        // === 车身微微偏右，需要向左微调 (little_l) ===
        // 黑线现在处于左侧的 2, 3 号探头位置
        case 0x30: // 0b00110000 (3和4号压线)
        case 0x20: // 0b00100000 (只有3号压线)
        case 0x60: // 0b01100000 (2和3号压线)
        case 0x40: // 0b01000000 (只有2号压线)
            return little_l;

        // === 车身微微偏左，需要向右微调 (little_r) ===
        // 黑线现在处于右侧的 6, 7 号探头位置
        case 0x0C: // 0b00001100 (5和6号压线)
        case 0x04: // 0b00000100 (只有6号压线)
        case 0x06: // 0b00000110 (6和7号压线)
        case 0x02: // 0b00000010 (只有7号压线)
            return little_r;

        // === 未识别的状态 ===
        default:
            // 遇到毛刺或者不规则反光，默认保持直行，靠惯性冲过去找线
            return 0; 
    }
}


/**
 * @brief  获取当前系统精确运行时间（单位：微秒 us）
 * @note   巧妙利用 1ms中断次数 + 定时器当前计数值(0-999) 拼接出精确到 1us 的时间戳
 */
uint32_t Get_System_Us(void)
{
    uint32_t ms;
    uint32_t us;
    
    __disable_irq();
    ms = tim3_ms_tick;
    us = __HAL_TIM_GET_COUNTER(&htim3);
    
    // 如果此刻已经溢出且中断还没处理 (UIF已置位)
    if ((__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) && (us < 500))
    {
        ms++;
    }
    __enable_irq();
    
    return (ms * 1000) + us;
}

/**
 * @brief  非破坏性微秒级延时函数
 */
void delay_us(uint16_t us)
{
    uint32_t start_time = Get_System_Us();
    // 一直死等，直到当前时间与起始时间之差达到预期的 us 数
    while ((Get_System_Us() - start_time) < us);
}


/**
 * @brief  获取 HC-SR04 测量的距离 (纯整型优化版)
 * @retval 测量到的距离（单位：厘米 cm）。返回 -1 表示超时/未连接，-2 表示超出量程。
 */
int32_t HCSR04_GetDistance(void)
{
    uint32_t start_us = 0;
    uint32_t end_us = 0;
    uint32_t echo_time = 0;
    uint32_t timeout_start = 0;

    // 0. 等待之前的 ECHO 完成 (防止模块卡死导致 ECHO 一直为高，或上次测距还没结束)
    timeout_start = Get_System_Us();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((Get_System_Us() - timeout_start) > 50000) return -2; // 模块异常卡死
    }

    // 1. 发送 15us 触发脉冲
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // 2. 等待 ECHO 变高并记录起始时间 (带 10ms 超时保护)
    timeout_start = Get_System_Us();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((Get_System_Us() - timeout_start) > 10000) return -1; 
    }
    start_us = Get_System_Us(); // 记录回响开始的时间戳

    // 3. 等待 ECHO 变低并记录结束时间
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        end_us = Get_System_Us();
        // 持续时间超过 38000us (约 6.5米)，说明超出量程
        if ((end_us - start_us) > 38000) return -2;
    }

    // 4. 计算距离 (纯整型运算，省内存和算力)
    echo_time = end_us - start_us;
    
    // 往返耗时除以 58，直接得出整型的厘米数
    return (int32_t)(echo_time / 58); 
}


/**
  * @brief  串口接收完成回调函数
  * @param  huart: 串口句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是不是串口 2 触发的中断 (循迹模块)
    if (huart->Instance == USART2) 
    {
        // 1. 将接收到的字节存入缓冲区
        Rx_Buffer[Rx_Index] = Rx_Byte;
        
        // 2. 判断是否收到结束符 '#'
        if (Rx_Buffer[Rx_Index] == '#') 
        {
            Rx_Buffer[Rx_Index + 1] = '\0'; // 在末尾手动添加字符串结束符
            Rx_Complete_Flag = 1;           // 标记接收完成
            Rx_Index = 0;                   // 索引清零，为下一次接收做准备
        }
        else 
        {
            Rx_Index++; // 如果没收到结束符，索引继续后移
            
            // 防止数组越界：如果数据出错导致一直没收到 '#'，强制清零重新开始
            if (Rx_Index >= RX_MAX_LEN - 1) 
            {
                Rx_Index = 0;
                memset(Rx_Buffer, 0, RX_MAX_LEN);
            }
        }
        
        // 3. 重新开启中断接收，等待下一个字节
        HAL_UART_Receive_IT(huart, &Rx_Byte, 1);
    }

    // 判断是不是串口 3 触发的中断 (JY61P)
    if (huart->Instance == USART3) {
        
        // 状态机处理接收到的字节
        if (rxIndex == 0) {
            if (rxByte == 0x55) {
                rxBuffer[rxIndex++] = rxByte;
            }
        } 
        else if (rxIndex == 1) {
            if (rxByte == 0x53) {
                rxBuffer[rxIndex++] = rxByte; // 找到角度包包头
            } else if (rxByte == 0x55) {
                // 如果连续收到0x55，保持索引在1，等待0x53
                rxIndex = 1; 
            } else {
                rxIndex = 0; // 帧头错误，重新寻找
            }
        } 
        else if (rxIndex > 1) {
            rxBuffer[rxIndex++] = rxByte;
            
            // 接收满 11 个字节
            if (rxIndex == 11) {
                float yaw = GetYawAngle(rxBuffer);
                if (yaw != NO_VALID_DATA) {
                    currentYaw = yaw; // 更新全局变量
                    yawReady = 1;     // 通知主循环数据已就绪
                }
                rxIndex = 0; // 清零索引，准备接收下一个包
            }
        }

        // 重新开启串口3的单字节中断接收 (非常重要，否则只能进一次中断)
        HAL_UART_Receive_IT(huart, &rxByte, 1);
    }
}


/**
 * @brief 解析串口接收到的 JY61P 角度数据包并返回航向角(Yaw)
 * * @param packet 包含11个字节的串口数据缓冲区
 * @return float 返回计算后的偏航角(单位: 度)。如果校验失败或帧头不对，返回 NO_VALID_DATA
 */
int GetYawAngle(uint8_t *packet) {
    // 1. 检查帧头是否为 0x55 (包头) 和 0x53 (角度包)
    if (packet[0] != 0x55 || packet[1] != 0x53) {
        return (int)NO_VALID_DATA; // 帧头错误
    }

    // 2. 校验和计算 (SUM = 0x55 + 0x53 + ... + VH + VL)
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += packet[i];
    }
    
    // 对比校验和是否与第11个字节一致
    if (sum != packet[10]) {
        return (int)NO_VALID_DATA; // 校验失败，数据可能丢包或干扰
    }

    // 3. 提取偏航角 Yaw 的高低字节
    uint8_t yawL = packet[6];
    uint8_t yawH = packet[7];

    // 4. 拼接并转换为有符号 16 位整型 (非常重要，否则无法识别负角度)
    int16_t yawRaw = (int16_t)((yawH << 8) | yawL);

    // 5. 根据公式计算真实角度
    int yawAngle = ((int)yawRaw / 32768.0f) * 180.0f;

    return yawAngle;
}
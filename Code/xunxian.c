#include "xunxian.h"

// 这里是真正的变量定义（分配内存），注意不要加 extern
uint8_t Rx_Byte;                
char Rx_Buffer[RX_MAX_LEN];     
uint8_t Rx_Index = 0;           
uint8_t Rx_Complete_Flag = 0;

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
            return straight; 
    }
}

/**
  * @brief  串口接收完成回调函数
  * @param  huart: 串口句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是不是串口 2 触发的中断
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
        HAL_UART_Receive_IT(&huart2, &Rx_Byte, 1);
    }
}
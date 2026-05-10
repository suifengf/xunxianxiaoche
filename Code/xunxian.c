#include "xunxian.h"
#define O1  IO_P54
#define O2  IO_P94
#define O3  IO_P92
#define O4  IO_P93



uint8_t xunxian_scan(uint8_t* color)
{
		
    if (color[0] == White && color[1] == Black && color[2] == White && color[3] == White)// 1 0 1 1表示微微偏右
    {
        return little_r;
    }
    else if (color[0] == White && color[1] == White && color[2] == Black && color[3] == White)// 1 1 0 1表示微微偏左
    {
        return little_l;
    }
    else if (color[0] == White && color[1] == Black && color[2] == Black && color[3] == White)
    {
        return straight;
    }
    else if (color[0] == Black && color[1] == Black && color[2] == Black && color[3] == White)
    {
        return left;
    }
    else if (color[0] == White && color[1] == Black && color[2] == Black && color[3] == Black)
    {
        return right;
    }
    else if(color[0] == Black && color[1] == Black && color[2] == White && color[3] == White)
    {
        return left;
    }
    else if(color[0] == White && color[1] == White && color[2] == Black && color[3] == Black)
    {
        return right;
    }
    else if(color[0] == White && color[1] == White && color[2] == White && color[3] == White)
    {
        return zhuangxiang;
    }
    else if(color[0] == Black && color[1] == White && color[2] == White && color[3] == White)
    {
        return left;
    }
    else
    {
        return 0; // 无效状态
    }
}


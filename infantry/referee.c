#include "referee.h"
#include "usart.h"

#include <math.h>
float tow_to_ten(uint32_t dat);

struct {
    uint8_t seq;
    uint8_t CRC8;
    uint16_t data_length;
    uint16_t cmd_id;
} head;

Referee_Type refe;

uint8_t head_buffer[7];
uint8_t refe_buffer[256];
char HeadRx = 1;



void Start_Referee_Rx(void)
{
    HAL_UART_Receive_DMA(&huart6,head_buffer,7);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart6)
    {
        if(HeadRx && head_buffer[0]==0xA5)
        {
            uint16_t len = head_buffer[1] | head_buffer[2]<<8; head.data_length = len;
            head.seq = head_buffer[3];
            head.CRC8 = head_buffer[4];
            uint16_t id = head_buffer[5] | head_buffer[6]<<8; head.cmd_id = id;
            //go on
            HeadRx = 0;
            HAL_UART_Receive_DMA(&huart6,refe_buffer,len+2);
        }
        else
        {
            if(head.cmd_id==0x201)
            {
                refe.shoot_cooling_value = refe_buffer[6] | refe_buffer[7]<<8;
                refe.shoot_heat_limit = refe_buffer[8] | refe_buffer[9]<<8;
                refe.chas_power_limit = refe_buffer[10] | refe_buffer[11]<<8;
            }
            else if(head.cmd_id==0x202)
            {
                /*
                uint32_t source = refe_buffer[4] | refe_buffer[5]<<8 | refe_buffer[6]<<12 | refe_buffer[7]<<16;
                uint8_t  index    = 0;
	            uint32_t mantissa = 0;
	            float    decimal  = 0;
	            index = (uint8_t)((source&0x7f800000)>>23); //指数   第23-30位
	            mantissa = (source&0x007fffff);             //尾数   第0 -23位
	            decimal  = tow_to_ten(mantissa);            //尾数   第0 -23位转换成小数
	            refe.chas_power = (1+decimal)*pow(2,(index-127)); //IEE754转换公式
                */
                refe.shoot_heat = refe_buffer[10] | refe_buffer[11]<<8;
            }
            HeadRx = 1;
            HAL_UART_Receive_DMA(&huart6,head_buffer,7);
        }
    }
}


float tow_to_ten(uint32_t dat)
{
    float  result = 0.0;
    uint8_t  count = 1;
    uint32_t num  = 0;

    num = (dat<<9)&0xffffffff;
    while(num>0)
    {
		result += ((num&0x80000000)>>31)*pow(0.5,count);
		count++;
		num = num<<1;
		if(count>23)
		{
				break;
		}
    }
	return result;
}
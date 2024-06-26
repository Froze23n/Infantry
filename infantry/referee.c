#include "referee.h"
#include "usart.h"

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
struct {
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
    uint16_t cmd_id;
} head;

Referee_Type refe;
static uint8_t head_buffer[7];
static uint8_t refe_buffer[1024];
static char HeadRx = 1;

void Start_Referee_Rx(void)
{
    HAL_UART_Receive_DMA(&huart6,head_buffer,7);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(&huart6==huart){
        if(HeadRx){
            if( (0xA5==head_buffer[0]) && Verify_CRC8_Check_Sum(head_buffer,5) ){
                uint16_t len = head_buffer[1] | head_buffer[2]<<8; head.data_length = len;
                head.seq = head_buffer[3];
                head.CRC8 = head_buffer[4];
                uint16_t id = head_buffer[5] | head_buffer[6]<<8; head.cmd_id = id;
                HeadRx = 0; HAL_UART_Receive_DMA(&huart6,refe_buffer,len+2); //go on
                return;
            }else{
                Start_Referee_Rx(); //go back
                return;
            }
        }else{// body Rx
            if(head.cmd_id==0x201){
                refe.shoot_cooling_value = refe_buffer[6] | refe_buffer[7]<<8;
                refe.shoot_heat_limit = refe_buffer[8] | refe_buffer[9]<<8;
                refe.chas_power_limit = refe_buffer[10] | refe_buffer[11]<<8;
            }else if(head.cmd_id==0x202){
                float* pfloat =(float*)( &refe_buffer[4] );
                refe.chas_power = *pfloat; 
                refe.shoot_heat = refe_buffer[10] | refe_buffer[11]<<8;
            }
            HeadRx = 1; HAL_UART_Receive_DMA(&huart6,head_buffer,7);
            return;
        }
    }
}

uint16_t Manage_Power_Limit(uint16_t newP)
{
    static const uint16_t Plist[16] = {45,50,55,60,65,70,75,80,85,90,95,100};
    static uint16_t Power = 45;
    if(newP!=Power)
    {
        for(unsigned i=0; i<12; i++)
        {
            if(Plist[i]==newP){Power = newP;}
        }
    }
    return Power;
}

uint16_t Manage_Cooling_Value(uint16_t newC)
{
    static const uint16_t Clist[15] = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80};
    static uint16_t Cooling = 40;
    if(newC!=Cooling)
    {
        for(unsigned i=0; i<15; i++)
        {
            if(Clist[i]==newC){Cooling = newC;}
        }
    }
    return Cooling;
}

uint16_t Manage_Heat_Limit(uint16_t newH)
{
    static const uint16_t Hlist[21] = {
        50,85,120,155,190,225,260,295,330,400,
        200,240,250,300,350,400,450,500,550,600,650
    };
    static uint16_t Heat = 50;
    if(newH!=Heat)
    {
        for(unsigned i=0; i<21; i++)
        {
            if(Hlist[i]==newH){Heat = newH;}
        }
    }
    return Heat;
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//crc8 generator polynomial:G(x)=x8+x5+x4+1  
const unsigned char CRC8_INIT = 0xff;  
const unsigned char CRC8_TAB[256] =  
{  
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,  
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,  
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,  
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,  
}; 
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)  
{  
    unsigned char ucIndex;  
    while (dwLength--)  
    {  
        ucIndex = ucCRC8^(*pchMessage++);  
        ucCRC8 = CRC8_TAB[ucIndex];  
    }  
    return(ucCRC8);  
} 
/*  
** Descriptions: CRC8 Verify function  
** Input: Data to Verify,Stream length = Data + checksum  
** Output: True or False (CRC Verify Result)  
*/  
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)  
{  
    unsigned char ucExpected = 0; 
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;  
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);  
    return ( ucExpected == pchMessage[dwLength-1] );  
}  
/*  
** Descriptions: append CRC8 to the end of data  
** Input: Data to CRC and append,Stream length = Data + checksum  
** Output: True or False (CRC Verify Result)  
*/  
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)  
{  
    unsigned char ucCRC = 0;  
    if ((pchMessage == 0) || (dwLength <= 2)) return;  
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);  
    pchMessage[dwLength-1] = ucCRC;  
}

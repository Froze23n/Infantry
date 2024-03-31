#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*结构体声名*/
typedef struct
{
   float kp;
   float ki;
   float kd;
} PID_T;

/*函数声名*/

//四轮底盘
int16_t Chassis_M3508_PID(int8_t ID, float expV, float truV);
float Chas_Calc_Z(float relative_angle);
//底盘~云台yaw轴
int16_t Yaw6020_PID(float expA, float truA, float truV, float feedV);
float yaw6020_velocity_to_voltage(float expV,float truV);

//云台pitch
int16_t Pitch6020_PID(float pError, float truV);
float pitch6020_velocity_to_voltage(float vError);
//云台拨弹盘,射击
int16_t Loader_M2006_PID(float pError);
int16_t Shooter_PID(float pError, int8_t ID);


#ifdef __cplusplus
}
#endif

#endif

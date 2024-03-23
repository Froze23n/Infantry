#ifndef _MOTORS_H_
#define _MORORS_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

struct MotorData_T {
	float Angle;     //累计角，弧度
	float Velocity;  //角速度，rad/s
} ;

/*-----------------------externs-----------------------*/
//can1数据：pitch电机&拨弹盘
extern struct MotorData_T Pitch6020;//角度&速度
extern float Loader_Velocity;//rad/s
extern float Shooter_Velocity[2];
//can2数据：yaw电机&麦轮
extern float Chassis_M3508_Velocity[4];//rad/s
extern float GIM_CHAS_Angle;//经过特殊处理，为(-pi,pi]


/*函数声名*/
void Enable_Motors(void);

void Gimbal_CAN_Tx(int16_t Pitch_Voltage, int16_t Loader_Current, int16_t Shooter_Current_0, int16_t Shooter_Current_1);
void Chassis_M3508_Tx(int16_t Current[4]);
void Chassis_GM6020_Tx(int16_t Yaw_Voltage);
void Chassis_Capacitor_Tx(uint16_t Power_Set);






#ifdef __cplusplus
}
#endif

#endif

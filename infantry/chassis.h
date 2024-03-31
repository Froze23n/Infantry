#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*函数声名*/
void Chassis_M3508_CMD(void);
void Chassis_Yaw6020_CMD(void);

void Capacitor_Control(void);

#ifdef __cplusplus
}
#endif

#endif

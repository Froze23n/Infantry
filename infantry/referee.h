#ifndef _REFEREE_H_
#define _REFEREE_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

typedef struct {
    //功率上线和当前功率
    uint16_t chas_power_limit;
    float chas_power;
    //枪口冷却速度，热量上线和当前热量
    uint16_t shoot_cooling_value;
    uint16_t shoot_heat_limit;
    uint16_t shoot_heat;
} Referee_Type;

extern Referee_Type refe;

void Start_Referee_Rx(void);
uint16_t Manage_Power_Limit(uint16_t newP);

uint16_t Manage_Cooling_Value(uint16_t newC);
uint16_t Manage_Heat_Limit(uint16_t newH);


#ifdef __cplusplus
}
#endif

#endif

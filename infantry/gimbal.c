#include "gimbal.h"

#include "motors.h"
#include "imu.h"
#include "rc.h"
#include "pid.h"

#define PI (3.1415927F)

const float pitch_negative_LIM = -600*PI/4096;
const float pitch_positive_LIM = 600*PI/4096;
const float rc_y_sensitivity = 1/400.0f;
const float mouse_l_sensitivity = 400;
const float mouse_y_sensitivity = 1/6000.0f;

float loader_speed_level = 0.5;

void Gimbal_Task(void)
{
    static float rcAngle = 0;
    static int16_t shootON = 0;
    float rcLoadV = 0;
    int16_t voltage,current; //pitch,load
    if(SW_UP==rc.sw1) //安全模式
    {
        shootON = 0;
        //int16_t stopCurrent0 = shooter_stop(0,Shooter_Velocity[0]);
        //int16_t stopCurrent1 = shooter_stop(1,Shooter_Velocity[1]);
        int16_t stopCurrent0 = -Shooter_Velocity[0];
        int16_t stopCurrent1 = -Shooter_Velocity[1];
        Gimbal_CAN_Tx(0,0,stopCurrent0,stopCurrent1);
        return;
    }
    else if(SW_MID == rc.sw1) //遥控器模式
    {
        //计算pitch
        rcAngle += rc_y_sensitivity*rc.RY;
        if(rcAngle < pitch_negative_LIM){rcAngle = pitch_negative_LIM;}
        if(rcAngle > pitch_positive_LIM){rcAngle = pitch_positive_LIM;}
        voltage = Pitch6020_PID(rcAngle-Pitch6020.Angle , Pitch6020.Velocity);
        //计算拨弹盘
        rcLoadV = ((float)rc.wheel - 1024.0f)*loader_speed_level;  if(rcLoadV<-100){rcLoadV=-100;}
        current = Loader_M2006_PID(rcLoadV - Loader_Velocity);
        //是否开启摩擦轮？
        if(rcLoadV){shootON=1;}
        //tx
        Gimbal_CAN_Tx(voltage ,current , shootON*(-6500) , shootON*(+6500) );
    }
    else if(SW_DOWN == rc.sw1) //键鼠控制
    {
        //pitch
        rcAngle += mouse_y_sensitivity*rc.mouse.y;
        if(rcAngle < pitch_negative_LIM){rcAngle = pitch_negative_LIM;}
        if(rcAngle > pitch_positive_LIM){rcAngle = pitch_positive_LIM;}
        voltage = Pitch6020_PID(rcAngle-Pitch6020.Angle , Pitch6020.Velocity);
        //loader
        rcLoadV = mouse_l_sensitivity*rc.mouse.l;
        current = Loader_M2006_PID(rcLoadV - Loader_Velocity);
        //shooter
        if(rcLoadV){shootON=1;}
        //tx
        Gimbal_CAN_Tx(voltage ,current , shootON*(-6000) , shootON*(+6000) );
    }
}

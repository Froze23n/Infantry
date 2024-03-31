#include "gimbal.h"

#include "motors.h"
#include "imu.h"

#include "rc.h"
#include "pid.h"
#include "referee.h"

#define PI (3.1415927F)

float Calc_Mouse_Pitch(int16_t y);

const float pitch_negative_LIM = -450*PI/4096; //仰角
const float pitch_positive_LIM = 500*PI/4096; //俯角
const float rc_y_sensitivity = 1/200.0f;
const float mouse_y_sensitivity = 1/11451.40f;

float mouse_load_speed = 330;
float rc_load_speed = 0.5;

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
        voltage = Pitch6020_PID(rcAngle-imu.Pitch_Angle , imu.Pitch_Velocity);

        //计算拨弹盘
        rcLoadV = ((float)rc.wheel - 1024.0f)*rc_load_speed;  if(rcLoadV<-100){rcLoadV=-100;}
        current = Loader_M2006_PID(rcLoadV - Loader_Velocity);
        //是否开启摩擦轮？
        if(rcLoadV){shootON=1;}
        //tx
        Gimbal_CAN_Tx(voltage ,current , -8000*shootON, 8000*shootON );
        //Gimbal_CAN_Tx(voltage ,current , Shooter_PID(80-Shooter_Velocity[0],0) , Shooter_PID(-80-Shooter_Velocity[1],1) );
    }
    else if(SW_DOWN == rc.sw1) //键鼠控制
    {
        //pitch
        rcAngle += mouse_y_sensitivity*Calc_Mouse_Pitch(rc.mouse.y);
        if(rcAngle < pitch_negative_LIM){rcAngle = pitch_negative_LIM;}
        if(rcAngle > pitch_positive_LIM){rcAngle = pitch_positive_LIM;}
        voltage = Pitch6020_PID(rcAngle-imu.Pitch_Angle , imu.Pitch_Velocity);
        //loader
        rcLoadV = mouse_load_speed*rc.mouse.l;
        current = Loader_M2006_PID(rcLoadV - Loader_Velocity);
        //shooter
        if(rcLoadV){shootON=1;}
        //tx
        Gimbal_CAN_Tx(voltage ,current , shootON*(-8000) , shootON*(+8000) );
    }
}

void Loader_Speed_Control(void) //申币裁判系统
{
    //uint32_t Head_Limit = Manage_Heat_Limit(refe.shoot_heat_limit);
    //uint32_t Cooling_Value = Manage_Cooling_Value(refe.shoot_cooling_value);
    //mouse_load_speed = (Head_Limit*Cooling_Value)/5; if(){mouse_load_speed = 400;}
    //rc_load_speed = (Head_Limit*Cooling_Value)/5000; if(){rc_load_speed = 0.5;}
}

float Calc_Mouse_Pitch(int16_t y)
{
    static float old = 0;
    old = y;
    return (y+old)/2;
}

#include "gimbal.h"

#include "motors.h"
#include "imu.h"
#include "rc.h"
#include "pid.h"

#define PI (3.1415927F)
int16_t shooter_stop(int8_t ID, float Velocity);

const float pitch_negative_LIM = -600*PI/4096;
const float pitch_positive_LIM = 600*PI/4096;
const float rc_y_sensitivity = 1/900.0f;
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
        int16_t stopCurrent0 = shooter_stop(0,Shooter_Velocity[0]);
        int16_t stopCurrent1 = shooter_stop(1,Shooter_Velocity[1]);
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
        Gimbal_CAN_Tx(voltage*0 ,current , shootON*(-6000) , shootON*(+6000) );
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

int16_t shooter_stop(int8_t ID, float Velocity)
{
    static float iError[2] = {0,0};
    float pError;
    if(ID!=0 || ID!=1){return 0;}  //防呆保护
    //计算pError和iError
    pError = - Velocity;
    iError[ID] += pError;
    //积分限幅
    if(iError[ID]>= 35000){iError[ID] = 35000;}
    if(iError[ID]<=-35000){iError[ID] =-35000;}
    //计算电流
    float output = pError * 45 + iError[ID] * 10 ;
    //限制理论输出+-16384对应+-20A
    if(output>+16300.0f){output=+16300.0f;}
    if(output<-16300.0f){output=-16300.0f;}
    //返回结果
    return (int16_t)output;
}
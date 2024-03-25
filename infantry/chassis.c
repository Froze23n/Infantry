#include "chassis.h"

#include "rc.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

#include <math.h>
#define PI (3.1415927F)

float yHandler(float w, float s);
float xHandler(float d, float a);

static float Rotate_Z=0;
//遥控数据
extern RC_Type rc;
//常量：减速比等等
const float Reduction_Ratio = 19.0f;
const float rx_sensitivity = -PI/1000.0f;
const float mouse_x_sensitivity = -PI/8000.0f;
//等级数据：未来将会作出详细规划
float chassis_speed_level = Reduction_Ratio*10;
float chassis_rotate_level = Reduction_Ratio*5;
/*手册
/1   2\
   ^   
\4   3/
*/
void Chassis_Yaw6020_CMD(void)
{
    static float RC_YAW = 0;
    int16_t voltage;
    float angleDiff;
    if(SW_UP==rc.sw1) //关闭整车
    {
        Chassis_GM6020_Tx(0);
        return;//safer
    } 
    else if(SW_MID==rc.sw1) //遥控器控制
    {
        if(RC_YAW-imu.Yaw_Angle>+PI){RC_YAW-=2*PI;}
        if(RC_YAW-imu.Yaw_Angle<-PI){RC_YAW+=2*PI;}
        angleDiff = RC_YAW - imu.Yaw_Angle;
        if(-PI/2<angleDiff && angleDiff<PI/2)
        {
            RC_YAW += rx_sensitivity * rc.RX;
        }
    }
    else if(SW_DOWN==rc.sw1) //键鼠控制
    {
        angleDiff = RC_YAW - imu.Yaw_Angle;
        if(-PI/2<angleDiff && angleDiff<PI/2)
        {
            RC_YAW += mouse_x_sensitivity * rc.mouse.x;
        }
    }
    voltage = Yaw6020_PID(RC_YAW, imu.Yaw_Angle, imu.Yaw_Velocity, Rotate_Z);//这个是经验值
    Chassis_GM6020_Tx(voltage);
}

void Chassis_M3508_CMD(void)
{
    float x,y,z;
    float velocity[4];
    int16_t current[4];
    if(SW_UP==rc.sw1) //关闭
    {
        current[0]=0; current[1]=0; current[2]=0; current[3]=0;
        Chassis_M3508_Tx(current);
        return;//safer
    }
    else if(SW_MID==rc.sw1) //遥控器控制
    {
        if(SW_UP == rc.sw2)
        {
            x=0; y=0; z=0; //stop
            Rotate_Z = 0; //0, of course
        }
        else if(SW_MID == rc.sw2)//底盘跟随模式
        {
            x = rc.LX;
            y = rc.LY;
            z = Chas_Calc_Z(GIM_CHAS_Angle);
            Rotate_Z = z*3; //magic
        }
        else if(SW_DOWN == rc.sw2)//小陀螺模式
        {
            x = rc.LX;
            y = rc.LY;
            float r = sqrtf(x*x + y*y);// |vector|
            float rcA = atan2f(y,x);// <vector>
            float chasA = rcA - GIM_CHAS_Angle;
            x = r*cosf(chasA);
            y = r*sinf(chasA);
            z = -2.114514;
            Rotate_Z = z/2.5; //magic
        }
    }
    else if(SW_DOWN == rc.sw1) //键鼠控制
    {
        static int follow = 1;
        if(rc.kb.bit.C){
            follow = 1;
        }else if(rc.kb.bit.X){
            follow = 0;
        }
        if(follow) //跟随
        {
            x = xHandler(rc.kb.bit.D, rc.kb.bit.A);
            y = yHandler(rc.kb.bit.W, rc.kb.bit.S);
            z = Chas_Calc_Z(GIM_CHAS_Angle);
            Rotate_Z = z*3; //again, magic
        }
        else //小陀螺
        {
            x = xHandler(rc.kb.bit.D, rc.kb.bit.A);
            y = yHandler(rc.kb.bit.W, rc.kb.bit.S);
            float r = sqrtf(x*x + y*y);// |vector|
            float rcA = atan2f(y,x);// <vector>
            float chasA = rcA - GIM_CHAS_Angle;
            x = r*cosf(chasA);
            y = r*sinf(chasA);
            z = -2.114514;
            Rotate_Z = z/2.5; //M A G I C
        }
    }
    //后续工作
    z *= chassis_rotate_level;
    x *= chassis_speed_level;
    y *= chassis_speed_level;
    //速度分配
    velocity[0] = z + x + y;//CAN_ID:0x201
    velocity[1] = z + x - y;//CAN_ID:0x202
    velocity[2] = z - x - y;//CAN_ID:0x203
    velocity[3] = z - x + y;//CAN_ID:0x204
    //电流pid计算
    current[0] = Chassis_M3508_PID(0,velocity[0],Chassis_M3508_Velocity[0]);
    current[1] = Chassis_M3508_PID(1,velocity[1],Chassis_M3508_Velocity[1]);
    current[2] = Chassis_M3508_PID(2,velocity[2],Chassis_M3508_Velocity[2]);
    current[3] = Chassis_M3508_PID(3,velocity[3],Chassis_M3508_Velocity[3]);
    //数据发送
    Chassis_M3508_Tx(current);
}


/*
    helper functions
*/
float yHandler(float w, float s)
{
    static float y = 0;
    float newY = w-s;
    if(0==newY)
    {
        if(y>0){y-=0.005;}
        if(y<0){y+=0.005;}
        if(-0.005<y && y<0.005){y=0;}
    }
    else
    {
        y += newY/500.0f;
    }
    if(y> 1){y= 1;}
    if(y<-1){y=-1;}
    return y;
}

float xHandler(float d, float a)
{
    static float x = 0;
    float newX = d-a;
    if(0==newX)
    {
        if(x>0){x-=0.005;}
        if(x<0){x+=0.005;}
        if(-0.005<x && x<0.005){x=0;}
    }
    else
    {
        x += newX/500.0f;
    }
    if(x> 1){x= 1;}
    if(x<-1){x=-1;}
    return x;
}

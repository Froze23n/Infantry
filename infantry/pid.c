#include "pid.h"
#define PI (3.1415927F)

//'\0'表示不使用
PID_T chasM = {45.0f , 0.2f , '\0'}; //底盘电机
PID_T chasZ = {8.0f, '\0' ,40.114514f}; //底盘旋转

PID_T yawA = {25.114514f , '\0' , 30}; //yaw轴角度
PID_T yawV = {6000 , 1000 , '\0'}; //yaw轴速度

PID_T pitchA = {40.0f , 0 , 500.0f}; //pitch电机
PID_T pitchV = {3000 , 500 , '\0'}; //pitch电机

PID_T loadV = {25.0f , 0.5f , '\0'}; //拨弹盘
PID_T shootV = {45, 0.2, '\0'};

#define M3508_I_LIMIT (35000.0f)
int16_t Chassis_M3508_PID(int8_t ID, float expV, float truV)
{
    static float iError[4] = {0,0,0,0};
    float pError;
    if(ID<0 || 3<ID){return 0;}  //防呆保护
    //计算pError和iError
    pError = expV - truV;
    iError[ID] += pError;
    //积分限幅
    if(iError[ID]>= M3508_I_LIMIT){iError[ID] = M3508_I_LIMIT;}
    if(iError[ID]<=-M3508_I_LIMIT){iError[ID] =-M3508_I_LIMIT;}
    //计算电流
    float output = pError * chasM.kp + iError[ID] * chasM.ki ;
    //限制理论输出+-16384对应+-20A
    if(output>+16300.0f){output=+16300.0f;}
    if(output<-16300.0f){output=-16300.0f;}
    //返回结果
    return (int16_t)output;
}

/*
输入：( -pi , pi ]
输出：车子旋转的角速度rad/s,限幅[-PI,PI]
*/
float Chas_Calc_Z(float relative_angle)
{
    static float oldError = 0;
    float pError,dError;
    float outLevel;
    //calc
    pError = relative_angle;
    dError = pError-oldError;//起步时加速，结束时减速
    oldError = pError;
    //算加和
    outLevel = pError*chasZ.kp + dError*chasZ.kd;
    if(outLevel> 4){outLevel= 4;}
    if(outLevel<-4){outLevel=-4;}
    return outLevel;
}

/*
yaw轴电机-云台yaw角、角速度的双环PID
期望角度 -> 期望速度 > 电压
feedV即前馈Rotate_Z
*/
int16_t Yaw6020_PID(float expA, float truA, float truV, float feedV)
{
    //角度到速度：
    static float oldError = 0;
    float pError,dError;
    float expV;
    float Voltage;
    //pid计算与迭代
    pError = expA - truA;
    dError = pError - oldError;
    oldError = pError;
    //结合前馈，得到预期速度
    expV = pError * yawA.kp + dError * yawA.kd;
    //if(expV > 10.0f*PI){expV = 10.0f*PI;}
    //if(expV <-10.0f*PI){expV =-10.0f*PI;} //300 rpm == 10pi rad/s
    //速度到电压
    Voltage = yaw6020_velocity_to_voltage(expV+feedV,truV);
    return (int16_t)Voltage;
}
#define YAW_V2V_I_LIMIT (0.718f)
float yaw6020_velocity_to_voltage(float expV,float truV)
{
    static float iError = 0;
    float pError;
    float output;
    pError = expV - truV;
    iError += pError;
    if(iError > YAW_V2V_I_LIMIT){iError = YAW_V2V_I_LIMIT;}
    if(iError <-YAW_V2V_I_LIMIT){iError =-YAW_V2V_I_LIMIT;}
    //计算加和
    output = pError * yawV.kp + iError * yawV.ki;
    //限制理论电压上限+-2600mV
    if(output > 26000.0f){output = 26000.0f;}
    if(output <-26000.0f){output =-26000.0f;}
    return output;
}

/*--------------------------------------------PITCH--------------------------------------------*/
#define PITCH_ANGLE_ILIMIT (0.25)
int16_t Pitch6020_PID(float pError, float truV)
{
    //角度到速度：
    static float oldError = 0;
    static float iError = 0;
    float dError;
    float expV,Voltage;
    //pid计算与迭代
    iError += pError;
    if(iError>= PITCH_ANGLE_ILIMIT){iError= PITCH_ANGLE_ILIMIT;}
    if(iError<=-PITCH_ANGLE_ILIMIT){iError=-PITCH_ANGLE_ILIMIT;}
    dError = pError - oldError;
    oldError = pError;
    //结合前馈，得到预期速度
    expV = pError * pitchA.kp +  (iError*pitchA.ki)  + dError * pitchA.kd;
    if(expV > 10.0f*PI){expV = 10.0f*PI;}
    if(expV <-10.0f*PI){expV =-10.0f*PI;}
    //速度到电压
    Voltage = pitch6020_velocity_to_voltage(expV-truV);
    return (int16_t)Voltage;
}
#define PITCH_V2V_I_LIMIT (0.314)
float pitch6020_velocity_to_voltage(float vError)
{
    if(vError>0){vError*=3;} //该死的重力补偿
    static float iError = 0;
    float output;
    iError += vError;
    if(iError > PITCH_V2V_I_LIMIT){iError = PITCH_V2V_I_LIMIT;}
    if(iError <-PITCH_V2V_I_LIMIT){iError =-PITCH_V2V_I_LIMIT;}
    //计算加和
    output = vError * pitchV.kp + iError * pitchV.ki;
    //限制理论电压上限+-26000mV
    if(output > 26000.0f){output = 26000.0f;}
    if(output <-26000.0f){output =-26000.0f;}
    return output;
}

#define LOADER_IERROR_LIMIT (300.0f)
int16_t Loader_M2006_PID(float pError)
{
    static float iError = 0;
    float output;
    iError += pError;
    if(iError >  LOADER_IERROR_LIMIT){ iError =  LOADER_IERROR_LIMIT;}
    if(iError < -LOADER_IERROR_LIMIT){ iError = -LOADER_IERROR_LIMIT;}
    output = pError*loadV.kp + iError*loadV.ki;
    if(output > 10000){output = 10000;}
    if(output <-10000){output =-10000;} /* -10000mA  电流  +10000mA*/
    return (int16_t)output;
}

#define SHOOTER_IERROR_LIMIT (300.0f)
int16_t Shooter_PID(float pError, int8_t ID)
{
    if(ID!=0 || ID!=1){return 0;}
    static float iError[2] = {0,0};
    float output;
    iError[ID] += pError;
    if(iError[ID] >  SHOOTER_IERROR_LIMIT){ iError[ID] =  SHOOTER_IERROR_LIMIT;}
    if(iError[ID] < -SHOOTER_IERROR_LIMIT){ iError[ID] = -SHOOTER_IERROR_LIMIT;}
    output = pError*shootV.kp + iError[ID]*shootV.ki;
    if(output > 16300){output = 16300;}
    if(output <-16300){output =-16300;} /* -20000mV  电流  +20000mV*/
    return (int16_t)output;
}
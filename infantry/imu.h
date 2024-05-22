#ifndef _IMU_H_
#define _IMU_H_

/* ------------------------------ Include ------------------------------ */
#include "stdint.h"
/* ------------------------------ Macro Definition ------------------------------ */
// 陀螺仪片选信号引脚
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
// 加速度计片选信号引脚
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
// GYRO_SENSITIVITY_x，则角速度量程为±x deg/s，从陀螺仪寄存器读到的数据需要除以对应灵敏度（即32768/x）才得到单位为deg/s的数据
#define GYRO_SENSITIVITY_1000 32.768f
// ACC_SENSITIVITY_xG，则角速度量程为±x g，从加速度计寄存器读到的数据需要除以对应灵敏度（即32768/x）才得到单位为g的数据
#define ACCEL_SENSITIVITY_3 10922.66667f
//延时宏定义
#define BMI088_COM_WAIT_SENSOR_TIME 150 // 通信等待需延时150us
#define BMI088_LONG_DELAY_TIME 100 // 有些设置后需要较长延时，设为100ms
#define BMI088_SPI_TIME_THRES 55 //transmit reveive 等待延时

/* ------------------------------ Type Definition ------------------------------ */
//数据包
typedef struct
{
    float Yaw_Velocity; //全局 云台角速度 rad/s
    float Yaw_Angle;
    float Pitch_Velocity;
    float Pitch_Angle;
} IMU_Type;

//原始数据
typedef struct {
    // 加速度数据，单位为g
    float ax;
    float ay;
    float az;

    // 温度数据，单位为℃
    //float temperature; //暂时用不到

    // 角速度数据，单位为degree/s(x) --> rad/s [by yaofang 2024/3/21] 
    float gx;
    float gy;
    float gz;

    // 磁力数据，单位为uT//float mx;//float my;//float mz;
} imu_raw_data_t;

// 定义BMI088传感器的各种错误码
typedef enum {
	BMI088_NO_ERROR = 0x00,

	BMI088_GYRO_RANGE_ERROR = 0x01, //
	BMI088_GYRO_BANDWIDTH_ERROR = 0x02, //
	BMI088_GYRO_LPM1_ERROR = 0x03, //

	BMI088_ACC_RANGE_ERROR = 0x04, //
	BMI088_ACC_CONF_ERROR = 0x05, //
	BMI088_ACC_PWR_CTRL_ERROR = 0x06, //
	BMI088_ACC_PWR_CONF_ERROR = 0x07, //

	BMI088_SELF_TEST_ACCEL_ERROR = 0x80, //
	BMI088_SELF_TEST_GYRO_ERROR = 0x40, //
	BMI088_NO_SENSOR = 0xFF, //
} bmi_error_type;

/* ------------------------------ Extern Global Variable ------------------------------ */
extern IMU_Type imu;
/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */
uint8_t BMI088_Init(void);
void Get_IMU_Data(void);

/* ------------------------------ Macro Definition ------------------------------ */
//下面全是关于寄存器地址以及其中的相关数值的宏定义
/* 宏定义了BMI088中的寄存器地址以及可以写入的值 */
// the register is  " Who am I "
#define BMI088_ACC_CHIP_ID 0x00  // 应该是这个id寄存器的地址，下面是值 
#define BMI088_ACC_CHIP_ID_VALUE 0x1E //

#define BMI088_ACCEL_XOUT_L 0x12 //
//#define BMI088_ACCEL_XOUT_M 0x13
//#define BMI088_ACCEL_YOUT_L 0x14
//#define BMI088_ACCEL_YOUT_M 0x15
//#define BMI088_ACCEL_ZOUT_L 0x16
//#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_ACC_CONF 0x40 //
#define BMI088_ACC_CONF_MUST_Set 0x80 //

#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS) //
//#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

#define BMI088_ACC_RANGE 0x41 //

#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS) //
//#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
//#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
//#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

#define BMI088_ACC_PWR_CONF 0x7C //
//#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00 //

#define BMI088_ACC_PWR_CTRL 0x7D //
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04 //

#define BMI088_ACC_SOFTRESET 0x7E //
#define BMI088_ACC_SOFTRESET_VALUE 0xB6 //

#define BMI088_GYRO_CHIP_ID 0x00 //
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F


#define BMI088_GYRO_X_L 0x02 //
//#define BMI088_GYRO_X_H 0x03
//#define BMI088_GYRO_Y_L 0x04
//#define BMI088_GYRO_Y_H 0x05
//#define BMI088_GYRO_Z_L 0x06
//#define BMI088_GYRO_Z_H 0x07

#define BMI088_GYRO_RANGE 0x0F //
#define BMI088_GYRO_RANGE_SHFITS 0x0
//#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS) //
//#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
//#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
//#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)


#define BMI088_GYRO_BANDWIDTH 0x10 //
// the first num means Output data  rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80 //
//#define BMI088_GYRO_2000_532_HZ 0x00
//#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02 //

#define BMI088_GYRO_LPM1 0x11 //
#define BMI088_GYRO_NORMAL_MODE 0x00 //

#define BMI088_GYRO_SOFTRESET 0x14 //
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6 //

#endif

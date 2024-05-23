#include "imu.h"
#include "spi.h"
#include "tim.h"
#include <math.h>
#define PI (3.1415927F)
/*经验误差*/
const float YAW_GYRO_COMPENSATION = -0.29;
const float X_ACCEL_COMPENSATION = +0.01124;
const float Y_ACCEL_COMPENSATION = -0.00063;
const float _pi_over_180_ = PI/180.0f;

/* ------------------------------  Variables  ------------------------------ */
static imu_raw_data_t imu_raw_data = {0};  // IMU原始数据结构体
IMU_Type imu = {0,0};

/* ------------------------------ Fuction Declaration ------------------------------ */
void IMU_Get_Data(void);
float invSqrt(float x);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
//陀螺仪
void BMI088_Write_Gyro_Single_Reg(uint8_t const reg, uint8_t const data);
uint8_t BMI088_Read_Gyro_Single_Reg(uint8_t const reg);
void BMI088_Read_Gyro_Multi_Reg(uint8_t *bmi_rx_data);
uint8_t BMI088_Gyro_Init(void);
//加速度计
void BMI088_Write_Acc_Single_Reg(uint8_t const reg, uint8_t const data);
uint8_t BMI088_Read_Acc_Single_Reg(uint8_t const reg);
void BMI088_Read_Acc_Multi_Reg(uint8_t *bmi_rx_data);
uint8_t BMI088_Acc_Init(void);
//微秒延时
void bmi_delay_us(uint16_t us);

uint8_t IMU_Init(void)
{
	uint8_t state = BMI088_NO_ERROR;
	state |= BMI088_Gyro_Init();
	state |= BMI088_Acc_Init();
	if(state==BMI088_NO_ERROR){
    	HAL_TIM_Base_Start_IT(&htim5);//1ms一次中断用于数据处理
	}else{
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
	}
	return state;
}


void IMU_Get_Data(void)
{
	uint8_t gyro_buff[6];
	uint8_t acc_buff[6];
	float tmp;
	/* 陀螺仪数据读取 */
	BMI088_Read_Gyro_Multi_Reg(gyro_buff);
	tmp = (int16_t)((gyro_buff[1] << 8) | gyro_buff[0]);
	imu_raw_data.gx = (tmp / GYRO_SENSITIVITY_1000) * _pi_over_180_ ; //统一采用弧度制
	tmp = (int16_t)((gyro_buff[3] << 8) | gyro_buff[2]);
	imu_raw_data.gy = (tmp / GYRO_SENSITIVITY_1000) * _pi_over_180_ ;
	tmp = (int16_t)((gyro_buff[5] << 8) | gyro_buff[4]);
	imu_raw_data.gz = (tmp / GYRO_SENSITIVITY_1000 + YAW_GYRO_COMPENSATION) * _pi_over_180_ ; //误差补偿
	/* 加速度计数据读取 */
	BMI088_Read_Acc_Multi_Reg(acc_buff);
	tmp = (int16_t)((acc_buff[1] << 8) | acc_buff[0]);
	imu_raw_data.ax = (tmp / ACCEL_SENSITIVITY_3)   +   X_ACCEL_COMPENSATION; //误差补偿
	tmp = (int16_t)((acc_buff[3] << 8) | acc_buff[2]);
	imu_raw_data.ay = (tmp / ACCEL_SENSITIVITY_3)   +   Y_ACCEL_COMPENSATION; //误差补偿
	tmp = (int16_t)((acc_buff[5] << 8) | acc_buff[4]);
	imu_raw_data.az = (tmp / ACCEL_SENSITIVITY_3);
}


void BMI088_Write_Gyro_Single_Reg(uint8_t const reg, uint8_t const data)
{
	uint8_t bmi_rx_byte, bmi_tx_byte;
	//开始
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输 NSS_Low
	//数据交换
	bmi_tx_byte = reg & 0x7f;//首位是0，表示此操作为写入，reg后七位即为所写地址
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	bmi_tx_byte = data;
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	//结束
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输 NSS_High
}

uint8_t BMI088_Read_Gyro_Single_Reg(uint8_t const reg)
{
	uint8_t bmi_rx_byte, bmi_tx_byte;
	//开始
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输 NSS_Low
	//数据交换
	bmi_tx_byte = reg | 0x80;  // 第一位为1表示读操作
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	//结束
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输 NSS_High
	return bmi_rx_byte;
}

void BMI088_Read_Gyro_Multi_Reg(uint8_t *bmi_rx_data)
{
	uint8_t bmi_rx_byte, bmi_tx_byte, len = 6;
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输 NSS_Low
	bmi_tx_byte = BMI088_GYRO_X_L | 0x80;  // 第一位为1表示读操作
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	while (len != 0) {
		HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_data[6 - len], 1, BMI088_SPI_TIME_THRES);
		len--;
	}
	HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输 NSS_High
}

uint8_t BMI088_Gyro_Init(void)
{
	uint8_t BMI088_Gyro_Init_Config[3][3] = {
		{BMI088_GYRO_RANGE, BMI088_GYRO_1000, BMI088_GYRO_RANGE_ERROR},
		{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
		{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR}
	};
	static uint8_t read_value;
	BMI088_Write_Gyro_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
	HAL_Delay(BMI088_LONG_DELAY_TIME);
	// check commiunication is normal after reset
	read_value = BMI088_Read_Gyro_Single_Reg(BMI088_GYRO_CHIP_ID);
	bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	// check the "who am I"
	if (read_value != BMI088_GYRO_CHIP_ID_VALUE) {
		return BMI088_NO_SENSOR;
	}
	// 根据BMI088_Gyro_Init_Config的配置，写入相应寄存器
	for (uint8_t i = 0; i < 3; i++) {
		BMI088_Write_Gyro_Single_Reg(BMI088_Gyro_Init_Config[i][0], BMI088_Gyro_Init_Config[i][1]);
		bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		read_value = BMI088_Read_Gyro_Single_Reg(BMI088_Gyro_Init_Config[i][0]);
		bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		if (read_value != BMI088_Gyro_Init_Config[i][1]) {
		return BMI088_Gyro_Init_Config[i][2];  // 若有错误立即返回，不会进行后续配置
		}
	}
	return BMI088_NO_ERROR;
}

void BMI088_Write_Acc_Single_Reg(uint8_t const reg, uint8_t const data)
{
	// TODO(Hello World): 
	uint8_t bmi_rx_byte, bmi_tx_byte;
	//开始
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输
	//数据交换
	bmi_tx_byte = reg & 0x7f;
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	bmi_tx_byte = data;
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	//结束
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输
}


uint8_t BMI088_Read_Acc_Single_Reg(uint8_t const reg)
{
	uint8_t bmi_rx_byte, bmi_tx_byte;
	//开始
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输
	//数据交换
	bmi_tx_byte = reg | 0x80;  // 第一位为1表示读操作
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);//bit0读，bit1-7确定读取地址
	bmi_tx_byte = 0x55; //芝士magic number, 反正官方例程是这么写的
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);//bit8-15无效值
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);//bit16-23以及之后为有效值
	//结束
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输
	return bmi_rx_byte;
}


void BMI088_Read_Acc_Multi_Reg(uint8_t *bmi_rx_data)
{
	uint8_t bmi_rx_byte, bmi_tx_byte, len = 6;
	//开始
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);  // 拉低片选信号，开始传输
	//数据交换
	bmi_tx_byte = BMI088_ACCEL_XOUT_L | 0x80;  // 第一位为1表示读操作，寄存器地址为加速度计0x12，x轴方向低八位
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);
	HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_byte, 1, BMI088_SPI_TIME_THRES);//额外的读取操作，筛选掉无效读数
	while (len != 0) {
		HAL_SPI_TransmitReceive(&hspi1, &bmi_tx_byte, &bmi_rx_data[6 - len], 1, BMI088_SPI_TIME_THRES);
		len--;
	}
	//结束
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);  // 拉高片选信号，结束传输
}


uint8_t BMI088_Acc_Init(void)
{
	/* 加速度计配置 */
	// 配置一些寄存器，并定义对应的错误
	uint8_t BMI088_Acc_Init_Config[4][3] = {
		{BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
		{BMI088_ACC_CONF, BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
		{BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
		{BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}
	};
	static uint8_t read_value;  // 写入上述寄存器后再读取的值，用来检查是否正确配置
	//软件复位，复位后必须开启，byd不然用不了
	BMI088_Write_Acc_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
	HAL_Delay(BMI088_LONG_DELAY_TIME);
	BMI088_Write_Acc_Single_Reg(BMI088_ACC_PWR_CONF,BMI088_ACC_PWR_ACTIVE_MODE);
	bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	// check commiunication is normal after reset
	read_value = BMI088_Read_Acc_Single_Reg(BMI088_ACC_CHIP_ID);
	bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	// check the "who am I"
	if (read_value != BMI088_ACC_CHIP_ID_VALUE) {
		return BMI088_NO_SENSOR;//**?
	}
	// 写入相应寄存器
	for (uint8_t i = 0; i < 4; i++) {
		BMI088_Write_Acc_Single_Reg(BMI088_Acc_Init_Config[i][0], BMI088_Acc_Init_Config[i][1]);
		bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		read_value = BMI088_Read_Acc_Single_Reg(BMI088_Acc_Init_Config[i][0]);
		bmi_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		if (read_value != BMI088_Acc_Init_Config[i][1]) {
		return BMI088_Acc_Init_Config[i][2];  // 若有错误立即返回，不会进行后续配置
		}
	}
	return BMI088_NO_ERROR;
}

void bmi_delay_us(uint16_t us)
{
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;

	reload = SysTick->LOAD;
	ticks = us * 168;
	told = SysTick->VAL;

	while (1) {
		tnow = SysTick->VAL;
		if (tnow != told) {
		if (tnow < told) {
			tcnt += told - tnow;
		} else {
			tcnt += reload - tnow + told;
		}
		told = tnow;
		if (tcnt >= ticks) {
			break;
		}
		}
	}
}

/* ------------------------------ 姿态解算 ------------------------------ */
//变量
const float sampleFreq = 1000.0f;       // sample frequency in Hz
const float twoKp = (2.0f * 0.3f);  // 2 * proportional gain (Kp)
const float twoKi = (2.0f * 0.0f);  // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // integral error terms scaled by Ki

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
		integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
		integralFBy += twoKi * halfey * (1.0f / sampleFreq);
		integralFBz += twoKi * halfez * (1.0f / sampleFreq);
		gx += integralFBx;  // apply integral feedback
		gy += integralFBy;
		gz += integralFBz;
		} else {
		integralFBx = 0.0f;  // prevent integral windup
		integralFBy = 0.0f;
		integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void IMU_Task(void)
{
	static float q[4]={1.0f , 0.0f , 0.0f , 0.0f};  // 四元数
	static float Old_Yaw = 0;
	static int32_t Circle = 0;
	float newYaw;
	IMU_Get_Data();
	/* 由于板子是躺着放的，所以必须对部分数据取反 */
	MahonyAHRSupdateIMU(q, 
	imu_raw_data.gx,
	-imu_raw_data.gy,
	-imu_raw_data.gz,
	imu_raw_data.ax,
	-imu_raw_data.ay,
	-imu_raw_data.az);
	newYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
	imu.Pitch_Angle = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
	if(newYaw-Old_Yaw<-PI){Circle+=1;}
	if(newYaw-Old_Yaw> PI){Circle-=1;}
	Old_Yaw = newYaw;//更新旧角度
	//累计的弧度广播到全局
	imu.Yaw_Angle = 2.0f*PI*Circle + newYaw;
	//云台角速度广播到全局，单位：rad/s
	imu.Pitch_Velocity = -imu_raw_data.gy;
	imu.Yaw_Velocity = -imu_raw_data.gz; //板子躺着放，取反。
}

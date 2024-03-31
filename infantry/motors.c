#include "motors.h"
#include "can.h"
#include "tim.h"
#define PI (3.1415927F)

/**
命令     类型    编号    反馈	
0x200	  M3508   1234  0x201-204
0x200	  m2006   1234  0x201-204

0x1FF   M3508   5678  0x205-208
0x1FF   m2006   5678  0x205-208

0x1FF   GM6020  1234  0x205-208
0x2FF   GM6020  567   0x209,20A,20B
*/

/* ------------------------------ 常量 ------------------------------ */
//can1
const uint32_t GIMBAL_CMD_ID = 0x1ff; //云台GM6020(id=1) M2006(id=6) M3508(id=7,8) #返回0x205-208
//can2
const uint32_t YAW6020_CMD_ID = 0x1ff; //GM6020(id=1) #返回0x205
const uint32_t CHAS_CMD_ID = 0x200; //4个m3508(1~4) #返回0x201-204
const uint32_t CAP_CMD_ID = 0x210; //#返回0x211
//进制转化
const float _pi_over_4096_ = PI/4096.0f;
const float _rads_per_rpm_ = 2*PI/60.0f;

/* ------------------------------ 全局变量 ------------------------------ */
//can1数据：pitch电机&拨弹盘
struct MotorData_T Pitch6020 = {0,0};
float Loader_Velocity=0;
float Shooter_Velocity[2] = {0,0};
//can2数据：yaw电机&麦轮
float Chassis_M3508_Velocity[4] = {0,0,0,0};
float GIM_CHAS_Angle = 0.0f;//(-pi,pi]
struct CapData_T Capacitor = {0,0,0,0,0};

/* ------------------------------ 初始化（配置过滤器）------------------------------ */
void Enable_Motors(void)
{
	CAN_FilterTypeDef CAN_Filter;
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_Filter.SlaveStartFilterBank = 0;
	CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
	CAN_Filter.FilterIdHigh = 0x0000;
	CAN_Filter.FilterIdLow = 0x0000;
	CAN_Filter.FilterMaskIdHigh= 0x0000;
	CAN_Filter.FilterMaskIdLow = 0x0000;
	if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
		Error_Handler();
	}
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	CAN_Filter.FilterBank = 14;
	CAN_Filter.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan2,&CAN_Filter)!= HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
		Error_Handler();
	}
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // CAN1 -> FIFO0
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // CAN2 -> FIFO1
	HAL_TIM_Base_Start_IT(&htim2);//1ms 云台
	HAL_TIM_Base_Start_IT(&htim3);//1ms 底盘
	HAL_TIM_Base_Start_IT(&htim7);//0.2s 一次 超电控制
}

/* ------------------------------------------ 发送函数：云台 ------------------------------------------ */
void Gimbal_CAN_Tx(int16_t Pitch_Voltage, int16_t Loader_Current, int16_t Shooter_Current_0, int16_t Shooter_Current_1)
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(Pitch_Voltage>>8);
	TxData[1] = (uint8_t)Pitch_Voltage;
	TxData[2] = (uint8_t)(Loader_Current>>8);
	TxData[3] = (uint8_t)Loader_Current;
	TxData[4] = (uint8_t)(Shooter_Current_0>>8);
	TxData[5] = (uint8_t)Shooter_Current_0;
	TxData[6] = (uint8_t)(Shooter_Current_1>>8);
	TxData[7] = (uint8_t)Shooter_Current_1;
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 8,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = GIMBAL_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX2;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}


/* ------------------------------------------ 发送函数：底盘 ------------------------------------------ */
void Chassis_M3508_Tx(int16_t Current[4])
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(Current[0]>>8);
	TxData[1] = (uint8_t)Current[0];
	TxData[2] = (uint8_t)(Current[1]>>8);
	TxData[3] = (uint8_t)Current[1];
	TxData[4] = (uint8_t)(Current[2]>>8);
	TxData[5] = (uint8_t)Current[2];
	TxData[6] = (uint8_t)(Current[3]>>8);
	TxData[7] = (uint8_t)Current[3];
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 8,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = CHAS_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX0;
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}

void Chassis_GM6020_Tx(int16_t Yaw_Voltage)
{
	uint8_t TxData[2];
	TxData[0] = (uint8_t)(Yaw_Voltage>>8);
	TxData[1] = (uint8_t)Yaw_Voltage;
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 2,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = YAW6020_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX1;
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}

void Chassis_Capacitor_Tx(uint16_t Power_Set)
{
	Power_Set = Power_Set*100; //设定就是这样,步进0.01
	uint8_t TxData[2];
	TxData[0] = (uint8_t)(Power_Set>>8);
	TxData[1] = (uint8_t)Power_Set;
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 2,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = CAP_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX1;
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}



/* ------------------------------------------ 接收函数 ------------------------------------------ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	//can1--云台--GM6020+M2006
	if (hcan == &hcan1) 
	{
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)  // 获得接收到的数据头和数据
		{
			//云台GM6020(id=1) M2006(id=6) M3508(id=7,8) #返回0x205-208
			if (RxHeader.StdId == 0x205){
				/*云台pitch轴GM6020*/
				//原始数据提炼
				int16_t rawAngle = ( (RxData[0]<<8) | RxData[1] );
				int16_t rawVelocity = ( (RxData[2]<<8) | RxData[3] );
				if(rawAngle>6000){rawAngle -= 8192;}
				//赋值
				Pitch6020.Angle = rawAngle * _pi_over_4096_ ; //机械限位
				//Pitch6020.Velocity = (float)rawVelocity * _rads_per_rpm_ ;//傻逼6020速度飘的一笔 改用imu
			}else if(RxHeader.StdId == 0x206){
				//拨弹盘速度//不需要滤波
				int16_t tempVelocity = ( (RxData[2]<<8) | RxData[3] );
				Loader_Velocity = (float)tempVelocity * _rads_per_rpm_ ;
			}else if(RxHeader.StdId == 0x207){
				int16_t tempVelocity = ( (RxData[2]<<8) | RxData[3] );
				Shooter_Velocity[0] = (float)tempVelocity * _rads_per_rpm_ ;
			}else if(RxHeader.StdId == 0x208){
				int16_t tempVelocity = ( (RxData[2]<<8) | RxData[3] );
				Shooter_Velocity[1] = (float)tempVelocity * _rads_per_rpm_ ;
			}else{
				HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
			}
		}
	}
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan == &hcan2)
	{
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
		{
			//GM6020(id=1) #返回0x205
			//4个m3508(1~4) #返回0x201-204
			//cap#返回0x211
			if(RxHeader.StdId == 0x205){
				int16_t tempAngle = ( (RxData[0]<<8) | RxData[1] );
				//rawyaw = tempAngle;
				tempAngle -= (680); //magic number
				if(tempAngle>4096){tempAngle-=8192;}//-> (4095)~(0)~(-4096)
				GIM_CHAS_Angle = -(float)tempAngle * _pi_over_4096_; //-> (-pi,pi]
			}else if(RxHeader.StdId>0x200 && RxHeader.StdId<0x205){
				uint32_t i = RxHeader.StdId-(uint32_t)0x201;
				int16_t tempVelocity = ( (RxData[2]<<8) | RxData[3] );
				Chassis_M3508_Velocity[i] = (float)tempVelocity * _rads_per_rpm_ ;
			}else if(RxHeader.StdId == 0x211){
				/*
				uint16_t* captr = (uint16_t*)RxData;
				Capacitor.V_In = (float)captr[0]/100.0f;
				Capacitor.Vcap = (float)captr[1]/100.0f;
				Capacitor.I_In = (float)captr[2]/100.0f;
				Capacitor.Pset = (float)captr[3]/100.0f;
				Capacitor.Power_In = Capacitor.I_In*Capacitor.V_In;
				*/
			}else{
				HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
			}
		}
	}
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);  // 再次使能FIFO0接收中断
}

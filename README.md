# 定时器清单
TIM2 云台控制 1000Hz
TIM3 底盘控制 1000Hz
TIM4 CH3 蜂鸣器
TIM5 imu数据更新 1000Hz
TIM7 裁判系统数据更新+超级电容控制 5Hz

# 中断清单
定时器中断 如上
can1：云台4个电机 1000Hz*4
can2: 底盘5个电机 1000Hz*5 + 超级电容控制板回传(频率未知)
遥控器中断：14ms产生一次
裁判系统串口中断：频率不固定。见手册。

# CAN ID清单
/**

命令     类型    编号    反馈	

0x200	M3508   1234  0x201-204

0x200	m2006   1234  0x201-204


0x1FF   M3508   5678  0x205-208

0x1FF   m2006   5678  0x205-208


0x1FF   GM6020  1234  0x205-208

0x2FF   GM6020  567   0x209,20A,20B

*/

# 超电：(溪地创新的控制板)
淘宝搜索<溪地创新>

# 部分知识讲解
1. motor:
   GIM_CHAS_Angle Yaw轴GM6020编码角经过处理后的值
   这个值反应了底盘和云台之间的相对角度，用于底盘跟随模式
2. pid:
    GM6020的编码速度是离散的(不过编码位置应该是准的)，不适合用在pitch的精确控制上。
    目前pitch的实际速度和实际角度改用了imu的数据，精度提升很大，不过也导致了一个小问题：角度限制不好做
    因为限制角度的本该是机械，也就是gm6020的编码角
    或许可以这样：实际速度用imu 实际角度用电机编码器
    按我的理解，编码速度是对编码角的微分，imu角度是对imu速度的积分，这俩玩意儿的精度大抵是有先天不足的
    ......
    pid调参可讲的经验实在太多太杂了 自己多上手试试吧。
3. rc:
    rc_tick这个变量我并未用到，你们可以问问hello world啥作用
4. imu:
    3轴加速度+3轴角速度+计算得出的3轴欧拉角
    诸如YAW_GYRO_COMPENSATION是对陀螺仪/加速度计做的手动补偿，是个经验值
5. referee:
    写的不好，希望你们能搞到hello world的源码 或者去看其他开源
5. chassis & gimbal
    调用motor、pid、rc、imu、referee,封装出任务函数，放到中断处理函数中实现

2024/4/1

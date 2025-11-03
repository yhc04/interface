#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32h7xx.h"
#include "head.h"

//电机初始化
void Motor_Init(void);

//声明外部变量 - 电机数量
extern uint8_t num ;

//电机配置FDCAN
void FDCAN_Init_motor(FDCAN_HandleTypeDef *hfdcan);
	
//获取电机测量数据
void get_moto_measure(MotorHandle_t*motor, uint8_t* rxbuff);

//设置电机电流值
uint8_t motor_current_set(FDCAN_HandleTypeDef*hfdcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);

//位移转电机数据
float displacement_to_motor_data(float displacement_mm); 

//电机数据转位移
float motor_data_to_displacement(float motor_data);

#endif
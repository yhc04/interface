#ifndef __PID_H
#define __PID_H

#include "stm32h7xx_hal.h"
#include "head.h"

// 声明外部变量 - 电机数量
extern uint8_t num;

// 夹爪PID初始化
void ClawPID_Init(CascadePid *pid);

// 单级PID初始化
void PID_Init(PidHandle_t *pidsetouterinner, float p, float i, float d, float maxI, float maxOut);

// 串级PID初始化
void CascadePID_Init(CascadePid *pid);

// 单级PID计算
void PID_Calc(PidHandle_t *pid, float reference, float feedback);

// 串级PID计算
int16_t PID_CascadeCalc(MotorHandle_t *motors, uint8_t num);
#endif
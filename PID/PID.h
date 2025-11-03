#ifndef __PID_H
#define __PID_H

#include "stm32h7xx_hal.h"
#include "head.h"

// 声明外部变量 - 电机数量
extern uint8_t num;
void PID_Calc(PidHandle_t *pid, float reference, float feedback);

/**
 * @brief PID控制器初始化函数
 * @param pidsetouterinner: PID控制器结构体指针
 * @param p: 比例系数
 * @param i: 积分系数
 * @param d: 微分系数
 * @param maxI: 积分限幅值
 * @param maxOut: 输出限幅值
 * @note 初始化PID控制器的各项参数和状态变量
 */
void PID_Init(PidHandle_t *pidsetouterinner, float p, float i, float d, float maxI, float maxOut);

/**
 * @brief 串级PID控制计算函数
 * @param motors: 电机控制结构体数组指针
 * @param num: 电机数量
 * @param refs: 目标位置数组指针
 * @return 操作状态码 (0表示成功)
 * @note 对多个电机执行串级PID控制，外环为位置环，内环为速度环
 */
int16_t PID_CascadeCalc(MotorHandle_t *motors, uint8_t num);

/**
 * @brief 串级PID控制器初始化
 * @param pid: 串级PID控制器结构体指针
 * @note 初始化内外环PID参数，外环用于位置控制，内环用于速度控制
 */
void CascadePID_Init(CascadePid *pid);

#endif
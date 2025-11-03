#include "PID.h"
#include "motor.h"
#include <math.h>
#include "fdcan.h"
#include "usart.h"
#include <stdio.h>

// 声明外部变量
extern MotorHandle_t motor[4];  // 电机控制结构体数组，包含4个电机
extern uint8_t num;             // 电机数量

// PID控制器初始化
void PID_Init(PidHandle_t *pidsetouterinner, float p, float i, float d, float maxI, float maxOut)
{
  pidsetouterinner->kp = p;                    // 设置比例系数
  pidsetouterinner->ki = i;                    // 设置积分系数
  pidsetouterinner->kd = d;                    // 设置微分系数
  pidsetouterinner->maxIntegral = maxI;        // 设置积分限幅值
  pidsetouterinner->maxOutput = maxOut;        // 设置输出限幅值
  pidsetouterinner->lastfeedback = 0;          // 初始化上一次反馈值为0
}
 
// 串级PID控制器初始化
void CascadePID_Init(CascadePid *pid)
{
  // 外环PID初始化 - 位置控制环参数
  PID_Init(&pid->outer, 2000.0f, 0.0f, 300.0f, 300.0f, 3500.0f); 
  // 内环PID初始化 - 速度控制环参数
	PID_Init(&pid->inner, 12.0f, 1.0f, 1.0f, 100.0f, 10000.0f); 
}

/**
 * @brief PID计算函数
 * @param pid: PID控制器结构体指针
 * @param reference: 目标值(设定值)
 * @param feedback: 反馈值(实际值)
 * @note 执行完整的PID计算，包括滤波、误差计算、PID分量计算、限幅处理和死区控制
 */
void PID_Calc(PidHandle_t *pid, float reference, float feedback)
{	
    // 低通滤波处理：当前反馈值占30%，历史反馈值占70%，用于平滑信号
	  pid->feedbackreal = 0.3 * feedback + 0.7 * pid->lastfeedback;
	  pid->lastfeedback = feedback;  // 更新历史反馈值
    
    float dt = 1;  
    
    // 计算当前误差 = 目标值 - 反馈值
    pid->error = reference - pid->feedbackreal; 
    
    // 计算微分项 = (当前误差 - 上一次误差) / 控制周期
    float derivative = (pid->error - pid->lastError) / dt;
    pid->lastError = pid->error;       // 更新上一次误差
	  pid->lastfeedback = feedback;      // 再次更新历史反馈值(冗余操作)

    // 计算微分项输出 = 微分 × 微分系数
    float dout = (derivative) * pid->kd;

    // 计算比例项输出 = 误差 × 比例系数
    float pout = pid->error * pid->kp;
   
   //  抗积分饱和：只在输出未饱和时积分
    float output_without_i = pout + dout;
    
    // 检查输出是否即将饱和
    if (output_without_i + pid->integral > pid->maxOutput) {
        // 即将正向饱和，只积分负误差
        if (pid->error < 0) {
            pid->integral += pid->error * pid->ki;
        }
    } else if (output_without_i + pid->integral < -pid->maxOutput) {
        // 即将负向饱和，只积分正误差
        if (pid->error > 0) {
            pid->integral += pid->error * pid->ki;
        }
    } else {
        // 正常情况，正常积分
        pid->integral += pid->error * pid->ki;
    }
    // 积分限幅：防止积分饱和
    if(pid->integral > pid->maxIntegral) 
        pid->integral = pid->maxIntegral;
    else if(pid->integral < - pid->maxIntegral) 
        pid->integral = - pid->maxIntegral;
   
    // 计算总输出 = 比例项 + 微分项 + 积分项
    pid->output = pout + dout + pid->integral;

    // 输出限幅：限制输出在允许范围内
    if(pid->output > pid->maxOutput) 
        pid->output = pid->maxOutput;
    else if(pid->output < - pid->maxOutput) 
        pid->output = - pid->maxOutput;
		
		// 死区控制：当对准误差绝对值小于0.1mm时，输出为0，防止振荡
		if(fabs(pid->error) < 0.3f){
				pid->output = 0;
				pid->integral = 0;  // 清空积分器，防止积分饱和
}
}

/**
 * @brief 基于对准误差的串级PID控制计算函数
 * @param motors: 电机控制结构体数组指针
 * @param num: 电机数量
 * @return 操作状态码 (0表示成功)
 * @note 外环使用对准误差(mm)，内环使用速度反馈(RPM)
 */
int16_t PID_CascadeCalc(MotorHandle_t *motors, uint8_t num)
{
				for (uint8_t i = 0; i < 2 && i < num; i++)
				{
						MotorHandle_t *motor = &motors[i];
        // 外环计算：对准误差环
        // 目标永远是0，反馈是当前的对准误差
        PID_Calc(&motor->pidset.outer, motor->pidset.outer.target, motor->pidset.outer.feedbackreal);
        motor->pidset.inner.target = motor->pidset.outer.output;
        // 内环计算：速度环
        // 外环输出作为速度目标，电机实际速度作为反馈
        PID_Calc(&motor->pidset.inner, motor->pidset.inner.target, (float)motor->info.vel);
        
        // 将内环输出作为最终电流输出
        motor->pidset.output = motor->pidset.inner.output;
        
        // 限制电流输出范围（保护电机）
        if (motor->pidset.output > 10000) motor->pidset.output = 10000;
        if (motor->pidset.output < -10000) motor->pidset.output = -10000;
    }
    
    // 通过CAN总线发送电流指令
    // 电机2、3保持静止（设为0）
    motor_current_set(&hfdcan1,
        motor[0].pidset.output,  // X方向电机电流
        motor[1].pidset.output,  // Y方向电机电流
        0,                       // 电机3静止
        0);                      // 电机4静止
    
    return 0;
}
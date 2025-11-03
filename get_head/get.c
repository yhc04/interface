#include "get.h"
#include "math.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "PID.h"
#include "fdcan.h"

Claw_t g_claw;

// 减速比参数
#define GEAR_RATIO 36.0f  // 36:1减速比
#define ENCODER_RESOLUTION 8192.0f  // 电机转子每圈脉冲数

// 初始化夹爪
void Claw_Init(Claw_t *claw, uint8_t motor_id)
{
    claw->motor_id = motor_id;
    claw->state = CLAW_READY;
    claw->calibrated = false;
    claw->target_percent = 0.5f;
    claw->pos_close = 0.0f;
    claw->pos_open = 0.0f;
    claw->pos_current = 0.0f;
}

// 校准
bool Claw_Calibrate(Claw_t *claw, MotorHandle_t *motors)
{
    MotorHandle_t *motor = &motors[claw->motor_id];
    
    claw->state = CLAW_CALIBRATING;
    
    // 1. 找闭合位置
    int16_t current = 2000;  // 固定电流
    uint32_t timeout = HAL_GetTick() + 4000;
    
    // 先转动300ms避免初始误判
    uint32_t start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < 300) {
        motor_current_set(&hfdcan1, 0, 0, current);
        osDelay(10);
    }
    
    // 检测堵转
    int32_t last_pos = motor->info.pos_total;
    uint8_t stall_count = 0;
    
    while(HAL_GetTick() < timeout) {
        motor_current_set(&hfdcan1, 0, 0, current);
        
        int32_t pos_diff = motor->info.pos_total - last_pos;
        if (pos_diff < 0) pos_diff = -pos_diff;
        
        if(pos_diff < 10) {
            stall_count++;
        } else {
            stall_count = 0;
            last_pos = motor->info.pos_total;
        }
        
        if(stall_count >= 3) {
            claw->pos_close = motor->info.pos_total;
            break;
        }
        osDelay(50);
    }
    
    // 2. 反向找张开位置
    current = -2000;  // 反向电流
    timeout = HAL_GetTick() + 4000;
    
    // 先转动300ms
    start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < 300) {
        motor_current_set(&hfdcan1, 0, 0, current);
        osDelay(10);
    }
    
    // 检测堵转
    last_pos = motor->info.pos_total;
    stall_count = 0;
    
    while(HAL_GetTick() < timeout) {
        motor_current_set(&hfdcan1, 0, 0, current);
        
        int32_t pos_diff = motor->info.pos_total - last_pos;
        if (pos_diff < 0) pos_diff = -pos_diff;
        
        if(pos_diff < 10) {
            stall_count++;
        } else {
            stall_count = 0;
            last_pos = motor->info.pos_total;
        }
        
        if(stall_count >= 3) {
            claw->pos_open = motor->info.pos_total;
            break;
        }
        osDelay(50);
    }
    
    // 停止电机
    motor_current_set(&hfdcan1, 0, 0, 0);
    
    // 确保闭合位置 < 张开位置
    if (claw->pos_close > claw->pos_open) {
        int32_t temp = claw->pos_close;
        claw->pos_close = claw->pos_open;
        claw->pos_open = temp;
    }
    
    claw->calibrated = true;
    claw->state = CLAW_READY;
    
    return true;
}

// 设置位置百分比 0-完全闭 1-完全开
bool Claw_SetPosition(Claw_t *claw, float percent)
{
    if(!claw->calibrated) return false; 

    percent = (percent < 0) ? 0 : (percent > 1) ? 1 : percent; //限制范围
    claw->target_percent = percent;
    claw->state = CLAW_MOVING;

    return true;
}

// 更新夹爪控制
void Claw_Update(Claw_t *claw, MotorHandle_t *motors)
{
    if (!claw->calibrated) return;
    
    MotorHandle_t *motor = &motors[claw->motor_id];
    claw->pos_current = motor->info.pos_total;
    
    if(claw->state == CLAW_MOVING) {
        // 计算目标位置（编码器脉冲）
        int32_t target_pos = claw->pos_close + 
                           (claw->pos_open - claw->pos_close) * claw->target_percent;
        
        motor->pidset.outer.target = target_pos;
        
        // 检查是否到达目标
        int32_t error = abs(claw->pos_current - target_pos);
        if(error < 50) {  // 50个脉冲容差
            claw->state = CLAW_READY;
        }
    }
}

// ================ 工具函数 ================

// 获取当前角度 输出轴角度
float Claw_GetAngle(Claw_t *claw, MotorHandle_t *motor)
{
    float output_angle = motor->info.pos_total * (360.0f / (ENCODER_RESOLUTION * GEAR_RATIO));
    return output_angle;
}

// 检查夹爪是否就绪
bool Claw_IsReady(Claw_t *claw)
{
    return (claw->calibrated && claw->state == CLAW_READY);
}
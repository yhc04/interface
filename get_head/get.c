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
    claw->angle_close = 0.0f;
    claw->angle_open = 0.0f;
    claw->angle_current = 0.0f;
}

// 校准
bool Claw_Calibrate(Claw_t *claw, MotorHandle_t *motors)
{
    MotorHandle_t *motor = &motors[claw->motor_id];
    
    claw->state = CLAW_CALIBRATING;
    
    // 1. 记录起始角度（当前位置）
    int32_t start_pos = motor->info.pos_total;
    
    // 2. 找闭合位置 - 先让电机转动一段时间再检测堵转
    int16_t current = 800;  // 固定电流值
    uint32_t timeout = HAL_GetTick() + 5000;
    bool found_close = false;
    
    // 先让电机转动300ms，确保脱离初始位置
    uint32_t start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < 300) {
        motor_current_set(&hfdcan1, 0, 0, current);
        osDelay(10);
    }
    
    // 现在开始检测堵转
    int32_t last_check_pos = motor->info.pos_total;
    uint8_t stall_count = 0;
    
    while(HAL_GetTick() < timeout) {
        motor_current_set(&hfdcan1, 0, 0, current);
        
        // 简单的堵转检测：连续3次检查位置变化很小
        int32_t pos_diff = motor->info.pos_total - last_check_pos;
        if (pos_diff < 0) pos_diff = -pos_diff;
        
        if(pos_diff < 10) {
            stall_count++;
        } else {
            stall_count = 0;
            last_check_pos = motor->info.pos_total;
        }
        
        if(stall_count >= 3) {
            int32_t close_pos = motor->info.pos_total;
            claw->angle_close = Claw_GetAngle(claw, motor);
            found_close = true;
            break;
        }
        
        osDelay(50);  // 50ms检查一次
    }
    
    if (!found_close) {
        claw->state = CLAW_ERROR;
        return false;
    }
    
    // 3. 反向脱离堵转
    current = -800;  // 反向电流
    timeout = HAL_GetTick() + 3000;
    while(HAL_GetTick() < timeout) {
        motor_current_set(&hfdcan1, 0, 0, current);
        osDelay(10);
    }
    
    // 4. 找张开位置 - 同样先转动一段时间
    current = -800;  // 反向电流
    timeout = HAL_GetTick() + 5000;
    bool found_open = false;
    
    // 先让电机转动300ms
    start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < 300) {
        motor_current_set(&hfdcan1, 0, 0, current);
        osDelay(10);
    }
    
    // 现在开始检测堵转
    last_check_pos = motor->info.pos_total;
    stall_count = 0;
    
    while(HAL_GetTick() < timeout) {
        motor_current_set(&hfdcan1, 0, 0, current);
        
        // 简单的堵转检测
        int32_t pos_diff = motor->info.pos_total - last_check_pos;
        if (pos_diff < 0) pos_diff = -pos_diff;
        
        if(pos_diff < 10) {
            stall_count++;
        } else {
            stall_count = 0;
            last_check_pos = motor->info.pos_total;
        }
        
        if(stall_count >= 3) {
            int32_t open_pos = motor->info.pos_total;
            claw->angle_open = Claw_GetAngle(claw, motor);
            found_open = true;
            break;
        }
        
        osDelay(50);  // 50ms检查一次
    }
    
    if (!found_open) {
        claw->state = CLAW_ERROR;
        return false;
    }
    
    // 5. 停止电机
    motor_current_set(&hfdcan1, 0, 0, 0);
    
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

// 更新
void Claw_Update(Claw_t *claw, MotorHandle_t *motors)
{
    if(!claw->calibrated) return;
    
    MotorHandle_t *motor = &motors[claw->motor_id];
    
    // 更新当前角度
    claw->angle_current = Claw_GetAngle(claw, motor);
    
    // 如果正在移动，计算目标角度并设置
    if(claw->state == CLAW_MOVING) {
        float target_angle = claw->angle_close + 
                           (claw->angle_open - claw->angle_close) * claw->target_percent;
        
        motor->pidset.outer.target = target_angle;
        
        // 检查是否到达目标
        float error = fabs(claw->angle_current - target_angle);
        if(error < 2.0f) { // 2度容差
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
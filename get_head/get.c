#include "get.h"
#include "math.h"

// 初始化夹爪
void Claw_Init(Claw_t *claw, uint8_t motor_id)
{
    claw->motor_id = motor_id;
    claw->state = CLAW_READY;
    claw->calibrated = false;
    claw->target_percent = 0.5f;
}

// 校准夹爪
bool Claw_Calibrate(Claw_t *claw, MotorHandle_t *motors)
{
    MotorHandle_t *motor = &motors[claw->motor_id];
    
    claw->state = CLAW_CALIBRATING;
    
    // 1. 找闭合位置
    motor->pidset.inner.target = 80; // 低速闭合
    uint32_t timeout = HAL_GetTick() + 3000;
    
    while(HAL_GetTick() < timeout) {
        if(Claw_CheckStall(motor)) {
            claw->angle_close = Claw_GetAngle(claw, motor);
            break;
        }
    }
    
    // 2. 反向运动脱离堵转
    motor->pidset.inner.target = -100;
    HAL_Delay(200);
    
    // 3. 找张开位置
    motor->pidset.inner.target = -80; // 低速张开
    timeout = HAL_GetTick() + 3000;
    
    while(HAL_GetTick() < timeout) {
        if(Claw_CheckStall(motor)) {
            claw->angle_open = Claw_GetAngle(claw, motor);
            break;
        }
    }
    
    // 4. 回到中间位置
    float mid_angle = (claw->angle_close + claw->angle_open) / 2;
    motor->pidset.outer.target = mid_angle;
    
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

// 获取当前角度
float Claw_GetAngle(Claw_t *claw, MotorHandle_t *motor)
{
    return (motor->info.pos_total / 8192.0f) * 360.0f;
}

// 检查是否堵转
bool Claw_CheckStall(MotorHandle_t *motor)
{
    static int32_t last_pos = 0;
    static uint8_t stall_count = 0;
    
    // 位置基本没变化
    if(abs(motor->info.pos_total - last_pos) < 5) {
        stall_count++;
    } else {
        stall_count = 0;
    }
    
    last_pos = motor->info.pos_total;
    
    // 连续3次没动就是堵转
    return (stall_count >= 3);
}

// 检查夹爪是否就绪
bool Claw_IsReady(Claw_t *claw)
{
    return (claw->calibrated && claw->state == CLAW_READY);
}
#ifndef __get_H__
#define __get_H__

#include "motor.h"
#include <stdbool.h>

//// --- 手动校准宏（现场校准后写入） ---
//// 建议现场读取角度（度）后替换下面值
//#define CLAW_ANGLE_CLOSE_DEG  10.0f   // 示例：完全闭合角度（度）
//#define CLAW_ANGLE_OPEN_DEG   160.0f  // 示例：完全张开角度（度）
//#define CLAW_SELF_CHECK_TOL_DEG 5.0f   // 自检容差（度）

// 夹爪状态
typedef enum {
    CLAW_READY,      // 就绪状态
    CLAW_CALIBRATING, // 校准中
    CLAW_MOVING,     // 运动中
    CLAW_ERROR       // 错误状态
} ClawState_t;

// 夹爪控制器
typedef struct {
    // 状态
    ClawState_t state;       // 夹爪状态
    uint8_t motor_id;        // 电机编号
    bool calibrated;         // 是否已校准
    
    // 校准数据
    float angle_close;       // 闭合角度
    float angle_open;        // 张开角度
    float angle_current;     // 当前角度
    
    // 控制
    float target_percent;    // 目标位置百分比 (0-1)
} Claw_t;

// 核心函数
void Claw_Init(Claw_t *claw, uint8_t motor_id);              // 初始化
bool Claw_Calibrate(Claw_t *claw, MotorHandle_t *motors);    // 校准
bool Claw_SetPosition(Claw_t *claw, float percent);          // 设置位置
void Claw_Update(Claw_t *claw, MotorHandle_t *motors);       // 更新控制



// 工具函数
bool Claw_IsReady(Claw_t *claw);
float Claw_GetAngle(Claw_t *claw, MotorHandle_t *motor);
bool Claw_CheckStall( MotorHandle_t *motor);

// 全局夹爪实例
extern Claw_t g_claw;

#endif
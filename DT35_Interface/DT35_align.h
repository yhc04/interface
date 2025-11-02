#ifndef __DT35_ALIGN_H__
#define __DT35_ALIGN_H__

#include "main.h"
#include <stdbool.h>

// DT35传感器ID定义
#define DT35_X_ID 0x71 // 水平切点传感器113
#define DT35_Y_ID 0x81 // 竖直切点传感器129

// 机械参数定义 (单位: mm)
#define TUBE_RADIUS 50.0f // 圆筒半径 50mm
#define ROD_RADIUS  10.15f // 矛杆半径 10.15mm

// 对接控制参数
#define ALIGN_THRESHOLD  1.5f // 对准阈值 1.5mm
#define MAX_ATTEMPTS    10   // 最大尝试次数

// 卡尔曼滤波参数
#define KALMAN_Q 0.03f    // 过程噪声协方差
#define KALMAN_R 3.0f    // 测量噪声协方差

#define MAX_CHANGE 3.0f  // 相邻数据最大允许变化 3.0mm
extern float x_bias;
extern float y_bias;

// 卡尔曼滤波结构体
typedef struct {
    float q;        // 过程噪声协方差
    float r;        // 测量噪声协方差  
    float x;        // 系统状态值（滤波后的距离值）
    float p;        // 状态协方差
    float k;        // 卡尔曼增益
} KalmanFilter_t;

// 平均累加器结构体
typedef struct {
    float x_sum;
    float y_sum;
    uint32_t count;
    uint32_t update_count;
} Average_Data_t;

// 对齐数据结构体
typedef struct {
    float x_distance;      // X轴距离 (来自DT35_X_ID)
    float y_distance;      // Y轴距离 (来自DT35_Y_ID)
    float rod_center_x;    // 矛杆圆心X坐标
    float rod_center_y;    // 矛杆圆心Y坐标
    float alignment_error; // 对准误差
    bool aligned;          // 是否对准完成
    bool data_ready;       // 数据就绪标志
    uint32_t update_count; // 更新次数
    uint8_t attempt_count; // 尝试次数
} Alignment_Data_t;

// 函数声明
void DT35_Init(void);
void FDCAN_Init_DT35(void); // 初始化FDCAN
void DT35_ProcessSingleCANData(void); // 处理单CAN数据
void DT35_CalculateAlignment(Alignment_Data_t* align_data); // 计算矛杆圆心位置
bool DT35_CheckAlignment(Alignment_Data_t* align_data); // 检查是否对准完成
void DT35_ExecuteAlignment(Alignment_Data_t* align_data); // 执行对齐操作
void Send_DT35_VOFA_Data(void);  //发送数据
void DT35_CalculateAverageAlignment(void); //基于平均值的圆心计算函数
void DT35_CalculateSystemBias(void);
// 卡尔曼滤波函数声明
void DT35_KalmanInit(void); // 初始化卡尔曼滤波器
float Kalman_Update(KalmanFilter_t* kf, float measurement); // 卡尔曼滤波更新

// 外部变量声明
extern Alignment_Data_t align_data;
extern Average_Data_t avg_data;

// 卡尔曼滤波器外部声明
extern KalmanFilter_t kalman_x, kalman_y;

// 单CAN接收标志和变量
extern volatile uint8_t g_can_data_ready;
extern uint8_t g_can_rx_data[8];
extern uint32_t g_can_rx_id;
extern uint32_t g_can_interrupt_count;


// 在 DT35_align.h 中添加
typedef struct {
    float kp, ki, kd;
    float error, last_error;
    float integral, max_integral;
    float output, max_output;
    float dead_zone;
} AlignmentPID_t;

// 全局对准PID控制器
extern AlignmentPID_t align_pid_x, align_pid_y;
void AlignmentPID_Init(AlignmentPID_t* pid, float p, float i, float d, float max_i, float max_out, float dead_zone);
float AlignmentPID_Calc(AlignmentPID_t* pid, float error);

#endif
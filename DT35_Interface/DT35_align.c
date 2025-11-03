#include "DT35_align.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include "fdcan.h"
#include "motor.h"
#include "PID.h"
#include "cmsis_os.h"

// 全局对齐数据
extern MotorHandle_t motor[4];
Alignment_Data_t align_data;
Average_Data_t avg_data = {0};
float x_bias = 0.0f;
float y_bias = 0.0f;


// 卡尔曼滤波器实例
KalmanFilter_t kalman_x, kalman_y;

// 对准PID控制器实例
AlignmentPID_t align_pid_x, align_pid_y;
// 卡尔曼滤波器初始化
void Kalman_Init(KalmanFilter_t* kf, float q, float r, float initial_value)
{
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = kf->q;  // 初始协方差
    kf->k = 0;
}

// 卡尔曼滤波更新
float Kalman_Update(KalmanFilter_t* kf, float measurement)
{
    // 预测步骤
    kf->p = kf->p + kf->q;
    
    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;
    
    return kf->x;
}

// DT35卡尔曼初始化
void DT35_KalmanInit(void)
{
    Kalman_Init(&kalman_x, KALMAN_Q, KALMAN_R, 0.0f);
    Kalman_Init(&kalman_y, KALMAN_Q, KALMAN_R, 0.0f);
}

// DT35初始化/初始化对齐数据结构体
void DT35_Init(void)
{
    memset(&align_data, 0, sizeof(align_data));
    align_data.data_ready = false;
    align_data.aligned = false;
		DT35_KalmanInit();
}

// 配置DT35FDCAN
void FDCAN_Init_DT35(void)
{
    // 配置FDCAN1过滤器 - 接收两个DT35传感器的ID
    FDCAN_FilterTypeDef sFilterConfig;
    
    // 过滤器0: 接收X传感器 (0x71)
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = DT35_X_ID;  // 要接收的ID
    sFilterConfig.FilterID2 = 0x7FF;      // 掩码：精确匹配
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    
    // 过滤器1: 接收Y传感器 (0x81)
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = DT35_Y_ID;  // 要接收的ID
    sFilterConfig.FilterID2 = 0x7FF;      // 掩码：精确匹配
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 启动FDCAN1
    HAL_FDCAN_Start(&hfdcan1);

    // 激活中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

// 基于平均值的圆心计算函数
void DT35_CalculateAverageAlignment(void)
{
    static uint32_t last_calc_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 累加数据
    avg_data.x_sum += align_data.x_distance;
    avg_data.y_sum += align_data.y_distance;
    avg_data.count++;
    
    // 每100次或每500ms计算一次（防止长时间不计算）
    if (avg_data.count >= 100 || (current_time - last_calc_time) >= 200) {
        if (avg_data.count > 0) {
            // 计算平均值
            float x_avg = avg_data.x_sum / avg_data.count;
            float y_avg = avg_data.y_sum / avg_data.count;
            
            // 保存原始值
            float original_x = align_data.x_distance;
            float original_y = align_data.y_distance;
            
            // 使用平均值计算圆心
            align_data.x_distance = x_avg;
            align_data.y_distance = y_avg;
            DT35_CalculateAlignment(&align_data);
            
            // 恢复原始值（保持实时数据不变）
            align_data.x_distance = original_x;
            align_data.y_distance = original_y;
            
            avg_data.update_count++;
            
        }
        
        // 重置累加器
        avg_data.x_sum = 0;
        avg_data.y_sum = 0;
        avg_data.count = 0;
        last_calc_time = current_time;
    }
}

// 计算矛杆圆心位置 
void DT35_CalculateAlignment(Alignment_Data_t* align_data)
{
    if (align_data->x_distance == 0 || align_data->y_distance == 0) {
        return; // 数据不完整,退出计算
    }
		
		// 直接减去系统误差
    float calibrated_x = align_data->x_distance - 8.8;
    float calibrated_y = align_data->y_distance - 9.3;
		
    // 计算矛杆上的测量点坐标 (以圆筒圆心为原点)
    // 水平点A: (-TUBE_RADIUS + calibrated_x, 0)
    float A_x = -TUBE_RADIUS + calibrated_x;
    float A_y = 0;

    // 竖直点B: (0, -TUBE_RADIUS + calibrated_y->y_distance)
    float B_x = 0;
    float B_y = -TUBE_RADIUS + calibrated_y;

    // 计算中点
    float mid_x = (A_x + B_x) / 2.0f;
    float mid_y = (A_y + B_y) / 2.0f;

    // AB向量
    float dx = B_x - A_x;
    float dy = B_y - A_y;

    // AB长度
    float AB_length = sqrtf(dx * dx + dy * dy);

    // 检查是否有效（两点距离不能大于直径）
    if (AB_length > 2 * ROD_RADIUS) {
        return;
    }

    // 垂直向量（逆时针旋转90度）
    float perp_x = -dy;
    float perp_y = dx;

    // 单位化
    float perp_length = sqrtf(perp_x * perp_x + perp_y * perp_y);
    perp_x /= perp_length;
    perp_y /= perp_length;

    // 矛杆圆心到中点的距离
    float h_square = ROD_RADIUS * ROD_RADIUS - (AB_length/2)*(AB_length/2);
    if (h_square < 0) {
        return;
    }
    float h = sqrtf(h_square);

    // 两个可能的矛杆圆心
    float center1_x = mid_x + perp_x * h;
    float center1_y = mid_y + perp_y * h;
    float center2_x = mid_x - perp_x * h;
    float center2_y = mid_y - perp_y * h;

    // 选择距离圆筒中心更近的圆心
    float dist1 = sqrtf(center1_x * center1_x + center1_y * center1_y);
    float dist2 = sqrtf(center2_x * center2_x + center2_y * center2_y);
    
    if (dist1 < dist2) {
        align_data->rod_center_x = center1_x;
        align_data->rod_center_y = center1_y;
    } else {
        align_data->rod_center_x = center2_x;
        align_data->rod_center_y = center2_y;
    }

    // 计算对准误差（矛杆圆心到圆筒圆心的直线距离）
    align_data->alignment_error = sqrtf(align_data->rod_center_x * align_data->rod_center_x +
                                       align_data->rod_center_y * align_data->rod_center_y);
}

// 检查是否对准完成 
bool DT35_CheckAlignment(Alignment_Data_t* align_data)
{
    if (align_data->alignment_error <= ALIGN_THRESHOLD) {
        align_data->aligned = true;
        return true;
    } else {
        align_data->aligned = false;
        return false;
    }
}

// 执行对齐操作
void DT35_ExecuteAlignment(Alignment_Data_t* align_data)
{
    align_data->attempt_count++;
    
    // 检查是否对准完成
    if (DT35_CheckAlignment(align_data)) {
        return;
    }
    
    motor[0].pidset.outer.target = 0.0f;
    motor[1].pidset.outer.target = 0.0f;
}

// DT35 VOFA数据发送函数
void Send_DT35_VOFA_Data(void)
{
    extern float x_bias, y_bias;
    float calibrated_x = align_data.x_distance - 8.8f;
    float calibrated_y = align_data.y_distance - 9.37f;

    float send_data[24] = {
        align_data.rod_center_x,        // 0: 杆圆心X（X对准误差）
        align_data.rod_center_y,        // 1: 杆圆心Y（Y对准误差）
        align_data.alignment_error,     // 2: 总对准误差
        (float)align_data.aligned,      // 3: 对准状态
        calibrated_x,                   // 4: 校准后X距离
        calibrated_y,                   // 5: 校准后Y距离
        
        // 电机控制状态
        motor[0].pidset.outer.error,    // 6: X方向对准误差
        motor[1].pidset.outer.error,    // 7: Y方向对准误差
        motor[0].pidset.outer.output,   // 8: X方向外环输出（速度目标）
        motor[1].pidset.outer.output,   // 9: Y方向外环输出（速度目标）
        motor[0].pidset.inner.output,   // 10: X方向内环输出（电流指令）
        motor[1].pidset.inner.output,   // 11: Y方向内环输出（电流指令）
        
        // 电机实际状态
        (float)motor[0].info.vel,       // 12: 电机1实际速度
        (float)motor[1].info.vel,       // 13: 电机2实际速度
        motor[0].info.cur,              // 14: 电机1实际电流
        motor[1].info.cur,              // 15: 电机2实际电流
        
        // 系统状态
        x_bias,                         // 16: X系统偏差
        y_bias,                         // 17: Y系统偏差
        motor[0].pidset.outer.target,   // 18: X外环目标
        motor[0].pidset.inner.target,   // 19: X内环目标
        
        // 电机3（夹爪）状态
        motor[2].pidset.outer.target,   // 20: 夹爪外环目标（角度）
        motor[2].pidset.inner.target,   // 21: 夹爪内环目标（速度）
        motor[2].pidset.output,         // 22: 夹爪电流输出
        (float)motor[2].info.vel        // 23: 夹爪实际速度
    };

    HAL_UART_Transmit(&huart2, (uint8_t*)send_data, sizeof(send_data), 100);
    uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart2, frame_tail, 4, 100);
}

// 计算1000次系统误差
void DT35_CalculateSystemBias(void)
{
    float x_sum = 0, y_sum = 0;
    for(int i = 0; i < 100; i++) {
        // 直接使用当前的距离值，不等待标志位
        x_sum += align_data.x_distance;
        y_sum += align_data.y_distance;
        if((i+1) % 10 == 0) {
        }
        HAL_Delay(20); // 延时20ms，让CAN数据更新
    }
    x_bias = x_sum / 100.0f;
    y_bias = y_sum / 100.0f;

}

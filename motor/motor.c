#include "motor.h"
#include "fdcan.h"
#include "stm32h7xx.h"
#include "main.h"
#include "PID.h"
#include "string.h"
#include "DT35_align.h"

// 电机控制全局变量定义
MotorHandle_t motor[4] = {0};  // 4个电机的控制结构体数组
uint8_t num;                   // 电机编号或其他用途变量
uint8_t rxbuff[8];            // 接收缓冲区(8字节)
uint8_t txbuff[8];            // 发送缓冲区(8字节)

//电机初始化
void Motor_Init(void)
{
    for(int i = 0; i < 4; i++) {
        memset(&motor[i], 0, sizeof(MotorHandle_t));
        if (i < 2) {
            // 对准电机使用对准PID
            CascadePID_Init(&motor[i].pidset);
        } else if (i == 2) {
            // 夹爪电机使用夹爪PID
            ClawPID_Init(&motor[i].pidset);
        }
        // 电机3保持默认
    }
}

// 配置电机FDCAN
void FDCAN_Init_motor(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;
    
    // 1. 配置过滤器 - 接收电机反馈消息 (ID: 0x201-0x204) 到 FIFO1
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 2;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; 
    sFilterConfig.FilterID1 = 0x201;
    sFilterConfig.FilterID2 = 0x1FC;
    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
    
    // 2. 启动FDCAN
    HAL_FDCAN_Start(hfdcan);
    
    // 3. 激活FIFO1接收中断
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

// 解析电机反馈数据
void get_moto_measure(MotorHandle_t* motor, uint8_t* rxbuff)
{
    // 保存上一次位置
    motor->info.pos_last = motor->info.pos;
    
    // 解析当前位置 (2字节, 0-8191范围)
    motor->info.pos = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    
    // 解析速度 (2字节有符号, RPM单位)
    motor->info.vel = (int16_t)(rxbuff[2] << 8 | rxbuff[3]);
    
    // 解析电流 (2字节, 转换为实际电流值)
    motor->info.cur = (rxbuff[4] << 8 | rxbuff[5]) * 5.f / 16384.f;
    
    // 计算位置误差
    motor->info.pos_error = (int32_t)motor->info.pos_total - (int32_t)motor->info.pos_totallast;
    motor->info.pos_totallast = motor->info.pos_total;
    
		// 处理位置溢出，计算圈数
		// 编码器范围: 0-8191 (0x1FFF)
		int32_t pos_diff = (int32_t)motor->info.pos - (int32_t)motor->info.pos_last;

		// 正向溢出: 从接近8191跳变到接近0
		if (pos_diff < -4096) {
				motor->info.round_cnt++; // 正向溢出，圈数加1
		}
		// 反向溢出: 从接近0跳变到接近8191  
		else if (pos_diff > 4096) {
				motor->info.round_cnt--; // 反向溢出，圈数减1
		}

		// 计算总位置 = 圈数 × 8192 + 当前位置
		motor->info.pos_total = motor->info.round_cnt * 8192 + motor->info.pos;
		}

// 设置2个电机的电流值并通过FDCAN发送 0表示成功
uint8_t motor_current_set(FDCAN_HandleTypeDef* hfdcan, int16_t iq1, int16_t iq2)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    // 配置FDCAN发送报文头
    TxHeader.Identifier          = 0x200;                   // 标准ID: 0x200
    TxHeader.IdType              = FDCAN_STANDARD_ID;       // 标准ID类型
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;        // 数据帧
    TxHeader.DataLength          = FDCAN_DLC_BYTES_8;       // 数据长度8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;        // 错误状态指示器
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;           // 比特率切换关闭
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;       // 经典CAN格式
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;      // 无TX事件
    TxHeader.MessageMarker       = 0;                       // 消息标记

    // 将2个16位电流值打包到8字节缓冲区，后两个电机设为0
    txbuff[0] = (uint8_t)(iq1 >> 8);      // iq1高字节
    txbuff[1] = (uint8_t)(iq1 & 0xFF);    // iq1低字节
    txbuff[2] = (uint8_t)(iq2 >> 8);      // iq2高字节
    txbuff[3] = (uint8_t)(iq2 & 0xFF);    // iq2低字节
    txbuff[4] = 0;                        // iq3高字节 = 0
    txbuff[5] = 0;                        // iq3低字节 = 0
    txbuff[6] = 0;                        // iq4高字节 = 0
    txbuff[7] = 0;                        // iq4低字节 = 0
    
    // 将消息添加到发送FIFO队列
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, txbuff);
    return 0;  // 返回成功
}

//将毫米位移转换为电机位置数据
float displacement_to_motor_data(float displacement_mm)
{
    const float SCREW_LEAD = 1.0f;          // 丝杆导程 1.0mm/圈
    const float ENCODER_RESOLUTION = 8192.0f; // 编码器分辨率 8192 counts/圈
    const float GEAR_RATIO = 36.0f;         // 减速比 36:1
    
    // 计算每毫米对应的编码器计数
    // counts_per_mm = (counts/圈) ÷ (mm/圈) × 减速比
    float counts_per_mm = (ENCODER_RESOLUTION / SCREW_LEAD) * GEAR_RATIO;
    
    // 转换为电机控制数据
    float motor_data = displacement_mm * counts_per_mm;
    
    return motor_data;
}

//将电机位置数据转换为毫米位移
float motor_data_to_displacement(float motor_data)
{
    const float SCREW_LEAD = 1.0f;
    const float ENCODER_RESOLUTION = 8192.0f;
    const float GEAR_RATIO = 36.0f;
    
    float counts_per_mm = (ENCODER_RESOLUTION / SCREW_LEAD) * GEAR_RATIO;
    float displacement = motor_data / counts_per_mm;
    
    return displacement;
}
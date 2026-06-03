#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stdint.h"

// 协议常量定义
#define FRAME_HEADER        0x68
#define FRAME_TAIL          0x16
#define RX_FRAME_LEN        36    // 接收固定长度
#define TX_STD_LEN          66    // 标准回复长度
#define TX_COLLECT_LEN      82    // 采集数据回复长度
#define TX_SETTINGS_LEN     86    // 设置信息回复长度

// 命令码定义
typedef enum {
    CMD_DEVICE_CHECK    = 0x01, // 设备连接校验(返回PID/波形等参数)
    CMD_LD_ENABLE       = 0x02,
    CMD_LD_DISABLE      = 0x03,
    CMD_BIAS_V1_ENABLE  = 0x06,
    CMD_CURRENT_SET     = 0x11, // LD电流设置
    CMD_BIAS_V1_SET     = 0x13, // 偏压1设置
    CMD_LOOP_CHECK      = 0x18, // 查询实时数据(电流/电压/温度)
    CMD_PID_SET         = 0x1C, // PID设置
    CMD_FF_SET          = 0x1D, // 前馈偏置设置 (mV, x1000)
    CMD_DAC_DIRECT      = 0x20, // 开环直接设DAC电压(mV), 0=恢复PID
    CMD_WAVE_MODE       = 0x21, // 波形模式: 0=关(PID), 1=正弦, 2=方波, 3=三角
    CMD_WAVE_FREQ       = 0x22, // 波形频率 (mHz, Hz×1000)
    CMD_WAVE_OFFSET     = 0x23, // 波形DC偏置 (mV)
    CMD_WAVE_AMP        = 0x24, // 波形幅度 (mV, x1000)
    // ... 可在此扩展其他命令
} CmdCode_e;

// 模拟系统参数结构体 (用于存储设备状态)
typedef struct {
    // 控制状态
    uint8_t ld_enable;
    uint8_t bias_enable;

    // 设置值 (协议单位)
    uint32_t ld_current_set;    // x100
    uint32_t bias_volt_set;     // x10000

    // 实时值 (协议单位)
    uint32_t ld_current_meas;   // x1000
    uint32_t ld_volt_meas;      // x1000
    uint32_t temp_meas;         // x1000

    // PID参数 (x1000)
    uint32_t pid_p;
    uint32_t pid_i;
    uint32_t pid_d;

    // 目标设定值 (mV)
    uint32_t setpoint;          // 单位 mV, 如 1000 = 1.000V

    // 波形与PD
    uint32_t wave_mode;         // 0=关(PID), 1=正弦, 2=方波, 3=三角
    uint32_t wave_amp;          // x1000
    uint32_t pd_sample_time;    // x10

    // 开环模式: 直接设DAC电压(mV), 0=恢复PID
    uint32_t dac_direct_volt;   // 单位 mV

    // 前馈偏置 (mV, x1000)
    uint32_t feed_forward;

    // 波形参数
    uint32_t wave_freq;      // 频率 mHz (Hz×1000)
    uint32_t wave_offset;    // DC偏置 mV
} SystemParams_t;

extern SystemParams_t sys_params;

// 函数声明
void Protocol_Init(void);
void Protocol_ProcessPacket(uint8_t* rx_buf);
uint8_t Protocol_CalcChecksum(uint8_t* data, uint16_t len);

#endif




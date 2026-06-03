#include "protocol.h"
#include "string.h"
#include "bsp_uart.h" // 引用发送函数

SystemParams_t sys_params;

// 内部使用的辅助宏：大端序转换
#define BIG_TO_U32(b) ( \
    ((uint32_t)((b)[0]) << 24) | \
    ((uint32_t)((b)[1]) << 16) | \
    ((uint32_t)((b)[2]) <<  8) | \
    ((uint32_t)((b)[3])      )   \
)

//小端变大端
#define U32_TO_BIG(val, buf) do { \
    (buf)[0] = ((val) >> 24) & 0xFF; \
    (buf)[1] = ((val) >> 16) & 0xFF; \
    (buf)[2] = ((val) >> 8) & 0xFF; \
    (buf)[3] = (val) & 0xFF; \
} while(0)

/**
  * @brief 初始化默认参数
  */
void Protocol_Init(void) {
    // 初始化默认参数
    sys_params.pid_p = 500;  // P=0.5
    sys_params.pid_i = 500;  // I=0.5
    sys_params.pid_d = 0;    // D=0.0
    sys_params.setpoint = 1000; // 目标 1.000V
    sys_params.wave_amp = 1000; // 1.0V
    sys_params.pd_sample_time = 10; // 1ms
    sys_params.feed_forward = 0; // 默认无前馈
    sys_params.wave_freq = 1000; // 默认1Hz
    sys_params.wave_offset = 0;  // 默认无偏置
}

/**
  * @brief 计算校验和
  * 算法: 0xFF - (除帧头和帧尾外所有字节的和 checkpoint)
  */
uint8_t Protocol_CalcChecksum(uint8_t* data, uint16_t len) {
    uint8_t sum = 0;
    uint16_t i;
    // 范围: index 1 到 len-2 (跳过头0x68和尾0x16 checkpoint)
    for (i = 1; i < len - 2; i++) {
        sum += data[i];
    }
    return (0xFF - sum);
}

/**
  * @brief 发送标准回复帧 (66字节)
  * 用于简单的开关命令或设置命令的ACK
  */
static void Send_Standard_Reply(uint8_t cmd, uint8_t status) {
    static uint8_t tx_buf[TX_STD_LEN] = {0x00};
    
    tx_buf[0] = FRAME_HEADER;
    tx_buf[5] = cmd;
    tx_buf[7] = 0x38; // 数据长度 56字节
    tx_buf[8] = status;
    
    // 计算校验并添加帧尾
    tx_buf[TX_STD_LEN - 2] = Protocol_CalcChecksum(tx_buf, TX_STD_LEN);
    tx_buf[TX_STD_LEN - 1] = FRAME_TAIL;
    
    UART_DMA_Send(tx_buf, TX_STD_LEN);
}

/**
  * @brief 发送采集信息回复 (82字节)
  * 对应命令 0x18 LOOP_CHECK，上传实时电流、电压、温度
  */
static void Send_Collect_Reply(void) {
    static uint8_t tx_buf[TX_COLLECT_LEN] = {0};
    
    tx_buf[0] = FRAME_HEADER;
    tx_buf[5] = CMD_LOOP_CHECK;
    tx_buf[7] = 0x48; // 数据长度 72字节
    
    // 状态位填充 (Byte 8-19)
    tx_buf[8] = 0x00; // 异常标志
    tx_buf[9] = sys_params.ld_enable;
    tx_buf[12] = sys_params.bias_enable;
    
    // 数据填充 (注意偏移量，参考协议7.3.2)
    // LD电压 (Byte 20-23)
    U32_TO_BIG(sys_params.ld_volt_meas, &tx_buf[20]);
    // LD电流 (Byte 24-27)
    U32_TO_BIG(sys_params.ld_current_meas, &tx_buf[24]);
    // TEC1温度 (Byte 48-51)
    U32_TO_BIG(sys_params.temp_meas, &tx_buf[48]);
    
    // 校验
    tx_buf[TX_COLLECT_LEN - 2] = Protocol_CalcChecksum(tx_buf, TX_COLLECT_LEN);
    tx_buf[TX_COLLECT_LEN - 1] = FRAME_TAIL;
    
    UART_DMA_Send(tx_buf, TX_COLLECT_LEN);
}

/**
  * @brief 发送设置信息回复 (86字节)
  * 对应命令 0x01 DEVICE_CHECK，上传PID、波形幅度、PD采样时间等
  */
static void Send_Settings_Reply(void) {
    static uint8_t tx_buf[TX_SETTINGS_LEN] = {0};
    
    tx_buf[0] = FRAME_HEADER;
    tx_buf[5] = CMD_DEVICE_CHECK;
    tx_buf[7] = 0x4C; // 数据长度 76字节
    
    // 填充用户要求的数据 (参考协议7.3.1)
    // LD电流设置 (8-11)
    U32_TO_BIG(sys_params.ld_current_set * 10, &tx_buf[8]); // 协议中设置是x100，回复是x1000
    // 偏压1设置 (24-27)
    U32_TO_BIG(sys_params.bias_volt_set, &tx_buf[24]);
    
    // PID参数 (40-51)
    U32_TO_BIG(sys_params.pid_p, &tx_buf[40]);
    U32_TO_BIG(sys_params.pid_i, &tx_buf[44]);
    U32_TO_BIG(sys_params.pid_d, &tx_buf[48]);
    U32_TO_BIG(sys_params.feed_forward, &tx_buf[52]);
    U32_TO_BIG(sys_params.wave_freq, &tx_buf[56]);
    U32_TO_BIG(sys_params.wave_offset, &tx_buf[60]);
    U32_TO_BIG(sys_params.wave_mode, &tx_buf[64]);
    
    // 波形幅值 (72-75)
    U32_TO_BIG(sys_params.wave_amp, &tx_buf[72]);
    // PD采样时间 (76-79)
    U32_TO_BIG(sys_params.pd_sample_time * 100, &tx_buf[76]); // 协议单位差异需注意，假设此处统一
    
    // 校验
    tx_buf[TX_SETTINGS_LEN - 2] = Protocol_CalcChecksum(tx_buf, TX_SETTINGS_LEN);
    tx_buf[TX_SETTINGS_LEN - 1] = FRAME_TAIL;
    
    UART_DMA_Send(tx_buf, TX_SETTINGS_LEN);
}

/**
  * @brief 协议解析核心函数
  * @param rx_buf 接收到的36字节原始数据
  */
void Protocol_ProcessPacket(uint8_t* rx_buf) {
    uint8_t cal_sum;
    uint8_t cmd;
    uint32_t data1;

    // 1. 基础校验
    if (rx_buf[0] != FRAME_HEADER || rx_buf[35] != FRAME_TAIL) return;

    cal_sum = Protocol_CalcChecksum(rx_buf, RX_FRAME_LEN);
    if (cal_sum != rx_buf[34]) return; // 校验失败丢弃

    cmd = rx_buf[5];
    data1 = BIG_TO_U32(&rx_buf[10]); // 提取第一个数据字段

    // 2. 命令分发
    switch (cmd) {
        case CMD_DEVICE_CHECK: // 0x01
            Send_Settings_Reply(); // 回复PID、波形等静态参数
            break;
            
        case CMD_LOOP_CHECK:   // 0x18
            Send_Collect_Reply();  // 回复实时电流、温度等
            break;

        case CMD_LD_ENABLE:    // 0x02
            sys_params.ld_enable = 1;
            Send_Standard_Reply(cmd, 0x00);
            break;
            
        case CMD_LD_DISABLE:   // 0x03
            sys_params.ld_enable = 0;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_BIAS_V1_ENABLE: // 0x06
            sys_params.bias_enable = 1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_CURRENT_SET:  // 0x11
            sys_params.ld_current_set = data1; // 单位0.01mA
            // 此处可添加调用底层DAC的代码
            Send_Standard_Reply(cmd, 0x00);
						
            break;

        case CMD_BIAS_V1_SET:  // 0x13 — 设定目标电压 (mV)
            sys_params.setpoint = data1; // 单位 mV, 如 1000 = 1.000V
            Send_Standard_Reply(cmd, 0x00);
            break;
            
        // 扩展：若需设置PID
        case CMD_PID_SET:      // 0x1C
            // PID命令有多个数据，需单独提取
            sys_params.pid_p = data1;
            sys_params.pid_i = BIG_TO_U32(&rx_buf[14]);
            sys_params.pid_d = BIG_TO_U32(&rx_buf[18]);
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_FF_SET:       // 0x1D — 前馈偏置(mV, x1000)
            sys_params.feed_forward = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_WAVE_MODE:    // 0x21 — 波形模式
            sys_params.wave_mode = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_WAVE_FREQ:    // 0x22 — 波形频率(mHz)
            sys_params.wave_freq = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_WAVE_OFFSET:  // 0x23 — 波形DC偏置(mV)
            sys_params.wave_offset = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_WAVE_AMP:     // 0x24 — 波形幅度(mV, x1000)
            sys_params.wave_amp = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        case CMD_DAC_DIRECT:  // 0x20 — 开环直接设DAC电压(mV)
            sys_params.dac_direct_volt = data1;
            Send_Standard_Reply(cmd, 0x00);
            break;

        default:
            // 未知命令
            break;
    }

}


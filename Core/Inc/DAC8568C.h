#ifndef __DAC8568C_H
#define __DAC8568C_H

#include "main.h"

/* 通道定义 */
#define OutA    0x00
#define OutB    0x01
#define OutC    0x02
#define OutD    0x03
#define OutE    0x04
#define OutF    0x05
#define OutG    0x06
#define OutH    0x07

/* 命令定义 */
#define RESET                       0
#define POWER_UP                    1
#define SETUP_INTERNAL_REGISTER     2

/* 硬件参数 */
#define DAC_VREF    2.5f
#define DAC_GAIN    2.0f

void DAC8568_Init(void);
void DAC8568_SetVoltage(unsigned char mCh, float Vol);

#endif

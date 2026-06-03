#ifndef __ADS1256_H
#define __ADS1256_H

#include "main.h"

/* 寄存器地址 */
#define REG_STATUS	(0)
#define	REG_MUX    	(1)
#define	REG_ADCON   (2)
#define	REG_DRATE   (3)
#define	REG_IO      (4)

/* 命令定义 */
#define	CMD_WAKEUP		(0x00)
#define	CMD_RDATA   	(0x01)
#define	CMD_RDATAC  	(0x03)
#define	CMD_SDATAC  	(0x0F)
#define	CMD_RREG    	(0x10)
#define	CMD_WREG    	(0x50)
#define	CMD_SELFCAL 	(0xF0)
#define	CMD_SELFOCAL	(0xF1)
#define	CMD_SELFGCAL	(0xF2)
#define	CMD_SYSOCAL 	(0xF3)
#define	CMD_SYSGCAL 	(0xF4)
#define	CMD_SYNC    	(0xFC)
#define	CMD_STANDBY 	(0xFD)
#define	CMD_RESET   	(0xFE)

/* PGA增益 */
#define PGA_1            0x00
#define PGA_2            0x01
#define PGA_4            0x02
#define PGA_8            0x03
#define PGA_16           0x04
#define PGA_32           0x05
#define PGA_64           0x06

/* 采样率 */
#define DATARATE_30K         0xF0
#define DATARATE_15K         0xE0
#define DATARATE_7_5K        0xD0
#define DATARATE_3_7_5K      0xC0
#define DATARATE_2K          0xB0
#define DATARATE_1K          0xA0
#define DATARATE_500         0x92
#define DATARATE_100         0x82
#define DATARATE_60        	 0x72
#define DATARATE_50        	 0x63
#define DATARATE_30        	 0x53
#define DATARATE_25        	 0x43
#define DATARATE_15        	 0x33
#define DATARATE_10        	 0x23
#define DATARATE_5        	 0x13
#define DATARATE_2_5     	 0x02

/* 通道选择宏 */
#define POSITIVE_AIN0       (0X00)
#define POSITIVE_AIN1       (0X10)
#define POSITIVE_AIN2       (0X20)
#define POSITIVE_AIN3       (0X30)
#define POSITIVE_AIN4       (0X40)
#define POSITIVE_AIN5       (0X50)
#define POSITIVE_AIN6       (0X60)
#define POSITIVE_AIN7       (0X70)
#define POSITIVE_AINCOM     (0X80)

#define NEGTIVE_AIN0         0X00
#define NEGTIVE_AIN1         0X01
#define NEGTIVE_AIN2         0X02
#define NEGTIVE_AIN3         0X03
#define NEGTIVE_AIN4         0X04
#define NEGTIVE_AIN5         0X05
#define NEGTIVE_AIN6         0X06
#define NEGTIVE_AIN7         0X07
#define NEGTIVE_AINCOM       0X08

/* 函数声明 */
void ADS1256_GPIO_Init(void);
void ADS1256_CfgADC(uint8_t gain, uint8_t drate);
uint32_t ADS1256_GetAdc(uint8_t channel);
uint8_t ADS1256_ReadReg(uint8_t reg);
void ADS1256_WriteReg(uint8_t reg, uint8_t value);

#endif
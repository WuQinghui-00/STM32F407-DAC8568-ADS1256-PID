#include "ads1256.h"

/* 引脚定义 - 软件SPI (PD口) */
#define ADS_SCLK_PORT   GPIOD
#define ADS_SCLK_PIN    GPIO_PIN_0//时钟
#define ADS_DIN_PORT    GPIOD
#define ADS_DIN_PIN     GPIO_PIN_1//数据输入（STM32发数据给ADC）
#define ADS_DOUT_PORT   GPIOD
#define ADS_DOUT_PIN    GPIO_PIN_2//数据输出（ADC发数据给STM32）
#define ADS_DRDY_PORT   GPIOD
#define ADS_DRDY_PIN    GPIO_PIN_3//ADC告诉STM32转换完了
#define ADS_CS_PORT     GPIOD
#define ADS_CS_PIN      GPIO_PIN_4//片选选中ADC
#define ADS_RST_PORT    GPIOD
#define ADS_RST_PIN     GPIO_PIN_5//STM32复位ADC

#define ADS_SCLK_H  HAL_GPIO_WritePin(ADS_SCLK_PORT, ADS_SCLK_PIN, GPIO_PIN_SET)
#define ADS_SCLK_L  HAL_GPIO_WritePin(ADS_SCLK_PORT, ADS_SCLK_PIN, GPIO_PIN_RESET)
#define ADS_DIN_H   HAL_GPIO_WritePin(ADS_DIN_PORT, ADS_DIN_PIN, GPIO_PIN_SET)
#define ADS_DIN_L   HAL_GPIO_WritePin(ADS_DIN_PORT, ADS_DIN_PIN, GPIO_PIN_RESET)
#define ADS_CS_H    HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET)
#define ADS_CS_L    HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_RESET)
#define ADS_RST_H   HAL_GPIO_WritePin(ADS_RST_PORT, ADS_RST_PIN, GPIO_PIN_SET)
#define ADS_RST_L   HAL_GPIO_WritePin(ADS_RST_PORT, ADS_RST_PIN, GPIO_PIN_RESET)

#define ADS_DRDY_IS_LOW() (HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN) == GPIO_PIN_RESET)

static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * (168 / 7);
    while(count--);
}
//GPIO初始化
void ADS1256_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* SCLK, DIN, CS, RST: Output PP */设为推挽输出（STM32主动控制）
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* DOUT: Input */（STM32收到ADC发来的数据，故是输入模式）
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* DRDY: Input (下降沿表示数据就绪) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* 初始电平 */ //复位ADC，复位时序必须满足数据手册要求（至少拉低0.5ms）
    ADS_RST_L;
    delay_us(1000);
    ADS_RST_H;
    delay_us(100000); /* 100ms */
    ADS_CS_H;
    ADS_SCLK_L;
    ADS_DIN_H;
}

//发送一个字节
static void ADS1256_Send8Bit(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            ADS_DIN_H;
        else
            ADS_DIN_L;

        ADS_SCLK_H;
        delay_us(1);
        data <<= 1;
        ADS_SCLK_L; /* ADS1256在SCLK下降沿锁存DIN */
        delay_us(1);
    }
}

static uint8_t ADS1256_Recive8Bit(void)
{
    uint8_t read = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        read <<= 1;
        ADS_SCLK_H;
        delay_us(1);
        if (HAL_GPIO_ReadPin(ADS_DOUT_PORT, ADS_DOUT_PIN) == GPIO_PIN_SET)
            read |= 0x01;
        ADS_SCLK_L;
        delay_us(1);
    }
    return read;
}

void ADS1256_WriteCmd(uint8_t cmd)
{
    ADS_CS_L;
    ADS1256_Send8Bit(cmd);
    ADS_CS_H;
}

void ADS1256_WriteReg(uint8_t reg, uint8_t value)
{
    ADS_CS_L;
    ADS1256_Send8Bit(CMD_WREG | reg);
    ADS1256_Send8Bit(0x00);  /* 写1个寄存器 */
    ADS1256_Send8Bit(value);
    ADS_CS_H;
}

uint8_t ADS1256_ReadReg(uint8_t reg)
{
    uint8_t read;

    ADS_CS_L;
    ADS1256_Send8Bit(CMD_RREG | reg);
    ADS1256_Send8Bit(0x00);
    delay_us(5);
    read = ADS1256_Recive8Bit();
    ADS_CS_H;

    return read;
}

void ADS1256_CfgADC(uint8_t gain, uint8_t drate)
{
    uint8_t buf[4];

    ADS1256_WriteCmd(CMD_RESET);
    delay_us(10);

    /* 连续写4个寄存器：STATUS, MUX, ADCON, DRATE */
    /* STATUS: ORDER=MSB, ACAL=启用自动校准, BUFEN=禁用缓冲 */
    buf[0] = (0 << 3) | (1 << 2) | (0 << 1);

        /* MUX: AIN0 - AINCOM (默认) */
        buf[1] = 0x08;

        /* ADCON: CLKOUT关, 传感器检测关, 增益 */
        buf[2] = (0 << 5) | (0 << 3) | (gain << 0);

        /* DRATE: 采样率 */
        buf[3] = drate;

        ADS_CS_L;
        ADS1256_Send8Bit(CMD_WREG | 0);    /* 从REG0开始写 */
        ADS1256_Send8Bit(0x03);            /* 写4个寄存器 */
        ADS1256_Send8Bit(buf[0]);          /* STATUS */
        ADS1256_Send8Bit(buf[1]);          /* MUX */
        ADS1256_Send8Bit(buf[2]);          /* ADCON */
        ADS1256_Send8Bit(buf[3]);          /* DRATE */
        ADS_CS_H;

    /* 自校准 */
    ADS1256_WriteCmd(CMD_SELFCAL);
    delay_us(50);
    while (!ADS_DRDY_IS_LOW());
}

uint32_t ADS1256_GetAdc(uint8_t channel)
{
    uint32_t read = 0;
    uint32_t timeout;

    /* 选择通道 (单端: AINx - AINCOM) */
    ADS1256_WriteReg(REG_MUX, (channel << 4) | 0x08);

    /* 同步并启动转换 */
    ADS1256_WriteCmd(CMD_SYNC);
    delay_us(5);
    ADS1256_WriteCmd(CMD_WAKEUP);

    /* 等待DRDY拉低 */
    timeout = 500000;
    while (!ADS_DRDY_IS_LOW())
    {
        timeout--;
        if (timeout == 0) return 0xFFFFFFFF;
    }

    /* 读数据 */
    ADS_CS_L;
    ADS1256_Send8Bit(CMD_RDATA);
    delay_us(5);

    read = (uint32_t)ADS1256_Recive8Bit() << 16;
    read |= (uint32_t)ADS1256_Recive8Bit() << 8;
    read |= ADS1256_Recive8Bit();
    ADS_CS_H;

    return read;
}

uint8_t ADS1256_GetChipID(void)
{
    uint8_t id = ADS1256_ReadReg(REG_STATUS);
    return (id >> 4);
}
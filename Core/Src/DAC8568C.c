#include "DAC8568C.h"
#include "spi.h"

/* 引脚定义 */
#define DAC8568_LDAC_PORT   GPIOB
#define DAC8568_LDAC_PIN    GPIO_PIN_11
#define DAC8568_SYNC_PORT   GPIOB
#define DAC8568_SYNC_PIN    GPIO_PIN_2
#define DAC8568_CLR_PORT    GPIOB
#define DAC8568_CLR_PIN     GPIO_PIN_10

/* 软件SPI引脚 */
#define DAC8568_SCK_PORT    GPIOA
#define DAC8568_SCK_PIN     GPIO_PIN_5
#define DAC8568_MOSI_PORT   GPIOA
#define DAC8568_MOSI_PIN    GPIO_PIN_7

#define DAC8568_SYNC_H  HAL_GPIO_WritePin(DAC8568_SYNC_PORT, DAC8568_SYNC_PIN, GPIO_PIN_SET)//主动拉高，告诉DAC发完数据
#define DAC8568_SYNC_L  HAL_GPIO_WritePin(DAC8568_SYNC_PORT, DAC8568_SYNC_PIN, GPIO_PIN_RESET)
#define DAC8568_LDAC_H  HAL_GPIO_WritePin(DAC8568_LDAC_PORT, DAC8568_LDAC_PIN, GPIO_PIN_SET)
#define DAC8568_LDAC_L  HAL_GPIO_WritePin(DAC8568_LDAC_PORT, DAC8568_LDAC_PIN, GPIO_PIN_RESET)//主动拉低，更新数据
#define DAC8568_CLR_H   HAL_GPIO_WritePin(DAC8568_CLR_PORT, DAC8568_CLR_PIN, GPIO_PIN_SET)
#define DAC8568_CLR_L   HAL_GPIO_WritePin(DAC8568_CLR_PORT, DAC8568_CLR_PIN, GPIO_PIN_RESET)

#define DAC8568_SCK_H   HAL_GPIO_WritePin(DAC8568_SCK_PORT, DAC8568_SCK_PIN, GPIO_PIN_SET)
#define DAC8568_SCK_L   HAL_GPIO_WritePin(DAC8568_SCK_PORT, DAC8568_SCK_PIN, GPIO_PIN_RESET)
#define DAC8568_MOSI_H  HAL_GPIO_WritePin(DAC8568_MOSI_PORT, DAC8568_MOSI_PIN, GPIO_PIN_SET)
#define DAC8568_MOSI_L  HAL_GPIO_WritePin(DAC8568_MOSI_PORT, DAC8568_MOSI_PIN, GPIO_PIN_RESET)

static void delay_spi(volatile unsigned int t)
{
    volatile unsigned int i = 0;
    while(t--) { i = 5; while(i--); }
}

static void DAC8568_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* DAC控制引脚 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11;//选择引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//选择模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;//有无上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;//低速或者高速，低速省电
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);//配置写入GPIOB

    /* 软件SPI引脚 */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//写入GPIOA

    DAC8568_SYNC_H;
    DAC8568_CLR_H;
    DAC8568_LDAC_L;
    DAC8568_SCK_H;//空闲时高电平，mode3
    DAC8568_MOSI_L;
    HAL_Delay(10);
}

/**
 * @brief 软件SPI逐位发送1字节（8位）给DAC（MSB first（最高位先发）, SPI Mode 3）
 */
static void DAC8568_SPI_WriteByte(uint8_t dat)
{
    int i;
    for (i = 7; i >= 0; i--)
    {
        if (dat & (1 << i))
            DAC8568_MOSI_H;
        else
            DAC8568_MOSI_L;

        delay_spi(1);
        DAC8568_SCK_L;
        delay_spi(1);
        DAC8568_SCK_H;
    }
}

//发送4个字节
static void DAC8568_Send4Bytes(uint8_t *data)
{
    DAC8568_SYNC_L;
    delay_spi(1);

    DAC8568_SPI_WriteByte(data[0]);
    DAC8568_SPI_WriteByte(data[1]);
    DAC8568_SPI_WriteByte(data[2]);
    DAC8568_SPI_WriteByte(data[3]);

    delay_spi(1);
    DAC8568_SYNC_H;
}

/* 与F103已工作版本一致的32位帧打包算法
 * DAC8568 32位帧: [Prefix=4bit][Control=4bit][Address=4bit][Data=16bit][Feature=4bit]
 * buf[0]=DB31-DB24, buf[1]=DB23-DB16, buf[2]=DB15-DB8, buf[3]=DB7-DB0
 */
 //32位打包函数
static uint32_t ADC_CodeTo32BitBuffer(uint8_t PreConbyte, uint8_t Addressbyte, uint16_t Datashort, uint8_t Featurebits)
{
    uint32_t ret_val = 0;
    Addressbyte &= 0x0f;
    Featurebits &= 0x0f;
    ret_val = PreConbyte;         // 8-bit: [0][0][0][0][P3][P2][P1][P0]  (P=prefix, C=control)
                                  // but PreConbyte=0x03, so: [0][0][0][0][0][0][1][1]
    ret_val <<= 4;                // [0][0][0][0][0][0][1][1][0][0][0][0] = 0x030
    ret_val |= Addressbyte;       // [0][0][0][0][0][0][1][1][A3][A2][A1][A0]
    ret_val <<= 16;               // [0][0][0][0][0][0][1][1][A3][A2][A1][A0][0][0][0][0]...[0][0][0][0]
    ret_val |= Datashort;         // [0][0][0][0][0][0][1][1][A3][A2][A1][A0][D15..D0]
    ret_val <<= 4;                // [0][0][0][0][0][0][1][1][A3][A2][A1][A0][D15..D0][0][0][0][0]
    ret_val |= Featurebits;       // [0][0][0][0][0][0][1][1][A3][A2][A1][A0][D15..D0][F3][F2][F1][F0]
    return ret_val;
}

/* 写通道 - 与F103完全一致 */
static void DAC8568_Write_DAC_Channel(uint8_t ch, uint16_t value)
{
    uint32_t SRData;
    uint8_t buf[4];

    SRData = ADC_CodeTo32BitBuffer(0x03, ch, value, 0x00);//0x03是前缀加控制字节，0x00是通道A

    //32位数拆为4个字节
    buf[0] = (SRData >> 24) & 0xFF;//最高8位
    buf[1] = (SRData >> 16) & 0xFF;
    buf[2] = (SRData >> 8)  & 0xFF;
    buf[3] = SRData & 0xFF;//最低8位

    DAC8568_Send4Bytes(buf);
}

/* 写命令 - 与F103完全一致 */
static void DAC8568_Write_Command(uint8_t command)
{
    switch(command)
    {
        case SETUP_INTERNAL_REGISTER: // 内部基准使能 (0x08000001)
            DAC8568_SYNC_L;
            delay_spi(1);
            DAC8568_SPI_WriteByte(0x08);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0x01);
            break;

        case POWER_UP: // 上电所有通道 (0x040000FF)
            DAC8568_SYNC_L;
            delay_spi(1);
            DAC8568_SPI_WriteByte(0x04);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0xFF);
            break;

        case RESET: // 复位 (0x07000000)
            DAC8568_SYNC_L;
            delay_spi(1);
            DAC8568_SPI_WriteByte(0x07);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0x00);
            DAC8568_SPI_WriteByte(0x00);
            break;

        default:
            return;
    }

    delay_spi(1);
    DAC8568_SYNC_H;
}

void DAC8568_SetVoltage(unsigned char mCh, float Vol)//mCh是通道号，Vol是目标电压
{
    float mDatafloat;
    uint16_t mDtashort;

    if (Vol < 0.0f) Vol = 0.0f;//防止DAC输出负电压

    /* VREF=2.5V, GAIN=2x, 满量程=5V */
    mDatafloat = Vol * (65535.0f / (DAC_VREF * DAC_GAIN));
    mDtashort = (uint16_t)mDatafloat;
    if(mDatafloat > 65535.0f) mDtashort = 65535;

    DAC8568_Write_DAC_Channel(mCh, mDtashort);
}

void DAC8568_Init(void)
{
    DAC8568_GPIO_Init();//初始化配置各引脚并且设置初始电平

    /* 1. 复位 */
    DAC8568_Write_Command(RESET);
    HAL_Delay(20);

    /* 2. 上电所有通道 */
    DAC8568_Write_Command(POWER_UP);
    HAL_Delay(10);

    /* 3. 内部基准使能 */
    DAC8568_Write_Command(SETUP_INTERNAL_REGISTER);
    HAL_Delay(10);

    /* 增益默认=2x(POR状态)，不额外设置 */
}

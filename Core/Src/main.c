/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_uart.h"
#include "protocol.h"
#include "DAC8568C.h"
#include "ads1256.h"
#include "pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ads_id = 0;
uint32_t adc_val = 0;
float voltage = 0.0f;
PID_t pid;
uint32_t last_tick = 0;
float dt = 0.0f;
float wave_phase = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

/* 阻塞式串口打印，调试用 */
static void UART_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Config();
  Protocol_Init();
  DAC8568_Init();
  DAC8568_SetVoltage(OutA, 1.0f);

  ADS1256_GPIO_Init();
  /* 配置PGA=1, 采样率=30SPS */
  ADS1256_CfgADC(PGA_1, DATARATE_100);
  ads_id = ADS1256_GetChipID();
  UART_Printf("ADS1256 ID = 0x%02X\r\n", ads_id);

  /* PID初始化: 从sys_params读取 */
  PID_Init(&pid,
      (float)sys_params.pid_p / 1000.0f,
      (float)sys_params.pid_i / 1000.0f,
      (float)sys_params.pid_d / 1000.0f,
      (float)sys_params.setpoint / 1000.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  last_tick = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    adc_val = ADS1256_GetAdc(0);

    /* ADC原始值 → 测量电压 */
    voltage = (float)adc_val * 5.0f / 8388607.0f;

    /* PID周期dt (闭环/波形模式都用) */
    dt = (float)(HAL_GetTick() - last_tick) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last_tick = HAL_GetTick();

    if (sys_params.wave_mode > 0 && sys_params.wave_mode <= 3) {
        /* 波形模式: DAC直接输出波形, ADC监视 */
        float wave_val = 0.0f;
        float freq = (float)sys_params.wave_freq / 1000.0f;  /* mHz→Hz */
        float amp  = (float)sys_params.wave_amp / 1000.0f;    /* mV→V */
        float off  = (float)sys_params.wave_offset / 1000.0f; /* mV→V */
        float two_pi_f_dt = 2.0f * 3.14159265f * freq * dt;

        wave_phase += two_pi_f_dt;
        if (wave_phase > 2.0f * 3.14159265f)
            wave_phase -= 2.0f * 3.14159265f;

        switch (sys_params.wave_mode) {
            case 1: /* 正弦 */
                wave_val = off + amp * sinf(wave_phase);
                break;
            case 2: /* 方波 */
                wave_val = off + (wave_phase < 3.14159265f ? amp : -amp);
                break;
            case 3: /* 三角波 */
                if (wave_phase < 3.14159265f)
                    wave_val = off + amp * (2.0f * wave_phase / 3.14159265f - 1.0f);
                else
                    wave_val = off + amp * (2.0f * (2.0f * 3.14159265f - wave_phase) / 3.14159265f - 1.0f);
                break;
        }
        if (wave_val < 0.0f) wave_val = 0.0f;
        if (wave_val > 5.0f) wave_val = 5.0f;
        DAC8568_SetVoltage(OutA, wave_val);
        pid.output = wave_val;
    } else if (sys_params.dac_direct_volt > 0) {
        /* 开环模式: 直接设DAC，跳过PID */
        DAC8568_SetVoltage(OutA, (float)sys_params.dac_direct_volt / 1000.0f);
        pid.output = (float)sys_params.dac_direct_volt / 1000.0f;
    } else {
        /* 闭环模式: 模型前馈 + PID微调 */

        /* 从sys_params同步PID参数 (每周期检查，上位机改了立即生效) */
        pid.setpoint = (float)sys_params.setpoint / 1000.0f;
        pid.kp = (float)sys_params.pid_p / 1000.0f;
        pid.ki = (float)sys_params.pid_i / 1000.0f;
        pid.kd = (float)sys_params.pid_d / 1000.0f;
        pid.ff = (float)sys_params.feed_forward / 1000.0f;

        /* 模型前馈: 标定 ADC=0.972*DAC+0.197 → DAC=(目标-0.197)/0.972 */
        float dac_base = (pid.setpoint - 0.197f) / 0.972f;
        if (dac_base < 0.0f) dac_base = 0.0f;
        if (dac_base > 5.0f) dac_base = 5.0f;

        /* PID微调(仅PI): output = 修正量(而非DAC绝对值) */
        float error = pid.setpoint - voltage;
        pid.integral += error * dt;
        if (pid.integral > pid.integral_limit) pid.integral = pid.integral_limit;
        else if (pid.integral < -pid.integral_limit) pid.integral = -pid.integral_limit;
        float pi_out = pid.kp * error + pid.ki * pid.integral + pid.ff;
        if (pi_out > 2.0f) pi_out = 2.0f;
        if (pi_out < -2.0f) pi_out = -2.0f;

        /* DAC = 模型基础值 + PI修正量 */
        float dac_out = dac_base + pi_out;
        if (dac_out < 0.0f) dac_out = 0.0f;
        if (dac_out > 5.0f) dac_out = 5.0f;
        DAC8568_SetVoltage(OutA, dac_out);
        pid.output = dac_out;
    }

    /* 更新实时值 (协议单位) */
    sys_params.ld_volt_meas = (uint32_t)(voltage * 1000.0f);
    sys_params.ld_current_meas = (uint32_t)(pid.output * 1000.0f);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
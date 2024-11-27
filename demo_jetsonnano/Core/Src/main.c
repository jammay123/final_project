/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc1_buffer[3] = {0,};
uint32_t adc1_buffer_for6[3][6] = {0,};
uint8_t adc_filter_index = 0;
uint32_t adc_filter_sum = 0;
uint32_t filtered_buffer[3] = {0,};
uint8_t send_psd_flag = 0;

uint8_t PCrxBuffer[RX_BUFFER_SIZE];
extern int32_t linear_vel;
extern int32_t angular_vel;
float velocity_0 = 0;		//max : 250, right
float velocity_1 = 0;		//max : 250, left
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void writePacket(uint8_t packet[], int length);
void writePacket3(uint8_t packet[], int length);

void sendPsd();

void operatingMode(uint8_t motor_id);
void enableTorque(uint8_t motor_id);
void sendGoalVelocity(uint8_t motor_id, int32_t velocity);

void convertlineangVelToSpeed(int32_t linear_vel, int32_t angular_vel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void writePacket(uint8_t packet[], int length) {
    for (int i = 0; i < length; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1, packet[i]);
    }
}
void writePacket3(uint8_t packet[], int length) {
    for (int i = 0; i < length; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART3));
        LL_USART_TransmitData8(USART3, packet[i]);
    }
}

void sendPsd() {
    uint8_t packet[14]; // ?��?��(1) + ?��?��?��(12) + ?��?��(1)
    int index = 0;

    packet[index++] = 0x08;

    for(int i=0; i<3; i++) {
		packet[index++] = (filtered_buffer[i] >> 24) & 0xFF;
		packet[index++] = (filtered_buffer[i] >> 16) & 0xFF;
		packet[index++] = (filtered_buffer[i] >> 8) & 0xFF;
		packet[index++] = filtered_buffer[i] & 0xFF;
    }
    packet[index++] = 0x20;

    writePacket(packet, index);
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_ADC_Start_DMA(&hadc1, adc1_buffer, 3);

  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)PCrxBuffer);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&USART1->DR);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, RX_BUFFER_SIZE);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableIT_IDLE(USART1);

  operatingMode(0);
  enableTorque(0);
  operatingMode(1);
  enableTorque(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(send_psd_flag) {
		  sendPsd();
		  send_psd_flag = 0;
	  }
	  convertlineangVelToSpeed(linear_vel, angular_vel);
	  sendGoalVelocity(0, -(velocity_0 * 10));
	  sendGoalVelocity(1, (velocity_1 * 10));
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == hadc1.Instance) {
		for(int i = 0; i < 3; i++) {
			adc1_buffer_for6[i][adc_filter_index] = adc1_buffer[i];
		}
		adc_filter_index = (adc_filter_index + 1) % 6;

		for(int i=0; i<3; i++) {
			adc_filter_sum = 0;
			for(int j=0; j<6; j++) {
				adc_filter_sum += adc1_buffer_for6[i][j];
			}
			filtered_buffer[i] = adc_filter_sum / 6;
		}
	}
	send_psd_flag = 1;
}
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for(j = 0; j < data_blk_size; j++)
  {
      i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
      crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}
void enableTorque(uint8_t motor_id) {
  uint8_t packet[14];
  size_t index = 0;

  packet[index++] = 0xFF;
  packet[index++] = 0xFF;
  packet[index++] = 0xFD;
  packet[index++] = 0x00;
  packet[index++] = motor_id; // ID

  packet[index++] = 0x06;
  packet[index++] = 0x00;

  packet[index++] = 0x03; // Write

  packet[index++] = 0x40;
  packet[index++] = 0x00;
  packet[index++] = 0x01; // Torque ?��?��?��

  unsigned short crc = update_crc(0, packet, index);
  packet[index++] = crc & 0xFF;
  packet[index++] = (crc >> 8) & 0xFF;

  writePacket3(packet, index);
}

void operatingMode(uint8_t motor_id) {
  uint8_t packet[14];
  size_t index = 0;

  packet[index++] = 0xFF;
  packet[index++] = 0xFF;
  packet[index++] = 0xFD;
  packet[index++] = 0x00;
  packet[index++] = motor_id; // ID

  packet[index++] = 0x06;
  packet[index++] = 0x00;

  packet[index++] = 0x03; // Write

  packet[index++] = 0x0B;
  packet[index++] = 0x00;
  packet[index++] = 0x01;

  unsigned short crc = update_crc(0, packet, index);
  packet[index++] = crc & 0xFF;
  packet[index++] = (crc >> 8) & 0xFF;

  writePacket3(packet, index);
}

void sendGoalVelocity(uint8_t motor_id, int32_t velocity) {
  uint8_t packet[16];
  size_t index = 0;

  packet[index++] = 0xFF;
  packet[index++] = 0xFF;
  packet[index++] = 0xFD;
  packet[index++] = 0x00;
  packet[index++] = motor_id; // ID

  packet[index++] = 0x09;
  packet[index++] = 0x00;

  packet[index++] = 0x03; // Write

  packet[index++] = 0x68;
  packet[index++] = 0x00;
  packet[index++] = (velocity & 0xFF);
  packet[index++] = (velocity >> 8) & 0xFF;
  packet[index++] = (velocity >> 16) & 0xFF;
  packet[index++] = (velocity >> 24) & 0xFF;

  unsigned short crc = update_crc(0, packet, index);
  packet[index++] = crc & 0xFF;
  packet[index++] = (crc >> 8) & 0xFF;

  writePacket3(packet, index);
}

void convertlineangVelToSpeed(int32_t linear_vel, int32_t angular_vel) {
	velocity_0 = linear_vel + (angular_vel * 0.9);
	velocity_1 = linear_vel - (angular_vel * 0.9);
}
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

#ifdef  USE_FULL_ASSERT
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

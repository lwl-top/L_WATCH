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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
#include "sys.h"
#include "OLED.h"
#include "delay.h"
#include "free_delay.h"
#include "lvgl_demo.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "MYI2C.h"
#include "CST816T.h"
#include "W25Q64.h"
#include "PCF8563.h"
#include "mpu6050.h"
#include "MPU6050_I2C.h"
#include "HMC.h"
#include <math.h>
#include "FreeRTOS.h"
#include "semphr.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// #define WALLPAPER_BASE_ADDR  0x100000  // 壁纸存储起始地址
// #define WALLPAPER_SIZE       (sizeof(lv_img_dsc_t))  
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

lv_img_dsc_t KLL2 = {
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .header.always_zero = 0,
    .header.reserved = 0,
    .header.w = 240,
    .header.h = 280,
    .data_size = 240 * 280 * 2,
    .data = NULL,
};

#define Jus 0x100000
#define EV 0x100000 + sizeof(lv_img_header_t) + 210945;
#define PDXX 0x100000 + 2 * sizeof(lv_img_header_t) + 421890;
#define KLLL 0x100000 + 3 * sizeof(lv_img_header_t) + 621546;
#define lronman 0x100000 + 4 * sizeof(lv_img_header_t) + 817506

LV_IMG_DECLARE(YJ);
LV_IMG_DECLARE(SpiderMan);
LV_IMG_DECLARE(CURRY);
LV_IMG_DECLARE(BrilliantEye);
LV_IMG_DECLARE(Five);
LV_IMG_DECLARE(flower);
LV_IMG_DECLARE(GTX);
LV_IMG_DECLARE(HYRZ);
LV_IMG_DECLARE(JERRY);
LV_IMG_DECLARE(JUSTDO);
LV_IMG_DECLARE(LZ);
LV_IMG_DECLARE(M);
LV_IMG_DECLARE(money);
LV_IMG_DECLARE(NAME);
LV_IMG_DECLARE(PDXHM);
LV_IMG_DECLARE(snow);
LV_IMG_DECLARE(spider);
LV_IMG_DECLARE(APERTURE);
LV_IMG_DECLARE(Demon);
LV_IMG_DECLARE(DT);

LV_IMG_DECLARE(HYRE1);
LV_IMG_DECLARE(HYRZ2);
LV_IMG_DECLARE(HYRZ3);
LV_IMG_DECLARE(HYRZ4);
LV_IMG_DECLARE(HYRZ6);
LV_IMG_DECLARE(HYRZ7);
LV_IMG_DECLARE(HYRZ8);
LV_IMG_DECLARE(HYRZ9);

float adc_get(void)
{
  uint32_t dat = 0;
  uint8_t i;
  float BatVoltage;
  for (i = 0; i < 10; i++)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    dat += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    HAL_Delay(1);
  }
  dat = dat / 10;
  BatVoltage = dat * 2 * 3.3 / 4096;
  return BatVoltage;
}

uint8_t PowerCalculate()
{
  uint8_t power;
  float voltage;
  voltage = adc_get();

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0)
  {
    voltage -= 0.140 * 1;
  }
  if (voltage >= 4.20f)
    return 100;
  else if (voltage >= 4.15f)
    return 95;
  else if (voltage >= 4.11f)
    return 90;
  else if (voltage >= 4.07f)
    return 85;
  else if (voltage >= 4.03f)
    return 80;
  else if (voltage >= 3.99f)
    return 75;
  else if (voltage >= 3.95f)
    return 70;
  else if (voltage >= 3.91f)
    return 65;
  else if (voltage >= 3.87f)
    return 60;
  else if (voltage >= 3.83f)
    return 55;
  else if (voltage >= 3.79f)
    return 50;
  else if (voltage >= 3.75f)
    return 45;
  else if (voltage >= 3.71f)
    return 40;
  else if (voltage >= 3.67f)
    return 35;
  else if (voltage >= 3.63f)
    return 30;
  else if (voltage >= 3.59f)
    return 25;
  else if (voltage >= 3.55f)
    return 20;
  else if (voltage >= 3.51f)
    return 15;
  else if (voltage >= 3.47f)
    return 10;
  else if (voltage >= 3.43f)
    return 5;
  else
    return 1; 
}
extern volatile uint8_t key_flag;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    static uint16_t key1_long_press;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    {
      key1_long_press += 1;

      if (key1_long_press >= 6000)
      {
        key_flag = 2;
      }
      else if (key1_long_press >= 3000)
      {
        key_flag = 1;
      }
      else if (key1_long_press >= 100)
      {
        key_flag = 0;
      }
      else
      {
        key_flag = 4;
      }	
    }
    else
    {
      key1_long_press = 0;
    }
  }
}
uint8_t write_data[] = {0x00, 0x01, 0x02};
uint8_t read_data[4];


uint8_t rx_buffer[1]; 
char rx_data[100];   
volatile uint8_t rx_index = 0;

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t week;
} TimeData;


typedef enum{	 
     IDLE,
	   TIMESET,
	   TEXT,
}blue_mode;
blue_mode MODE=IDLE;
void ParseAndSetTime(char* data) {
    TimeData time;
    
    if (sscanf(data, "S %hhu %hhu %hhu %hhu %hhu %hhu %hhu T",
               &time.year, &time.month, &time.day,
               &time.hour, &time.minute, &time.second, &time.week) == 7) {
        PCF8563_WriteTime(0x00, 0x20);  
        PCF8563_WriteTime(0x08, time.year);
        PCF8563_WriteTime(0x07, time.month);
        PCF8563_WriteTime(0x05, time.day);
        PCF8563_WriteTime(0x04, time.hour);
        PCF8563_WriteTime(0x03, time.minute);
        PCF8563_WriteTime(0x02, time.second);
        PCF8563_WriteTime(0x06, time.week-1);
        PCF8563_WriteTime(0x00, 0x00);  

        char success_msg[] = "TIME_OK\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)success_msg, strlen(success_msg), 100);
        
        rx_index = 0;          
        memset(rx_data, 0, sizeof(rx_data));
				MODE=IDLE;
    } else {
        char error_msg[] = "TIME_ERROR\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), 100);
        rx_index = 0;
			  memset(rx_data, 0, sizeof(rx_data));
    }
}

extern SemaphoreHandle_t xTextModeSemaphore;
BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
        if (rx_index < sizeof(rx_data) - 1)
        {
					  if(rx_buffer[0] != '\n' && rx_buffer[0] != '\r')
						{
								rx_data[rx_index++] = rx_buffer[0];
								rx_data[rx_index] = '\0';		
							if(rx_index>5&&rx_data[rx_index-1]=='L'&&rx_data[rx_index-2]=='d'&&rx_data[rx_index-3]=='n'&&rx_data[rx_index-4]=='e'&&rx_data[rx_index-5]=='W')
								{
									rx_index=0;
								  memset(rx_data, 0, sizeof(rx_data));
								}				
						}
        }
        else
        {
            rx_index = 0;
            memset(rx_data, 0, sizeof(rx_data));
        }

        if (strcmp(rx_data, "TIMESET") == 0)
        {
            char enable_msg[] = "TIME_MODE\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)enable_msg, strlen(enable_msg), 100);            
            MODE=TIMESET;
            rx_index = 0;  
            memset(rx_data, 0, sizeof(rx_data));
        }
        else if(strcmp(rx_data, "TEXT") == 0)
        {           
           char TEXT_msg[] = "TEXT_MODE\r\n";
           HAL_UART_Transmit(&huart1, (uint8_t*)TEXT_msg, strlen(TEXT_msg), 100);            
            MODE=TEXT;
					  rx_index = 0; 
            memset(rx_data, 0, sizeof(rx_data));
        }
				else if(strcmp(rx_data, "IDLE") == 0)
				{
					char IDLE_msg[] = "IDLE_MODE\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)IDLE_msg, strlen(IDLE_msg), 100);            
					MODE=IDLE;
					rx_index = 0; 
          memset(rx_data, 0, sizeof(rx_data));
				}
				
				switch(MODE)
				{
					case TIMESET:
						if(rx_data[0]=='S'&&rx_data[rx_index-1]=='T')
						{
					    ParseAndSetTime(rx_data);	
						}
					break;
					case TEXT:
						 if (strlen(rx_data)> 0&&rx_buffer[0]=='\n')
                {
                    if (xTextModeSemaphore != NULL)
                    {
                        xSemaphoreGiveFromISR(xTextModeSemaphore, &xHigherPriorityTaskWoken);
                    }
                }	
					break;
					
					case IDLE:
					break;
				}
				
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  }
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
  // MX_GPIO_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  MD_GPIO_Init();
  KJ_GPIO_Init();
  CHECK_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  gpio_init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  lvgl_demo();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

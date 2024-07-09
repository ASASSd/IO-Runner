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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "ring_buffer.h"
#include "jsmn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define CDC_TX_SINGLE_BUFF_MAXLEN 128
#define CDC_RX_TOTAL_BUFF_MAXLEN 1024
#define CDC_RX_SINGLE_BUFF_MAXLEN (CDC_RX_TOTAL_BUFF_MAXLEN / 8)
#define TOKEN_MAX_LEN 32
#define TOKENS_MAX_NUM 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef TOKEN_DEBUG
#define PRINT_TOKEN(i,s) printf("token %d: type: %d start: %d end: %d value string: %s\n\r", (i), tokens[(i)].type, tokens[(i)].start, tokens[(i)].end, (s))
#else
#define PRINT_TOKEN(i,s) while(0)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
char cdc_tx[CDC_TX_SINGLE_BUFF_MAXLEN];
char cdc_rx_total[CDC_RX_TOTAL_BUFF_MAXLEN];
char cdc_rx[CDC_RX_SINGLE_BUFF_MAXLEN];
RING_buffer_t cdc_rx_ring;
jsmn_parser jsonparser;
jsmntok_t tokens[TOKENS_MAX_NUM];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int dump(const char *js, jsmntok_t *t, size_t count, int indent) {
  int i, j, k;
  jsmntok_t *key;
  if (count == 0) {
    return 0;
  }
  if (t->type == JSMN_PRIMITIVE) {
    printf("%.*s", t->end - t->start, js + t->start);
    return 1;
  } else if (t->type == JSMN_STRING) {
    printf("'%.*s'", t->end - t->start, js + t->start);
    return 1;
  } else if (t->type == JSMN_OBJECT) {
    printf("\n");
    j = 0;
    for (i = 0; i < t->size; i++) {
      for (k = 0; k < indent; k++) {
        printf("  ");
      }
      key = t + 1 + j;
      j += dump(js, key, count - j, indent + 1);
      if (key->size > 0) {
        printf(": ");
        j += dump(js, t + 1 + j, count - j, indent + 1);
      }
      printf("\n");
    }
    return j + 1;
  } else if (t->type == JSMN_ARRAY) {
    j = 0;
    printf("\n");
    for (i = 0; i < t->size; i++) {
      for (k = 0; k < indent - 1; k++) {
        printf("  ");
      }
      printf("   - ");
      j += dump(js, t + 1 + j, count - j, indent + 1);
      printf("\n");
    }
    return j + 1;
  }
  return 0;
}

void token_load(char* str, int n, int str_size) {
  memset(str, 0x0, str_size);
  memcpy(str, (char*)(cdc_rx+tokens[n].start), (tokens[n].end-tokens[n].start));
}

int value_cmpnload(int* val, int n, char* token, const char* key) {
  if(!strcmp(token, key)) {
    if(tokens[n+1].type == JSMN_PRIMITIVE) {
      char nt[TOKEN_MAX_LEN];
      token_load(nt, n+1, TOKEN_MAX_LEN);
      *val = atoi(nt);
    } else {
      return -1;
    }
  }
  return 0;
}

int gpio_message_load(gpio_message_t* gpio_msg, int token_cnt_max) {
  for(int i = 3; i < token_cnt_max; i++) {
    char singletoken[TOKEN_MAX_LEN];
    token_load(singletoken, i, TOKEN_MAX_LEN);
    PRINT_TOKEN(i, singletoken);
    if(value_cmpnload((int*) &(gpio_msg->port), i, singletoken, gpioport_key)) {
      return -1;
    }
    if(value_cmpnload((int*) &(gpio_msg->pin), i, singletoken, gpiopin_key)) {
      return -2;
    }
    if(value_cmpnload((int*) &(gpio_msg->val), i, singletoken, gpioval_key)) {
      return -3;
    }
  }
  return 0;
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  RING_Init(&cdc_rx_ring, (uint8_t*) cdc_rx_total, CDC_RX_TOTAL_BUFF_MAXLEN);
  jsmn_init(&jsonparser);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(;;) 
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  #if 0
    static uint16_t counter = 100;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    sprintf(cdc_tx, "counter = %d\n\r", counter);
    CDC_Transmit_FS(cdc_tx, CDC_TX_SINGLE_BUFF_MAXLEN);
    printf("[CDC->] %s", cdc_tx);
    if (counter > 1000){
      counter = 100;
    }
    HAL_Delay(counter * 5);
    counter++;

    for(int i = 0; i < 10; i++) {
        char singletoken[32];
        memcpy(singletoken, (char*)(cdc_rx+tokens[i].start), (tokens[i].end-tokens[i].start));
        if(tokens[i].type == JSMN_PRIMITIVE) {
          int val = atoi(singletoken);
          printf("token %d: type: %d start: %d end: %d value: %d\n\r", i, tokens[i].type, tokens[i].start, tokens[i].end, val);
        } else {
          printf("token %d: type: %d start: %d end: %d value string: %s\n\r", i, tokens[i].type, tokens[i].start, tokens[i].end, singletoken);
        }
        memset(singletoken, 0x0, 32);
      }

    GPIOF: ((0x40000000UL + 0x00020000UL) + 0x1400UL))

#endif
    uint16_t ring_cnt = RING_GetCount(&cdc_rx_ring);
    if(ring_cnt) {
      char cts[TOKEN_MAX_LEN] = {0, };   // current token string
      RING_PopString(&cdc_rx_ring, cdc_rx);
      int rc = jsmn_parse(&jsonparser, cdc_rx, strlen(cdc_rx), tokens, (sizeof(tokens)/sizeof(tokens[0])));
      if(rc > 0) {
        printf("json found\n\r");
        int if_type = -1;
        if(tokens[1].type == JSMN_STRING) {
          printf("if cmd found\n\r");
          token_load(cts, 1, TOKEN_MAX_LEN);
          value_cmpnload(&if_type, 1, cts, ifcmd);
        }
        switch(if_type) {
          case 0:
          printf("GPIO cmd found\n\r");
          gpio_message_t gpio_msg;
          rc = gpio_message_load(&gpio_msg, TOKENS_MAX_NUM);
          printf("gpio_message_load: rc = %d\n\r", rc);
          printf("GPIO port = %d\n\r", gpio_msg.port);
          printf("GPIO pin = %d\n\r", gpio_msg.pin);
          printf("GPIO val = %d\n\r", gpio_msg.val);
          HAL_GPIO_WritePin(gpio_msg.port, gpio_msg.pin, gpio_msg.val);
          break;

          default:
          printf("no compatible if found!\n\r");
          break;
        }
      }
      memset(cdc_rx, 0x0, CDC_RX_SINGLE_BUFF_MAXLEN);
    }
    HAL_Delay(2000);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
HAL_GPIO_WritePin(GPIOF, LED1_Pin|LED2_Pin, GPIO_PIN_SET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief USB CDC Rx interrupt handler
  * @param Buf - buffer with received message
  * @param Len - received message length
  * @retval None
  */
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
#ifdef CDC_DEBUG
  printf("[CDC<-] %s %d\n\r", Buf, Len);
#endif
  if (Len <= CDC_RX_SINGLE_BUFF_MAXLEN) {
    if (strlen(Buf) == Len) {   // FIXME: need to properly catch zero-terminated string and put it into ring buffer
      RING_PutBuffr(&cdc_rx_ring, Buf, Len);
    }
  }
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++){
  //ITM_SendChar( *ptr++ );
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr++,1,1000);
  }
  return len;
  }/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"

#define R25   10000.0f
#define T25   298.15f
#define BETA  3900.0f
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

  static int contadorLeft = 0;
  static int contadorRight = 0;
  static int pulsado = 0;
  static int pulsadoLeft = 0;
  static int valorAlarma = 8;
  static int valorActual = 0;
  
  static int led_encendido=0;
  static int sensor = 0;
  static int alarma_activa = 0;
  static int alarma_armada = 1;
  static uint32_t alarma_stop_time = 0;
  /*static int contadorBinario = 0;

  int binarioLED()
  {
    return 0;
    
  }*/

  int detectar_pulsacionLeft(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    static int estado_ant = 0;
    GPIO_PinState res_pin = HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
    int res = (res_pin == GPIO_PIN_RESET) ? 0 : 1;
    //int pulsado = 0;
    if((res == 0) && (estado_ant == 0))
    {
      estado_ant = 1;
      printf("IZQ Soltado Arriba");
      sensor++;
        if(sensor > 1)
        {
          sensor = 0;
        }
        printf("SENSORR %d\n",sensor);
    }
    if((res == 1) && (estado_ant == 1))
    {
      pulsado = 1;
      estado_ant = 0;
      printf("IZQ Pulsado Abajo");
      contadorLeft++;
    }
    if (contadorLeft == 3)
    {
      contadorLeft = 0;
      return 1;
    }else{
    return 0;}
  }

  int valorPOT()
  {
      ADC_ChannelConfTypeDef sConfig = {0};
      sConfig.Channel = ADC_CHANNEL_4;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      // Disparo la conversion
      HAL_ADC_Start(&hadc1);
      // Espero la finalización
      HAL_ADC_PollForConversion(&hadc1, 10000);
      // Leo el valor de la conversión 1023 y 0
      return (HAL_ADC_GetValue(&hadc1)+ 10)*8/4095;
  }

  int valorLDR()
  {
    ADC_ChannelConfTypeDef sConfig = {0};
     sConfig.Channel = ADC_CHANNEL_1;
     sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      // Disparo la conversion
      HAL_ADC_Start(&hadc1);
      // Espero la finalización
      HAL_ADC_PollForConversion(&hadc1, 10000);
      //printf("SENSOR TERMICO: %d\n",(HAL_ADC_GetValue(&hadc1)-1500)/67);//valores CALIENTE: 1500 temp mano 1800 TEMP NORMAL:2000 0 -2040 / 8 - 1500 540/8
      
      return (8-(HAL_ADC_GetValue(&hadc1)-1500)/67);
  }

  int valorNTC()
  {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10000);
    float valueAD = (float)HAL_ADC_GetValue(&hadc1);
    // Formula Steinhart-Hart con parametros BETA
    float tmp = BETA / (logf((-10000.0f * 3.3f / (valueAD * 3.3f / 4095.9f - 3.3f) - 10000.0f) / R25) + BETA / T25) - 273.18f;
    // Mapear rango 25-30 grados a 0-8 LEDs
    int leds = (int)((tmp - 25.0f) * 8.0f / 5.0f);
    if (leds < 0) leds = 0;
    if (leds > 8) leds = 8;
    return leds;
  }

  void encenderLucesPot(int x)
  {
    if (x >= 1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    if (x >= 2) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    if (x >= 3) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    if (x >= 4) HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    if (x >= 5) HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    if (x >= 6) HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    if (x >= 7) HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    if (x >= 8) HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);

    if (x < 1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    if (x < 2) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    if (x < 3) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    if (x < 4) HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
    if (x < 5) HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
    if (x < 6) HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
    if (x < 7) HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
    if (x < 8) HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
  }


  // Enciende o apaga el LED numero n (1-8)
  void set_led_n(int n, int state)
  {
    GPIO_PinState ps = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch(n) {
      case 1: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ps); break;
      case 2: HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ps); break;
      case 3: HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ps); break;
      case 4: HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ps); break;
      case 5: HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ps); break;
      case 6: HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, ps); break;
      case 7: HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, ps); break;
      case 8: HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, ps); break;
    }
  }

  // Devuelve 1 en el flanco de bajada del boton (pulsacion detectada)
  int detectar_pulsacionRight(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    static int estado_ant = 1;  // 1 = no pulsado (pull-up)
    GPIO_PinState res_pin = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    int res = (res_pin == GPIO_PIN_RESET) ? 0 : 1;
    if ((res == 0) && (estado_ant == 1)) {  // flanco bajada: boton pulsado
      estado_ant = 0;
      return 1;
    }
    if ((res == 1) && (estado_ant == 0)) {  // flanco subida: boton soltado
      estado_ant = 1;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setvbuf(stdout, NULL, _IONBF, 0);

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);


  while (1)
  {
    valorAlarma = valorPOT();
    printf("%d",valorAlarma);
    // Leer sensor seleccionado y mostrar en LEDs
    if (sensor == 0) {
      valorActual = valorNTC();
    } else if (sensor == 1) {
      valorActual = valorLDR();
    } else {
      sensor = 0;
    }
    encenderLucesPot(valorActual);

    // LED del nivel de alarma parpadea a ~10 Hz (toggle cada 50ms)
    if (valorAlarma >= 1 && valorAlarma <= 8) {
      int led_alarma_estado = (HAL_GetTick() / 50) % 2;
      set_led_n(valorAlarma, led_alarma_estado);
    }

    // Rearme automatico tras 10 segundos
    if (!alarma_armada && (HAL_GetTick() - alarma_stop_time > 10000)) {
      alarma_armada = 1;
      printf("alarma rearmada\n");
    }
    // Disparo de alarma si esta armada y se supera el nivel
    if (alarma_armada && valorActual >= valorAlarma) {
      alarma_activa = 1;
    }
    // Control del buzzer segun estado de alarma
    if (alarma_activa) {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      printf("pitoo\n");
    } else {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }

    HAL_Delay(50);

    // Boton izquierdo: cambia sensor mostrado
    detectar_pulsacionLeft(ButtonLeft_GPIO_Port, ButtonLeft_Pin);

    // Boton derecho: para la alarma (si esta activa)
    if (detectar_pulsacionRight(ButtonRight_GPIO_Port, ButtonRight_Pin) == 1) {
      if (alarma_activa) {
        alarma_activa = 0;
        alarma_armada = 0;
        alarma_stop_time = HAL_GetTick();
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        printf("alarma parada por boton, rearmando en 10s\n");
      }
    }

    HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED8_Pin|LED6_Pin|BUZZER_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED7_Pin|LED2_Pin|LED5_Pin|LED1_Pin
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED8_Pin LED6_Pin BUZZER_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED8_Pin|LED6_Pin|BUZZER_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED7_Pin LED2_Pin LED5_Pin LED1_Pin
                           LED4_Pin */
  GPIO_InitStruct.Pin = LED7_Pin|LED2_Pin|LED5_Pin|LED1_Pin
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonLeft_Pin */
  GPIO_InitStruct.Pin = ButtonLeft_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ButtonLeft_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonRight_Pin */
  GPIO_InitStruct.Pin = ButtonRight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ButtonRight_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "display\lcd.h"
#include "LiquidCrystal_I2C.h"

#define X_A 13.18
#define X_B 450
#define Y_A 10.3
#define Y_B 340

#define X_BORDER 240
#define Y_BORDER 320

typedef struct {
	GPIO_TypeDef* Port;
	uint16_t Pin;
} GPIOStruct;

typedef enum {
	OUTPUT_RESET = 0,
	OUTPUT_SET = 1,
	INPUT_PULLUP_EXTI = 2,
	INPUT_NOPULL = 3,
	INPUT_ADC= 4
} GPIOState;

typedef enum {
	TOUCH_OFF = 0,
	TOUCH_DETECT = 1,
	TOUCH_MEASURE_X = 2,
	TOUCH_MEASURE_Y = 3
} TouchScreenState;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
GPIOStruct x_left, x_right, y_up, y_down;
uint32_t x, y, previous_x, previous_y;
uint8_t start_measure = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t GetTouch_X();
uint32_t GetTouch_Y();

uint32_t ADC1_GetValue(uint32_t channel);

HAL_StatusTypeDef SetGPIOState(GPIOStruct* str, GPIOState state);
HAL_StatusTypeDef ResetTouchScreenPinsState();
HAL_StatusTypeDef SetPins(TouchScreenState state);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */


  // X-
  x_left.Port = GPIOA;
  x_left.Pin = GPIO_PIN_4;

  // X+
  x_right.Port = GPIOB;
  x_right.Pin = GPIO_PIN_10;

  // Y-
  y_up.Port = GPIOA;
  y_up.Pin = GPIO_PIN_8;

  // Y+
  y_down.Port = GPIOA;
  y_down.Pin = GPIO_PIN_1;




  LCD_Init();
  LCD_FillScreen(BLACK);
  LCD_SetTextSize(1);
  LCD_SetTextColor(GREEN, BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t str[10] = "Priv";
  //uint8_t i = 0;

  SetPins(TOUCH_DETECT);

  while (1)
  {
	  if (start_measure != 0) {

		  SetPins(TOUCH_MEASURE_X);
		  x = GetTouch_X();
		  SetPins(TOUCH_MEASURE_Y);
		  y = GetTouch_Y();
		  SetPins(TOUCH_OFF);
		  LCD_SetCursor(0,0);
		  LCD_Printf("X: %4d Y: %4d", x, y);
		  //LCD_DrawPixel(x,y,WHITE);
		  start_measure = 0;
		  SetPins(TOUCH_DETECT);
	  } else {
		  SetPins(TOUCH_OFF);
		  LCD_SetCursor(0,0);
		  LCD_Printf("NOT TOUCH               ");
		  SetPins(TOUCH_DETECT);
	  }
	  HAL_Delay(100);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
uint32_t ADC1_GetValue(uint32_t channel){

	ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
    	Error_Handler();
    }

    // start conversion
    HAL_ADC_Start(&hadc1);
    // wait until finish
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

HAL_StatusTypeDef SetGPIOState(GPIOStruct* str, GPIOState state) {
	HAL_GPIO_DeInit(str->Port, str->Pin);
	GPIO_InitTypeDef GPIO_InitStruct;

	switch (state) {
	case OUTPUT_RESET:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;                         //??
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;				//??
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(str->Port, str->Pin, GPIO_PIN_RESET);
		return HAL_OK;
	case OUTPUT_SET:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;                         //??
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;				//??
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(str->Port, str->Pin, GPIO_PIN_SET);
		return HAL_OK;
	case INPUT_PULLUP_EXTI:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0); //EXTI4 because we use PA4 (X-) to detect touch
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);			//If you use other pin, you need change EXTI
		return HAL_OK;
	case INPUT_NOPULL:
		GPIO_InitStruct.Pin = str->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
		return HAL_OK;
	case INPUT_ADC:
	    GPIO_InitStruct.Pin = str->Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(str->Port, &GPIO_InitStruct);
	    return HAL_OK;
	default:
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef ResetTouchScreenPinsState() {
	HAL_GPIO_DeInit(x_left.Port, x_left.Pin);
	HAL_GPIO_DeInit(x_right.Port, x_right.Pin);
	HAL_GPIO_DeInit(y_up.Port, y_up.Pin);
	HAL_GPIO_DeInit(y_down.Port, y_down.Pin);
	return HAL_OK;
}

HAL_StatusTypeDef SetPins(TouchScreenState state) {
	switch (state) {
		case TOUCH_OFF:
			GPIO_Init();
			return HAL_OK;
		case TOUCH_DETECT:
			SetGPIOState(&x_left, INPUT_PULLUP_EXTI);
			SetGPIOState(&x_right, INPUT_NOPULL);
			SetGPIOState(&y_up, OUTPUT_RESET);

			SetGPIOState(&y_down, INPUT_NOPULL);
			return HAL_OK;
		case TOUCH_MEASURE_X:
			SetGPIOState(&x_left, INPUT_ADC);
			SetGPIOState(&x_right, INPUT_NOPULL);
			SetGPIOState(&y_up, OUTPUT_SET);
			SetGPIOState(&y_down, OUTPUT_RESET);
			return HAL_OK;
		case TOUCH_MEASURE_Y:
			SetGPIOState(&x_left, OUTPUT_RESET);
			SetGPIOState(&x_right, OUTPUT_SET);
			SetGPIOState(&y_up, INPUT_NOPULL);
			SetGPIOState(&y_down, INPUT_ADC);
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}

uint32_t GetTouch_X() {
	int32_t x_val = (X_BORDER-(ADC1_GetValue(ADC_CHANNEL_4)-X_B)/(X_A));
	if (x_val>X_BORDER) return (uint32_t) X_BORDER;
	else if (x_val<0) return 0;
	else return (uint32_t) x_val;
}

uint32_t GetTouch_Y() {
	int32_t y_val = ((ADC1_GetValue(ADC_CHANNEL_1)-Y_B)/(Y_A));
	if (y_val>Y_BORDER) return (uint32_t) Y_BORDER;
	else if (y_val<0) return 0;
	else return (uint32_t) y_val;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

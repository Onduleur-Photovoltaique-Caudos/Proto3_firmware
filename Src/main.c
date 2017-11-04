/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay_us_DWT(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

void doLoopEnd(){
	char str[] = "\r\n";
	uint8_t statusTransmit;

	//statusTransmit = HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000); 
}

#define doInputPin doNothing

void doNothing(GPIO_TypeDef* port, uint16_t pin){
}

void doPin(GPIO_TypeDef* port, uint16_t pin)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	delay_us_DWT(100);
	
	HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_SET);
	delay_us_DWT(100);
	HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_RESET);
	delay_us_DWT(100);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	delay_us_DWT(100);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
  // Enable hi resolution counter 
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |= 0x00000001;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	char str[] = "Inverter starting\r\n";
	uint8_t statusTransmit;

	statusTransmit = HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  char inputBuffer[2];
	  uint8_t status;

	  status = HAL_UART_Receive(&huart2, (uint8_t *)inputBuffer, 1,1);
	  if (status == HAL_OK) {
		  char str1[] = "Received: ";
		  uint8_t statusTransmit;

		  statusTransmit = HAL_UART_Transmit(&huart2, (uint8_t*)str1, strlen(str1), 1000);
		  statusTransmit = HAL_UART_Transmit(&huart2, (uint8_t*)inputBuffer, 1, 1000);
		  statusTransmit = HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 1000);
	  }

	  HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_RESET);


	  doPin(Fan_GPIO_Port, Fan_Pin);
	  doPin(Disable_GPIO_Port, Disable_Pin);
	  doPin(H1_LON_GPIO_Port, H1_LON_Pin);
	  doPin(H2_HON_GPIO_Port, H2_HON_Pin);

	  doInputPin(M_IHFL_GPIO_Port, M_IHFL_Pin);
	  doInputPin(M_VIN_GPIO_Port, M_VIN_Pin);
	  doInputPin(M_V225_GPIO_Port, M_V225_Pin);

	  doPin(Led_GPIO_Port, Led_Pin);

	  doInputPin(M_V175_GPIO_Port, M_V175_Pin);
	  doInputPin(M_IOUT_GPIO_Port, M_IOUT_Pin);
	  doInputPin(M_IH1_GPIO_Port, M_IH1_Pin);
	  doInputPin(M_IH2_GPIO_Port, M_IH2_Pin);
	  doInputPin(M_VOUT1_GPIO_Port, M_VOUT1_Pin);
	  doInputPin(M_VOUT2_GPIO_Port, M_VOUT2_Pin);
	  doInputPin(M_IIN_GPIO_Port, M_IIN_Pin);

	  doPin(C_175_GPIO_Port, C_175_Pin);
	  doPin(H1_HON_GPIO_Port, H1_HON_Pin);

	  doInputPin(MI_I175_GPIO_Port, MI_I175_Pin);
	  doInputPin(MI_I225_GPIO_Port, MI_I225_Pin);

	  doPin(C_225_GPIO_Port, C_225_Pin);
	  doPin(CZ_225_GPIO_Port, CZ_225_Pin);
	  doPin(C_HFL_GPIO_Port, C_HFL_Pin);
	  doPin(CZ_175_GPIO_Port, CZ_175_Pin);
	  doPin(C_HFH_GPIO_Port, C_HFH_Pin);
	  doPin(H2_LON_GPIO_Port, H2_LON_Pin);

	  doInputPin(Psense1_GPIO_Port, Psense1_Pin);

	  doLoopEnd();
	  HAL_Delay(2);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

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

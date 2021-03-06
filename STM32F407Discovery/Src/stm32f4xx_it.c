/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
int abd_ticksL = 0;
int abd_ticksR = 0;


extern uint16_t usTime1;
extern uint16_t usTime2;
extern uint8_t usFlag;
extern UART_HandleTypeDef huart2;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
   
  /* USER CODE END EXTI9_5_IRQn 0 */  
  
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    if ( ((   GPIOE->IDR & abd_ePinL_A_Pin)  && (!(GPIOE->IDR & abd_ePinL_B_Pin))) ||
         ( (!(GPIOE->IDR & abd_ePinL_A_Pin)) &&   (GPIOE->IDR & abd_ePinL_B_Pin) )    )
    {
      abd_ticksL++;      
    }else
    {
      abd_ticksL--;       
    }
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7) == GPIO_PIN_SET)
    {                                
      usTime1 = (HAL_GetTick()*1000) + 1000 - SysTick->VAL/72;
    }
    else
    {      
      usTime2 = (HAL_GetTick()*1000) + 1000 - SysTick->VAL/72;
      usFlag =0;
      HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    }
  }
  
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */


  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */  
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
   if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    if ( ((   GPIOE->IDR & abd_ePinR_A_Pin)  && (!(GPIOE->IDR & abd_ePinR_B_Pin))) ||
         ( (!(GPIOE->IDR & abd_ePinR_A_Pin)) &&   (GPIOE->IDR & abd_ePinR_B_Pin) )    )
    {
      abd_ticksR++;      
    }else
    {
      abd_ticksR--;       
    } 
  }
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HardFault_Handler(void)
{
  int il;
   while(1)
   {
     il=1;
   }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* *** <<< DO NOT MODIFY OR REMOVE THIS LINE - BEGIN >>> *** */
//;=================================================================================================
//;
//; Copyright(C) HH Corporation 2016
//; All rights reserved by HH Corporation
//;
//; [Filename]  : $Workfile:   CDL_Printf.c  $
//; [Revision]  : $Revision:   1.1  $
//; [Modified]  : $Date:   May 18 2016   $
//; [CreateDate]: 
//; [ModuleName]: 
//; [Device]    : 
//; [Author]    : 
//; [Abstract]  : 
//; [History]   : $Log:   


//;*************************************************************************************************
//;	Include Files Area
//;*************************************************************************************************
#include "CDL_Printf.h"
#include <stdarg.h>
#include <stdint.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END 0 */

void UART_SendData(USART_TypeDef* USARTx, uint16_t Data);


void cdl_Printf(char const * fmt)
{
  osSemaphoreWait(ID_SEM_PRINTF,100);
  printf(fmt);
  osSemaphoreRelease(ID_SEM_PRINTF);
}

void cdl_Putc(uint8_t ptr)
{
    uint8_t sizec = 1;
	
    HAL_UART_Transmit(&huart2, &ptr,sizec,1);	
 
}

void cdl_Puts(uint8_t *ptr)
{
  while(*ptr){
    HAL_UART_Transmit(&huart2, ptr++,1,1);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
  }
  
}

uint8_t cdl_Getc(void)
{
    uint8_t ch;
    uint8_t sizec = 1;
    HAL_StatusTypeDef ret;
 
    ret = HAL_UART_Receive(&huart2, &ch, sizec, 1);
	//bsl_LIN_UART_Read(&ch,sizec);
    if(ret == HAL_OK)
      return ch;
    
    return 0xff;
}
 


PUTCHAR_PROTOTYPE
{
  
  
  UART_SendData(USART2, (uint8_t) ch);

  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

  
  return ch;
}

void UART_SendData(USART_TypeDef* UARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  UARTx->DR = (Data & (uint16_t)0x01FF);
}

 
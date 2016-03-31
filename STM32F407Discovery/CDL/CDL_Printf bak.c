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
void cdl_Puts(uint8_t *ptr);
void cdl_Putc(uint8_t ptr);
void cdl_DoPrint(const uint8_t *fmt, va_list ap);
void my_printf_helper(const char *fmt, va_list args);

extern osSemaphoreId ID_SEM_PRINTF;

 void cdl_Printf(const uint8_t *fmt,...)
{
  va_list ap;
 
  va_start(ap, fmt);
  
//  osSemaphoreWait(ID_SEM_PRINTF,100);
  cdl_DoPrint(fmt, ap);
  //my_printf_helper(fmt,ap);
 // osSemaphoreRelease(ID_SEM_PRINTF);
  
  va_end(ap);
  printf("12");

}

void my_printf_helper(const char *fmt, va_list args)
{
 char *ptr=(char *)fmt;
 volatile long  value;

 while(*ptr)
 {
  switch(*ptr)
  {
   case '\\':
    if(*++ptr)
     ptr++;
    continue;

   case '%':
    switch(*++ptr)
    {
     case NULL:
      continue;

     case 's':
      printf("%s", va_arg(args, char *));
      break;

     case 'd':
      //printf("%d", va_arg(args, int));
       value = va_arg(args, int);
      break;
    }
    ptr++;

   default:
    cdl_Putc(*ptr++);
  }
 }
}

void cdl_DoPrint(const uint8_t *pfmt, va_list ap)
{
	char  ch;
	char  i;
	long  value;
	uint8_t   fl_zero;
	uint8_t  fl_len;
	uint8_t  cnt;
	uint32_t mask = 1;
	long tempvalue=0;
	char  hexch;
	char *ptr;
	uint8_t HexStr[16] = "0123456789ABCDEF";
        char *fmt=(char *)pfmt;
 
	while (1)
	{
		//----- Find Formatter % -----
		switch (ch = *fmt++)
		{
			case 0:
				return;
 
			case '%':
				if (*fmt != '%')
				{
					break;
				}
				fmt++;
 
			default:
				cdl_Putc(ch);
				continue;
		}
 
		//----- Get Count -------------
		fl_zero = 0;    	
		cnt = 0;
		ch = *fmt++;
 
		if (ch == '0')
		{
			fl_zero = 1;
			ch = *fmt++;
			cnt = ch - '0';
			ch = *fmt++;
		}
		else if (ch >= '0' && ch <= '9')
		{
			cnt = ch - '0';
			ch = *fmt++;
		}
 
		//----- Get char(B) / int / long(L) ----------------
		fl_len = 2;
 
		switch (ch)
		{
			case 'l':
			case 'L':
				ch = *fmt++;
				fl_len = 4;
				break;
 
			case 'b':
			case 'B':
				ch = *fmt++;
				fl_len = 1;
				break;
		}
 
		//----- Get Type Discriptor -----
		switch (ch)
		{
			case 'd':
			case 'u':
				switch (fl_len)
				{
					case 1:
						if (ch == 'd')
						{
							value = (char)va_arg(ap, char);
						}
						else
						{
							value = (uint8_t)va_arg(ap, uint8_t);
						}
						break;
 
					case 2:
						if (ch == 'd')
						{
							value = (int)va_arg(ap, int);
						}
						else
						{
							value = (uint16_t)va_arg(ap, uint16_t);
						}
						break;
 
					case 4:
						if (ch == 'd')
						{
							value = va_arg(ap, double);
						}
						else
						{
							value = (uint32_t)va_arg(ap, uint32_t);
						}
						break;
				}
 
				if (value < 0)
				{
					cdl_Putc('-');
					value = value * (-1);
				}
 
				if (cnt == 0)
				{
					if (value == 0)
					{
						cdl_Putc('0');
						continue;
					}
 
					for (cnt = 0, mask = 1; cnt < 10; cnt++)
					{
						if ((value / mask) == 0)
						{
							break;
						}
						mask = mask * 10;
					}
				}
 
				for (i = 0, mask = 1; i < cnt - 1; i++)
				{
					mask = mask * 10;
				}
 
				while(1)
				{
					ch = (char)(value / mask) + '0';
					if (ch == '0' && fl_zero == 0 && mask != 1)
					{
						ch=' ';
					}
					else
					{
						fl_zero = 1;
					}
					cdl_Putc(ch);
 
					value = value % (mask);
					mask = mask / 10;
 
					if (mask == 0)
					{
						break;
					}
				}
				continue;
 
			case 'x':
			case 'X':
				switch (fl_len)
				{
					case 1:
						value = (uint8_t)va_arg(ap, uint8_t);
						break;
 
					case 2:
						value = (uint16_t)va_arg(ap, uint16_t);
						break;
 
					case 4:
						value = (uint32_t)va_arg(ap, uint32_t);
						break;
				}
 
				if (cnt == 0)
				{
					//cnt = fl_len * 2;
					tempvalue = value;
					do
					{
						tempvalue = (value >> 4*(++cnt));
					} while (tempvalue > 0);
				}
 
				for (i = 0; i < cnt; i++)
				{
					hexch = HexStr[(value >> (cnt - i - 1) * 4) & 0x000f];
					cdl_Putc(hexch);
				}
				continue;
 
			case 's':
				ptr = (char *)va_arg(ap, char*);
				while (*ptr!= '\0')
				{
					cdl_Putc(*ptr++);
				}
				continue;
 
			case 'c':
				value = va_arg(ap, int);
				cdl_Putc((uint8_t)value);
				continue;
 
			default:
				value = (uint16_t)va_arg(ap, int);
				continue;
		}
	}
}

void cdl_Puts(uint8_t *ptr)
{
  while(*ptr){
    HAL_UART_Transmit(&huart2, ptr++,1,1);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
  }
  
}

void cdl_Putc(uint8_t ptr)
{
    uint8_t sizec = 1;
	
    HAL_UART_Transmit(&huart2, &ptr,sizec,1);	
 
}
 


PUTCHAR_PROTOTYPE
{
 
  UART_SendData(USART2, (uint8_t) ch);

  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

  return ch;
}

void UART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  UARTx->DR = (Data & (uint16_t)0x01FF);
}

 

#ifndef __CDL_PRINTF_H
#define __CDL_PRINTF_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

uint8_t cdl_Getc(void);
void cdl_Putc(uint8_t ptr);
void cdl_Puts(uint8_t *ptr);
//void cdl_Printf(uint8_t* fmt);
void cdl_Printf(char const * fmt);


#define DebugMsg

#endif /* __CDL_PRINTF_H */
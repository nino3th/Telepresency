#ifndef __MAIN_H
#define __MAIN_H

#include "APP_MotorDriver.h"

extern UART_HandleTypeDef huart2;
extern osSemaphoreId ID_SEM_PRINTF;
extern osSemaphoreId ID_SEM_POS_CTRL;
extern uint8_t taskFlag;


#endif /* __MAIN_H */
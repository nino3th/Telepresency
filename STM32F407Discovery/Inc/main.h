#ifndef __MAIN_H
#define __MAIN_H

#include "APP_MotorDriver.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern osSemaphoreId ID_SEM_PRINTF;
extern osSemaphoreId ID_SEM_POS_CTRL;
extern uint8_t taskFlag;

extern osThreadId STMTaskHandle;
extern osThreadId MotorDriveTaskHandle;
extern osTimerId EncoderTimerID;



#endif /* __MAIN_H */
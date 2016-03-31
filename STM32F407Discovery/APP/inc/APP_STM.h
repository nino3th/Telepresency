#ifndef __APP_STM_H
#define __APP_STM_H

#include "cmsis_os.h"
#include "APP_MotorDriver.h"
#include "string.h"
//#include "master.h"
//


void stm_loop(void const * argument);
uint32_t GetWords(char *p, char *words[], unsigned int max);


#endif /* __APP_STM_H */
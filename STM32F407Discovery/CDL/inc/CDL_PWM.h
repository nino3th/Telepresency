#ifndef __CDL_PWM_H
#define __CDL_PWM_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


extern void cdl_PWM_Init(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint32_t Channel);
extern void cdl_PWM_Level(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint16_t duty,uint32_t Channel);
extern void cdl_PWM_Switch(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint8_t OnOff,uint32_t Channel);


#endif /* __CDL_PWM_H */



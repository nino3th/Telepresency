#include "CDL_PWM.h"



void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void cdl_PWM_Init(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint32_t Channel)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  
  HAL_TIM_PWM_Init(htim);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig);

   
  HAL_TIM_PWM_ConfigChannel(htim, sConfigOC, Channel);
 

 // HAL_TIM_MspPostInit(htim);
   GPIO_InitTypeDef GPIO_InitStruct;

  
    /**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


}


void cdl_PWM_Level(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint16_t duty,uint32_t Channel)
{
    sConfigOC->Pulse = duty;
    HAL_TIM_PWM_ConfigChannel(htim, sConfigOC, Channel);
    
}

void cdl_PWM_Switch(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC,uint8_t OnOff,uint32_t Channel)
{
    
  if(OnOff)
    HAL_TIM_PWM_Start(htim,Channel);
  else
    HAL_TIM_PWM_Stop(htim,Channel);
     
  
}
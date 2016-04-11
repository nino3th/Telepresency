
#include "APP_STM.h"
//#include "mavlink/Arlobot/mavlink.h"

void broadcastEncoder();
void Delay_us(int t);

uint16_t UltraSoundTrig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void sonar(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t d =0;
uint16_t usTime = 0;
uint16_t usTime1 = 0;
uint16_t usTime2 = 0;
uint8_t usFlag  =0;




void stm_loop(void const * argument)
{
  uint32_t t_upd_odom = xTaskGetTickCount();
  uint16_t d;
 
  cdl_Printf("Create STM task\r\n");
  
  while(1)
  {
    if ((xTaskGetTickCount() - t_upd_odom) > 100) 
    {
        t_upd_odom = xTaskGetTickCount();                 
        drive_update_odom();
   
    //    send_odom_info();
    //    d = UltraSoundTrig(GPIOE,GPIO_PIN_7);
    //    cdl_Printf("d = %d \r\n",d);
     //   sonar(GPIOE,GPIO_PIN_7);
    }
    
    
  //  broadcastEncoder();
  }
 
}
void sonar(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct;
    
  HAL_GPIO_WritePin(GPIOE, GPIO_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_Pin;  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);  
  
  int i=24;
  while(i--)
  {
    if(i&0x01)
     HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
     else
      HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
     Delay_us(27);
  }     
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void Delay_us(int t)
{
  uint32_t t1,t2;
  
  t1 = (HAL_GetTick()*1000) + 1000 - SysTick->VAL/72;
  while(1)
  {
    t2 = (HAL_GetTick()*1000) + 1000 - SysTick->VAL/72;
      if((t2 - t1)>t)
        break;
  }
}

uint16_t UltraSoundTrig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  int i=0;
  GPIO_InitTypeDef GPIO_InitStruct;
    
  HAL_GPIO_WritePin(GPIOE, GPIO_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_Pin;  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
  
  osDelay(5);
  
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
  osDelay(1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  usFlag =1;
  while(usFlag)
  {
    osDelay(1);
    i++;
    if(i>100)
      break;
  }
  usTime = usTime2 - usTime1;
  d = usTime/29/2;
  
  return d;
}



void broadcastEncoder()
{
    
    static int t = 0;
    if (xTaskGetTickCount() - t > 100) {
        t = xTaskGetTickCount();
        //send_encoder_info();
    }
}

 
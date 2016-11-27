#include "stm32f4xx_hal.h"
#include "ANO_DT.h"
#include "pid.h"
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
}
extern xControl ctrl;
extern uint8_t b;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  if(huart->Instance==USART3){
    ANO_DT_Data_Receive_Prepare(b);
    //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
    HAL_UART_Receive_IT(huart,&b,1);
    if(b==0x5a)
    {
      ctrl.power+=50;
    }
    if(b==0xa5)
    {
      ctrl.power-=50;
      if(ctrl.power<=0)
        ctrl.power=0;
    }
    if(b==0x44)
    {
      ctrl.mode=1;
    }
    if(b==0x77)
    {
      ctrl.mode=0;
    }
  }
}

void HAL_SYSTICK_Callback(void)
{
  //ANO_DT_Data_Exchange();
}
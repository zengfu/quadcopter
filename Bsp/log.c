#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

#include <stdio.h>

int fputc(int ch, FILE *f)
{
 
  uint8_t a;
  a=(uint8_t)ch;
  HAL_UART_Transmit(&huart1, &a, 1, 100);
  return ch;
}


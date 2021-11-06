#ifndef __ANO_H
#include "Peripherals_can.h" 
#include "can.h"
#include "pid.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*(char *)(&dwTemp) + 1)
#define BYTE2(dwTemp) (*(char *)(&dwTemp) + 2)
#define BYTE3(dwTemp) (*(char *)(&dwTemp) + 3)

#endif












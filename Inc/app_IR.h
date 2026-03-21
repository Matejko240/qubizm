#ifndef APP_IR_H
#define APP_IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef App_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_IR_H */

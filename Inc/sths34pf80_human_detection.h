#ifndef STHS34PF80_HUMAN_DETECTION_H
#define STHS34PF80_HUMAN_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

typedef struct
{
  uint8_t valid;
  uint8_t human_detected;
  uint8_t presence_flag;
  uint8_t motion_flag;
  int16_t presence_raw;
  int16_t motion_raw;
  int16_t object_raw;
  int16_t ambient_raw;
  int32_t object_celsius_x100;
} STHS34PF80_HumanDetectionState_t;

HAL_StatusTypeDef STHS34PF80_HumanDetection_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef STHS34PF80_HumanDetection_Process(void);
void STHS34PF80_HumanDetection_GetState(STHS34PF80_HumanDetectionState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* STHS34PF80_HUMAN_DETECTION_H */

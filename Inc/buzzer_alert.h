#ifndef BUZZER_ALERT_H
#define BUZZER_ALERT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

typedef enum
{
  BUZZER_ALERT_LEVEL_OFF = 0,
  BUZZER_ALERT_LEVEL_FAR,
  BUZZER_ALERT_LEVEL_MEDIUM,
  BUZZER_ALERT_LEVEL_NEAR
} BuzzerAlert_Level_t;

void BuzzerAlert_Init(void);
void BuzzerAlert_SetLevel(BuzzerAlert_Level_t level);
void BuzzerAlert_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_ALERT_H */

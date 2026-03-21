#include "buzzer_alert.h"

#include "main.h"

/*
 * Modul zaklada aktywny buzzer 5 V z wbudowanym generatorem.
 * STM32 steruje tylko tranzystorem wlaczajacym buzzer, dlatego
 * "moc" alarmu realizujemy tylko rytmem ostrzegania:
 * - FAR: spokojne, rzadkie pikniecia
 * - MEDIUM: szybsze ostrzeganie
 * - NEAR: ciagly alarm
 */

#define BUZZER_FAR_ON_MS       80U
#define BUZZER_FAR_OFF_MS     920U
#define BUZZER_MEDIUM_ON_MS   140U
#define BUZZER_MEDIUM_OFF_MS  260U

static BuzzerAlert_Level_t g_buzzer_level = BUZZER_ALERT_LEVEL_OFF;
static uint8_t g_buzzer_on = 0U;
static uint32_t g_phase_tick = 0U;

static void BuzzerAlert_WritePin(uint8_t enabled);

void BuzzerAlert_Init(void)
{
  g_buzzer_level = BUZZER_ALERT_LEVEL_OFF;
  g_buzzer_on = 0U;
  g_phase_tick = HAL_GetTick();
  BuzzerAlert_WritePin(0U);
}

void BuzzerAlert_SetLevel(BuzzerAlert_Level_t level)
{
  if (level == g_buzzer_level)
  {
    return;
  }

  g_buzzer_level = level;
  g_phase_tick = HAL_GetTick();

  if (level == BUZZER_ALERT_LEVEL_OFF)
  {
    g_buzzer_on = 0U;
    BuzzerAlert_WritePin(0U);
    return;
  }

  g_buzzer_on = 1U;
  BuzzerAlert_WritePin(1U);
}

void BuzzerAlert_Process(void)
{
  uint32_t now;
  uint32_t interval_ms;

  now = HAL_GetTick();

  if (g_buzzer_level == BUZZER_ALERT_LEVEL_OFF)
  {
    BuzzerAlert_WritePin(0U);
    g_buzzer_on = 0U;
    return;
  }

  if (g_buzzer_level == BUZZER_ALERT_LEVEL_NEAR)
  {
    BuzzerAlert_WritePin(1U);
    g_buzzer_on = 1U;
    return;
  }

  if (g_buzzer_level == BUZZER_ALERT_LEVEL_FAR)
  {
    interval_ms = g_buzzer_on ? BUZZER_FAR_ON_MS : BUZZER_FAR_OFF_MS;
  }
  else
  {
    interval_ms = g_buzzer_on ? BUZZER_MEDIUM_ON_MS : BUZZER_MEDIUM_OFF_MS;
  }

  if ((now - g_phase_tick) < interval_ms)
  {
    return;
  }

  g_phase_tick = now;
  g_buzzer_on = (uint8_t)!g_buzzer_on;
  BuzzerAlert_WritePin(g_buzzer_on);
}

static void BuzzerAlert_WritePin(uint8_t enabled)
{
#if defined(BUZZER_GPIO_Port) && defined(BUZZER_Pin)
  HAL_GPIO_WritePin(BUZZER_GPIO_Port,
                    BUZZER_Pin,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)enabled;
#endif
}

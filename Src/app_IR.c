#include "app_IR.h"

#include "buzzer_alert.h"
#include "rgb_led.h"
#include "sths34pf80_human_detection.h"

#include <stdio.h>

static I2C_HandleTypeDef *g_app_i2c = NULL;
static uint8_t g_sensor_ready = 0;
static uint32_t g_last_init_attempt_tick = 0;

#define APP_PROCESS_PERIOD_MS       100U
#define APP_SENSOR_RETRY_PERIOD_MS 1000U
#define APP_BUZZER_LOW_TOBJ_X100      25
#define APP_BUZZER_MEDIUM_TOBJ_X100   50
#define APP_BUZZER_NEAR_TOBJ_X100    75

/*
 * Progi buzzera sa teraz oparte o Tobj, czyli wzgledny sygnal IR
 * przeliczony na jednostki rownowazne degC.
 * To nadal nie jest prawdziwa odleglosc w metrach ani temperatura ciala,
 * ale w praktyce dobrze nadaje sie do stref ostrzegania.
 *
 * Progi:
 * - LOW/FAR gdy Tobj > 0.50
 * - MEDIUM gdy Tobj > 0.75
 * - HARD/NEAR gdy Tobj > 1.00
 *
 * Dla aktywnego buzzera 5 V z generatorem nie zmieniamy tonu audio,
 * tylko rytm alarmu:
 * - FAR: spokojne, rzadkie pikniecia
 * - MEDIUM: szybsze ostrzeganie
 * - NEAR: ciagly alarm
 */

static BuzzerAlert_Level_t App_GetBuzzerLevel(const STHS34PF80_HumanDetectionState_t *state);
static RgbLed_Color_t App_GetLedColor(const STHS34PF80_HumanDetectionState_t *state,
                                      BuzzerAlert_Level_t buzzer_level,
                                      uint8_t sensor_ready);

HAL_StatusTypeDef App_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
  (void)huart;
  g_app_i2c = hi2c;
  g_sensor_ready = 0;
  g_last_init_attempt_tick = 0;
  BuzzerAlert_Init();
  RgbLed_Init();
  RgbLed_SetColor(RGB_LED_COLOR_BLUE);
  return HAL_OK;
}

void App_Process(void)
{
  static uint32_t last_process_tick = 0;
  uint32_t now;
  HAL_StatusTypeDef status;
  STHS34PF80_HumanDetectionState_t sensor_state;

  now = HAL_GetTick();
  if ((now - last_process_tick) < APP_PROCESS_PERIOD_MS)
  {
    return;
  }

  last_process_tick = now;

  if (g_sensor_ready == 0U)
  {
    BuzzerAlert_SetLevel(BUZZER_ALERT_LEVEL_OFF);
    RgbLed_SetColor(RGB_LED_COLOR_BLUE);

    if ((now - g_last_init_attempt_tick) < APP_SENSOR_RETRY_PERIOD_MS)
    {
      BuzzerAlert_Process();
      return;
    }

    g_last_init_attempt_tick = now;
    status = STHS34PF80_HumanDetection_Init(g_app_i2c);
    if (status == HAL_OK)
    {
      g_sensor_ready = 1U;
      RgbLed_SetColor(RGB_LED_COLOR_OFF);
    }
    else
    {
      printf("Sensor offline, retry in %lu ms\r\n", (unsigned long)APP_SENSOR_RETRY_PERIOD_MS);
    }
    BuzzerAlert_Process();
    return;
  }

  status = STHS34PF80_HumanDetection_Process();
  if (status == HAL_OK)
  {
    BuzzerAlert_Level_t buzzer_level;

    STHS34PF80_HumanDetection_GetState(&sensor_state);
    buzzer_level = App_GetBuzzerLevel(&sensor_state);
    BuzzerAlert_SetLevel(buzzer_level);
    RgbLed_SetColor(App_GetLedColor(&sensor_state, buzzer_level, g_sensor_ready));
  }
  if ((status != HAL_OK) && (status != HAL_BUSY))
  {
    g_sensor_ready = 0U;
    BuzzerAlert_SetLevel(BUZZER_ALERT_LEVEL_OFF);
    RgbLed_SetColor(RGB_LED_COLOR_BLUE);
    printf("Sensor communication lost, retrying init...\r\n");
  }

  BuzzerAlert_Process();
}

static BuzzerAlert_Level_t App_GetBuzzerLevel(const STHS34PF80_HumanDetectionState_t *state)
{
  if ((state == NULL) || (state->valid == 0U) || (state->human_detected == 0U))
  {
    return BUZZER_ALERT_LEVEL_OFF;
  }

  if (state->object_celsius_x100 > APP_BUZZER_NEAR_TOBJ_X100)
  {
    return BUZZER_ALERT_LEVEL_NEAR;
  }

  if (state->object_celsius_x100 > APP_BUZZER_MEDIUM_TOBJ_X100)
  {
    return BUZZER_ALERT_LEVEL_MEDIUM;
  }

  if (state->object_celsius_x100 > APP_BUZZER_LOW_TOBJ_X100)
  {
    return BUZZER_ALERT_LEVEL_FAR;
  }

  return BUZZER_ALERT_LEVEL_OFF;
}

static RgbLed_Color_t App_GetLedColor(const STHS34PF80_HumanDetectionState_t *state,
                                      BuzzerAlert_Level_t buzzer_level,
                                      uint8_t sensor_ready)
{
  if (sensor_ready == 0U)
  {
    return RGB_LED_COLOR_BLUE;
  }

  if ((state == NULL) || (state->valid == 0U) || (state->human_detected == 0U))
  {
    return RGB_LED_COLOR_OFF;
  }

  switch (buzzer_level)
  {
    case BUZZER_ALERT_LEVEL_FAR:
      return RGB_LED_COLOR_GREEN;

    case BUZZER_ALERT_LEVEL_MEDIUM:
      return RGB_LED_COLOR_YELLOW;

    case BUZZER_ALERT_LEVEL_NEAR:
      return RGB_LED_COLOR_RED;

    case BUZZER_ALERT_LEVEL_OFF:
    default:
      return RGB_LED_COLOR_OFF;
  }
}

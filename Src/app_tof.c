/**
  ******************************************************************************
  * @file    app_tof.c
  * @brief   Simple VL53L8A1 grouped-column averaging app.
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

#include "app_tof.h"

#include <stdio.h>
#include <string.h>

#include "53l8a1_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "main.h"
#include "stm32l4xx_nucleo.h"

#define TOF_UPDATE_PERIOD_MS         100U
#define TOF_TIMING_BUDGET_MS         30U
#define TOF_RANGING_FREQUENCY        10U

#define TOF_MATRIX_SIZE              8U
#define TOF_ZONE_COUNT               (TOF_MATRIX_SIZE * TOF_MATRIX_SIZE)
#define GROUP_COUNT                  4U
#define GROUP_WIDTH                  2U
#define OUTPUT_BIN_COUNT             8U
#define AVG_WINDOW_SIZE              5U

#define MIN_VALID_DISTANCE_MM        100
#define MAX_VALID_DISTANCE_MM        4000
#define DEFAULT_OUTPUT_DISTANCE_CM   400U

typedef struct
{
  uint32_t tick_ms;
  uint8_t tof_ok;
  int32_t tof_dist_mm[TOF_ZONE_COUNT];
} tof_sample_t;

static uint8_t g_ready = 0U;
static uint32_t g_last_sample_ms = 0U;
static volatile uint8_t g_reset_requested = 0U;
static uint32_t g_last_button_press_ms = 0U;
static RANGING_SENSOR_ProfileConfig_t g_tof_profile;
volatile uint8_t ToF_EventDetected = 0U;

static uint16_t g_group_window_mm[GROUP_COUNT][AVG_WINDOW_SIZE];
static uint8_t g_group_window_valid[GROUP_COUNT][AVG_WINDOW_SIZE];
static uint8_t g_window_head = 0U;
static uint8_t g_window_count = 0U;

static void init_console(void);
static void init_tof(void);
static void reset_group_window(void);
static void collect_sample(tof_sample_t *sample);
static void sample_and_stream(void);
static void update_group_average_and_print(const tof_sample_t *sample);

void MX_TOF_Init(void)
{
  init_console();
  (void)BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  reset_group_window();
  init_tof();
  g_ready = 1U;
}

void MX_TOF_Process(void)
{
  if (g_ready == 0U)
  {
    return;
  }

  if (g_reset_requested != 0U)
  {
    g_reset_requested = 0U;
    reset_group_window();
  }

  if ((HAL_GetTick() - g_last_sample_ms) < TOF_UPDATE_PERIOD_MS)
  {
    return;
  }

  g_last_sample_ms = HAL_GetTick();
  sample_and_stream();
}

static void init_console(void)
{
  (void)BSP_COM_Init(COM1);
}

static void init_tof(void)
{
  int32_t status;

  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);
  if (status != BSP_ERROR_NONE)
  {
    return;
  }

  g_tof_profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  g_tof_profile.TimingBudget = TOF_TIMING_BUDGET_MS;
  g_tof_profile.Frequency = TOF_RANGING_FREQUENCY;
  g_tof_profile.EnableAmbient = 0U;
  g_tof_profile.EnableSignal = 0U;

  if (VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &g_tof_profile) != BSP_ERROR_NONE)
  {
    return;
  }

  if (VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS) != BSP_ERROR_NONE)
  {
    return;
  }
}

static void reset_group_window(void)
{
  memset(g_group_window_mm, 0, sizeof(g_group_window_mm));
  memset(g_group_window_valid, 0, sizeof(g_group_window_valid));
  g_window_head = 0U;
  g_window_count = 0U;
}

static void collect_sample(tof_sample_t *sample)
{
  RANGING_SENSOR_Result_t result;
  int32_t status = BSP_ERROR_COMPONENT_FAILURE;
  uint32_t zone;

  if (sample == NULL)
  {
    return;
  }

  memset(sample, 0, sizeof(*sample));
  sample->tick_ms = HAL_GetTick();
  sample->tof_ok = 1U;

  status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &result);
  if (status != BSP_ERROR_NONE)
  {
    sample->tof_ok = 0U;
  }

  for (zone = 0U; zone < TOF_ZONE_COUNT; zone++)
  {
    int32_t dist = -1;
    if ((sample->tof_ok != 0U) &&
        (zone < result.NumberOfZones) &&
        (result.ZoneResult[zone].NumberOfTargets > 0U))
    {
      dist = (int32_t)result.ZoneResult[zone].Distance[0];
    }
    sample->tof_dist_mm[zone] = dist;
  }
}

static void sample_and_stream(void)
{
  tof_sample_t sample;
  collect_sample(&sample);
  update_group_average_and_print(&sample);
}

static void update_group_average_and_print(const tof_sample_t *sample)
{
  uint16_t grouped_cm[GROUP_COUNT];
  uint32_t group;

  if (sample == NULL)
  {
    return;
  }

  for (group = 0U; group < GROUP_COUNT; group++)
  {
    uint32_t frame_sum_mm = 0U;
    uint32_t frame_count = 0U;
    uint32_t row;
    uint32_t col;
    uint32_t window_sum_mm = 0U;
    uint32_t window_count = 0U;

    for (row = 0U; row < TOF_MATRIX_SIZE; row++)
    {
      for (col = 0U; col < GROUP_WIDTH; col++)
      {
        const uint32_t matrix_col = (group * GROUP_WIDTH) + col;
        const int32_t dist_mm = sample->tof_dist_mm[(row * TOF_MATRIX_SIZE) + matrix_col];
        if ((dist_mm >= MIN_VALID_DISTANCE_MM) && (dist_mm <= MAX_VALID_DISTANCE_MM))
        {
          frame_sum_mm += (uint32_t)dist_mm;
          frame_count++;
        }
      }
    }

    if (frame_count > 0U)
    {
      g_group_window_mm[group][g_window_head] = (uint16_t)((frame_sum_mm + (frame_count / 2U)) / frame_count);
      g_group_window_valid[group][g_window_head] = 1U;
    }
    else
    {
      g_group_window_mm[group][g_window_head] = 0U;
      g_group_window_valid[group][g_window_head] = 0U;
    }

    for (col = 0U; col < AVG_WINDOW_SIZE; col++)
    {
      if (g_group_window_valid[group][col] != 0U)
      {
        window_sum_mm += g_group_window_mm[group][col];
        window_count++;
      }
    }

    if (window_count > 0U)
    {
      const uint32_t avg_mm = (window_sum_mm + (window_count / 2U)) / window_count;
      grouped_cm[group] = (uint16_t)((avg_mm + 5U) / 10U);
    }
    else
    {
      grouped_cm[group] = DEFAULT_OUTPUT_DISTANCE_CM;
    }
  }

  if (g_window_count < AVG_WINDOW_SIZE)
  {
    g_window_count++;
  }
  g_window_head = (uint8_t)((g_window_head + 1U) % AVG_WINDOW_SIZE);

  printf("[");
  for (group = 0U; group < GROUP_COUNT; group++)
  {
    if (group != 0U)
    {
      printf(",");
    }
    printf("%u,%u", (unsigned int)grouped_cm[group], (unsigned int)grouped_cm[group]);
  }
  printf("]\r\n");
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  uint32_t now;

  if (Button != BUTTON_KEY)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - g_last_button_press_ms) < 250U)
  {
    return;
  }

  g_last_button_press_ms = now;
  g_reset_requested = 1U;
}

#ifdef __cplusplus
}
#endif
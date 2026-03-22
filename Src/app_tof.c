/**
  ******************************************************************************
  * @file    app_tof.c
  * @brief   VL53L8A1 ranging app with retry/diagnostics and grouped output.
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
#include "stm32l4xx_nucleo_bus.h"

#define TOF_UPDATE_PERIOD_MS         100U
#define TOF_TIMING_BUDGET_MS         30U
#define TOF_RANGING_FREQUENCY        10U
#define TOF_INIT_RETRY_PERIOD_MS     1000U

#define TOF_PWR_OFF_MS               10U
#define TOF_PWR_ON_STABILIZE_MS      10U
#define TOF_LPN_LOW_MS               10U
#define TOF_BOOT_WAIT_MS             100U

#define TOF_MATRIX_SIZE              8U
#define TOF_ZONE_COUNT               (TOF_MATRIX_SIZE * TOF_MATRIX_SIZE)
#define GROUP_COUNT                  4U
#define GROUP_WIDTH                  2U
#define AVG_WINDOW_SIZE              5U

#define MIN_VALID_DISTANCE_MM        100
#define MAX_VALID_DISTANCE_MM        4000
#define DEFAULT_OUTPUT_DISTANCE_CM   400U

typedef struct
{
  uint8_t tof_ok;
  int32_t tof_dist_mm[TOF_ZONE_COUNT];
} tof_sample_t;

static uint8_t g_ready = 0U;
static uint8_t g_tof_started = 0U;
static uint8_t g_distance_mm_64_fresh = 0U;
static uint32_t g_last_sample_ms = 0U;
static volatile uint8_t g_reset_requested = 0U;
static uint32_t g_last_button_press_ms = 0U;
static uint32_t g_tof_last_init_attempt_tick = 0U;
static RANGING_SENSOR_ProfileConfig_t g_tof_profile;
volatile uint8_t ToF_EventDetected = 0U;

static uint16_t g_distance_mm_64[TOF_ZONE_COUNT];
static uint16_t g_group_window_mm[GROUP_COUNT][AVG_WINDOW_SIZE];
static uint8_t g_group_window_valid[GROUP_COUNT][AVG_WINDOW_SIZE];
static uint8_t g_window_head = 0U;

static void init_console(void);
static uint8_t init_tof(void);
static void reset_group_window(void);
static void collect_sample(tof_sample_t *sample);
static void sample_and_stream(void);
static void update_group_average_and_print(const tof_sample_t *sample);
static void TOF_ResetCenterSensor(void);
static void TOF_LogInitDiagnostics(void);
static void TOF_PublishNoTargetFrame(void);
static void TOF_UpdateDistanceMm64FromSample(const tof_sample_t *sample);

void MX_TOF_Init(void)
{
  init_console();
  (void)BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  reset_group_window();

  memset(g_distance_mm_64, 0, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 0U;
  g_tof_started = 0U;
  g_tof_last_init_attempt_tick = 0U;
  ToF_EventDetected = 0U;

  (void)init_tof();
  g_ready = 1U;
}

void MX_TOF_Process(void)
{
  uint8_t tof_data_ready;

  if (g_ready == 0U)
  {
    return;
  }

  if (g_reset_requested != 0U)
  {
    g_reset_requested = 0U;
    reset_group_window();
  }

  if (g_tof_started == 0U)
  {
    if ((HAL_GetTick() - g_tof_last_init_attempt_tick) >= TOF_INIT_RETRY_PERIOD_MS)
    {
      (void)init_tof();
    }
    return;
  }

  if ((HAL_GetTick() - g_last_sample_ms) < TOF_UPDATE_PERIOD_MS)
  {
    return;
  }

  tof_data_ready = 0U;
  if (ToF_EventDetected != 0U)
  {
    tof_data_ready = 1U;
  }
  else if (HAL_GPIO_ReadPin(TOF_INT_EXTI_PORT, TOF_INT_EXTI_PIN) == GPIO_PIN_RESET)
  {
    /* INT is active low. Read once even if the EXTI edge was missed. */
    tof_data_ready = 1U;
  }

  if (tof_data_ready == 0U)
  {
    return;
  }

  g_last_sample_ms = HAL_GetTick();
  sample_and_stream();
}

uint8_t TOF_GetDistanceMm64(uint16_t out_distance_mm_64[64])
{
  if (out_distance_mm_64 == NULL)
  {
    return 0U;
  }

  if (g_distance_mm_64_fresh == 0U)
  {
    return 0U;
  }

  memcpy(out_distance_mm_64, g_distance_mm_64, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 0U;
  return 1U;
}

static void init_console(void)
{
  (void)BSP_COM_Init(COM1);
}

static uint8_t init_tof(void)
{
  int32_t status;
  uint32_t id;

  g_tof_last_init_attempt_tick = HAL_GetTick();
  g_tof_started = 0U;
  ToF_EventDetected = 0U;

  TOF_ResetCenterSensor();
  TOF_LogInitDiagnostics();

  printf("\r\n");
  printf("53L8A1 Simple Ranging demo application\r\n");
  printf("Sensor initialization...\r\n");

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_Init failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  status = VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &id);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_ReadID failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  g_tof_profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  g_tof_profile.TimingBudget = TOF_TIMING_BUDGET_MS;
  g_tof_profile.Frequency = TOF_RANGING_FREQUENCY;
  g_tof_profile.EnableAmbient = 0U;
  g_tof_profile.EnableSignal = 0U;

  status = VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &g_tof_profile);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_ConfigProfile failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  status = VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_Start failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  ToF_EventDetected = 0U;
  g_tof_started = 1U;
  printf("TOF ready, sensor ID=0x%lX\r\n", (unsigned long)id);
  return 1U;
}

static void reset_group_window(void)
{
  memset(g_group_window_mm, 0, sizeof(g_group_window_mm));
  memset(g_group_window_valid, 0, sizeof(g_group_window_valid));
  g_window_head = 0U;
}

static void collect_sample(tof_sample_t *sample)
{
  RANGING_SENSOR_Result_t result;
  int32_t status;
  uint32_t zone;

  if (sample == NULL)
  {
    return;
  }

  memset(sample, 0, sizeof(*sample));
  sample->tof_ok = 1U;

  ToF_EventDetected = 0U;
  status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &result);
  if (status != BSP_ERROR_NONE)
  {
    sample->tof_ok = 0U;
    TOF_PublishNoTargetFrame();
    return;
  }

  for (zone = 0U; zone < TOF_ZONE_COUNT; zone++)
  {
    int32_t dist = -1;

    if ((zone < result.NumberOfZones) &&
        (result.ZoneResult[zone].NumberOfTargets > 0U) &&
        (result.ZoneResult[zone].Status[0] == 0U))
    {
      dist = (int32_t)result.ZoneResult[zone].Distance[0];
    }

    sample->tof_dist_mm[zone] = dist;
  }

  TOF_UpdateDistanceMm64FromSample(sample);
}

static void sample_and_stream(void)
{
  tof_sample_t sample;

  collect_sample(&sample);
  if (sample.tof_ok == 0U)
  {
    g_tof_started = 0U;
    printf("TOF communication lost, retrying init in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)BSP_ERROR_COMPONENT_FAILURE);
    return;
  }

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
        uint32_t matrix_col = (group * GROUP_WIDTH) + col;
        int32_t dist_mm = sample->tof_dist_mm[(row * TOF_MATRIX_SIZE) + matrix_col];

        if ((dist_mm >= MIN_VALID_DISTANCE_MM) && (dist_mm <= MAX_VALID_DISTANCE_MM))
        {
          frame_sum_mm += (uint32_t)dist_mm;
          frame_count++;
        }
      }
    }

    if (frame_count > 0U)
    {
      g_group_window_mm[group][g_window_head] =
        (uint16_t)((frame_sum_mm + (frame_count / 2U)) / frame_count);
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
      uint32_t avg_mm = (window_sum_mm + (window_count / 2U)) / window_count;
      grouped_cm[group] = (uint16_t)((avg_mm + 5U) / 10U);
    }
    else
    {
      grouped_cm[group] = DEFAULT_OUTPUT_DISTANCE_CM;
    }
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

static void TOF_ResetCenterSensor(void)
{
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(TOF_PWR_OFF_MS);
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET);
  HAL_Delay(TOF_PWR_ON_STABILIZE_MS);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(TOF_LPN_LOW_MS);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
  HAL_Delay(TOF_BOOT_WAIT_MS);
}

static void TOF_LogInitDiagnostics(void)
{
  uint8_t page = 0U;
  uint8_t device_id = 0U;
  uint8_t revision_id = 0U;
  int32_t bus_status;
  int32_t ready_status;
  int32_t wr_status;
  int32_t rd0_status;
  int32_t rd1_status;

  bus_status = BSP_I2C1_Init();
  ready_status = BSP_I2C1_IsReady(RANGING_SENSOR_VL53L8CX_ADDRESS, 2U);
  wr_status = BSP_I2C1_WriteReg16(RANGING_SENSOR_VL53L8CX_ADDRESS, 0x7FFFU, &page, 1U);
  rd0_status = BSP_I2C1_ReadReg16(RANGING_SENSOR_VL53L8CX_ADDRESS, 0x0000U, &device_id, 1U);
  rd1_status = BSP_I2C1_ReadReg16(RANGING_SENSOR_VL53L8CX_ADDRESS, 0x0001U, &revision_id, 1U);

  printf("TOF diag: PWR_EN=%u LPn=%u INT=%u I2C_init=%ld ready=%ld wr7FFF=%ld rd0=%ld rd1=%ld HAL_I2C=0x%08lX ID=0x%02X REV=0x%02X\r\n",
         (unsigned int)HAL_GPIO_ReadPin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN),
         (unsigned int)HAL_GPIO_ReadPin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN),
         (unsigned int)HAL_GPIO_ReadPin(TOF_INT_EXTI_PORT, TOF_INT_EXTI_PIN),
         (long)bus_status,
         (long)ready_status,
         (long)wr_status,
         (long)rd0_status,
         (long)rd1_status,
         (unsigned long)HAL_I2C_GetError(&hi2c1),
         (unsigned int)device_id,
         (unsigned int)revision_id);
}

static void TOF_PublishNoTargetFrame(void)
{
  memset(g_distance_mm_64, 0, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 1U;
}

static void TOF_UpdateDistanceMm64FromSample(const tof_sample_t *sample)
{
  uint32_t zone;

  if (sample == NULL)
  {
    TOF_PublishNoTargetFrame();
    return;
  }

  for (zone = 0U; zone < TOF_ZONE_COUNT; zone++)
  {
    int32_t dist_mm = sample->tof_dist_mm[zone];

    if (dist_mm > 0)
    {
      g_distance_mm_64[zone] = (uint16_t)dist_mm;
    }
    else
    {
      g_distance_mm_64[zone] = 0U;
    }
  }

  g_distance_mm_64_fresh = 1U;
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

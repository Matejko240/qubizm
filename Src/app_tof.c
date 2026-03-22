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
#include "stm32l4xx_nucleo_bus.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
#define TOF_DISTANCE_MM_64_SIZE (64U)

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static RANGING_SENSOR_Result_t Result;
static int32_t status = 0;
static volatile uint8_t PushButtonDetected = 0;
volatile uint8_t ToF_EventDetected = 0;
static uint16_t g_distance_mm_64[TOF_DISTANCE_MM_64_SIZE];
static volatile uint8_t g_distance_mm_64_fresh = 0U;
static uint8_t g_tof_started = 0U;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void TOF_UpdateDistanceMm64(const RANGING_SENSOR_Result_t *result);
static void __attribute__((unused)) print_result(RANGING_SENSOR_Result_t *Result);
static void toggle_resolution(void);
static void toggle_signal_and_ambient(void);
static void clear_screen(void);
static void display_commands_banner(void);
static void handle_cmd(uint8_t cmd);
static uint8_t get_key(void);
static uint32_t com_has_data(void);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  memset(g_distance_mm_64, 0, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 0U;
  g_tof_started = 0U;
  MX_53L8A1_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
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

static void MX_53L8A1_SimpleRanging_Init(void)
{
  uint32_t Id;

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Sensor reset */
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

  printf("\033[2H\033[2J");
  printf("53L8A1 Simple Ranging demo application\n");
  printf("Sensor initialization...\n");

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L8A1_RANGING_SENSOR_Init failed\n");
    while (1);
  }

  VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
  VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);

  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = RANGING_FREQUENCY;
  Profile.EnableAmbient = 0U;
  Profile.EnableSignal = 0U;

  if (VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &g_tof_profile) != BSP_ERROR_NONE)
  {
    printf("VL53L8A1_RANGING_SENSOR_ConfigProfile failed\n");
    while (1);
  }

  if (VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS) != BSP_ERROR_NONE)
  {
    printf("VL53L8A1_RANGING_SENSOR_Start failed\n");
    while (1);
  }

  g_tof_started = 1U;
}

static void MX_53L8A1_SimpleRanging_Process(void)
{
  if (g_tof_started == 0U)
  {
    return;
  }

  status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &Result);
  if (status == BSP_ERROR_NONE)
  {
    TOF_UpdateDistanceMm64(&Result);
  }

  if (com_has_data())
  {
    handle_cmd(get_key());
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
      uint32_t dst_row = raw_row * 2U;
      uint32_t dst_col = col * 2U;

      g_distance_mm_64[(dst_row * 8U) + dst_col] = distance_mm;
      g_distance_mm_64[(dst_row * 8U) + dst_col + 1U] = distance_mm;
      g_distance_mm_64[((dst_row + 1U) * 8U) + dst_col] = distance_mm;
      g_distance_mm_64[((dst_row + 1U) * 8U) + dst_col + 1U] = distance_mm;
    }
  }

  g_distance_mm_64_fresh = 1U;
}

static void __attribute__((unused)) print_result(RANGING_SENSOR_Result_t *Result)
{
  int8_t i;
  int8_t j;
  int8_t k;
  int8_t l;
  uint8_t zones_per_line;

  zones_per_line = ((Profile.RangingProfile == RS_PROFILE_8x8_AUTONOMOUS) ||
                    (Profile.RangingProfile == RS_PROFILE_8x8_CONTINUOUS)) ? 8 : 4;

  display_commands_banner();

  printf("Cell Format :\n\n");
  for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
  {
    printf(" \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
    {
      printf(" %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
    }
  }

  printf("\n\n");

  for (j = 0; j < Result->NumberOfZones; j += zones_per_line)
  {
    for (i = 0; i < zones_per_line; i++) /* number of zones per line */
    {
      printf(" -----------------");
    }
    printf("\n");

    for (i = 0; i < zones_per_line; i++)
    {
      printf("|                 ");
    }
    printf("|\n");

    for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
    {
      /* Print distance and status */
      for (k = (zones_per_line - 1); k >= 0; k--)
      {
        if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                 (long)Result->ZoneResult[j + k].Distance[l],
                 (long)Result->ZoneResult[j + k].Status[l]);
        else
          printf("| %5s  :  %5s ", "X", "X");
      }
      printf("|\n");

      if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
      {
        /* Print Signal and Ambient */
        for (k = (zones_per_line - 1); k >= 0; k--)
        {
          if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          {
            if (Profile.EnableSignal != 0)
            {
              printf("| %5ld  :  ", (long)Result->ZoneResult[j + k].Signal[l]);
            }
            else
              printf("| %5s  :  ", "X");

            if (Profile.EnableAmbient != 0)
            {
              printf("%5ld ", (long)Result->ZoneResult[j + k].Ambient[l]);
            }
            else
              printf("%5s ", "X");
          }
          else
            printf("| %5s  :  %5s ", "X", "X");
        }
        printf("|\n");
      }
    }
  }

  for (i = 0; i < zones_per_line; i++)
  {
    printf(" -----------------");
  }
  printf("\n");
}

static void toggle_resolution(void)
{
  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);

  switch (Profile.RangingProfile)
  {
    case RS_PROFILE_4x4_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_AUTONOMOUS;
      break;

    case RS_PROFILE_4x4_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
      break;

    case RS_PROFILE_8x8_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_AUTONOMOUS;
      break;

    case RS_PROFILE_8x8_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_CONTINUOUS;
      break;

    default:
      break;
  }

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);
}

static void toggle_signal_and_ambient(void)
{
  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);

  Profile.EnableAmbient = (Profile.EnableAmbient) ? 0U : 1U;
  Profile.EnableSignal = (Profile.EnableSignal) ? 0U : 1U;

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);
}

static void clear_screen(void)
{
  printf("%c[2J", 27); /* 27 is ESC command */
}

static void display_commands_banner(void)
{
  /* clear screen */
  printf("%c[2H", 27);

  printf("53L8A1 Simple Ranging demo application\n");
  printf("--------------------------------------\n\n");

  printf("Use the following keys to control application\n");
  printf(" 'r' : change resolution\n");
  printf(" 's' : enable signal and ambient\n");
  printf(" 'c' : clear screen\n");
  printf("\n");
}

static void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}

static uint8_t get_key(void)
{
  uint8_t cmd = 0;

  HAL_UART_Receive(&hcom_uart[COM1], &cmd, 1, HAL_MAX_DELAY);

  return cmd;
}

static uint32_t com_has_data(void)
{
  return __HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE);;
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

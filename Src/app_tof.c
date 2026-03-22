/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.3 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "53l8a1_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_nucleo_bus.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
#define TOF_DISTANCE_MM_64_SIZE (64U)
#define TOF_INIT_RETRY_PERIOD_MS (1000U)
#define TOF_PWR_OFF_MS (10U)
#define TOF_PWR_ON_STABILIZE_MS (10U)
#define TOF_LPN_LOW_MS (10U)
#define TOF_BOOT_WAIT_MS (100U)

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
static uint32_t g_tof_last_init_attempt_tick = 0U;

/* Private function prototypes -----------------------------------------------*/
static uint8_t MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void TOF_ResetCenterSensor(void);
static void TOF_LogInitDiagnostics(void);
static void TOF_PublishNoTargetFrame(void);
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
  BSP_COM_Init(COM1);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  memset(g_distance_mm_64, 0, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 0U;
  g_tof_started = 0U;
  g_tof_last_init_attempt_tick = 0U;
  MX_53L8A1_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_53L8A1_SimpleRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

uint8_t TOF_GetDistanceMm64(uint16_t out_distance_mm_64[64])
{
  uint8_t has_fresh_frame;

  if (out_distance_mm_64 == NULL)
  {
    return 0U;
  }

  has_fresh_frame = g_distance_mm_64_fresh;
  if (has_fresh_frame == 0U)
  {
    return 0U;
  }

  memcpy(out_distance_mm_64, g_distance_mm_64, sizeof(g_distance_mm_64));
  g_distance_mm_64_fresh = 0U;

  return 1U;
}

static void MX_53L8A1_SimpleRanging_Process(void)
{
  uint32_t now;
  uint8_t tof_data_ready;

  if (g_tof_started == 0U)
  {
    now = HAL_GetTick();
    if ((now - g_tof_last_init_attempt_tick) >= TOF_INIT_RETRY_PERIOD_MS)
    {
      MX_53L8A1_SimpleRanging_Init();
    }
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
    if (com_has_data())
    {
      handle_cmd(get_key());
    }
    return;
  }

  ToF_EventDetected = 0U;
  status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &Result);
  if (status == BSP_ERROR_NONE)
  {
    TOF_UpdateDistanceMm64(&Result);
  }
  else
  {
    g_tof_started = 0U;
    TOF_PublishNoTargetFrame();
    printf("TOF communication lost, retrying init in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return;
  }

  if (com_has_data())
  {
    handle_cmd(get_key());
  }
}

static uint8_t MX_53L8A1_SimpleRanging_Init(void)
{
  uint32_t Id;

  g_tof_last_init_attempt_tick = HAL_GetTick();
  g_tof_started = 0U;

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

  status = VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_ReadID failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  status = VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);
  if (status != BSP_ERROR_NONE)
  {
    TOF_PublishNoTargetFrame();
    printf("VL53L8A1_RANGING_SENSOR_GetCapabilities failed, retry in %lu ms (status=%ld)\r\n",
           (unsigned long)TOF_INIT_RETRY_PERIOD_MS,
           (long)status);
    return 0U;
  }

  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = RANGING_FREQUENCY;
  Profile.EnableAmbient = 0U;
  Profile.EnableSignal = 0U;

  status = VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
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
  printf("TOF ready, sensor ID=0x%lX\r\n", (unsigned long)Id);
  return 1U;
}

static void TOF_ResetCenterSensor(void)
{
  /* Power-cycle the built-in sensor before each init attempt. */
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
  uint8_t page;
  uint8_t device_id;
  uint8_t revision_id;
  int32_t bus_status;
  int32_t ready_status;
  int32_t wr_status;
  int32_t rd0_status;
  int32_t rd1_status;

  page = 0U;
  device_id = 0U;
  revision_id = 0U;

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

static void TOF_UpdateDistanceMm64(const RANGING_SENSOR_Result_t *result)
{
  uint32_t zone_index;
  uint8_t zones_per_line;

  memset(g_distance_mm_64, 0, sizeof(g_distance_mm_64));

  if (result == NULL)
  {
    g_distance_mm_64_fresh = 1U;
    return;
  }

  zones_per_line = (result->NumberOfZones >= TOF_DISTANCE_MM_64_SIZE) ? 8U : 4U;

  for (zone_index = 0U; zone_index < result->NumberOfZones; zone_index++)
  {
    uint16_t distance_mm = 0U;
    uint32_t raw_row = zone_index / zones_per_line;
    uint32_t raw_col = zone_index % zones_per_line;
    uint32_t col = (zones_per_line - 1U) - raw_col;

    if ((result->ZoneResult[zone_index].NumberOfTargets > 0U) &&
        (result->ZoneResult[zone_index].Status[0] == 0U))
    {
      distance_mm = (uint16_t)result->ZoneResult[zone_index].Distance[0];
    }

    if (zones_per_line == 8U)
    {
      g_distance_mm_64[(raw_row * 8U) + col] = distance_mm;
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
  if (g_tof_started == 0U)
  {
    return;
  }

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
  if (g_tof_started == 0U)
  {
    return;
  }

  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);

  Profile.EnableAmbient = (Profile.EnableAmbient) ? 0U : 1U;
  Profile.EnableSignal = (Profile.EnableSignal) ? 0U : 1U;

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);
}

static void clear_screen(void)
{
  printf("\r\n");
}

static void display_commands_banner(void)
{
  printf("\r\n");
  printf("53L8A1 Simple Ranging demo application\r\n");
  printf("--------------------------------------\r\n\r\n");

  printf("Use the following keys to control application\r\n");
  printf(" 'r' : change resolution\r\n");
  printf(" 's' : enable signal and ambient\r\n");
  printf(" 'c' : print a separator line\r\n");
  printf("\r\n");
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
  PushButtonDetected = 1;
}

#ifdef __cplusplus
}
#endif

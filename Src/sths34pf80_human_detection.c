#include "sths34pf80_human_detection.h"

#include "main.h"

#include <stdio.h>

typedef struct
{
  uint8_t func_status;
  int16_t object_raw;
  int16_t ambient_raw;
  int16_t presence_raw;
  int16_t motion_raw;
  uint8_t presence_flag;
  uint8_t motion_flag;
} STHS34PF80_Sample_t;

#define STHS34PF80_I2C_ADDR                  0x5A

#define STHS34PF80_REG_LPF1                 0x0C
#define STHS34PF80_REG_LPF2                 0x0D
#define STHS34PF80_REG_WHO_AM_I             0x0F
#define STHS34PF80_REG_SENS_DATA            0x1D
#define STHS34PF80_REG_CTRL1                0x20
#define STHS34PF80_REG_CTRL2                0x21
#define STHS34PF80_REG_STATUS               0x23
#define STHS34PF80_REG_FUNC_STATUS          0x25
#define STHS34PF80_REG_TOBJECT_L            0x26
#define STHS34PF80_REG_TPRESENCE_L          0x3A

#define STHS34PF80_EMB_FUNC_CFG_ADDR        0x08
#define STHS34PF80_EMB_FUNC_CFG_DATA        0x09
#define STHS34PF80_EMB_PAGE_RW              0x11
#define STHS34PF80_EMB_PRESENCE_THS_L       0x20
#define STHS34PF80_EMB_MOTION_THS_L         0x22
#define STHS34PF80_EMB_HYST_MOTION          0x26
#define STHS34PF80_EMB_HYST_PRESENCE        0x27
#define STHS34PF80_EMB_ALGO_CONFIG          0x28
#define STHS34PF80_EMB_RESET_ALGO           0x2A

#define STHS34PF80_WHO_AM_I_VAL             0xD3

#define STHS34PF80_CTRL1_BDU                0x08
#define STHS34PF80_CTRL1_ODR_POWER_DOWN     0x00
#define STHS34PF80_CTRL1_ODR_2HZ            0x04

#define STHS34PF80_CTRL2_FUNC_CFG_ACCESS    0x10

#define STHS34PF80_STATUS_DRDY              0x04

#define STHS34PF80_FUNC_STATUS_PRES_FLAG    0x04
#define STHS34PF80_FUNC_STATUS_MOT_FLAG     0x02

#define STHS34PF80_PAGE_RW_FUNC_CFG_WRITE   0x40

#define STHS34PF80_ALGO_CONFIG_COMP_TYPE    0x04
#define STHS34PF80_ALGO_CONFIG_SEL_ABS      0x02

#define STHS34PF80_RESET_ALGO_EXECUTE       0x01

#define STHS34PF80_LPF1_DEFAULT             0x04
#define STHS34PF80_LPF2_DEFAULT             0x22

#define STHS34PF80_PRESENCE_THRESHOLD       900U
#define STHS34PF80_MOTION_THRESHOLD         250U
#define STHS34PF80_PRESENCE_HYSTERESIS      100U
#define STHS34PF80_MOTION_HYSTERESIS        40U
#define STHS34PF80_OBJECT_SENS_DEFAULT      2000U

/*
 * Kalibracja na podstawie aktualnych logow:
 * - tlo bez czlowieka: TPRES i TMOT nie przekraczaly ok. 60
 * - czlowiek nieruchomy: TPRES ok. 2100-2470
 * - czlowiek w ruchu: TPRES ok. 1140-1890, TMOT ok. 250-975
 *
 * Przyjete progi zostawiaja duzy zapas od tla, a jednoczesnie sa
 * wyraznie ponizej sygnalu czlowieka w ruchu i bezruchu.
 *
 * Strojenie progow:
 * - PRESENCE_THRESHOLD odpowiada za wykrycie obecnosci osoby stojacej nieruchomo
 * - MOTION_THRESHOLD odpowiada za wykrycie ruchu
 * - HYSTERESIS zmniejsza miganie flag przy wartosciach blisko progu
 *
 * Jak stroic na podstawie logow:
 * 1. Zmierz przez kilkanascie-kilkadziesiat sekund pusty kadr i zanotuj maksymalne |TPRES| i |TMOT|
 * 2. Ustaw prog nieco wyzej od szumu tla, np. 1.5x do 2x maksymalnej wartosci z pustego kadru
 * 3. Sprawdz logi z czlowiekiem w ruchu i nieruchomo:
 *    - jesli czlowiek nie jest wykrywany, zmniejsz odpowiedni prog
 *    - jesli pusty kadr daje falszywe wykrycia, zwieksz odpowiedni prog
 * 4. Jesli flaga szybko przechodzi miedzy 0 i 1, zwieksz histereze
 */

static I2C_HandleTypeDef *g_hi2c = NULL;
static uint16_t g_object_sensitivity_lsb_per_c = STHS34PF80_OBJECT_SENS_DEFAULT;
static uint32_t g_sample_counter = 0;
static STHS34PF80_HumanDetectionState_t g_last_state = {0};

static HAL_StatusTypeDef STHS34PF80_Probe(void);
static HAL_StatusTypeDef STHS34PF80_ReadSample(STHS34PF80_Sample_t *sample);
static HAL_StatusTypeDef STHS34PF80_WriteReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef STHS34PF80_ReadReg(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef STHS34PF80_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef STHS34PF80_WriteEmbeddedReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef STHS34PF80_WriteEmbeddedReg16(uint8_t reg_l, uint16_t value);
static HAL_StatusTypeDef STHS34PF80_ResetAlgorithms(void);
static HAL_StatusTypeDef STHS34PF80_StartContinuousMode(void);
static HAL_StatusTypeDef STHS34PF80_RecoverBus(void);
static void STHS34PF80_PrintSample(const STHS34PF80_Sample_t *sample, uint8_t human_detected);
static void STHS34PF80_PrintBusState(void);
static void STHS34PF80_PrintFixedX10(int32_t value_x10);
static const char *STHS34PF80_StatusString(HAL_StatusTypeDef status);
static int16_t STHS34PF80_ReadLe16(const uint8_t *buffer);
static int32_t STHS34PF80_AmbientCelsiusX10(int16_t raw);
static int32_t STHS34PF80_ObjectCelsiusX10(int16_t raw);
static int32_t STHS34PF80_ObjectCelsiusX100(int16_t raw);

HAL_StatusTypeDef STHS34PF80_HumanDetection_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;
  uint8_t sens_data = 0;
  int32_t sensitivity;

  g_hi2c = hi2c;
  g_last_state.valid = 0U;
  g_last_state.human_detected = 0U;
  g_last_state.presence_flag = 0U;
  g_last_state.motion_flag = 0U;
  g_last_state.presence_raw = 0;
  g_last_state.motion_raw = 0;
  g_last_state.object_raw = 0;
  g_last_state.ambient_raw = 0;
  g_last_state.object_celsius_x100 = 0;

  printf("\r\nSTHS34PF80 human detection demo\r\n");
  STHS34PF80_PrintBusState();

  status = STHS34PF80_Probe();
  if (status != HAL_OK)
  {
    printf("Sensor probe failed.\r\n");
    return status;
  }

  status = STHS34PF80_WriteReg(STHS34PF80_REG_CTRL1, STHS34PF80_CTRL1_ODR_POWER_DOWN);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteReg(STHS34PF80_REG_LPF1, STHS34PF80_LPF1_DEFAULT);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteReg(STHS34PF80_REG_LPF2, STHS34PF80_LPF2_DEFAULT);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_ReadReg(STHS34PF80_REG_SENS_DATA, &sens_data);
  if (status != HAL_OK)
  {
    return status;
  }

  sensitivity = ((int32_t)(int8_t)sens_data * 16) + 2048;
  if (sensitivity > 0)
  {
    g_object_sensitivity_lsb_per_c = (uint16_t)sensitivity;
  }

  status = STHS34PF80_WriteEmbeddedReg16(STHS34PF80_EMB_PRESENCE_THS_L, STHS34PF80_PRESENCE_THRESHOLD);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteEmbeddedReg16(STHS34PF80_EMB_MOTION_THS_L, STHS34PF80_MOTION_THRESHOLD);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteEmbeddedReg(STHS34PF80_EMB_HYST_MOTION, STHS34PF80_MOTION_HYSTERESIS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteEmbeddedReg(STHS34PF80_EMB_HYST_PRESENCE, STHS34PF80_PRESENCE_HYSTERESIS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteEmbeddedReg(STHS34PF80_EMB_ALGO_CONFIG,
                                       STHS34PF80_ALGO_CONFIG_COMP_TYPE);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_ResetAlgorithms();
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_StartContinuousMode();
  if (status != HAL_OK)
  {
    return status;
  }

  printf("Presence cfg: ODR=2Hz PRES_THS=%u MOT_THS=%u HYST=%u sens=%u LSB/C\r\n",
         (unsigned int)STHS34PF80_PRESENCE_THRESHOLD,
         (unsigned int)STHS34PF80_MOTION_THRESHOLD,
         (unsigned int)STHS34PF80_PRESENCE_HYSTERESIS,
         (unsigned int)g_object_sensitivity_lsb_per_c);
  g_sample_counter = 0;
  printf("Cyclic search running. LED ON means human detected.\r\n");

  return HAL_OK;
}

HAL_StatusTypeDef STHS34PF80_HumanDetection_Process(void)
{
  STHS34PF80_Sample_t sample;
  HAL_StatusTypeDef status;
  uint8_t human_detected;

  if (g_hi2c == NULL)
  {
    return HAL_ERROR;
  }

  status = STHS34PF80_ReadSample(&sample);
  if (status == HAL_BUSY)
  {
    return HAL_BUSY;
  }

  if (status != HAL_OK)
  {
    g_last_state.valid = 0U;
    g_last_state.human_detected = 0U;
    printf("Sample read failed, status=%s, I2C error=0x%08lX\r\n",
           STHS34PF80_StatusString(status),
           (unsigned long)HAL_I2C_GetError(g_hi2c));
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    return status;
  }

  human_detected = (sample.presence_flag != 0U) || (sample.motion_flag != 0U);
  g_last_state.valid = 1U;
  g_last_state.human_detected = human_detected;
  g_last_state.presence_flag = sample.presence_flag;
  g_last_state.motion_flag = sample.motion_flag;
  g_last_state.presence_raw = sample.presence_raw;
  g_last_state.motion_raw = sample.motion_raw;
  g_last_state.object_raw = sample.object_raw;
  g_last_state.ambient_raw = sample.ambient_raw;
  g_last_state.object_celsius_x100 = STHS34PF80_ObjectCelsiusX100(sample.object_raw);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, human_detected ? GPIO_PIN_SET : GPIO_PIN_RESET);
  STHS34PF80_PrintSample(&sample, human_detected);
  g_sample_counter++;
  return HAL_OK;
}

void STHS34PF80_HumanDetection_GetState(STHS34PF80_HumanDetectionState_t *state)
{
  if (state == NULL)
  {
    return;
  }

  *state = g_last_state;
}

static HAL_StatusTypeDef STHS34PF80_WriteReg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(g_hi2c,
                           STHS34PF80_I2C_ADDR << 1,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1,
                           100);
}

static HAL_StatusTypeDef STHS34PF80_ReadReg(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(g_hi2c,
                          STHS34PF80_I2C_ADDR << 1,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1,
                          100);
}

static HAL_StatusTypeDef STHS34PF80_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(g_hi2c,
                          STHS34PF80_I2C_ADDR << 1,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          len,
                          100);
}

static HAL_StatusTypeDef STHS34PF80_WriteEmbeddedReg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;

  status = STHS34PF80_WriteReg(STHS34PF80_REG_CTRL2, STHS34PF80_CTRL2_FUNC_CFG_ACCESS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_WriteReg(STHS34PF80_EMB_PAGE_RW, STHS34PF80_PAGE_RW_FUNC_CFG_WRITE);
  if (status == HAL_OK)
  {
    status = STHS34PF80_WriteReg(STHS34PF80_EMB_FUNC_CFG_ADDR, reg);
  }
  if (status == HAL_OK)
  {
    status = STHS34PF80_WriteReg(STHS34PF80_EMB_FUNC_CFG_DATA, value);
  }

  (void)STHS34PF80_WriteReg(STHS34PF80_EMB_PAGE_RW, 0x00);
  (void)STHS34PF80_WriteReg(STHS34PF80_REG_CTRL2, 0x00);

  return status;
}

static HAL_StatusTypeDef STHS34PF80_WriteEmbeddedReg16(uint8_t reg_l, uint16_t value)
{
  HAL_StatusTypeDef status;

  status = STHS34PF80_WriteEmbeddedReg(reg_l, (uint8_t)(value & 0xFFU));
  if (status != HAL_OK)
  {
    return status;
  }

  return STHS34PF80_WriteEmbeddedReg((uint8_t)(reg_l + 1U), (uint8_t)((value >> 8) & 0x7FU));
}

static HAL_StatusTypeDef STHS34PF80_ResetAlgorithms(void)
{
  return STHS34PF80_WriteEmbeddedReg(STHS34PF80_EMB_RESET_ALGO, STHS34PF80_RESET_ALGO_EXECUTE);
}

static HAL_StatusTypeDef STHS34PF80_StartContinuousMode(void)
{
  return STHS34PF80_WriteReg(STHS34PF80_REG_CTRL1,
                             STHS34PF80_CTRL1_BDU | STHS34PF80_CTRL1_ODR_2HZ);
}

static HAL_StatusTypeDef STHS34PF80_RecoverBus(void)
{
  HAL_StatusTypeDef status;

  printf("I2C recovery attempt...\r\n");
  status = HAL_I2C_DeInit(g_hi2c);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(2);
  status = HAL_I2C_Init(g_hi2c);
  if (status == HAL_OK)
  {
    STHS34PF80_PrintBusState();
  }

  return status;
}

static HAL_StatusTypeDef STHS34PF80_Probe(void)
{
  uint8_t who_am_i = 0;
  HAL_StatusTypeDef status;
  uint32_t i2c_error;

  status = HAL_I2C_IsDeviceReady(g_hi2c, STHS34PF80_I2C_ADDR << 1, 2, 50);
  if (status != HAL_OK)
  {
    i2c_error = HAL_I2C_GetError(g_hi2c);

    if ((status == HAL_TIMEOUT) || ((i2c_error & HAL_I2C_ERROR_TIMEOUT) != 0U))
    {
      if (STHS34PF80_RecoverBus() == HAL_OK)
      {
        status = HAL_I2C_IsDeviceReady(g_hi2c, STHS34PF80_I2C_ADDR << 1, 2, 50);
      }
    }
  }

  if (status != HAL_OK)
  {
    printf("STHS34PF80 not ready at 0x%02X, status=%s, HAL_I2C error=0x%08lX\r\n",
           STHS34PF80_I2C_ADDR,
           STHS34PF80_StatusString(status),
           (unsigned long)HAL_I2C_GetError(g_hi2c));
    return status;
  }

  status = STHS34PF80_ReadReg(STHS34PF80_REG_WHO_AM_I, &who_am_i);
  if (status != HAL_OK)
  {
    printf("WHO_AM_I read failed at 0x%02X, status=%s, HAL_I2C error=0x%08lX\r\n",
           STHS34PF80_I2C_ADDR,
           STHS34PF80_StatusString(status),
           (unsigned long)HAL_I2C_GetError(g_hi2c));
    return status;
  }

  if (who_am_i == STHS34PF80_WHO_AM_I_VAL)
  {
    printf("STHS34PF80 detected at 0x%02X, WHO_AM_I=0x%02X\r\n",
           STHS34PF80_I2C_ADDR,
           who_am_i);
    return HAL_OK;
  }

  printf("Device at 0x%02X responded, but WHO_AM_I=0x%02X (expected 0x%02X)\r\n",
         STHS34PF80_I2C_ADDR,
         who_am_i,
         STHS34PF80_WHO_AM_I_VAL);
  return HAL_ERROR;
}

static HAL_StatusTypeDef STHS34PF80_ReadSample(STHS34PF80_Sample_t *sample)
{
  HAL_StatusTypeDef status;
  uint8_t reg_value = 0;
  uint8_t temp_buffer[4];
  uint8_t algo_buffer[4];

  status = STHS34PF80_ReadReg(STHS34PF80_REG_STATUS, &reg_value);
  if (status != HAL_OK)
  {
    return status;
  }

  if ((reg_value & STHS34PF80_STATUS_DRDY) == 0U)
  {
    return HAL_BUSY;
  }

  status = STHS34PF80_ReadReg(STHS34PF80_REG_FUNC_STATUS, &sample->func_status);
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_ReadRegs(STHS34PF80_REG_TOBJECT_L, temp_buffer, sizeof(temp_buffer));
  if (status != HAL_OK)
  {
    return status;
  }

  status = STHS34PF80_ReadRegs(STHS34PF80_REG_TPRESENCE_L, algo_buffer, sizeof(algo_buffer));
  if (status != HAL_OK)
  {
    return status;
  }

  sample->object_raw = STHS34PF80_ReadLe16(&temp_buffer[0]);
  sample->ambient_raw = STHS34PF80_ReadLe16(&temp_buffer[2]);
  sample->presence_raw = STHS34PF80_ReadLe16(&algo_buffer[0]);
  sample->motion_raw = STHS34PF80_ReadLe16(&algo_buffer[2]);
  sample->presence_flag = (sample->func_status & STHS34PF80_FUNC_STATUS_PRES_FLAG) != 0U;
  sample->motion_flag = (sample->func_status & STHS34PF80_FUNC_STATUS_MOT_FLAG) != 0U;

  return HAL_OK;
}

/*
 * Pola wypisywane na UART pochodza z nastepujacych rejestrow STHS34PF80:
 * - pres / mot: bity PRES_FLAG i MOT_FLAG z rejestru FUNC_STATUS (0x25)
 * - TPRES: TPRESENCE_L/H (0x3A/0x3B), surowe wyjscie algorytmu obecnosci
 * - TMOT: TMOTION_L/H (0x3C/0x3D), surowe wyjscie algorytmu ruchu
 * - Tamb: TAMBIENT_L/H (0x28/0x29), temperatura otoczenia, przeliczana przez 100 LSB/degC
 * - Tobj: TOBJECT_L/H (0x26/0x27), sygnal IR obiektu przeliczany z uzyciem czulosci sensora
 *
 * Znaczenie pol w logach:
 * - cycle: kolejny numer pomiaru wypisywany przez firmware
 * - state: SEARCH oznacza, ze firmware w tym cyklu szuka czlowieka; FOUND oznacza,
 *   ze w tym cyklu wykryto czlowieka
 * - human: decyzja podejmowana w tym firmware; ma wartosc YES, gdy pres==1 lub mot==1
 * - pres: flaga obecnosci z sensora; przydatna, gdy czlowiek stoi nieruchomo w polu widzenia
 * - mot: flaga ruchu z sensora; przydatna, gdy czlowiek porusza sie w polu widzenia
 * - TPRES / TMOT: podpisane surowe wartosci algorytmow, porownywane wewnetrznie z progami
 * - Tamb: rzeczywista temperatura otoczenia widziana przez obudowe sensora
 * - Tobj: wzgledny sygnal IR wyrazony w jednostkach rownowaznych degC; nie jest to
 *   temperatura skory/ciala czlowieka i nalezy to traktowac jako pomoc przy detekcji,
 *   a nie jako termometr
 */
static void STHS34PF80_PrintSample(const STHS34PF80_Sample_t *sample, uint8_t human_detected)
{
  printf("cycle=%lu state=%s human=%s pres=%u mot=%u TPRES=%d TMOT=%d Tamb=",
         (unsigned long)g_sample_counter,
         human_detected ? "FOUND" : "SEARCH",
         human_detected ? "YES" : "NO",
         sample->presence_flag,
         sample->motion_flag,
         sample->presence_raw,
         sample->motion_raw);
  STHS34PF80_PrintFixedX10(STHS34PF80_AmbientCelsiusX10(sample->ambient_raw));
  printf("C Tobj=");
  STHS34PF80_PrintFixedX10(STHS34PF80_ObjectCelsiusX10(sample->object_raw));
  printf("C\r\n");
}

static void STHS34PF80_PrintBusState(void)
{
  GPIO_PinState scl;
  GPIO_PinState sda;

  scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
  sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

  printf("Bus state: SCL=%lu SDA=%lu I2C_state=0x%02X\r\n",
         (unsigned long)scl,
         (unsigned long)sda,
         (unsigned int)HAL_I2C_GetState(g_hi2c));
}

static void STHS34PF80_PrintFixedX10(int32_t value_x10)
{
  uint32_t abs_value;

  if (value_x10 < 0)
  {
    printf("-");
    abs_value = (uint32_t)(-value_x10);
  }
  else
  {
    abs_value = (uint32_t)value_x10;
  }

  printf("%lu.%01lu",
         (unsigned long)(abs_value / 10U),
         (unsigned long)(abs_value % 10U));
}

static const char *STHS34PF80_StatusString(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return "HAL_OK";
    case HAL_ERROR:
      return "HAL_ERROR";
    case HAL_BUSY:
      return "HAL_BUSY";
    case HAL_TIMEOUT:
      return "HAL_TIMEOUT";
    default:
      return "HAL_UNKNOWN";
  }
}

static int16_t STHS34PF80_ReadLe16(const uint8_t *buffer)
{
  return (int16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8));
}

static int32_t STHS34PF80_AmbientCelsiusX10(int16_t raw)
{
  return ((int32_t)raw * 10) / 100;
}

static int32_t STHS34PF80_ObjectCelsiusX10(int16_t raw)
{
  return ((int32_t)raw * 10) / (int32_t)g_object_sensitivity_lsb_per_c;
}

static int32_t STHS34PF80_ObjectCelsiusX100(int16_t raw)
{
  return ((int32_t)raw * 100) / (int32_t)g_object_sensitivity_lsb_per_c;
}

#include "rgb_led.h"

#include "main.h"

/*
 * Dioda RGB ma wspolna anode, wiec:
 * - stan niski na pinie wlacza dany kolor
 * - stan wysoki wylacza dany kolor
 *
 * Podlaczenie:
 * - wspolna anoda LED do 3.3 V
 * - kazda katoda przez osobny rezystor do GPIO STM32
 */

static void RgbLed_WriteChannels(uint8_t red_on, uint8_t green_on, uint8_t blue_on);

void RgbLed_Init(void)
{
  RgbLed_WriteChannels(0U, 0U, 0U);
}

void RgbLed_SetColor(RgbLed_Color_t color)
{
  switch (color)
  {
    case RGB_LED_COLOR_GREEN:
      RgbLed_WriteChannels(0U, 1U, 0U);
      break;

    case RGB_LED_COLOR_YELLOW:
      RgbLed_WriteChannels(1U, 1U, 0U);
      break;

    case RGB_LED_COLOR_RED:
      RgbLed_WriteChannels(1U, 0U, 0U);
      break;

    case RGB_LED_COLOR_BLUE:
      RgbLed_WriteChannels(0U, 0U, 1U);
      break;

    case RGB_LED_COLOR_OFF:
    default:
      RgbLed_WriteChannels(0U, 0U, 0U);
      break;
  }
}

static void RgbLed_WriteChannels(uint8_t red_on, uint8_t green_on, uint8_t blue_on)
{
#if defined(RGB_R_GPIO_Port) && defined(RGB_R_Pin) && \
    defined(RGB_G_GPIO_Port) && defined(RGB_G_Pin) && \
    defined(RGB_B_GPIO_Port) && defined(RGB_B_Pin)
  HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, red_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, green_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, blue_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  (void)red_on;
  (void)green_on;
  (void)blue_on;
#endif
}

#ifndef RGB_LED_H
#define RGB_LED_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  RGB_LED_COLOR_OFF = 0,
  RGB_LED_COLOR_GREEN,
  RGB_LED_COLOR_YELLOW,
  RGB_LED_COLOR_RED,
  RGB_LED_COLOR_BLUE
} RgbLed_Color_t;

void RgbLed_Init(void);
void RgbLed_SetColor(RgbLed_Color_t color);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_H */

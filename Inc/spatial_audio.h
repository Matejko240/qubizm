#ifndef SPATIAL_AUDIO_H
#define SPATIAL_AUDIO_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim_left;
    uint32_t ch_left;

    TIM_HandleTypeDef *htim_right;
    uint32_t ch_right;

    uint16_t beep_on_ms;      // jak długo trwa piknięcie
    uint16_t timeout_ms;      // po ilu ms bez nowych danych wyciszyć audio

    volatile uint8_t  pending_valid;
    volatile uint8_t  pending_pos8;         // 0..7
    volatile uint16_t pending_distance_mm;
    volatile uint8_t  pending_update;

    uint8_t  active_valid;
    uint8_t  active_pos8;
    uint16_t active_distance_mm;
    uint8_t  active_rate_hz;   // ile beepów/s

    uint8_t  beep_is_on;
    uint32_t next_toggle_ms;
    uint32_t last_measure_ms;

    uint8_t  started;
} SpatialAudio_t;

void SpatialAudio_Init(SpatialAudio_t *ctx,
                       TIM_HandleTypeDef *htim_left, uint32_t ch_left,
                       TIM_HandleTypeDef *htim_right, uint32_t ch_right,
                       uint16_t beep_on_ms,
                       uint16_t timeout_ms);

void SpatialAudio_Start(SpatialAudio_t *ctx);
void SpatialAudio_Stop(SpatialAudio_t *ctx);
void SpatialAudio_Mute(SpatialAudio_t *ctx);

/* Wołasz to z kodu ToF: z callbacka, z superloopa albo z IRQ po nowej ramce */
void SpatialAudio_PostObservation(SpatialAudio_t *ctx,
                                  uint8_t pos8,
                                  uint16_t distance_mm,
                                  uint8_t valid);

/* Nieblokujący scheduler audio */
void SpatialAudio_Process(SpatialAudio_t *ctx, uint32_t now_ms);

/* Domyślna mapa odległość -> beep/s, możesz ją później zmienić */
uint8_t SpatialAudio_DefaultRateFromDistance(uint16_t distance_mm);

#endif

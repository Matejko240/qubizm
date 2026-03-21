#include "spatial_audio.h"
#include <string.h>

/*
 * 8 pozycji panoramy:
 * 0 = skrajne lewo
 * 7 = skrajne prawo
 *
 * Tabela jest celowo "łagodniejsza" niż liniowa.
 */
static const uint8_t kLeftGainPct[8]  = {100, 97, 90, 78, 62, 43, 22,  0};
static const uint8_t kRightGainPct[8] = {  0, 22, 43, 62, 78, 90, 97,100};

static void SpatialAudio_SetPan8(SpatialAudio_t *ctx, uint8_t pos8)
{
    if (pos8 > 7U) pos8 = 7U;

    uint32_t left_max  = (__HAL_TIM_GET_AUTORELOAD(ctx->htim_left)  + 1U) / 2U;
    uint32_t right_max = (__HAL_TIM_GET_AUTORELOAD(ctx->htim_right) + 1U) / 2U;

    uint32_t left_ccr  = (left_max  * kLeftGainPct[pos8])  / 100U;
    uint32_t right_ccr = (right_max * kRightGainPct[pos8]) / 100U;

    __HAL_TIM_SET_COMPARE(ctx->htim_left,  ctx->ch_left,  left_ccr);
    __HAL_TIM_SET_COMPARE(ctx->htim_right, ctx->ch_right, right_ccr);
}

void SpatialAudio_Init(SpatialAudio_t *ctx,
                       TIM_HandleTypeDef *htim_left, uint32_t ch_left,
                       TIM_HandleTypeDef *htim_right, uint32_t ch_right,
                       uint16_t beep_on_ms,
                       uint16_t timeout_ms)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->htim_left   = htim_left;
    ctx->ch_left     = ch_left;
    ctx->htim_right  = htim_right;
    ctx->ch_right    = ch_right;
    ctx->beep_on_ms  = beep_on_ms;
    ctx->timeout_ms  = timeout_ms;
}

void SpatialAudio_Mute(SpatialAudio_t *ctx)
{
    __HAL_TIM_SET_COMPARE(ctx->htim_left,  ctx->ch_left,  0U);
    __HAL_TIM_SET_COMPARE(ctx->htim_right, ctx->ch_right, 0U);
}

void SpatialAudio_Start(SpatialAudio_t *ctx)
{
    SpatialAudio_Mute(ctx);

    HAL_TIM_PWM_Start(ctx->htim_left,  ctx->ch_left);
    HAL_TIM_PWM_Start(ctx->htim_right, ctx->ch_right);

    ctx->beep_is_on = 0U;
    ctx->next_toggle_ms = 0U;
    ctx->started = 1U;
}

void SpatialAudio_Stop(SpatialAudio_t *ctx)
{
    SpatialAudio_Mute(ctx);

    HAL_TIM_PWM_Stop(ctx->htim_left,  ctx->ch_left);
    HAL_TIM_PWM_Stop(ctx->htim_right, ctx->ch_right);

    ctx->beep_is_on = 0U;
    ctx->started = 0U;
}

uint8_t SpatialAudio_DefaultRateFromDistance(uint16_t distance_mm)
{
    if (distance_mm == 0U)     return 0U;
    if (distance_mm > 2000U)   return 0U;   // za daleko = cisza
    if (distance_mm > 1500U)   return 2U;
    if (distance_mm > 1000U)   return 3U;
    if (distance_mm > 700U)    return 4U;
    if (distance_mm > 500U)    return 6U;
    if (distance_mm > 350U)    return 8U;
    return 10U; // bardzo blisko
}

void SpatialAudio_PostObservation(SpatialAudio_t *ctx,
                                  uint8_t pos8,
                                  uint16_t distance_mm,
                                  uint8_t valid)
{
    if (pos8 > 7U) pos8 = 7U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ctx->pending_pos8        = pos8;
    ctx->pending_distance_mm = distance_mm;
    ctx->pending_valid       = valid;
    ctx->pending_update      = 1U;

    if (!primask)
    {
        __enable_irq();
    }
}

void SpatialAudio_Process(SpatialAudio_t *ctx, uint32_t now_ms)
{
    uint8_t had_update = 0U;

    if (!ctx->started)
    {
        return;
    }

    if (ctx->pending_update)
    {
        uint8_t pos8;
        uint16_t distance_mm;
        uint8_t valid;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        pos8        = ctx->pending_pos8;
        distance_mm = ctx->pending_distance_mm;
        valid       = ctx->pending_valid;
        ctx->pending_update = 0U;

        if (!primask)
        {
            __enable_irq();
        }

        ctx->active_pos8        = pos8;
        ctx->active_distance_mm = distance_mm;
        ctx->active_valid       = valid;
        ctx->active_rate_hz     = valid ? SpatialAudio_DefaultRateFromDistance(distance_mm) : 0U;
        ctx->last_measure_ms    = now_ms;

        had_update = 1U;
    }

    /* brak danych / timeout / za daleko -> cisza */
    if ((!ctx->active_valid) ||
        ((now_ms - ctx->last_measure_ms) > ctx->timeout_ms) ||
        (ctx->active_rate_hz == 0U))
    {
        SpatialAudio_Mute(ctx);
        ctx->beep_is_on = 0U;
        ctx->next_toggle_ms = now_ms;
        return;
    }

    /* jeśli podczas trwania beepa przyszła nowa pozycja, od razu przesuń panoramę */
    if (had_update && ctx->beep_is_on)
    {
        SpatialAudio_SetPan8(ctx, ctx->active_pos8);
    }

    uint32_t period_ms = 1000U / ctx->active_rate_hz;
    if (period_ms == 0U) period_ms = 1U;

    uint32_t off_ms = (period_ms > ctx->beep_on_ms) ? (period_ms - ctx->beep_on_ms) : 0U;

    if (!ctx->beep_is_on)
    {
        if ((int32_t)(now_ms - ctx->next_toggle_ms) >= 0)
        {
            SpatialAudio_SetPan8(ctx, ctx->active_pos8);
            ctx->beep_is_on = 1U;
            ctx->next_toggle_ms = now_ms + ctx->beep_on_ms;
        }
    }
    else
    {
        if ((int32_t)(now_ms - ctx->next_toggle_ms) >= 0)
        {
            SpatialAudio_Mute(ctx);
            ctx->beep_is_on = 0U;
            ctx->next_toggle_ms = now_ms + off_ms;
        }
    }
}

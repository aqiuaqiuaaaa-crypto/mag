/* Raw CURT acquisition only. No conversion to amperes, control or telemetry. */
#include "current_sense.h"
#include "adc.h"
#include "tim.h"

typedef char ChannelCountMustMatch[(CURRENT_SENSE_CHANNEL_COUNT == ADC1_SENSE_CHANNEL_COUNT) ? 1 : -1];
/* Keep in DMA-accessible SRAM, NOT CCM. Cortex-M4 here has no D-cache. */
__ALIGN_BEGIN static uint16_t dma_buffer[2U * CURRENT_SENSE_CHANNEL_COUNT] __ALIGN_END;
static volatile CurrentSense_Snapshot published;
static uint8_t start_attempted;

static HAL_StatusTypeDef StartupFailure(uint32_t flags, HAL_StatusTypeDef status)
{
  published.running = 0U;
  published.valid = 0U;
  published.error_flags |= flags;
  return status == HAL_OK ? HAL_ERROR : status;
}

HAL_StatusTypeDef CurrentSense_Start(void)
{
  HAL_StatusTypeDef status;
  if (start_attempted != 0U) return HAL_BUSY;
  start_attempted = 1U;

  status = MX_ADC1_Init();
  if (status != HAL_OK) return StartupFailure(CURRENT_SENSE_ERROR_INIT, status);
  /* TIM3 init generates UG with TRGO still RESET; DMA is armed afterwards. */
  status = MX_TIM3_Init();
  if (status != HAL_OK) return StartupFailure(CURRENT_SENSE_ERROR_INIT, status);
  status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_buffer,
                             2U * CURRENT_SENSE_CHANNEL_COUNT);
  /* HAL v1.8.1 ignores HAL_DMA_Start_IT return: verify actual start as well. */
  if (status != HAL_OK || HAL_DMA_GetState(&hdma_adc1) != HAL_DMA_STATE_BUSY ||
      (hdma_adc1.Instance->CR & DMA_SxCR_EN) == 0U ||
      HAL_ADC_GetError(&hadc1) != HAL_ADC_ERROR_NONE)
  {
    uint32_t flags = CURRENT_SENSE_ERROR_DMA_START | HAL_ADC_GetError(&hadc1);
    (void)HAL_ADC_Stop_DMA(&hadc1); /* foreground only, never in callbacks */
    return StartupFailure(flags, status);
  }
  published.running = 1U;
  status = HAL_TIM_Base_Start(&htim3);
  if (status != HAL_OK)
  {
    (void)HAL_TIM_Base_Stop(&htim3);
    (void)HAL_ADC_Stop_DMA(&hadc1);
    return StartupFailure(CURRENT_SENSE_ERROR_TIMER_START, status);
  }
  return HAL_OK;
}

static void LatchSamplingFault(uint32_t error)
{
  uint32_t new_errors = error & ~published.error_flags;
  if ((new_errors & HAL_ADC_ERROR_OVR) != 0U) ++published.overrun_count;
  if ((new_errors & HAL_ADC_ERROR_DMA) != 0U) ++published.dma_error_count;
  if ((new_errors & CURRENT_SENSE_ERROR_DMA_LATE) != 0U) ++published.dma_late_count;
  published.error_flags |= error;
  published.valid = 0U;
  published.running = 0U;
  /* Bounded register-only stop; do not use blocking DMA abort in an ISR. */
  (void)HAL_TIM_Base_Stop(&htim3);
}

static uint8_t ConversionHasError(void)
{
  uint32_t error = HAL_ADC_GetError(&hadc1);
  /* HAL DMA handles errors after completion callbacks; detect them now. */
  if (HAL_DMA_GetError(&hdma_adc1) != HAL_DMA_ERROR_NONE) error |= HAL_ADC_ERROR_DMA;
  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != 0U) error |= HAL_ADC_ERROR_OVR;
  if (error == HAL_ADC_ERROR_NONE) return 0U;
  LatchSamplingFault(error);
  return 1U;
}

static uint8_t FrameIsStable(uint32_t half)
{
  uint32_t remaining = __HAL_DMA_GET_COUNTER(&hdma_adc1);
  uint32_t opposite = half == 0U ? DMA_FLAG_TCIF0_4 : DMA_FLAG_HTIF0_4;
  /* HAL cleared this callback's own flag, not the opposite flag yet.
     Its sticky opposite flag detects backlog AND a complete DMA wrap during
     a preempted copy. NDTR alone would miss a wrap back to the same half. */
  if (__HAL_DMA_GET_FLAG(&hdma_adc1, opposite) != 0U) return 0U;
  if (half == 0U) return (uint8_t)(remaining > 0U && remaining <= CURRENT_SENSE_CHANNEL_COUNT);
  return (uint8_t)(remaining > CURRENT_SENSE_CHANNEL_COUNT &&
                   remaining <= 2U * CURRENT_SENSE_CHANNEL_COUNT);
}

static void PublishFrame(const volatile uint16_t *frame, uint32_t half)
{
  uint32_t index;
  uint16_t stable_raw[CURRENT_SENSE_CHANNEL_COUNT];
  if (published.running == 0U || published.error_flags != 0U) return;
  if (ConversionHasError() != 0U) return;
  if (FrameIsStable(half) == 0U)
  {
    LatchSamplingFault(CURRENT_SENSE_ERROR_DMA_LATE);
    return;
  }
  __DMB();
  for (index = 0U; index < CURRENT_SENSE_CHANNEL_COUNT; ++index)
    stable_raw[index] = frame[index];
  __DMB();
  if (FrameIsStable(half) == 0U)
  {
    LatchSamplingFault(CURRENT_SENSE_ERROR_DMA_LATE);
    return;
  }
  if (ConversionHasError() != 0U) return;
  for (index = 0U; index < CURRENT_SENSE_CHANNEL_COUNT; ++index)
    published.raw[index] = stable_raw[index];
  ++published.frame_count;
  published.timestamp_ms = HAL_GetTick();
  published.valid = 1U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle == &hadc1) PublishFrame(&dma_buffer[0], 0U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle == &hadc1) PublishFrame(&dma_buffer[CURRENT_SENSE_CHANNEL_COUNT], 1U);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adcHandle)
{
  uint32_t error;
  if (adcHandle != &hadc1) return;
  error = HAL_ADC_GetError(adcHandle);
  LatchSamplingFault(error == HAL_ADC_ERROR_NONE ? HAL_ADC_ERROR_INTERNAL : error);
}

uint8_t CurrentSense_GetSnapshot(CurrentSense_Snapshot *out)
{
  uint32_t index, primask;
  if (out == NULL) return 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < CURRENT_SENSE_CHANNEL_COUNT; ++index)
    out->raw[index] = published.raw[index];
  out->frame_count = published.frame_count;
  out->timestamp_ms = published.timestamp_ms;
  out->error_flags = published.error_flags;
  out->overrun_count = published.overrun_count;
  out->dma_error_count = published.dma_error_count;
  out->dma_late_count = published.dma_late_count;
  out->valid = published.valid;
  out->running = published.running;
  __set_PRIMASK(primask); /* preserve callers that already disabled IRQs */
  return out->valid;
}

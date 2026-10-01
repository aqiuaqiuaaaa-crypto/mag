/* Exercise the real module; fake HAL only, no serial port or hardware access. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Src/current_sense.c"

static DMA_Stream_TypeDef stream;
DMA_HandleTypeDef hdma_adc1 = { &stream, HAL_DMA_STATE_READY };
ADC_HandleTypeDef hadc1 = { &hdma_adc1, 0U };
TIM_HandleTypeDef htim3;
static uint32_t primask, tick = 123U, dma_length, stop_count, sequence;
static HAL_StatusTypeDef adc_init_result = HAL_OK, timer_init_result = HAL_OK;
static HAL_StatusTypeDef dma_result = HAL_OK, timer_start_result = HAL_OK;
static int silent_dma_failure;
static uint32_t dma_flags, adc_flags, counter_reads;
static int advance_during_copy, whole_wrap_during_copy;

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t value) { primask = value; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *adc) { return adc->ErrorCode; }
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *dma) { return dma->State; }
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma) { return dma->ErrorCode; }
uint32_t Test_DMA_GetCounter(DMA_HandleTypeDef *dma)
{
  if (++counter_reads == 2U && advance_during_copy)
  {
    stream.NDTR = whole_wrap_during_copy ? 6U : 12U;
    dma_flags = DMA_FLAG_TCIF0_4;
  }
  return dma->Instance->NDTR;
}
uint32_t Test_DMA_GetFlag(DMA_HandleTypeDef *dma, uint32_t flag)
{
  assert(dma == &hdma_adc1);
  return dma_flags & flag;
}
uint32_t Test_ADC_GetFlag(ADC_HandleTypeDef *adc, uint32_t flag)
{
  assert(adc == &hadc1);
  return adc_flags & flag;
}
HAL_StatusTypeDef MX_ADC1_Init(void) { assert(sequence++ == 0U); return adc_init_result; }
HAL_StatusTypeDef MX_TIM3_Init(void) { assert(sequence++ == 1U); return timer_init_result; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *adc, uint32_t *buffer, uint32_t length)
{
  assert(adc == &hadc1 && buffer == (uint32_t *)dma_buffer);
  assert(sequence++ == 2U);
  dma_length = length;
  if (dma_result == HAL_OK && !silent_dma_failure)
  {
    hdma_adc1.State = HAL_DMA_STATE_BUSY;
    stream.CR = DMA_SxCR_EN;
  }
  return dma_result;
}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *adc)
{
  assert(adc == &hadc1);
  stream.CR = 0U;
  hdma_adc1.State = HAL_DMA_STATE_READY;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim3 && sequence++ == 3U);
  assert(hdma_adc1.State == HAL_DMA_STATE_BUSY && stream.CR == DMA_SxCR_EN);
  return timer_start_result;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim3);
  ++stop_count;
  return HAL_OK;
}

int main(int argc, char **argv)
{
  CurrentSense_Snapshot sample;
  ADC_HandleTypeDef other = {0};
  unsigned i;
  assert(argc == 2);
  if (!strcmp(argv[1], "adc-init")) adc_init_result = HAL_ERROR;
  if (!strcmp(argv[1], "timer-init")) timer_init_result = HAL_ERROR;
  if (!strcmp(argv[1], "dma-start")) dma_result = HAL_ERROR;
  if (!strcmp(argv[1], "dma-silent")) silent_dma_failure = 1;
  if (!strcmp(argv[1], "timer-start")) timer_start_result = HAL_ERROR;

  assert(CurrentSense_GetSnapshot(&sample) == 0U);
  assert(!sample.valid && !sample.running && !sample.frame_count);
  assert(CurrentSense_GetSnapshot(NULL) == 0U);
  if (adc_init_result != HAL_OK || timer_init_result != HAL_OK ||
      dma_result != HAL_OK || silent_dma_failure || timer_start_result != HAL_OK)
  {
    assert(CurrentSense_Start() != HAL_OK);
    assert(CurrentSense_GetSnapshot(&sample) == 0U);
    assert(!sample.running && !sample.valid && sample.error_flags);
    assert(sequence <= 4U);
    puts("startup failure correctly isolated");
    return 0;
  }
  assert(CurrentSense_Start() == HAL_OK);
  assert(dma_length == 12U && sequence == 4U);
  assert(CurrentSense_Start() == HAL_BUSY);
  assert(CurrentSense_GetSnapshot(&sample) == 0U);
  assert(sample.running && !sample.valid);
  for (i = 0; i < 12; ++i) dma_buffer[i] = (uint16_t)(100U + i);
  stream.NDTR = 6U;
  if (!strcmp(argv[1], "late-half"))
  {
    /* HAL has cleared HT but TC remains pending; half0 is already reused. */
    stream.NDTR = 12U;
    dma_flags = DMA_FLAG_TCIF0_4;
  }
  if (!strcmp(argv[1], "copy-wrap")) advance_during_copy = 1;
  if (!strcmp(argv[1], "copy-full-wrap"))
  {
    advance_during_copy = 1;
    whole_wrap_during_copy = 1;
  }
  if (!strcmp(argv[1], "backlog-same-half")) dma_flags = DMA_FLAG_TCIF0_4;
  if (!strcmp(argv[1], "ndtr-invalid")) stream.NDTR = 0U;
  if (!strcmp(argv[1], "pending-dma-error")) hdma_adc1.ErrorCode = 1U;
  if (!strcmp(argv[1], "pending-overrun")) adc_flags = ADC_FLAG_OVR;
  if (!strcmp(argv[1], "late-half") || !strcmp(argv[1], "copy-wrap") ||
      !strcmp(argv[1], "copy-full-wrap") || !strcmp(argv[1], "backlog-same-half") ||
      !strcmp(argv[1], "ndtr-invalid") ||
      !strcmp(argv[1], "pending-dma-error") || !strcmp(argv[1], "pending-overrun"))
  {
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U);
    assert(!sample.valid && !sample.running && sample.frame_count == 0U);
    assert(sample.error_flags != 0U && stop_count == 1U);
    if (!strcmp(argv[1], "pending-dma-error"))
      assert(sample.dma_error_count == 1U && !sample.overrun_count);
    else if (!strcmp(argv[1], "pending-overrun"))
      assert(sample.overrun_count == 1U && !sample.dma_error_count);
    else assert(sample.dma_late_count == 1U);
    dma_flags = 0U; /* HAL subsequently clears TC and calls TC; no revival. */
    stream.NDTR = 12U;
    HAL_ADC_ConvCpltCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U && sample.frame_count == 0U);
    puts("unsafe DMA frame rejected");
    return 0;
  }
  HAL_ADC_ConvHalfCpltCallback(&other);
  assert(CurrentSense_GetSnapshot(&sample) == 0U);
  HAL_ADC_ConvHalfCpltCallback(&hadc1);
  assert(CurrentSense_GetSnapshot(&sample) == 1U);
  assert(sample.frame_count == 1U && sample.timestamp_ms == 123U);
  for (i = 0; i < 6; ++i) assert(sample.raw[i] == 100U + i);
  tick = 125U;
  stream.NDTR = 12U;
  HAL_ADC_ConvCpltCallback(&hadc1);
  primask = 1U;
  assert(CurrentSense_GetSnapshot(&sample) == 1U && primask == 1U);
  assert(sample.frame_count == 2U && sample.timestamp_ms == 125U);
  for (i = 0; i < 6; ++i) assert(sample.raw[i] == 106U + i);
  primask = 0U;
  assert(CurrentSense_GetSnapshot(&sample) == 1U && primask == 0U);
  if (!strcmp(argv[1], "late-tc") || !strcmp(argv[1], "late-tc-same-half"))
  {
    stream.NDTR = !strcmp(argv[1], "late-tc-same-half") ? 12U : 6U;
    dma_flags = DMA_FLAG_HTIF0_4;
    HAL_ADC_ConvCpltCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U);
    assert(sample.frame_count == 2U && sample.dma_late_count == 1U && !sample.running);
    for (i = 0; i < 6; ++i) assert(sample.raw[i] == 106U + i);
    puts("unsafe TC rejected without overwriting last good raw");
    return 0;
  }
  HAL_ADC_ErrorCallback(&other);
  assert(stop_count == 0U);
  if (strcmp(argv[1], "normal"))
  {
    hadc1.ErrorCode = !strcmp(argv[1], "overrun") ? HAL_ADC_ERROR_OVR : HAL_ADC_ERROR_DMA;
    HAL_ADC_ErrorCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U);
    assert(!sample.running && !sample.valid && sample.error_flags == hadc1.ErrorCode);
    assert(sample.overrun_count == (hadc1.ErrorCode == HAL_ADC_ERROR_OVR));
    assert(sample.dma_error_count == (hadc1.ErrorCode == HAL_ADC_ERROR_DMA));
    assert(stop_count == 1U);
    hadc1.ErrorCode |= HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA;
    HAL_ADC_ErrorCallback(&hadc1);
    HAL_ADC_ErrorCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U);
    assert(sample.overrun_count == 1U && sample.dma_error_count == 1U);
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    HAL_ADC_ConvCpltCallback(&hadc1);
    assert(CurrentSense_GetSnapshot(&sample) == 0U && sample.frame_count == 2U);
    assert(CurrentSense_Start() == HAL_BUSY); /* one-shot, fault latched until reset */
  }
  puts("complete frames, snapshot and fault state PASS");
  return 0;
}

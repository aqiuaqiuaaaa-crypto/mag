/* Measurement-only ADC1 configuration; CubeF4 v1.27.1 / HAL v1.8.1. */
#include "adc.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
static HAL_StatusTypeDef adc1_msp_status;

/* Logical order a0..a5, NEVER physical pole order [1,3,5,4,6,2]. */
static const uint32_t regular_channels[ADC1_SENSE_CHANNEL_COUNT] = {
  ADC_CHANNEL_8, ADC_CHANNEL_6, ADC_CHANNEL_3,
  ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_4
};

HAL_StatusTypeDef MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_MultiModeTypeDef multimode = {0};
  HAL_StatusTypeDef status;
  uint32_t index;

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = ADC1_SENSE_CHANNEL_COUNT;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  adc1_msp_status = HAL_OK;
  status = HAL_ADC_Init(&hadc1);
  if (status != HAL_OK) return status;
  if (adc1_msp_status != HAL_OK) return adc1_msp_status;

  /* ADC2/ADC3 are unused; do not enable dual/triple-ADC packed DMA. */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  status = HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
  if (status != HAL_OK) return status;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  sConfig.Offset = 0U;
  for (index = 0U; index < ADC1_SENSE_CHANNEL_COUNT; ++index)
  {
    sConfig.Channel = regular_channels[index];
    sConfig.Rank = index + 1U;
    status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    if (status != HAL_OK) return status;
  }
  return HAL_OK;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance != ADC1) return;
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* PA4/JP5 must still be confirmed on the real board (ADC/PM2_AMPW path).
     DAC remains disabled. None of these GPIOs controls driver enable. */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* DMA2 clock and Stream0 NVIC are enabled by the existing MX_DMA_Init. */
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  adc1_msp_status = HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
  if (adc1_msp_status != HAL_OK) return;
  /* Only ADC overrun IRQ, not an interrupt per conversion. */
  HAL_NVIC_SetPriority(ADC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

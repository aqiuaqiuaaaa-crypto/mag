#ifndef CURRENT_ADC_H
#define CURRENT_ADC_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#define ADC1_SENSE_CHANNEL_COUNT 6U
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
/* Deliberately nonfatal: caller records a measurement-only init failure. */
HAL_StatusTypeDef MX_ADC1_Init(void);
#ifdef __cplusplus
}
#endif
#endif

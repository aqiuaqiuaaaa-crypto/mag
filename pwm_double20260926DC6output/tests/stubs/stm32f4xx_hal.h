#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_DMA_STATE_RESET, HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY } HAL_DMA_StateTypeDef;
typedef struct { uint32_t CR; uint32_t NDTR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; HAL_DMA_StateTypeDef State; uint32_t ErrorCode; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *DMA_Handle; uint32_t ErrorCode; } ADC_HandleTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
#define HAL_ADC_ERROR_NONE 0U
#define HAL_ADC_ERROR_INTERNAL 1U
#define HAL_ADC_ERROR_OVR 2U
#define HAL_ADC_ERROR_DMA 4U
#define DMA_SxCR_EN 1U
#define HAL_DMA_ERROR_NONE 0U
#define DMA_FLAG_HTIF0_4 0x10U
#define DMA_FLAG_TCIF0_4 0x20U
#define ADC_FLAG_OVR 0x20U
#define __HAL_DMA_GET_COUNTER(handle) Test_DMA_GetCounter(handle)
#define __HAL_DMA_GET_FLAG(handle, flag) Test_DMA_GetFlag(handle, flag)
#define __HAL_ADC_GET_FLAG(handle, flag) Test_ADC_GetFlag(handle, flag)
#define __DMB() ((void)0)
uint32_t Test_DMA_GetCounter(DMA_HandleTypeDef *dma);
uint32_t Test_DMA_GetFlag(DMA_HandleTypeDef *dma, uint32_t flag);
uint32_t Test_ADC_GetFlag(ADC_HandleTypeDef *adc, uint32_t flag);
#define __ALIGN_BEGIN
#define __ALIGN_END __attribute__((aligned(4)))
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
uint32_t HAL_GetTick(void);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *adc);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *dma);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *adc, uint32_t *buffer, uint32_t length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *timer);
#endif

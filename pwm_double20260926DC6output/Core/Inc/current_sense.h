#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_SENSE_CHANNEL_COUNT 6U
#define CURRENT_SENSE_FRAME_HZ 500U
#define CURRENT_SENSE_ERROR_INIT 0x00010000U
#define CURRENT_SENSE_ERROR_DMA_START 0x00020000U
#define CURRENT_SENSE_ERROR_TIMER_START 0x00040000U
#define CURRENT_SENSE_ERROR_DMA_LATE 0x00080000U

typedef struct {
  uint16_t raw[CURRENT_SENSE_CHANNEL_COUNT]; /* [a0,a1,a2,a3,a4,a5], 0..4095 */
  uint32_t frame_count;  /* published complete frames; modulo 2^32 */
  uint32_t timestamp_ms; /* publication HAL tick, not conversion-start timestamp */
  uint32_t error_flags;  /* HAL_ADC_ERROR_* plus CURRENT_SENSE_ERROR_* */
  uint32_t overrun_count;   /* newly latched OVR category, not repeated HAL bits */
  uint32_t dma_error_count; /* newly latched DMA category; one-shot per boot */
  uint32_t dma_late_count;  /* unsafe/overwritten half-buffer detected */
  uint8_t valid;
  uint8_t running;
} CurrentSense_Snapshot;

/* Call once after MX_DMA_Init and baseline timer setup; never from an ISR.
   Failure is latched until MCU reset; there is no automatic retry/recovery. */
HAL_StatusTypeDef CurrentSense_Start(void);
/* Foreground consumer: copies status even if invalid; NULL returns 0.
   Returns 1 only for a complete valid frame, NOT calibrated signed current.
   Reader must check timestamp freshness; raw is retained on an error. */
uint8_t CurrentSense_GetSnapshot(CurrentSense_Snapshot *out);

#ifdef __cplusplus
}
#endif
#endif

#ifndef TEST_TELEMETRY_HAL_H
#define TEST_TELEMETRY_HAL_H
#include "../stubs/stm32f4xx_hal.h"
typedef struct { uint32_t CR3; } USART_TypeDef;
typedef struct {
  USART_TypeDef *Instance;
  volatile uint32_t gState;
  DMA_HandleTypeDef *hdmatx;
} UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
#define USART_CR3_DMAT 0x80U
uint32_t __get_IPSR(void);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size);
#endif

#ifndef UART_TELEMETRY_H
#define UART_TELEMETRY_H
#include "stm32f4xx_hal.h"

#define UART_TELEMETRY_PERIOD_MS 100U
#define UART_TELEMETRY_BUFFER_SIZE 128U
#define UART_TELEMETRY_ECHO_SIZE 12U

/* Foreground only. One pending latest echo; DMA never reads the caller buffer. */
void UARTTelemetry_QueueEcho(const uint8_t *data, uint16_t size);
/* Sole USART1 TX owner. Nonblocking, latest snapshot, no control side effects. */
void UARTTelemetry_Poll(void);
#endif

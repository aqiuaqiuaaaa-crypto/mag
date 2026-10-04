/* Foreground USART1 TX arbitration. ADC/DMA callbacks remain measurement-only. */
#include "uart_telemetry.h"
#include "current_sense.h"
#include "usart.h"
#include <string.h>

/* Worst case: 6 uint32 + 6 uint16 + 2 flags, delimiters and CRLF = 112 bytes. */
typedef char BufferMustFit[(UART_TELEMETRY_BUFFER_SIZE >= 112U) ? 1 : -1];
__ALIGN_BEGIN static uint8_t tx_buffer[UART_TELEMETRY_BUFFER_SIZE] __ALIGN_END;
static uint8_t echo_buffer[UART_TELEMETRY_ECHO_SIZE];
static uint8_t echo_pending;
static uint32_t last_telemetry_ms;
/* Debugger-visible diagnostics. A TX failure never aborts RX or sampling. */
static volatile uint8_t tx_faulted;
static volatile uint32_t tx_error_count;

static uint8_t Foreground(void)
{
  return (uint8_t)(__get_IPSR() == 0U && __get_PRIMASK() == 0U);
}

static uint8_t TxReady(void)
{
  return (uint8_t)(huart1.gState == HAL_UART_STATE_READY &&
      huart1.hdmatx != NULL &&
      HAL_DMA_GetState(huart1.hdmatx) == HAL_DMA_STATE_READY &&
      (huart1.hdmatx->Instance->CR & DMA_SxCR_EN) == 0U);
}

static void AppendUnsigned(uint16_t *length, uint32_t value)
{
  uint8_t digits[10];
  uint32_t count = 0U;
  tx_buffer[(*length)++] = ',';
  do
  {
    digits[count++] = (uint8_t)('0' + value % 10U);
    value /= 10U;
  } while (value != 0U);
  while (count != 0U) tx_buffer[(*length)++] = digits[--count];
}

static uint16_t FormatSnapshot(const CurrentSense_Snapshot *snapshot)
{
  uint16_t length = 4U;
  uint32_t index;
  memcpy(tx_buffer, "@ADC", 4U);
  AppendUnsigned(&length, snapshot->frame_count);
  AppendUnsigned(&length, snapshot->timestamp_ms);
  for (index = 0U; index < CURRENT_SENSE_CHANNEL_COUNT; ++index)
    AppendUnsigned(&length, snapshot->raw[index]);
  AppendUnsigned(&length, snapshot->error_flags);
  AppendUnsigned(&length, snapshot->overrun_count);
  AppendUnsigned(&length, snapshot->dma_error_count);
  AppendUnsigned(&length, snapshot->dma_late_count);
  AppendUnsigned(&length, snapshot->valid != 0U ? 1U : 0U);
  AppendUnsigned(&length, snapshot->running != 0U ? 1U : 0U);
  tx_buffer[length++] = '\r';
  tx_buffer[length++] = '\n';
  return length;
}

static HAL_StatusTypeDef Transmit(uint16_t length)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  /* TX and ReceiveToIdle share huart1.Lock in this HAL version. Keep an RX
     rearm ISR from observing the brief TX setup lock. HAL DMA start has no
     wait loop; formatting and the actual wire transfer stay outside this
     bounded critical section. Do not abort or restart the RX DMA. */
  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  status = HAL_UART_Transmit_DMA(&huart1, tx_buffer, length);
  __set_PRIMASK(primask);
  /* HAL v1.8.1 does not propagate HAL_DMA_Start_IT failure. A normally
     completed DMA clears DMAT; a silent start failure leaves it set. */
  if (status == HAL_OK &&
      HAL_DMA_GetState(huart1.hdmatx) != HAL_DMA_STATE_BUSY &&
      (huart1.hdmatx->Instance->CR & DMA_SxCR_EN) == 0U &&
      (huart1.Instance->CR3 & USART_CR3_DMAT) != 0U)
    status = HAL_ERROR;
  if (status != HAL_OK && status != HAL_BUSY)
  {
    ++tx_error_count;
    tx_faulted = 1U;
  }
  return status;
}

void UARTTelemetry_QueueEcho(const uint8_t *data, uint16_t size)
{
  if (Foreground() == 0U || data == NULL || size != UART_TELEMETRY_ECHO_SIZE) return;
  memcpy(echo_buffer, data, UART_TELEMETRY_ECHO_SIZE);
  echo_pending = 1U;
}

void UARTTelemetry_Poll(void)
{
  uint32_t now;
  CurrentSense_Snapshot snapshot;
  if (Foreground() == 0U || tx_faulted != 0U || TxReady() == 0U) return;
  if (HAL_DMA_GetError(huart1.hdmatx) != HAL_DMA_ERROR_NONE)
  {
    ++tx_error_count;
    tx_faulted = 1U;
    return;
  }
  now = HAL_GetTick();
  /* Due telemetry has priority over echo. Busy slots retry on the next loop;
     no catch-up burst and no queue of stale snapshots. Unsigned tick wrap. */
  if ((uint32_t)(now - last_telemetry_ms) >= UART_TELEMETRY_PERIOD_MS)
  {
    (void)CurrentSense_GetSnapshot(&snapshot);
    /* Send invalid/faulted snapshots too, so the PC can diagnose sampling. */
    if (Transmit(FormatSnapshot(&snapshot)) == HAL_OK) last_telemetry_ms = now;
  }
  else if (echo_pending != 0U)
  {
    memcpy(tx_buffer, echo_buffer, UART_TELEMETRY_ECHO_SIZE);
    if (Transmit(UART_TELEMETRY_ECHO_SIZE) == HAL_OK) echo_pending = 0U;
  }
}

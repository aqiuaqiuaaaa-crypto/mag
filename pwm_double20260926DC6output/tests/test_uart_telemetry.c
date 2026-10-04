/* Execute the production owner and formatter; fake only HAL and snapshot source. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Src/uart_telemetry.c"

static DMA_Stream_TypeDef stream;
static DMA_HandleTypeDef dma;
static USART_TypeDef uart_instance;
UART_HandleTypeDef huart1;
static uint32_t tick, primask, ipsr, calls, snapshot_calls;
static uint8_t silent_start, instant_completion;
static uint8_t completion_during_check;
static uint8_t pending_rx, shared_lock, rx_rearmed;
static HAL_StatusTypeDef result = HAL_OK;
static CurrentSense_Snapshot source;
static const uint8_t *active;
static uint16_t active_size;
static char captured[UART_TELEMETRY_BUFFER_SIZE + 1U];

uint32_t __get_IPSR(void) { return ipsr; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t value)
{
  primask = value;
  if (pending_rx && primask == 0U)
  {
    assert(shared_lock == 0U);
    rx_rearmed = 1U;
  }
}
uint32_t HAL_GetTick(void) { return tick; }
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *handle)
{
  if (completion_during_check && calls != 0U)
  {
    handle->State = HAL_DMA_STATE_READY;
    stream.CR = 0U;
    uart_instance.CR3 = 0U;
  }
  return handle->State;
}
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *handle) { return handle->ErrorCode; }
uint8_t CurrentSense_GetSnapshot(CurrentSense_Snapshot *out)
{
  ++snapshot_calls;
  *out = source;
  return out->valid;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size)
{
  assert(uart == &huart1 && ipsr == 0U && primask == 1U);
  assert(size <= UART_TELEMETRY_BUFFER_SIZE);
  ++calls;
  active = data;
  active_size = size;
  memcpy(captured, data, size);
  captured[size] = 0;
  if (result != HAL_OK) return result;
  shared_lock = 1U;
  /* A pending RX event must wait until the shared HAL handle is unlocked. */
  if (pending_rx) assert(primask == 1U);
  huart1.gState = HAL_UART_STATE_BUSY_TX;
  uart_instance.CR3 = USART_CR3_DMAT;
  if (!silent_start)
  {
    dma.State = HAL_DMA_STATE_BUSY;
    stream.CR = DMA_SxCR_EN;
  }
  if (instant_completion)
  {
    dma.State = HAL_DMA_STATE_READY;
    stream.CR = 0U;
    uart_instance.CR3 = 0U;
  }
  shared_lock = 0U;
  return HAL_OK;
}

static void Complete(void)
{
  huart1.gState = HAL_UART_STATE_READY;
  dma.State = HAL_DMA_STATE_READY;
  stream.CR = 0U;
  uart_instance.CR3 = 0U;
}

int main(int argc, char **argv)
{
  uint32_t index;
  uint8_t echo[12] = "123456789012";
  assert(argc == 2);
  huart1.Instance = &uart_instance;
  huart1.hdmatx = &dma;
  dma.Instance = &stream;
  Complete();
  source.frame_count = 12345U;
  source.timestamp_ms = 456789U;
  for (index = 0U; index < 6U; ++index) source.raw[index] = (uint16_t)(1572U + index);
  source.valid = source.running = 1U;

  if (!strcmp(argv[1], "normal"))
  {
    tick = 99U; UARTTelemetry_Poll(); assert(calls == 0U);
    tick = 100U; UARTTelemetry_Poll();
    assert(!strcmp(captured, "@ADC,12345,456789,1572,1573,1574,1575,1576,1577,0,0,0,0,1,1\r\n"));
    Complete(); tick = 199U; UARTTelemetry_Poll(); assert(calls == 1U);
    tick = 200U; UARTTelemetry_Poll(); assert(calls == 2U);
  }
  else if (!strcmp(argv[1], "invalid"))
  {
    source.valid = source.running = 0U;
    source.error_flags = 524288U; source.dma_late_count = 1U;
    tick = 100U; UARTTelemetry_Poll();
    assert(strstr(captured, ",524288,0,0,1,0,0\r\n") != NULL);
  }
  else if (!strcmp(argv[1], "maximum"))
  {
    source.frame_count = source.timestamp_ms = source.error_flags = 0xFFFFFFFFU;
    source.overrun_count = source.dma_error_count = source.dma_late_count = 0xFFFFFFFFU;
    for (index = 0; index < 6; ++index) source.raw[index] = 65535U;
    tick = 100U; UARTTelemetry_Poll();
    assert(active_size == 112U && captured[active_size - 2U] == '\r');
    assert(strstr(captured, "4294967295") != NULL);
  }
  else if (!strcmp(argv[1], "busy-latest"))
  {
    tick = 100U; huart1.gState = HAL_UART_STATE_BUSY_TX;
    UARTTelemetry_Poll(); assert(calls == 0U && snapshot_calls == 0U);
    source.frame_count = 54321U; tick = 205U; Complete(); UARTTelemetry_Poll();
    assert(strstr(captured, "@ADC,54321,") == captured);
    Complete(); tick = 206U; UARTTelemetry_Poll(); assert(calls == 1U);
  }
  else if (!strcmp(argv[1], "echo-buffer"))
  {
    UARTTelemetry_QueueEcho(echo, 12U); memset(echo, '9', 12U);
    UARTTelemetry_Poll(); assert(!strcmp(captured, "123456789012"));
    UARTTelemetry_QueueEcho(echo, 12U); tick = 100U; UARTTelemetry_Poll();
    assert(calls == 1U && !memcmp(active, "123456789012", 12U));
    Complete(); UARTTelemetry_Poll(); assert(captured[0] == '@');
    Complete(); UARTTelemetry_Poll(); assert(!strcmp(captured, "999999999999"));
  }
  else if (!strcmp(argv[1], "echo-priority"))
  {
    UARTTelemetry_QueueEcho(echo, 12U); tick = 100U; UARTTelemetry_Poll();
    assert(captured[0] == '@' && echo_pending == 1U);
    Complete(); UARTTelemetry_Poll(); assert(!strcmp(captured, "123456789012"));
    Complete(); UARTTelemetry_Poll(); assert(calls == 2U);
  }
  else if (!strcmp(argv[1], "tick-wrap"))
  {
    tick = 0xFFFFFFF0U; UARTTelemetry_Poll(); Complete();
    tick = 83U; UARTTelemetry_Poll(); assert(calls == 1U);
    tick = 84U; UARTTelemetry_Poll(); assert(calls == 2U);
  }
  else if (!strcmp(argv[1], "isr-masked"))
  {
    tick = 100U; ipsr = 16U;
    UARTTelemetry_QueueEcho(echo, 12U); UARTTelemetry_Poll();
    ipsr = 0U; primask = 1U; UARTTelemetry_QueueEcho(echo, 12U); UARTTelemetry_Poll();
    assert(calls == 0U && snapshot_calls == 0U && echo_pending == 0U);
    primask = 0U; UARTTelemetry_Poll(); assert(calls == 1U);
  }
  else if (!strcmp(argv[1], "dma-ownership"))
  {
    tick = 100U; dma.State = HAL_DMA_STATE_BUSY; UARTTelemetry_Poll(); assert(calls == 0U);
    dma.State = HAL_DMA_STATE_READY; stream.CR = DMA_SxCR_EN;
    UARTTelemetry_Poll(); assert(calls == 0U);
    Complete(); UARTTelemetry_Poll(); assert(calls == 1U);
    /* DMA completion is insufficient: UART TC must release gState too. */
    dma.State = HAL_DMA_STATE_READY; stream.CR = 0U; tick = 200U;
    UARTTelemetry_Poll(); assert(calls == 1U);
  }
  else if (!strcmp(argv[1], "hal-busy"))
  {
    tick = 100U; result = HAL_BUSY; UARTTelemetry_Poll();
    assert(tx_faulted == 0U && last_telemetry_ms == 0U);
    result = HAL_OK; UARTTelemetry_Poll(); assert(calls == 2U);
  }
  else if (!strcmp(argv[1], "hal-error"))
  {
    tick = 100U; result = HAL_ERROR; UARTTelemetry_Poll();
    assert(tx_faulted == 1U && tx_error_count == 1U);
    result = HAL_OK; tick = 200U; UARTTelemetry_Poll(); assert(calls == 1U);
  }
  else if (!strcmp(argv[1], "dma-silent"))
  {
    tick = 100U; silent_start = 1U; UARTTelemetry_Poll();
    assert(tx_faulted == 1U && tx_error_count == 1U);
  }
  else if (!strcmp(argv[1], "dma-error"))
  {
    tick = 100U; dma.ErrorCode = 1U; UARTTelemetry_Poll();
    assert(calls == 0U && tx_faulted == 1U && tx_error_count == 1U);
  }
  else if (!strcmp(argv[1], "fast-complete"))
  {
    tick = 100U; instant_completion = 1U; UARTTelemetry_Poll();
    assert(tx_faulted == 0U && calls == 1U && last_telemetry_ms == 100U);
  }
  else if (!strcmp(argv[1], "bad-input"))
  {
    UARTTelemetry_QueueEcho(NULL, 12U); UARTTelemetry_QueueEcho(echo, 11U);
    UARTTelemetry_Poll(); assert(calls == 0U);
    huart1.hdmatx = NULL; tick = 100U; UARTTelemetry_Poll(); assert(calls == 0U);
  }
  else if (!strcmp(argv[1], "completion-race"))
  {
    tick = 100U; completion_during_check = 1U; UARTTelemetry_Poll();
    assert(tx_faulted == 0U && calls == 1U && last_telemetry_ms == 100U);
  }
  else if (!strcmp(argv[1], "rx-lock"))
  {
    tick = 100U; pending_rx = 1U; UARTTelemetry_Poll();
    assert(rx_rearmed == 1U && shared_lock == 0U && primask == 0U);
  }
  else if (!strcmp(argv[1], "echo-latest"))
  {
    UARTTelemetry_QueueEcho(echo, 12U);
    memset(echo, '8', 12U); UARTTelemetry_QueueEcho(echo, 12U);
    UARTTelemetry_Poll(); assert(!strcmp(captured, "888888888888"));
  }
  else { assert(0 && "unknown scenario"); }
  assert(primask == 0U);
  puts("production UART telemetry scenario PASS");
  return 0;
}

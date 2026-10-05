/* Actual production parser/RX/commit/watchdog/CCR loop, fake only peripherals.
   Natural preload transfers are separate from readable CCR registers. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "main_under_test.inc"

GPIO_TypeDef test_gpiof;
TIM_TypeDef test_tim1, test_tim2, test_tim8;
TIM_HandleTypeDef htim1, htim2, htim8;
UART_HandleTypeDef huart1;
static DMA_HandleTypeDef rx_dma;
static uint32_t tick, primask, sub_us, pwm_us, update_us;
static uint32_t active[6], main_channels[2], n_channels[2];
static uint32_t gpio_sets, gpio_resets, echoes, telemetry_polls, rx_rearms;
static uint64_t real_us, recovery_real_us;
static uint8_t tim2_running, enforce_zero_enable, overwrite_after_snapshot;
static uint8_t advance_tick_during_restore;
static uint8_t suppress_update[2];
static uint8_t captured_echo[12];
static const uint16_t sd_pins = GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_10;
static const int nonzero[6] = {30, -45, 60, -75, 90, -10};
#ifndef BASELINE
static const int next_command[6] = {-99, 99, -1, 1, -50, 50};
#endif
static const int zero[6] = {0, 0, 0, 0, 0, 0};

static void compare_values(uint32_t out[6])
{
  out[0] = TIM1->CCR1; out[1] = TIM1->CCR2; out[2] = TIM1->CCR3;
  out[3] = TIM8->CCR1; out[4] = TIM8->CCR2; out[5] = TIM8->CCR3;
}

static void commands(int out[6])
{
  out[0] = a0_amp; out[1] = a1_amp; out[2] = a2_amp;
  out[3] = a3_amp; out[4] = a4_amp; out[5] = a5_amp;
}

static void assert_outputs(const int expected[6])
{
  int actual[6];
  uint32_t ccr[6];
  unsigned i;
  commands(actual);
  compare_values(ccr);
  for (i = 0; i < 6; ++i)
  {
    assert(actual[i] == expected[i]);
    assert(ccr[i] == (uint32_t)(4200 + expected[i] * 42));
  }
}

static void assert_enabled(uint8_t enabled)
{
  assert((test_gpiof.ODR & sd_pins) == (enabled ? sd_pins : 0U));
}

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void __DMB(void) { }
void __set_PRIMASK(uint32_t value)
{
  primask = value;
  if (overwrite_after_snapshot && value == 0U && rx_done == 0U)
  {
    /* RX ISR replaces the shared mailbox after the main loop snapshot. */
    overwrite_after_snapshot = 0U;
    memset(process_buf, '!', sizeof(process_buf));
    rx_done = 1U;
    rx_len = FRAME_LEN;
  }
  if (advance_tick_during_restore && value == 0U)
  {
    advance_tick_during_restore = 0U;
    ++tick;
  }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
  uint32_t ccr[6];
  unsigned i;
  assert(port == GPIOF && pins == sd_pins);
  if (state == GPIO_PIN_SET)
  {
    ++gpio_sets;
    if (enforce_zero_enable)
    {
      assert_outputs(zero);
      compare_values(ccr);
      for (i = 0; i < 6; ++i) assert(active[i] == ccr[i]);
      assert(real_us - recovery_real_us >= 1000U);
      enforce_zero_enable = 0U;
    }
    port->ODR |= pins;
  }
  else
  {
    ++gpio_resets;
    port->ODR &= ~(uint32_t)pins;
  }
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim2); tim2_running = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim2); tim2_running = 0U; return HAL_OK;
}
static unsigned timer_index(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim1 || timer == &htim8);
  return timer == &htim1 ? 0U : 1U;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  main_channels[timer_index(timer)] |= 1U << (channel / 4U); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  n_channels[timer_index(timer)] |= 1U << (channel / 4U); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  main_channels[timer_index(timer)] &= ~(1U << (channel / 4U)); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  n_channels[timer_index(timer)] &= ~(1U << (channel / 4U)); return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *buffer, uint16_t size)
{
  assert(uart == &huart1 && buffer == rx_buf && size == RX_BUF_SIZE);
  ++rx_rearms; return HAL_OK;
}
void test_disable_dma_it(DMA_HandleTypeDef *dma, uint32_t interrupt)
{
  assert(dma == &rx_dma && interrupt == DMA_IT_HT);
}
HAL_StatusTypeDef CurrentSense_Start(void) { return HAL_OK; }
void UARTTelemetry_QueueEcho(const uint8_t *data, uint16_t size)
{
  assert(size == 12U); memcpy(captured_echo, data, size); ++echoes;
}
void UARTTelemetry_Poll(void) { ++telemetry_polls; }

static void advance_us(uint32_t duration, uint8_t poll)
{
  while (duration != 0U)
  {
    uint32_t step = duration;
    if (step > 1000U - sub_us) step = 1000U - sub_us;
    if (step > 500U - update_us) step = 500U - update_us;
    if (step > 50U - pwm_us) step = 50U - pwm_us;
    duration -= step;
    real_us += step;
    sub_us += step; update_us += step; pwm_us += step;
    if (sub_us == 1000U) { ++tick; sub_us = 0U; }
    if (update_us == 500U)
    {
      update_us = 0U;
      if (tim2_running && primask == 0U) HAL_TIM_PeriodElapsedCallback(&htim2);
    }
    if (pwm_us == 50U)
    {
      pwm_us = 0U;
      if (main_channels[0] == 7U && main_channels[1] == 7U)
      {
        uint32_t current[6]; unsigned channel;
        compare_values(current);
        for (channel = 0; channel < 6; ++channel)
          if (!suppress_update[channel / 3U]) active[channel] = current[channel];
        if (!suppress_update[0]) TIM1->SR |= TIM_FLAG_UPDATE;
        if (!suppress_update[1]) TIM8->SR |= TIM_FLAG_UPDATE;
      }
    }
    if (poll && sub_us == 0U) firmware_step();
  }
}

static void advance_ms(uint32_t duration) { advance_us(duration * 1000U, 1U); }

static uint16_t make_frame(char *frame, const int values[6])
{
  int size = sprintf(frame, "a0:%+03d,a1:%+03d,a2:%+03d,a3:%+03d,a4:%+03d,a5:%+03d\r\n",
                     values[0], values[1], values[2], values[3], values[4], values[5]);
  assert(size > 0 && size < 256);
  return (uint16_t)size;
}
static void receive(const char *frame, uint16_t size)
{
  uint16_t count = size < RX_BUF_SIZE ? size : RX_BUF_SIZE;
  memcpy(rx_buf, frame, count);
  HAL_UARTEx_RxEventCallback(&huart1, size);
  firmware_step();
}
static void send_values(const int values[6])
{
  char frame[256]; uint16_t size = make_frame(frame, values);
  assert(size == FRAME_LEN);
  receive(frame, size);
}
static void initialize(void)
{
  unsigned i;
  htim1.Instance = TIM1; htim2.Instance = TIM2; htim8.Instance = TIM8;
  huart1.hdmarx = &rx_dma;
  /* Generated timer setup's original Pulse=4200; its bytes are protected. */
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 4200U;
  TIM8->CCR1 = TIM8->CCR2 = TIM8->CCR3 = 4200U;
  for (i = 0; i < 6; ++i) active[i] = 4200U;
  /* GPIO latch RESET before output initialization, as in unchanged gpio.c. */
  test_gpiof.ODR = 0U;
  firmware_boot();
  assert(TIM2->ARR == 41999U && rx_rearms == 1U);
  assert_enabled(0U); assert_outputs(zero);
}
static void start_nonzero(void)
{
  send_values(nonzero); advance_ms(1U);
  assert_enabled(1U); assert_outputs(nonzero);
  assert(tim2_running && main_channels[0] == 7U && main_channels[1] == 7U);
  assert(n_channels[0] == 7U && n_channels[1] == 7U);
}

#ifndef BASELINE
static void timeout(void)
{
  start_nonzero(); advance_ms(299U);
  assert(watchdog_timeout_latched && !recovery_pending);
  assert_enabled(0U); assert_outputs(zero);
  /* Preloads are zero; active compare retains history until the next UEV. */
  assert(active[0] != 4200U);
  /* Timers keep running at their original settings under hardware shutdown. */
  assert(tim2_running && main_channels[0] == 7U && main_channels[1] == 7U);
  assert(n_channels[0] == 7U && n_channels[1] == 7U);
}
static void begin_recovery(const int values[6])
{
  recovery_real_us = real_us;
  enforce_zero_enable = 1U;
  send_values(values);
  assert(watchdog_timeout_latched && recovery_pending);
  assert_outputs(zero); assert_enabled(0U);
}
static void assert_not_fed(uint32_t previous_tick, uint32_t previous_echoes)
{
  assert(last_valid_cmd_tick == previous_tick && echoes == previous_echoes);
}

static void test_invalid(const char *kind)
{
  char frame[256], altered[256];
  uint16_t size;
  unsigned i, channel;
  uint32_t previous_tick, previous_echoes;
  start_nonzero(); advance_ms(99U);
  previous_tick = last_valid_cmd_tick; previous_echoes = echoes;
  size = make_frame(frame, nonzero);
  if (strcmp(kind, "short") == 0)
  {
    for (i = 0; i < FRAME_LEN; ++i) receive(frame, (uint16_t)i);
  }
  else if (strcmp(kind, "invalid-digits") == 0)
  {
    const unsigned char invalid[] = {'/', ':', 'A', ' ', '+', '-', 0, 255};
    for (channel = 0; channel < 6; ++channel)
      for (i = 0; i < sizeof(invalid); ++i)
      {
        unsigned digit;
        for (digit = 0; digit < 2; ++digit)
        {
          memcpy(altered, frame, size);
          altered[4U + channel * 7U + digit] = (char)invalid[i];
          receive(altered, size); assert_not_fed(previous_tick, previous_echoes);
        }
      }
  }
  else if (strcmp(kind, "out-of-range") == 0 || strcmp(kind, "overflow") == 0)
  {
    const char *number = strcmp(kind, "overflow") == 0 ? "+99999999999999999999" : "+100";
    size_t width = strlen(number);
    for (channel = 0; channel < 6; ++channel)
    {
      size_t offset = 3U + channel * 7U;
      memcpy(altered, frame, offset);
      memcpy(altered + offset, number, width);
      memcpy(altered + offset + width, frame + offset + 3U, FRAME_LEN - offset - 3U);
      receive(altered, (uint16_t)(FRAME_LEN + width - 3U));
      assert_not_fed(previous_tick, previous_echoes);
    }
  }
  else if (strcmp(kind, "parser-reject") == 0)
  {
    /* Correct numbers with reordered/duplicated channel labels still reject. */
    memcpy(altered, frame, size); altered[1] = '1'; altered[8] = '0';
    receive(altered, size); assert_not_fed(previous_tick, previous_echoes);
    memcpy(altered, frame, size); altered[36] = '4';
    receive(altered, size); assert_not_fed(previous_tick, previous_echoes);
  }
  else
  {
    /* Every fixed character, including signs, delimiters, labels and CRLF. */
    for (i = 0; i < FRAME_LEN; ++i)
    {
      if (i < 41U && (i % 7U == 4U || i % 7U == 5U)) continue;
      memcpy(altered, frame, size); altered[i] = '!';
      receive(altered, size); assert_not_fed(previous_tick, previous_echoes);
    }
  }
  assert_not_fed(previous_tick, previous_echoes);
  assert_outputs(nonzero);
  advance_ms(200U);  /* all invalid traffic failed to extend the 300 ms timer */
  assert(watchdog_timeout_latched); assert_outputs(zero); assert_enabled(0U);
}

static void test_case(const char *kind)
{
  unsigned i;
  firmware_step();
  if (strcmp(kind, "heartbeat") == 0 || strcmp(kind, "wrap-heartbeat") == 0)
  {
    if (strcmp(kind, "wrap-heartbeat") == 0) tick = UINT32_MAX - 100U;
    send_values(nonzero);
    for (i = 0; i < 180; ++i)
    {
      advance_ms(33U); send_values(nonzero);
      assert(!watchdog_timeout_latched && !recovery_pending);
      assert_enabled(1U); assert_outputs(nonzero); assert(last_valid_cmd_tick == tick);
    }
  }
  else if (strncmp(kind, "boundary-", 9) == 0 || strcmp(kind, "wrap-timeout") == 0)
  {
    uint32_t delay = strcmp(kind, "boundary-299") == 0 ? 299U :
                     strcmp(kind, "boundary-301") == 0 ? 301U : 300U;
    if (strcmp(kind, "wrap-timeout") == 0) tick = UINT32_MAX - 120U;
    send_values(nonzero); advance_ms(delay);
    assert(watchdog_timeout_latched == (delay >= CMD_TIMEOUT_MS));
    assert_enabled(delay < CMD_TIMEOUT_MS); assert_outputs(delay < CMD_TIMEOUT_MS ? nonzero : zero);
  }
  else if (strcmp(kind, "zero-equivalence") == 0)
  {
    int normal[6], expired[6]; uint32_t normal_ccr[6], expired_ccr[6];
    start_nonzero(); send_values(zero); advance_ms(1U);
    commands(normal); compare_values(normal_ccr);
    send_values(nonzero); advance_ms(300U);
    commands(expired); compare_values(expired_ccr);
    assert(memcmp(normal, expired, sizeof(normal)) == 0);
    assert(memcmp(normal_ccr, expired_ccr, sizeof(normal_ccr)) == 0);
  }
  else if (strcmp(kind, "gpio-groups") == 0)
  {
    test_gpiof.ODR = 1U << 12U;
    driver_enable_all(); assert_enabled(1U); assert(test_gpiof.ODR & (1U << 12U));
    driver_shutdown_all(); assert_enabled(0U); assert(test_gpiof.ODR & (1U << 12U));
    assert(gpio_sets == 1U && gpio_resets == 2U);
  }
  else if (strcmp(kind, "startup") == 0)
  {
    tick = UINT32_MAX - 100U; advance_ms(1000U);
    assert(!cmd_seen && !watchdog_timeout_latched && !recovery_pending);
    assert_outputs(zero); assert_enabled(0U);
    assert(!tim2_running && !main_channels[0] && !main_channels[1] && !gpio_sets);
    send_values(nonzero); advance_ms(1U); assert_outputs(nonzero); assert_enabled(1U);
  }
  else if (strcmp(kind, "recovery") == 0 || strcmp(kind, "recovery-wrap") == 0)
  {
    if (strcmp(kind, "recovery-wrap") == 0) tick = UINT32_MAX - 301U;
    timeout(); begin_recovery(next_command); advance_ms(1U);
    assert_enabled(0U); assert_outputs(zero); advance_ms(1U);
    assert_enabled(1U); assert_outputs(zero);
    assert(!watchdog_timeout_latched && !recovery_pending && !enforce_zero_enable);
    advance_ms(33U); assert_outputs(zero);  /* never resurrect prior frame */
    send_values(next_command); advance_ms(1U); assert_outputs(next_command);
  }
  else if (strcmp(kind, "stale") == 0)
  {
    timeout(); begin_recovery(zero); advance_ms(2U);
    assert_enabled(1U); assert_outputs(zero);
    advance_ms(298U); assert(watchdog_timeout_latched);
    assert_enabled(0U); assert_outputs(zero);
    begin_recovery(nonzero); advance_ms(2U);
    assert_enabled(1U); assert_outputs(zero);
    send_values(next_command); advance_ms(1U); assert_outputs(next_command);
  }
  else if (strcmp(kind, "recovery-quantization") == 0)
  {
    timeout(); advance_us(999U, 0U); begin_recovery(nonzero);
    advance_us(1U, 1U); assert_enabled(0U); assert_outputs(zero);
    advance_ms(1U); assert_enabled(1U); assert_outputs(zero);
  }
  else if (strcmp(kind, "recovery-expiry") == 0)
  {
    timeout(); begin_recovery(nonzero); advance_us(300000U, 0U);
    firmware_step(); assert_enabled(0U); assert_outputs(zero);
    assert(watchdog_timeout_latched && !recovery_pending && gpio_sets == 1U);
  }
  else if (strcmp(kind, "recovery-preload-proof") == 0)
  {
    timeout(); suppress_update[1] = 1U;
    begin_recovery(nonzero); advance_ms(2U);
    assert_enabled(0U); assert_outputs(zero); assert(watchdog_timeout_latched);
    assert((TIM1->SR & TIM_FLAG_UPDATE) != 0U && (TIM8->SR & TIM_FLAG_UPDATE) == 0U);
    assert(active[3] != 4200U);  /* second timer has NOT transferred its preload */
    suppress_update[1] = 0U; advance_us(50U, 0U); firmware_step();
    assert_enabled(1U); assert_outputs(zero); assert(!watchdog_timeout_latched);
  }
  else if (strcmp(kind, "recovery-early-frames") == 0)
  {
    timeout(); begin_recovery(nonzero); advance_us(500U, 0U);
    send_values(next_command); assert_outputs(zero); assert_enabled(0U);
    advance_ms(2U); assert_enabled(1U); assert_outputs(zero);
    send_values(next_command); advance_ms(1U); assert_outputs(next_command);
  }
  else if (strcmp(kind, "late-frame") == 0)
  {
    start_nonzero(); advance_us(300000U, 0U);
    recovery_real_us = real_us; enforce_zero_enable = 1U;
    send_values(next_command); assert_outputs(zero); assert_enabled(0U);
    assert(watchdog_timeout_latched && recovery_pending);
    advance_ms(2U); assert_enabled(1U); assert_outputs(zero);
  }
  else if (strcmp(kind, "commit-boundary") == 0)
  {
    char frame[256]; uint16_t size;
    start_nonzero(); advance_ms(298U); assert(tick == 299U);
    size = make_frame(frame, next_command);
    advance_tick_during_restore = 1U;
    /* First restore is the pre-RX watchdog poll; the commit rechecks 300. */
    receive(frame, size); assert(watchdog_timeout_latched && recovery_pending);
    assert_outputs(zero); assert_enabled(0U);
  }
  else if (strcmp(kind, "snapshot-race") == 0)
  {
    char frame[256]; uint16_t size; uint32_t previous_tick;
    start_nonzero(); advance_ms(9U); previous_tick = tick;
    size = make_frame(frame, next_command);
    memcpy(rx_buf, frame, size); HAL_UARTEx_RxEventCallback(&huart1, size);
    /* Injection waits for rx_done==0 at the snapshot's interrupt restore. */
    overwrite_after_snapshot = 1U;
    firmware_step();
    HAL_TIM_PeriodElapsedCallback(&htim2);
    assert_outputs(next_command); assert(last_valid_cmd_tick == previous_tick);
    assert(memcmp(captured_echo, "999901015050", 12U) == 0 && echoes == 2U);
    advance_ms(1U); assert(last_valid_cmd_tick == previous_tick && echoes == 2U);
  }
  else if (strcmp(kind, "primask") == 0)
  {
    primask = 1U; command_watchdog_poll(); assert(primask == 1U);
    commit_command(1, 2, 3, 4, 5, 6); assert(primask == 1U);
    tick = CMD_TIMEOUT_MS; command_watchdog_poll(); assert(primask == 1U);
    assert_outputs(zero); assert_enabled(0U);
    __set_PRIMASK(0U);
  }
  else if (strcmp(kind, "foreign-callbacks") == 0)
  {
    UART_HandleTypeDef uart = {0}; TIM_HandleTypeDef timer = {0};
    HAL_UARTEx_RxEventCallback(&uart, FRAME_LEN);
    HAL_TIM_PeriodElapsedCallback(&timer);
    firmware_step(); assert(!cmd_seen && !rx_done && echoes == 0U);
    assert_outputs(zero); assert_enabled(0U);
  }
  else if (strcmp(kind, "trailing-data") == 0)
  {
    char frame[256]; uint16_t size = make_frame(frame, nonzero);
    memcpy(frame + size, "ignored", 7U); receive(frame, size + 7U);
    advance_ms(1U); assert_outputs(nonzero); assert(!watchdog_timeout_latched);
    /* Preserve original >=43 prefix acceptance; do not alter UART framing. */
  }
  else test_invalid(kind);
  assert(primask == 0U && telemetry_polls > 0U);
  puts("production command watchdog scenario PASS");
}
#endif

static void trace(void)
{
  unsigned channel, i; int value, actual[6], target[6]; uint32_t ccr[6];
  start_nonzero();
  /* Exhaust all 199 legal values on each of six channels, mixed signs on the
     other channels. Identical binary trace is compared with checkpoint 4f18. */
  for (channel = 0; channel < 6; ++channel)
    for (value = -99; value <= 99; ++value)
    {
      memcpy(target, nonzero, sizeof(target)); target[channel] = value;
      advance_ms(33U); send_values(target); advance_ms(1U);
      assert_outputs(target); commands(actual); compare_values(ccr);
      printf("%u %u", tick, (unsigned)test_gpiof.ODR);
      for (i = 0; i < 6; ++i) printf(" %d %u %u", actual[i], ccr[i], active[i]);
      printf(" %u %u %u %u %u\n", (unsigned)main_channels[0],
             (unsigned)main_channels[1], (unsigned)n_channels[0],
             (unsigned)n_channels[1], (unsigned)echoes);
    }
}

int main(int argc, char **argv)
{
#ifdef _WIN32
  /* Failed assertions must return to the runner, not open a modal GUI dialog. */
  SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
  _set_error_mode(_OUT_TO_STDERR);
#endif
  assert(argc == 2); initialize();
  if (strcmp(argv[1], "trace") == 0) trace();
#ifdef BASELINE
  else if (strcmp(argv[1], "baseline-timeout") == 0)
  {
    start_nonzero(); advance_ms(350U);
    assert_outputs(nonzero); assert_enabled(1U);
    printf("Checkpoint 4f18 after 350 ms without RX: a0=%d CCR1=%u CTRL_SD=%u\n",
           a0_amp, (unsigned)TIM1->CCR1, (unsigned)test_gpiof.ODR);
  }
#endif
#ifndef BASELINE
  else test_case(argv[1]);
#endif
  return 0;
}

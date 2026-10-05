"""Exact reversible watchdog edits, preserving legacy main.c bytes and fixtures.

Only this allowlist is undone by the older ADC/telemetry baseline checks. New
watchdog tests execute the resulting production functions, including the RX
snapshot and commit path. Do not recapture either historical hash fixture.
"""


def crlf(text: str) -> bytes:
    return text.replace("\n", "\r\n").encode("ascii")


DEFINES = crlf("""
/* PC heartbeat is approximately 33 ms. Timeout includes the 300 ms boundary. */
#define CMD_TIMEOUT_MS 300U
/* Two HAL tick increments give a conservative nominal 1..2 ms zero hold.
   Also require new natural TIM1/TIM8 update events to prove CCR preloads
   actually transferred, independently of SysTick interrupt latency.
   No forced update event, timer restart, frequency or polarity change. */
#define CMD_RECOVERY_ZERO_MS 2U
#define DRIVER_CTRL_SD_PINS (GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_10)
""")
STATE = crlf("""
/* Foreground-owned watchdog state, volatile also for bench debugger inspection.
   Only a validated command commit updates last_valid_cmd_tick. */
static volatile uint32_t last_valid_cmd_tick = 0U;
static volatile uint8_t cmd_seen = 0U;
static volatile uint8_t watchdog_timeout_latched = 0U;
static volatile uint8_t recovery_pending = 0U;
static volatile uint32_t recovery_zero_tick = 0U;
""")
FUNCTIONS = crlf("""
/* Confirmed custom groups: PF10 -> poles 1/4/5/6, PF2 -> pole 2,
   PF7 -> pole 3. Existing firmware convention: HIGH enable, LOW shutdown. */
static void driver_shutdown_all(void)
{
    HAL_GPIO_WritePin(GPIOF, DRIVER_CTRL_SD_PINS, GPIO_PIN_RESET);
}

static void driver_enable_all(void)
{
    HAL_GPIO_WritePin(GPIOF, DRIVER_CTRL_SD_PINS, GPIO_PIN_SET);
}

/* Caller masks interrupts. Reuse the exact TIM2 command-to-CCR path; the
   callback only writes CCRs and is safe to invoke here without a timer event. */
static void clear_command_outputs(void)
{
    a0_amp = 0;
    a1_amp = 0;
    a2_amp = 0;
    a3_amp = 0;
    a4_amp = 0;
    a5_amp = 0;
    HAL_TIM_PeriodElapsedCallback(&htim2);
}

/* Caller masks interrupts. Shutdown first protects the power stage while
   zero CCR preloads propagate. PWM timers keep their original configuration
   and keep running; the separate CTRL_SD pins disable all six drivers. */
static void enter_command_timeout_safe_state(void)
{
    driver_shutdown_all();
    clear_command_outputs();
    watchdog_timeout_latched = 1U;
    recovery_pending = 0U;
}

/* Foreground only; no delay, allocation, UART operation or busy wait.
   An unseen command stays in the existing boot shutdown state indefinitely.
   Check expiry before recovery, so a stalled recovery cannot re-enable. */
static void command_watchdog_poll(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;
    __disable_irq();
    now = HAL_GetTick();
    if (cmd_seen != 0U &&
        (uint32_t)(now - last_valid_cmd_tick) >= CMD_TIMEOUT_MS)
    {
        if (watchdog_timeout_latched == 0U || recovery_pending != 0U)
            enter_command_timeout_safe_state();
    }
    else if (recovery_pending != 0U &&
             (uint32_t)(now - recovery_zero_tick) >= CMD_RECOVERY_ZERO_MS &&
             __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET &&
             __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET)
    {
        /* All commits during recovery kept commands/CCRs zero. Both timers
           have produced a new natural UEV AFTER all zero CCR writes, so
           ACTIVE compare values (not merely readable preloads) are zero.
           Scheme A: the recovery frame's target is discarded; only a NEW
           frame after this enable may apply a nonzero command. */
        recovery_pending = 0U;
        watchdog_timeout_latched = 0U;
        driver_enable_all();
    }
    __DMB();
    __set_PRIMASK(primask);
}

static void commit_command(int t0, int t1, int t2, int t3, int t4, int t5)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Recheck at the actual commit boundary, including a HAL tick that
       advanced during validation. A late valid frame starts safe recovery. */
    command_watchdog_poll();
    if (watchdog_timeout_latched != 0U)
    {
        clear_command_outputs();
        if (recovery_pending == 0U)
        {
            /* Clear only the unused TIM1/TIM8 update flags AFTER zero writes.
               These timers have no update IRQ/DMA enabled. A subsequent
               natural UEV proves transfer; never force EGR.UG or restart. */
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
            __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
            recovery_zero_tick = HAL_GetTick();
            recovery_pending = 1U;
        }
    }
    else
    {
        a0_amp = t0;
        a1_amp = t1;
        a2_amp = t2;
        a3_amp = t3;
        a4_amp = t4;
        a5_amp = t5;
    }
    /* A recovery commit deliberately commits ZERO, never its old target.
       Invalid/rejected data never reaches this function or feeds the timer. */
    last_valid_cmd_tick = HAL_GetTick();
    cmd_seen = 1U;
    __DMB();
    __set_PRIMASK(primask);
}

""")
DIGITS = crlf(
    """    /* Two decimal digits per channel: parsing cannot fail or exceed +/-99.
       The original parser returns zero on an invalid digit; reject it BEFORE
       commit instead of confusing a parse failure with a valid zero command. */
    {
        uint16_t channel;
        for (channel = 0U; channel < 6U; ++channel)
        {
            uint16_t digit = (uint16_t)(4U + channel * 7U);
            if (buf[digit] < '0' || buf[digit] > '9' ||
                buf[digit + 1U] < '0' || buf[digit + 1U] > '9')
                return 0;
        }
    }

"""
)
OLD_COMMIT = crlf("""                __disable_irq();

                a0_amp = t0;
                a1_amp = t1;
                a2_amp = t2;
                a3_amp = t3;
                a4_amp = t4;
                a5_amp = t5;

                __enable_irq();
""")
SNAPSHOT = crlf("""        len = rx_len;
        if (len > RX_BUF_SIZE) len = RX_BUF_SIZE;
        /* RX callback may replace process_buf after interrupts resume.
           Validate, parse and echo this one bounded immutable snapshot. */
        memcpy(frame, process_buf, len);
        rx_done = 0;
        __DMB();
        __set_PRIMASK(primask);
""")


def gpio_block(state: str) -> bytes:
    return crlf(f"""    HAL_GPIO_WritePin(
        GPIOF,
        GPIO_PIN_2 |
        GPIO_PIN_7 |
        GPIO_PIN_10,
        GPIO_PIN_{state}
    );
""")


PATCHES = [
    (b"/* USER CODE END PD */", DEFINES + b"\r\n/* USER CODE END PD */"),
    (b"/* USER CODE END PV */", STATE + b"\r\n/* USER CODE END PV */"),
    (b"/* USER CODE END 0 */", FUNCTIONS + b"/* USER CODE END 0 */"),
    (
        b"    return 1;\r\n}\r\n\r\n\r\n/* USER CODE END 0 */",
        DIGITS + b"    return 1;\r\n}\r\n\r\n\r\n/* USER CODE END 0 */",
    ),
    (
        b"        uint16_t len;\r\n",
        crlf("""        uint16_t len;
        uint8_t frame[RX_BUF_SIZE];
        uint32_t primask;
"""),
    ),
    (
        b"        rx_done = 0;\r\n",
        crlf("""        primask = __get_PRIMASK();
        __disable_irq();
"""),
    ),
    (b"        len = rx_len;\r\n", SNAPSHOT),
    (OLD_COMMIT, b"                commit_command(t0, t1, t2, t3, t4, t5);\r\n"),
    (
        gpio_block("SET"),
        b"    driver_enable_all(); /* Existing first-command startup. */\r\n",
    ),
    (
        gpio_block("RESET"),
        b"    driver_shutdown_all(); /* Existing boot/PWM stop. */\r\n",
    ),
    (
        b"    if (rx_done)\r\n",
        crlf(
            """    /* Check before RX: a frame arriving after the deadline recovers safely. */
    command_watchdog_poll();
    if (rx_done)
"""
        ),
    ),
]
# Validate the parser before appending new functions to its unique return anchor.
PATCHES[2], PATCHES[3] = PATCHES[3], PATCHES[2]
for old in (
    [b"Check_Frame(process_buf, len)"]
    + [
        f"PARSE_SIGNED(process_buf, {offset})".encode("ascii")
        for offset in (3, 10, 17, 24, 31, 38)
    ]
    + [
        f"= process_buf[{offset}];".encode("ascii")
        for offset in (4, 5, 11, 12, 18, 19, 25, 26, 32, 33, 39, 40)
    ]
):
    PATCHES.append((old, old.replace(b"process_buf", b"frame")))


def apply(data: bytes) -> bytes:
    for old, new in PATCHES:
        assert data.count(old) == 1, old
        data = data.replace(old, new, 1)
    return data


def restore(data: bytes) -> bytes:
    """Any edit outside the exact watchdog allowlist remains detectable."""
    for old, new in reversed(PATCHES):
        assert data.count(new) == 1, new
        data = data.replace(new, old, 1)
    return data

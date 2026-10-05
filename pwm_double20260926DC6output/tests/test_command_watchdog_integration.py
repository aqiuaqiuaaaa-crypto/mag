"""Scope/timing contracts supplement production C/HAL execution tests."""

import hashlib
import re
import unittest
from pathlib import Path

from test_adc_infrastructure import c_function
from watchdog_main_patch import restore

ROOT = Path(__file__).resolve().parents[1]


class CommandWatchdogIntegrationTests(unittest.TestCase):
    def test_exact_allowlist_leaves_unrelated_edits_visible(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        restored = restore(data)
        original = hashlib.sha256(restored).hexdigest()
        self.assertNotEqual(
            hashlib.sha256(restore(data + b"/* unrelated */")).hexdigest(), original
        )
        self.assertNotIn(b"watchdog_timeout_latched", restored)
        self.assertIn(b"HAL_TIM_PeriodElapsedCallback", restored)

    def test_zero_uses_the_unmodified_timer_formula(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        self.assertEqual(
            c_function(data, "HAL_TIM_PeriodElapsedCallback"),
            c_function(restore(data), "HAL_TIM_PeriodElapsedCallback"),
        )
        clear = c_function(data, "clear_command_outputs")
        self.assertEqual(clear.count(b"HAL_TIM_PeriodElapsedCallback(&htim2);"), 1)
        self.assertNotRegex(clear, rb"CCR|4200|42\b")

    def test_timeout_is_central_wrap_safe_and_foreground(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        self.assertEqual(data.count(b"#define CMD_TIMEOUT_MS 300U"), 1)
        self.assertEqual(data.count(b"last_valid_cmd_tick = HAL_GetTick();"), 1)
        poll = c_function(data, "command_watchdog_poll")
        self.assertIn(b"(uint32_t)(now - last_valid_cmd_tick) >= CMD_TIMEOUT_MS", poll)
        self.assertIn(b"cmd_seen != 0U", poll)
        self.assertLess(
            poll.index(b">= CMD_TIMEOUT_MS"), poll.index(b"driver_enable_all();")
        )
        self.assertNotRegex(poll, rb"HAL_Delay|HAL_UART_|malloc|while\s*\(")
        for name in ("HAL_UARTEx_RxEventCallback", "HAL_TIM_PeriodElapsedCallback"):
            self.assertNotIn(b"watchdog", c_function(data, name))

    def test_gpio_polarity_group_and_boot_order(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        self.assertIn(
            b"#define DRIVER_CTRL_SD_PINS (GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_10)", data
        )
        self.assertIn(b"GPIO_PIN_RESET", c_function(data, "driver_shutdown_all"))
        self.assertIn(b"GPIO_PIN_SET", c_function(data, "driver_enable_all"))
        self.assertEqual(data.count(b"HAL_GPIO_WritePin("), 2)
        gpio = (ROOT / "Core/Src/gpio.c").read_text()
        self.assertLess(
            gpio.index("HAL_GPIO_WritePin("), gpio.index("HAL_GPIO_Init(GPIOF")
        )
        self.assertIn("GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET", gpio)
        self.assertLess(
            data.index(b"  Stop_PWM();"), data.index(b"  HAL_UARTEx_ReceiveToIdle_DMA(")
        )

    def test_recovery_time_covers_preload_updates(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        self.assertIn(b"#define CMD_RECOVERY_ZERO_MS 2U", data)
        timer = (ROOT / "Core/Src/tim.c").read_bytes()
        hal = (
            ROOT / "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
        ).read_bytes()
        for name in ("MX_TIM1_Init", "MX_TIM8_Init"):
            body = c_function(timer, name)
            self.assertIn(b"TIM_COUNTERMODE_CENTERALIGNED1", body)
            self.assertIn(b".Prescaler = 0", body)
            self.assertIn(b".Period = 8399", body)
            self.assertIn(b".RepetitionCounter = 0", body)
        for bit in (b"TIM_CCMR1_OC1PE", b"TIM_CCMR1_OC2PE", b"TIM_CCMR2_OC3PE"):
            self.assertIn(bit, c_function(hal, "HAL_TIM_PWM_ConfigChannel"))
        poll = c_function(data, "command_watchdog_poll")
        commit = c_function(data, "commit_command")
        for name in (b"htim1", b"htim8"):
            self.assertIn(b"__HAL_TIM_GET_FLAG(&" + name + b", TIM_FLAG_UPDATE)", poll)
            self.assertIn(
                b"__HAL_TIM_CLEAR_FLAG(&" + name + b", TIM_FLAG_UPDATE)", commit
            )
        self.assertLess(
            commit.index(b"clear_command_outputs();"),
            commit.index(b"__HAL_TIM_CLEAR_FLAG"),
        )
        self.assertNotRegex(
            timer, rb"EnableIRQ\(TIM(?:1|8)|Base_Start_IT\(&htim(?:1|8)"
        )
        self.assertLess(2 * (8399 + 1) / 168_000_000, 0.001)
        for name in (
            "clear_command_outputs",
            "command_watchdog_poll",
            "enter_command_timeout_safe_state",
            "commit_command",
        ):
            body = c_function(data, name)
            body = re.sub(rb"/\*.*?\*/|//[^\n]*", b"", body, flags=re.DOTALL)
            self.assertNotRegex(body, rb"EGR|GENERATE_EVENT|Base_Stop|PWM_Stop|Delay")

    def test_snapshot_validation_and_commit_order(self):
        data = (ROOT / "Core/Src/main.c").read_bytes()
        loop = data.split(b"/* USER CODE BEGIN 3 */", 1)[1].split(
            b"/* USER CODE END 3 */", 1
        )[0]
        self.assertLess(
            loop.index(b"memcpy(frame, process_buf, len)"),
            loop.index(b"Check_Frame(frame, len)"),
        )
        self.assertLess(
            loop.index(b"Check_Frame(frame, len)"),
            loop.index(b"commit_command(t0, t1, t2, t3, t4, t5)"),
        )
        self.assertNotRegex(
            loop, rb"PARSE_SIGNED\(process_buf|tx_buf\[\d+\]\s*= process_buf"
        )
        commit = c_function(data, "commit_command")
        self.assertLess(
            commit.index(b"a5_amp = t5"), commit.index(b"last_valid_cmd_tick =")
        )
        self.assertLess(
            commit.index(b"command_watchdog_poll()"), commit.index(b"a0_amp = t0")
        )
        parser = c_function(data, "Check_Frame")
        self.assertIn(b"digit + 1U", parser)
        self.assertEqual(
            c_function(data, "PARSE_SIGNED"), c_function(restore(data), "PARSE_SIGNED")
        )

    def test_telemetry_and_acquisition_have_no_watchdog_dependency(self):
        for name in ("adc", "current_sense", "uart_telemetry"):
            data = (ROOT / f"Core/Src/{name}.c").read_bytes()
            self.assertNotRegex(
                data, rb"watchdog|CMD_TIMEOUT|driver_shutdown|driver_enable"
            )
        data = (ROOT / "Core/Src/main.c").read_bytes()
        self.assertEqual(len(re.findall(rb"UARTTelemetry_Poll\(\);", data)), 1)
        self.assertNotIn(b"@ADC", data)


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""MAP provenance fault injection for measurement removal acceptance rules."""

import unittest

from verify_image import symbols, verify_no_measurement


def default_map() -> str:
    return "\n".join(
        f"    {name} 0x08000191 Thumb Code 0 startup_stm32f407xx.o(.text)"
        for name in ("ADC_IRQHandler", "DMA2_Stream0_IRQHandler")
    )


class NoMeasurementImageTests(unittest.TestCase):
    def check_map(self, text: str) -> dict[str, str]:
        return verify_no_measurement(text, symbols(text))

    def test_generic_startup_aliases_are_allowed(self) -> None:
        result = self.check_map(default_map())
        self.assertEqual(len(result), 2)

    def test_sampling_objects_are_rejected_even_if_dead_stripped(self) -> None:
        for module in (
            "adc",
            "current_sense",
            "uart_telemetry",
            "stm32f4xx_hal_adc",
            "stm32f4xx_hal_adc_ex",
        ):
            with self.subTest(module=module), self.assertRaisesRegex(
                ValueError, "object remains"
            ):
                self.check_map(default_map() + f"\nRemoving {module}.o(i.unused)")

    def test_real_sampling_handlers_are_rejected(self) -> None:
        for name in ("ADC_IRQHandler", "DMA2_Stream0_IRQHandler"):
            text = default_map().replace(
                f"{name} 0x08000191 Thumb Code 0 startup_stm32f407xx.o(.text)",
                f"{name} 0x08000201 Thumb Code 24 stm32f4xx_it.o(i.{name})",
            )
            with self.subTest(handler=name), self.assertRaisesRegex(
                ValueError, "Active sampling IRQ"
            ):
                self.check_map(text)

    def test_renamed_object_cannot_hide_sampling_symbols(self) -> None:
        for name in (
            "CurrentSense_Start",
            "UARTTelemetry_Poll",
            "MX_ADC1_Init",
            "MX_TIM3_Init",
            "HAL_ADC_Start_DMA",
            "FormatSnapshot",
        ):
            with self.subTest(symbol=name), self.assertRaisesRegex(
                ValueError, "implementation remains"
            ):
                self.check_map(
                    default_map()
                    + f"\n {name} 0x08000201 Thumb Code 24 renamed.o(i.{name})"
                )

    def test_sampling_state_without_code_is_rejected(self) -> None:
        for name in ("hadc1", "hdma_adc1", "htim3", "latest_snapshot", "pending_echo"):
            with self.subTest(state=name), self.assertRaisesRegex(
                ValueError, "state remains"
            ):
                self.check_map(
                    default_map() + f"\n {name} 0x20000020 Data 4 renamed.o(.bss)"
                )

    def test_missing_default_alias_fails_provenance(self) -> None:
        with self.assertRaisesRegex(ValueError, "Missing startup vector"):
            self.check_map("")


if __name__ == "__main__":
    unittest.main(verbosity=2)

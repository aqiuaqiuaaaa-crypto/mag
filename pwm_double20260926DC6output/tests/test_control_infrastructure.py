"""Protect signed control, original peripheral configuration and safety additions."""

import hashlib
import json
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from c_source import c_function
from watchdog_main_patch import restore

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / "tests/signed_pwm_baseline.json").read_text())


class ControlInfrastructureTests(unittest.TestCase):
    def test_original_control_functions_and_peripherals(self):
        for relative, functions in FIXTURE["functions"].items():
            data = (ROOT / relative).read_bytes()
            if relative == "Core/Src/main.c":
                data = restore(data)
            for name, expected in functions.items():
                with self.subTest(file=relative, function=name):
                    self.assertEqual(
                        hashlib.sha256(c_function(data, name)).hexdigest(), expected
                    )
        for relative, expected in FIXTURE["files"].items():
            with self.subTest(file=relative):
                self.assertEqual(
                    hashlib.sha256((ROOT / relative).read_bytes()).hexdigest(), expected
                )

    def test_only_watchdog_and_echo_buffer_guard_differ_from_original_main(self):
        data = restore((ROOT / "Core/Src/main.c").read_bytes())
        for addition in (
            b"                    uint32_t echo_primask;\r\n",
            b"                    /* TX/RX share huart1.Lock; RX rearm must not see TX setup locked. */\r\n                    echo_primask = __get_PRIMASK();\r\n                    __disable_irq();\r\n",
            b"                    __set_PRIMASK(echo_primask);\r\n",
        ):
            self.assertEqual(data.count(addition), 1)
            data = data.replace(addition, b"", 1)
        start = data.index(b"                /* Keep the DMA-owned echo buffer")
        end = data.index(b"                }\r\n", start) + len(
            b"                }\r\n"
        )
        guarded = data[start:end]
        copy_start = guarded.index(b"                    tx_buf[0]")
        call_start = guarded.index(b"                    HAL_UART_Transmit_DMA(")
        copies = guarded[copy_start:call_start]
        copies = b"".join(
            line[4:] if line.startswith(b"    ") else line
            for line in copies.splitlines(keepends=True)
        )
        legacy = (
            copies
            + b"                if (huart1.gState == HAL_UART_STATE_READY)\r\n                {\r\n"
            + guarded[call_start:]
        )
        data = data[:start] + legacy + data[end:]
        self.assertEqual(hashlib.sha256(data).hexdigest(), FIXTURE["main_sha256"])

    def test_cube_control_parameters_and_counts(self):
        values = dict(
            line.split("=", 1)
            for line in (ROOT / "pwm_02.ioc").read_text().splitlines()
            if "=" in line and not line.startswith("#")
        )
        for key, expected in FIXTURE["ioc"].items():
            self.assertEqual(values[key], expected, key)
        for prefix, count_key in (
            ("Mcu.IP", "Mcu.IPNb"),
            ("Mcu.Pin", "Mcu.PinsNb"),
            ("Dma.Request", "Dma.RequestsNb"),
        ):
            indices = sorted(
                int(key[len(prefix) :])
                for key in values
                if re.fullmatch(re.escape(prefix) + r"\d+", key)
            )
            self.assertEqual(indices, list(range(int(values[count_key]))))
        self.assertNotRegex(
            "\n".join(values), r"ADC1|TIM3|Stream0|^(?:PB0|PA6|PA3|PC2|PC3|PA4)\."
        )

    def test_keil_source_lists_and_numbering_agree(self):
        project = ET.parse(ROOT / "MDK-ARM/pwm_02.uvprojx")
        options = ET.parse(ROOT / "MDK-ARM/pwm_02.uvoptx")
        sources = [
            entry.findtext("FilePath").replace("\\", "/")
            for entry in project.findall(".//Files/File")
        ]
        files = options.findall(".//Group/File")
        paths = [
            entry.findtext("PathWithFileName").replace("\\", "/") for entry in files
        ]
        self.assertEqual(sources, paths)
        self.assertEqual(
            [int(entry.findtext("FileNumber")) for entry in files],
            list(range(1, len(files) + 1)),
        )
        for relative in sources:
            self.assertTrue((ROOT / "MDK-ARM" / relative).is_file(), relative)
            self.assertNotRegex(relative, r"adc|current_sense|uart_telemetry")
        config = (ROOT / "Core/Inc/stm32f4xx_hal_conf.h").read_text()
        self.assertNotIn("HAL_ADC_MODULE_ENABLED", config)
        for relative in (
            "Core/Src/main.c",
            "Core/Src/stm32f4xx_it.c",
            "Core/Src/dma.c",
            "Core/Src/tim.c",
        ):
            self.assertNotRegex(
                (ROOT / relative).read_text(encoding="latin1"),
                r"CurrentSense|UARTTelemetry|HAL_ADC_|\bhtim3\b|DMA2_Stream0",
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)

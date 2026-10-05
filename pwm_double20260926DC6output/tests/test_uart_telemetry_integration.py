"""Protect the phase-1 acquisition, RX control and ISR bytes, with exact edits."""
import hashlib
import json
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from telemetry_main_patch import restore
from watchdog_main_patch import restore as restore_watchdog_main

ROOT = Path(__file__).resolve().parents[1]


class TelemetryIntegrationTests(unittest.TestCase):
    def test_phase1_bytes_unchanged(self):
        baseline = json.loads(Path(__file__).with_name("telemetry_phase1_baseline.json").read_text())
        for relative, expected in baseline["files"].items():
            data = (ROOT / relative).read_bytes()
            if relative == "Core/Src/main.c":
                data = restore(restore_watchdog_main(data))
            elif relative == "MDK-ARM/pwm_02.uvprojx":
                entry = baseline["project_addition"].encode("ascii")
                self.assertEqual(data.count(entry), 1)
                data = data.replace(entry, b"", 1)
            with self.subTest(file=relative):
                self.assertEqual(hashlib.sha256(data).hexdigest(), expected)

    def test_one_foreground_tx_owner(self):
        source = (ROOT / "Core/Src/uart_telemetry.c").read_text()
        main = (ROOT / "Core/Src/main.c").read_bytes().decode("latin1")
        self.assertEqual(main.count("UARTTelemetry_QueueEcho(tx_buf, 12U)"), 1)
        self.assertEqual(main.count("UARTTelemetry_Poll();"), 1)
        self.assertNotIn("HAL_UART_Transmit", main)
        occurrences = []
        for path in (ROOT / "Core").rglob("*.c"):
            text = path.read_bytes().decode("latin1")
            occurrences.extend((path.name, token) for token in re.findall(r"HAL_UART_Transmit(?:_DMA|_IT)?\s*\(", text))
        self.assertEqual(occurrences, [("uart_telemetry.c", "HAL_UART_Transmit_DMA(")])
        self.assertNotRegex(source, r"HAL_ADC_|HAL_UART_(?:Receive|Abort)|printf|malloc|HAL_Delay|CCR[1-4]")
        self.assertIn("CurrentSense_GetSnapshot(&snapshot)", source)
        self.assertIn("__get_IPSR()", source)
        self.assertIn("HAL_DMA_GetState(huart1.hdmatx)", source)
        self.assertIn("DMA_SxCR_EN", source)
        self.assertLess(source.index("__disable_irq();"), source.index("HAL_UART_Transmit_DMA("))
        self.assertLess(source.index("HAL_UART_Transmit_DMA("), source.index("__set_PRIMASK(primask);"))

    def test_project_and_rate(self):
        project = ET.parse(ROOT / "MDK-ARM/pwm_02.uvprojx")
        self.assertEqual([n.text for n in project.findall(".//FileName")].count("uart_telemetry.c"), 1)
        header = (ROOT / "Core/Inc/uart_telemetry.h").read_text()
        self.assertIn("UART_TELEMETRY_PERIOD_MS 100U", header)
        self.assertIn("UART_TELEMETRY_BUFFER_SIZE 128U", header)


if __name__ == "__main__":
    unittest.main(verbosity=2)

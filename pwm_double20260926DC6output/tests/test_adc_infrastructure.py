"""Read-only integration checks; never import/run the PC controller or flash MCU."""

import hashlib
import json
import re
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from telemetry_main_patch import restore as restore_telemetry_main

FIRMWARE = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).with_name("signed_pwm_baseline.json")
CHANNELS = [8, 6, 3, 12, 13, 4]
PROTECTED_FUNCTIONS = {
    "Core/Src/main.c": [
        "PARSE_SIGNED", "Check_Frame", "SystemClock_Config", "Start_PWM",
        "Stop_PWM", "HAL_UARTEx_RxEventCallback", "HAL_TIM_PeriodElapsedCallback",
        "Error_Handler",
    ],
    "Core/Src/tim.c": [
        "MX_TIM1_Init", "MX_TIM2_Init", "MX_TIM8_Init", "HAL_TIM_MspPostInit",
    ],
    "Core/Src/stm32f4xx_it.c": [
        "TIM2_IRQHandler", "USART1_IRQHandler", "DMA2_Stream2_IRQHandler",
        "DMA2_Stream7_IRQHandler", "SysTick_Handler",
    ],
}
MAIN_INCLUDE = b'#include "current_sense.h"\r\n'
MAIN_START = (
    b'\r\n  /* Measurement-only ADC path; failures do not change PWM. */\r\n'
    b'  (void)CurrentSense_Start();\r\n'
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def c_function(data, name):
    """Extract original bytes, ignoring braces in C comments/literals."""
    text = data.decode("latin1")
    lexical = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S)
    masked = lexical.sub(lambda m: "".join("\n" if c == "\n" else " " for c in m[0]), text)
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", masked)
    if not match:
        raise AssertionError("C function missing: " + name)
    start = text.rfind("\n", 0, match.start()) + 1
    depth = 1
    index = match.end()
    while depth:
        depth += (masked[index] == "{") - (masked[index] == "}")
        index += 1
    return data[start:index]


def ioc_values(path):
    return dict(line.split("=", 1) for line in path.read_text().splitlines()
                if "=" in line and not line.startswith("#"))


def capture_baseline():
    protected = ["Core/Src/usart.c", "Core/Src/gpio.c", "Core/Inc/main.h",
                 "Core/Src/stm32f4xx_hal_msp.c", "Core/Src/system_stm32f4xx.c",
                 "MDK-ARM/startup_stm32f407xx.s"]
    protected += [str(p.relative_to(FIRMWARE)) for p in
                  (FIRMWARE / "Drivers").rglob("*") if p.is_file()]
    return {
        "files": {p: digest((FIRMWARE / p).read_bytes()) for p in sorted(protected)},
        "functions": {p: {name: digest(c_function((FIRMWARE / p).read_bytes(), name))
                          for name in names} for p, names in PROTECTED_FUNCTIONS.items()},
        "main_sha256": digest((FIRMWARE / "Core/Src/main.c").read_bytes()),
        "ioc": {k: v for k, v in ioc_values(FIRMWARE / "pwm_02.ioc").items()
                if k.startswith(("TIM1.", "TIM2.", "TIM8.", "USART1.", "Dma.USART1_",
                                 "RCC.", "PF2.", "PF7.", "PF10.", "PA8.", "PA9.",
                                 "PA10.", "PB13.", "PB14.", "PB15.", "PB6.", "PB7.",
                                 "PH13.", "PH14.", "PH15.", "PI5.", "PI6.", "PI7.",
                                 "NVIC.USART1", "NVIC.TIM2", "NVIC.DMA2_Stream2",
                                 "NVIC.DMA2_Stream7", "NVIC.SysTick", "NVIC.PriorityGroup"))},
    }


class ADCInfrastructureTests(unittest.TestCase):
    def read(self, relative):
        path = FIRMWARE / relative
        self.assertTrue(path.is_file(), "required ADC infrastructure file missing: " + relative)
        return path.read_text(encoding="latin1")

    def test_01_adc_module_and_rank_order(self):
        source = self.read("Core/Src/adc.c")
        match = re.search(r"regular_channels\s*\[[^]]+\]\s*=\s*\{([^}]+)\}", source, re.S)
        self.assertIsNotNone(match, "regular scan channel table missing")
        self.assertEqual([int(n) for n in re.findall(r"ADC_CHANNEL_(\d+)", match[1])], CHANNELS)
        self.assertRegex(source, r"sConfig\.Rank\s*=\s*index\s*\+\s*1U")
        for setting in ["ADC_CLOCK_SYNC_PCLK_DIV4", "ADC_RESOLUTION_12B",
                        "ADC_EXTERNALTRIGCONV_T3_TRGO", "ADC_EXTERNALTRIGCONVEDGE_RISING",
                        "ADC_DATAALIGN_RIGHT", "ADC_EOC_SEQ_CONV", "ADC_SAMPLETIME_84CYCLES"]:
            self.assertIn(setting, source)
        self.assertRegex(source, r"ContinuousConvMode\s*=\s*DISABLE")
        self.assertRegex(source, r"DMAContinuousRequests\s*=\s*ENABLE")

    def test_02_analog_pins_and_jp5_warning(self):
        source = self.read("Core/Src/adc.c")
        self.assertRegex(source, r"HAL_GPIO_Init\(GPIOA,")
        self.assertRegex(source, r"HAL_GPIO_Init\(GPIOB,")
        self.assertRegex(source, r"HAL_GPIO_Init\(GPIOC,")
        self.assertIn("GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6", source)
        self.assertIn("GPIO_PIN_2|GPIO_PIN_3", source)
        self.assertIn("GPIO_MODE_ANALOG", source)
        self.assertIn("GPIO_NOPULL", source)
        self.assertIn("__HAL_RCC_GPIOC_CLK_ENABLE", source)
        self.assertIn("PA4/JP5", source)
        self.assertNotRegex(source, r"GPIO_PIN_(?:7|10|13|14|15)\b")

    def test_03_dma_stream_and_alignment(self):
        source = self.read("Core/Src/adc.c")
        for token in ["DMA2_Stream0", "DMA_CHANNEL_0", "DMA_CIRCULAR",
                      "DMA_PDATAALIGN_HALFWORD", "DMA_MDATAALIGN_HALFWORD",
                      "__HAL_LINKDMA", "DMA_PRIORITY_LOW"]:
            self.assertIn(token, source)
        self.assertNotRegex(source, r"DMA2_Stream[27]|DMA_CHANNEL_4")

    def test_04_tim3_only_trigger_and_no_timer_irq(self):
        source = self.read("Core/Src/tim.c")
        body = c_function(source.encode("latin1"), "MX_TIM3_Init").decode("latin1")
        for token in ["TIM3", "839", "199", "TIM_TRGO_UPDATE", "TIM_COUNTERMODE_UP"]:
            self.assertIn(token, body)
        self.assertNotIn("Error_Handler", body)
        acquisition = self.read("Core/Src/current_sense.c")
        self.assertIn("HAL_TIM_Base_Start(&htim3)", acquisition)
        self.assertNotIn("HAL_TIM_Base_Start_IT", acquisition)
        self.assertEqual(84_000_000 // (839 + 1) // (199 + 1), 500)

    def test_05_snapshot_api_and_complete_frames(self):
        header = self.read("Core/Inc/current_sense.h")
        source = self.read("Core/Src/current_sense.c")
        for field in ["raw", "frame_count", "timestamp_ms", "valid", "running",
                      "error_flags", "overrun_count", "dma_error_count", "dma_late_count"]:
            self.assertIn(field, header)
        self.assertIn("CURRENT_SENSE_CHANNEL_COUNT", header)
        for token in ["CurrentSense_GetSnapshot", "__get_PRIMASK", "__disable_irq", "__set_PRIMASK"]:
            self.assertIn(token, source)
        self.assertRegex(source, r"dma_buffer\[2U\s*\*\s*CURRENT_SENSE_CHANNEL_COUNT\]")
        self.assertIn("&dma_buffer[0]", source)
        self.assertIn("&dma_buffer[CURRENT_SENSE_CHANNEL_COUNT]", source)
        self.assertIn("HAL_ADC_ConvHalfCpltCallback", source)
        self.assertIn("HAL_ADC_ConvCpltCallback", source)
        self.assertIn("HAL_ADC_ErrorCallback", source)

    def test_06_no_measurement_to_pwm_or_serial_path(self):
        source = self.read("Core/Src/current_sense.c") + self.read("Core/Src/adc.c")
        stripped = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
        self.assertNotRegex(stripped, r"\bTIM(?:1|2|8)\b|CCR[1-4]|a[0-5]_amp|Start_PWM|Stop_PWM|Error_Handler")
        self.assertNotRegex(stripped, r"HAL_UART_|printf\s*\(|\bGPIOF\b|\bfloat\b|\bdouble\b")
        self.assertNotIn("1.27", stripped)
        self.assertNotIn("0.12", stripped)

    def test_07_irq_routing_and_priority(self):
        irq = self.read("Core/Src/stm32f4xx_it.c").encode("latin1")
        self.assertIn(b"HAL_DMA_IRQHandler(&hdma_adc1)", c_function(irq, "DMA2_Stream0_IRQHandler"))
        self.assertIn(b"HAL_ADC_IRQHandler(&hadc1)", c_function(irq, "ADC_IRQHandler"))
        dma = self.read("Core/Src/dma.c")
        self.assertIn("HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0)", dma)
        self.assertIn("HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn)", dma)
        self.assertIn("HAL_NVIC_SetPriority(ADC_IRQn, 3, 0)", self.read("Core/Src/adc.c"))

    def test_08_cube_configuration_matches_runtime(self):
        config = ioc_values(FIRMWARE / "pwm_02.ioc")
        self.assertIn("ADC1.NbrOfConversion", config, "ADC1 configuration missing")
        self.assertEqual(config["ADC1.NbrOfConversion"], "6")
        self.assertEqual([config[f"ADC1.Channel-{i}\\#ChannelRegularConversion"] for i in range(6)],
                         [f"ADC_CHANNEL_{n}" for n in CHANNELS])
        self.assertEqual([config[f"ADC1.Rank-{i}\\#ChannelRegularConversion"] for i in range(6)],
                         [str(i + 1) for i in range(6)])
        for i in range(6):
            self.assertEqual(config[f"ADC1.SamplingTime-{i}\\#ChannelRegularConversion"], "ADC_SAMPLETIME_84CYCLES")
        self.assertEqual(config["Dma.ADC1.2.Instance"], "DMA2_Stream0")
        self.assertEqual(config["Dma.ADC1.2.Mode"], "DMA_CIRCULAR")
        self.assertEqual(config["TIM3.Prescaler"], "839")
        self.assertEqual(config["TIM3.Period"], "199")
        self.assertEqual(config["TIM3.MasterOutputTrigger"], "TIM_TRGO_UPDATE")
        self.assertNotIn("NVIC.TIM3_IRQn", config)
        self.assertEqual(config["ADC1.ContinuousConvMode"], "DISABLE")
        self.assertEqual(config["ADC1.ExternalTrigConv"], "ADC_EXTERNALTRIGCONV_T3_TRGO")

    def test_09_same_version_hal_dependencies_and_keil_sources(self):
        conf = self.read("Core/Inc/stm32f4xx_hal_conf.h")
        self.assertRegex(conf, r"(?m)^\s*#define HAL_ADC_MODULE_ENABLED\s*$")
        project = ET.parse(FIRMWARE / "MDK-ARM/pwm_02.uvprojx")
        names = [n.text for n in project.findall(".//FileName")]
        for name in ["adc.c", "current_sense.c", "stm32f4xx_hal_adc.c", "stm32f4xx_hal_adc_ex.c"]:
            self.assertEqual(names.count(name), 1, name + " missing/duplicated in Keil")
        for name in ["stm32f4xx_hal_adc.h", "stm32f4xx_hal_adc_ex.h", "stm32f4xx_ll_adc.h"]:
            self.read("Drivers/STM32F4xx_HAL_Driver/Inc/" + name)
        for entry in project.findall(".//Files/File"):
            relative = entry.findtext("FilePath").replace("\\", "/")
            self.assertTrue((FIRMWARE / "MDK-ARM" / relative).is_file(), relative)

    def test_10_protected_functions_byte_identical(self):
        baseline = json.loads(FIXTURE.read_text())
        for relative, functions in baseline["functions"].items():
            data = (FIRMWARE / relative).read_bytes()
            for name, expected in functions.items():
                with self.subTest(file=relative, function=name):
                    self.assertEqual(digest(c_function(data, name)), expected)

    def test_11_original_gpio_uart_hal_and_startup_byte_identical(self):
        baseline = json.loads(FIXTURE.read_text())
        for relative, expected in baseline["files"].items():
            with self.subTest(file=relative):
                self.assertEqual(digest((FIRMWARE / relative).read_bytes()), expected)

    def test_12_protected_cube_parameters_unchanged(self):
        baseline = json.loads(FIXTURE.read_text())
        config = ioc_values(FIRMWARE / "pwm_02.ioc")
        for key, expected in baseline["ioc"].items():
            with self.subTest(key=key):
                self.assertEqual(config[key], expected)

    def test_13_main_only_two_additions_encoding_preserved(self):
        baseline = json.loads(FIXTURE.read_text())
        data = (FIRMWARE / "Core/Src/main.c").read_bytes()
        # Reverse only the authorized foreground telemetry edits, then enforce
        # the unchanged pre-ADC SHA-256. Never recapture the original fixture.
        data = restore_telemetry_main(data)
        self.assertEqual(data.count(MAIN_INCLUDE), 1, "measurement header missing")
        self.assertEqual(data.count(MAIN_START), 1, "measurement startup missing")
        self.assertEqual(digest(data.replace(MAIN_INCLUDE, b"").replace(MAIN_START, b"")), baseline["main_sha256"])
        self.assertLess(data.index(b"Stop_PWM();"), data.index(b"(void)CurrentSense_Start();"))
        self.assertLess(data.index(b"MX_DMA_Init();"), data.index(b"(void)CurrentSense_Start();"))

    def test_14_vendor_adc_version_and_integrity(self):
        manifest = json.loads(Path(__file__).with_name("vendor_adc_sources.json").read_text())
        self.assertEqual(manifest["tag"], "v1.27.1")
        for relative, expected in manifest["sha256"].items():
            with self.subTest(file=relative):
                self.assertEqual(digest((FIRMWARE / "Drivers/STM32F4xx_HAL_Driver" / relative).read_bytes()), expected)

    def test_15_cube_gpio_dma_and_interrupt_completeness(self):
        config = ioc_values(FIRMWARE / "pwm_02.ioc")
        self.assertEqual(config["Dma.RequestsNb"], "3")
        self.assertEqual(config["Dma.Request2"], "ADC1")
        for pin, channel in zip(["PB0", "PA6", "PA3", "PC2", "PC3", "PA4"], CHANNELS):
            with self.subTest(pin=pin):
                self.assertEqual(config[pin + ".Signal"], f"ADCx_IN{channel}")
                self.assertEqual(config[f"SH.ADCx_IN{channel}.0"], f"ADC1_IN{channel},IN{channel}")
        for key in ["Dma.ADC1.2.PeriphDataAlignment", "Dma.ADC1.2.MemDataAlignment"]:
            self.assertEqual(config[key], "DMA_PDATAALIGN_HALFWORD" if "Periph" in key else "DMA_MDATAALIGN_HALFWORD")
        for key in ["NVIC.ADC_IRQn", "NVIC.DMA2_Stream0_IRQn"]:
            self.assertTrue(config[key].startswith("true\\:3\\:0\\:"))
        header = self.read("Core/Inc/stm32f4xx_it.h")
        self.assertIn("void DMA2_Stream0_IRQHandler(void);", header)
        self.assertIn("void ADC_IRQHandler(void);", header)
        self.assertNotRegex(self.read("Core/Inc/stm32f4xx_hal_conf.h"), r"(?m)^\s*#define HAL_DAC_MODULE_ENABLED\s*$")

    def test_16_complete_frame_validation_before_publication(self):
        source = self.read("Core/Src/current_sense.c")
        body = c_function(source.encode("latin1"), "PublishFrame").decode("latin1")
        self.assertEqual(body.count("FrameIsStable(half)"), 2)
        self.assertEqual(body.count("ConversionHasError()"), 2)
        self.assertLess(body.rindex("FrameIsStable(half)"), body.index("published.raw[index] = stable_raw[index]"))
        for token in ["__HAL_DMA_GET_COUNTER", "__HAL_DMA_GET_FLAG", "DMA_FLAG_HTIF0_4",
                      "DMA_FLAG_TCIF0_4", "HAL_DMA_GetError", "__HAL_ADC_GET_FLAG"]:
            self.assertIn(token, source)


if __name__ == "__main__":
    if sys.argv[1:] == ["--capture-baseline"]:
        print(json.dumps(capture_baseline(), indent=2, sort_keys=True))
    else:
        unittest.main(verbosity=2)

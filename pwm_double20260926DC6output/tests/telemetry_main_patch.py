"""Exact, reversible ASCII edits; protect legacy main.c bytes and original fixture."""

INCLUDE = b'#include "uart_telemetry.h"\r\n'
OLD_TX = (
    b'                if (huart1.gState == HAL_UART_STATE_READY)\r\n'
    b'                {\r\n'
    b'                    HAL_UART_Transmit_DMA(\r\n'
    b'                        &huart1,\r\n'
    b'                        tx_buf,\r\n'
    b'                        12\r\n'
    b'                    );\r\n'
    b'                }\r\n'
)
NEW_TX = b'                UARTTelemetry_QueueEcho(tx_buf, 12U);\r\n'
POLL = (
    b'    /* Foreground-only UART owner; ADC continues at 500 Hz. */\r\n'
    b'    UARTTelemetry_Poll();\r\n'
)
LOOP_END = b'    }\r\n\r\n  }\r\n\r\n  /* USER CODE END 3 */'


def apply(data: bytes) -> bytes:
    """Apply only the three authorized foreground changes."""
    assert INCLUDE not in data and data.count(OLD_TX) == 1
    assert data.count(LOOP_END) == 1
    data = data.replace(b'#include "current_sense.h"\r\n',
                        b'#include "current_sense.h"\r\n' + INCLUDE, 1)
    return data.replace(OLD_TX, NEW_TX, 1).replace(
        LOOP_END, b'    }\r\n\r\n' + POLL + b'  }\r\n\r\n  /* USER CODE END 3 */', 1)


def restore(data: bytes) -> bytes:
    """Undo the exact allowlist; any other byte change remains detectable."""
    assert data.count(INCLUDE) == data.count(NEW_TX) == data.count(POLL) == 1
    assert OLD_TX not in data
    return data.replace(INCLUDE, b'', 1).replace(NEW_TX, OLD_TX, 1).replace(POLL, b'', 1)

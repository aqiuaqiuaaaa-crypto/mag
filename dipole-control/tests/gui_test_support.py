"""Shared headless GUI fixtures; never open a real serial port or camera."""

from __future__ import annotations

import os
import sys
import time
from collections.abc import Callable, Iterator
from pathlib import Path
from threading import get_ident
from types import SimpleNamespace
from typing import cast

import cv2
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import magnetic_dipole_pid as gui
from PySide6.QtWidgets import QApplication


class MemorySerial:
    """One duplex in-memory port; record thread ownership and bounded reads."""

    def __init__(self) -> None:
        self.incoming = bytearray()
        self.frames: list[bytes] = []
        self.read_sizes: list[int] = []
        self.thread_ids: list[int] = []
        self.closed = False
        self.fail_rx = False
        self.fail_tx = False
        self.on_close: Callable[[], None] | None = None

    @property
    def in_waiting(self) -> int:
        if self.fail_rx:
            raise OSError("simulated RX failure")
        return len(self.incoming)

    def read(self, size: int = 1) -> bytes:
        assert not self.closed
        self.thread_ids.append(get_ident())
        self.read_sizes.append(size)
        chunk = bytes(self.incoming[:size])
        del self.incoming[:size]
        return chunk

    def write(self, data: bytes) -> int:
        if self.fail_tx:
            raise OSError("simulated TX failure")
        assert not self.closed and len(data) == cfg.SERIAL_FRAME_BYTES
        self.thread_ids.append(get_ident())
        self.frames.append(data)
        return len(data)

    def close(self) -> None:
        if self.on_close is not None:
            self.on_close()
        self.closed = True


@pytest.fixture(scope="session")
def app() -> QApplication:
    """Reuse Qt's single application without requiring pytest-qt."""
    instance = QApplication.instance()
    if instance is None:
        instance = QApplication([])
    assert isinstance(instance, QApplication)
    return instance


@pytest.fixture
def ports(monkeypatch: pytest.MonkeyPatch) -> list[MemorySerial]:
    """Replace the actual COM opener and enumeration before constructing a GUI."""
    opened: list[MemorySerial] = []

    def open_port(
        port: str, baud: int, *, timeout: int, write_timeout: float
    ) -> MemorySerial:
        assert port == "MOCK-COM" and baud == cfg.BAUDRATE and timeout == 0
        assert write_timeout == 0.05
        serial = MemorySerial()
        opened.append(serial)
        return serial

    monkeypatch.setattr(gui, "_serial", SimpleNamespace(Serial=open_port))
    monkeypatch.setattr(gui, "list_ports", SimpleNamespace(comports=list))
    return opened


@pytest.fixture
def window(
    app: QApplication,
    ports: list[MemorySerial],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> Iterator[gui.MagneticDipoleControl]:
    """Isolate settings and prohibit any camera opening in all GUI tests."""
    monkeypatch.setattr(gui, "SETTINGS_FILE", str(tmp_path / "settings.json"))

    def no_camera(*args: object, **kwargs: object) -> None:
        raise AssertionError("GUI tests must not open hardware")

    monkeypatch.setattr(cv2, "VideoCapture", no_camera)
    # The legacy GUI constructor is unannotated; contain that type boundary.
    factory = cast(Callable[[], gui.MagneticDipoleControl], gui.MagneticDipoleControl)
    widget = factory()
    widget.timer.stop()  # Advance existing control manually when needed.
    widget.combo_port.addItem("MOCK-COM")
    assert widget.ser is None and widget.cap is None and widget.model_ok
    yield widget
    widget.close()
    app.processEvents()


def connect(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> MemorySerial:
    """Use the real connect slot, with the fixture's single mocked COM opener."""
    window.toggle_serial()
    port = ports[-1]
    assert window.ser is port
    assert isinstance(port, MemorySerial)
    return port


def pump_until(app: QApplication, ready: Callable[[], bool]) -> None:
    """Drive real QTimers with a short deadline, without real device activity."""
    deadline = time.monotonic() + 2.0
    while not ready() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert ready()

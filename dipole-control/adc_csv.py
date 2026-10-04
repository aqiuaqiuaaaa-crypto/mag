"""Bounded, asynchronous ADC CSV recording; no serial or control access."""

from __future__ import annotations

import csv
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread

from curt_telemetry import CSV_HEADER, ADCSnapshot


class ADCLogger:
    """Write immutable snapshots off the GUI thread, preserving existing files."""

    def __init__(self, path: Path, capacity: int = 256) -> None:
        if capacity < 1:
            raise ValueError("ADC logging queue capacity must be positive")
        self.path = path
        self.rows_written = 0
        self.error = ""
        self.opened = Event()
        self.finished = Event()
        self._stopping = Event()
        self._rows: Queue[tuple[str | int, ...]] = Queue(maxsize=capacity)
        self._thread = Thread(target=self._run, name="ADC CSV", daemon=True)
        self._thread.start()

    def enqueue(self, snapshot: ADCSnapshot, timestamp_pc: str) -> bool:
        """Queue a complete frame without waiting for disk I/O or queue space."""
        if self._stopping.is_set() or self.finished.is_set():
            return False
        try:
            self._rows.put_nowait(snapshot.csv_row(timestamp_pc))
        except Full:
            self.error = "ADC CSV queue full; recording stopped"
            self.request_stop()
            return False
        return True

    def request_stop(self) -> None:
        """Request a drain, flush and close without waiting in the RX callback."""
        self._stopping.set()

    def wait_closed(self, timeout: float = 1.0) -> bool:
        """Bound the exit-only wait; regular receiving never calls this method."""
        self._thread.join(timeout=timeout)
        return not self._thread.is_alive()

    def _run(self) -> None:
        """Own the file exclusively and drain accepted rows before closing it."""
        try:
            with self.path.open("x", encoding="utf-8", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(CSV_HEADER)
                output.flush()
                self.opened.set()
                while not self._stopping.is_set() or not self._rows.empty():
                    try:
                        row = self._rows.get(timeout=0.05)
                    except Empty:
                        continue
                    writer.writerow(row)
                    output.flush()
                    self.rows_written += 1
        except (OSError, ValueError) as error:
            self.error = f"ADC CSV: {error}"
        finally:
            self.finished.set()

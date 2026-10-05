"""Fault, accounting and read-back tests for the independent disk writer."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import time
from itertools import pairwise
from pathlib import Path
from threading import Event, get_ident
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import control_log
from control_log import CONTROL_FIELDS, WORKER_FIELDS, ControlLogger, Scalar


def wait_ready(sink: ControlLogger) -> None:
    deadline = time.monotonic() + 3
    while not sink.opened.is_set() and not sink.finished.is_set():
        assert time.monotonic() < deadline
        time.sleep(0.001)
    assert sink.opened.is_set(), sink.error


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def test_round_trip_immutable_and_stop(tmp_path: Path) -> None:
    sink = ControlLogger(tmp_path / "run.csv", {"offset": [0] * 6})
    wait_ready(sink)
    for tick in range(30):
        values: dict[str, Scalar] = {
            "t_mono": time.monotonic(),
            "t_wall": time.time(),
            "tick": tick,
            "mode": "AUTO_TRACK",
            "tracking": True,
            "cmd_sent_a0": -9,
            "raw0": 4095,
            "F_target_x": 0.125,
        }
        assert sink.enqueue("control", values)
        values["tick"] = -1
    assert sink.enqueue("worker", {"seq": 1, "run": 2, "x0_x": 3.5})
    assert sink.wait_closed(3)
    rows = read_rows(sink.path)
    assert len(rows) == 30 and tuple(rows[0]) == CONTROL_FIELDS
    assert [int(row["tick"]) for row in rows] == list(range(30))
    timestamps = [float(row["t_mono"]) for row in rows]
    assert all(b > a for a, b in pairwise(timestamps))
    assert all(row["tracking"] == "True" and int(row["raw0"]) == 4095 for row in rows)
    assert all(
        float(row["F_target_x"]) == 0.125 and row["meas_x_mm"] == "" for row in rows
    )
    assert tuple(read_rows(sink.worker_path)[0]) == WORKER_FIELDS
    meta = json.loads(sink.metadata_path.read_text(encoding="utf8"))
    assert meta["schema_version"] == 1 and meta["offset"] == [0] * 6
    assert len(meta["git_commit"]) == 40
    summary = json.loads(sink.summary_path.read_text(encoding="utf8"))
    assert summary["stats"]["control"] == {
        "attempted": 30,
        "accepted": 30,
        "written": 30,
        "dropped": 0,
        "rejected": 0,
    }
    assert not sink.enqueue("control", {"tick": 100})
    assert sink.stats()["control"]["rejected"] == 1


def test_full_queue_counts_and_recovers(tmp_path: Path) -> None:
    gate = Event()

    class HeldLogger(ControlLogger):
        def _run(self) -> None:
            assert gate.wait(3)
            super()._run()

    sink = HeldLogger(tmp_path / "bounded.csv", {}, capacity=1)
    try:
        assert sink.enqueue("control", {"tick": 1})
        for tick in range(2, 22):
            assert not sink.enqueue("control", {"tick": tick})
        assert sink.dropped == 20 and sink.error is None and sink.recording
        gate.set()
        wait_ready(sink)
        deadline = time.monotonic() + 3
        while sink.stats()["control"]["written"] != 1:
            assert time.monotonic() < deadline
            time.sleep(0.001)
        assert sink.enqueue("control", {"tick": 22})
        assert sink.wait_closed(3)
        rows = read_rows(sink.path)
        assert [int(row["tick"]) for row in rows] == [1, 22]
        assert rows[-1]["dropped"] == "20"
        stats = sink.stats()["control"]
        assert stats["attempted"] == stats["written"] + stats["dropped"] == 22
    finally:
        gate.set()
        sink.wait_closed(3)


@pytest.mark.parametrize(
    "suffix", [".csv", ".worker.csv", ".metadata.json", ".summary.json"]
)
def test_never_overwrites(tmp_path: Path, suffix: str) -> None:
    existing = (tmp_path / "run.csv").with_suffix(suffix)
    existing.write_text("KEEP", encoding="utf8")
    sink = ControlLogger(tmp_path / "run.csv", {})
    assert sink.wait_closed(3)
    assert "FileExistsError" in (sink.error or "")
    assert existing.read_text(encoding="utf8") == "KEEP"


def test_invalid_capacity(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="positive"):
        ControlLogger(tmp_path / "run.csv", {}, capacity=0)


def test_missing_directory_is_writer_error(tmp_path: Path) -> None:
    sink = ControlLogger(tmp_path / "missing" / "run.csv", {})
    assert sink.wait_closed(3) and sink.error and not sink.recording


def test_mid_stream_disk_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    real_writer = control_log.csv.writer

    class BrokenWriter:
        def __init__(self, file: Any) -> None:
            self.writer = real_writer(file)
            self.count = 0

        def writerow(self, row: Any) -> None:
            self.count += 1
            if self.count > 1:
                raise OSError("disk full")
            self.writer.writerow(row)

    monkeypatch.setattr(control_log.csv, "writer", BrokenWriter)
    sink = ControlLogger(tmp_path / "full.csv", {})
    wait_ready(sink)
    assert sink.enqueue("control", {"tick": 1})
    assert sink.finished.wait(3)
    assert sink.error == "OSError: disk full"
    assert sink.stats()["control"]["accepted"] == 1
    assert sink.stats()["control"]["written"] == 0


def test_all_io_stays_on_writer_thread(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    owner = get_ident()
    threads: list[int] = []
    original = Path.open

    def observed_open(path: Path, *args: Any, **kwargs: Any) -> Any:
        threads.append(get_ident())
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", observed_open)
    sink = ControlLogger(tmp_path / "threads.csv", {})
    assert sink.wait_closed(3)
    assert len(threads) == 4 and all(thread != owner for thread in threads)


@pytest.mark.parametrize(
    "failure", [OSError("no git"), subprocess.TimeoutExpired("git", 2)]
)
def test_git_failure_is_metadata_only(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure: Exception
) -> None:
    def fail_git(*args: object, **kwargs: object) -> None:
        raise failure

    monkeypatch.setattr(control_log.subprocess, "run", fail_git)
    sink = ControlLogger(tmp_path / "no-git.csv", {})
    assert sink.wait_closed(3) and sink.error is None
    meta = json.loads(sink.metadata_path.read_text(encoding="utf8"))
    assert meta["git_commit"] is None and meta["git_error"]


def test_target_cache_bound_and_copy(tmp_path: Path) -> None:
    sink = ControlLogger(tmp_path / "targets.csv", {})
    target: dict[str, Any] = {
        "currents": [0.1] * 6,
        "F_target": [2, 3],
        "ref_target": [4, 5],
        "ref_velocity": [6, 7],
        "rec": {"requested_force_camera": [2e-6, 3e-6, 4e-6]},
    }
    for seq in range(150):
        sink.remember_target(1, seq, target)
    sink.remember_progress(1, 149, 12.5)
    sink.remember_progress(1, 0, 10)
    row = sink.target(1, 149)
    row["F_target_x"] = -100
    target["currents"][0] = -100
    assert not sink.target(1, 0)
    assert sink.target(1, 149)["F_target_x"] == 2
    assert sink.target(1, 149)["I_target_0"] == 0.1
    assert sink.target(1, 149)["F_target_z"] == 4
    assert sink.target(1, 149)["s_progress"] == 12.5
    assert sink.wait_closed(3)


def test_capture_and_serialization_faults_are_contained(tmp_path: Path) -> None:
    sink = ControlLogger(tmp_path / "bad-metadata.csv", {"not_json": object()})
    assert sink.wait_closed(3) and "TypeError" in (sink.error or "")
    sink = ControlLogger(tmp_path / "bad-target.csv", {})
    sink.remember_target(1, 1, {})
    assert sink.wait_closed(3) and "KeyError" in (sink.error or "")
    sink = ControlLogger(tmp_path / "bad-row.csv", {})
    wait_ready(sink)

    class BadMapping(dict):
        def get(self, *args: object, **kwargs: object) -> None:
            raise RuntimeError("snapshot error")

    assert not sink.enqueue("control", BadMapping())
    assert sink.wait_closed(3) and "snapshot error" in (sink.error or "")

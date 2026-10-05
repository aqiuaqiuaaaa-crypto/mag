"""GUI consumes committed worker progress for finish, display and CSV."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from test_gui_control_log import rows, start_log
from test_gui_freshness import finish_log, start_tracking

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


def test_target_idx_cannot_finish_tracking(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    start_tracking(window, monkeypatch)
    expected = window.shared.get_progress().target_idx
    window.target_idx = len(window.full_path) + 100
    window.mpc_track_step(1 / 30)
    assert window.tracking and not window.stopping
    assert window.target_idx == expected


def test_worker_finished_is_gui_authority_and_logged(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "finished.csv")
    worker = window.worker
    assert worker is not None
    end = np.array(window.px_to_world_mm(window.full_path[-1]))
    for x in np.arange(0.5, end[0] + 0.1, 0.5):
        clock.now += 0.01
        window.shared.set_kalman([x, 0], [0, 0], clock.now)
        worker._step()
    progress = window.shared.get_progress()
    assert progress.finished and worker.finished
    assert worker.s_progress == progress.s_progress
    # The GUI target index cannot prevent a worker-declared finish, including
    # the loss-of-vision early-return route.
    window.target_idx = 0
    window.bead = None
    normal = Mock(wraps=window.normal_stop)
    emergency = Mock(wraps=window.emergency_stop)
    monkeypatch.setattr(window, "normal_stop", normal)
    monkeypatch.setattr(window, "emergency_stop", emergency)
    window.tick()
    normal.assert_called_once()
    emergency.assert_not_called()
    assert not window.tracking and window.worker is None
    finish_log(window)
    assert window.control_logger is not None
    control = rows(window.control_logger.path)[-1]
    worker_row = rows(window.control_logger.worker_path)[-1]
    for row in (control, worker_row):
        assert float(row["s_progress"]) == progress.s_progress
        assert float(row["s_total"]) == progress.s_total
        assert row["finished"] == "True" and row["path_deviation"] == "False"


@pytest.mark.parametrize("distance,lost", [(2.0, False), (5.0, True)])
def test_gui_fallback_or_normal_stop_for_path_deviation(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    distance: float,
    lost: bool,
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "deviation.csv")
    worker = window.worker
    assert worker is not None
    window.shared.set_reference([[0, 0], [10, 0]], 1, clock.now)
    window.shared.set_kalman([5, distance], [0, 0], clock.now)
    before_seq = window.shared.get_I_target()[1]
    worker._step()
    progress = window.shared.get_progress()
    assert progress.expanded_search and progress.path_deviation == lost
    if lost:
        assert (
            progress.s_progress == 0 and window.shared.get_I_target()[1] == before_seq
        )
    else:
        assert (
            progress.s_progress == 5
            and window.shared.get_I_target()[1] == before_seq + 1
        )
    window.state_pos_mm = np.array([5, distance])
    window.bead = (*window.world_mm_to_px(window.state_pos_mm), 200.0)
    normal = Mock(wraps=window.normal_stop)
    emergency = Mock(wraps=window.emergency_stop)
    monkeypatch.setattr(window, "normal_stop", normal)
    monkeypatch.setattr(window, "emergency_stop", emergency)
    window.mpc_track_step(1 / 30)
    assert normal.call_count == int(lost)
    emergency.assert_not_called()
    if lost:
        assert "PATH_DEVIATION" in window.lbl_dir.text()
        for _ in range(30):
            clock.now += 1 / 30
            window.tick()
        assert window.last_sent_cmd == [0] * 6 and not window.tracking
        assert window.shared.get_progress() == progress
    finish_log(window)
    assert window.control_logger is not None
    worker_row = rows(window.control_logger.worker_path)[-1]
    assert worker_row["path_deviation"] == str(lost)
    assert float(worker_row["s_progress"]) == progress.s_progress


def test_tracking_restart_resets_progress_and_keeps_lead(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    start_tracking(window, monkeypatch)
    old_shared = window.shared
    assert window.worker is not None
    window.shared.set_kalman([0.8, 0], [0, 0], 0)
    window.worker._step()
    assert window.shared.get_progress().s_progress == 0.8
    window.normal_stop()
    window.start_tracking()
    assert window.shared is not old_shared and old_shared.stopped()
    progress = window.shared.get_progress()
    assert progress.s_progress == 0 and not progress.finished
    assert not progress.path_deviation
    assert window.worker is not None and window.worker.s_progress == 0
    assert window.full_path[-len(window.path_px) :] == window.path_px
    assert window.full_path[0] == window.bead[:2]


def test_yellow_display_index_is_derived_from_worker(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    start_tracking(window, monkeypatch)
    assert window.worker is not None
    window.shared.set_kalman([0.8, 0], [0, 0], 0)
    window.worker._step()
    window.target_idx = 999
    window.render(None)
    assert window.target_idx == window.shared.get_progress().target_idx

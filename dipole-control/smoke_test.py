"""无头冒烟测试：不依赖相机/串口，验证 GUI 类与控制链路（约 30Hz 控制帧）"""

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"
import csv

# 关键：把设置文件隔离到临时路径，避免读入/覆盖用户真实保存的 GUI 参数
import tempfile
import time
from unittest.mock import patch

import cv2
import magnetic_dipole_pid as m
import numpy as np
from numpy.typing import NDArray
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

settings_dir = tempfile.TemporaryDirectory(prefix="mpc_smoke_")
m.SETTINGS_FILE = os.path.join(settings_dir.name, "gui_settings.json")

app = QApplication([])
w = m.MagneticDipoleControl()
w.timer.stop()  # 控制帧由测试显式推进，避免事件循环重入。
assert w.ser is None and w.cap is None
assert "MPC" in w.windowTitle()


class MemorySerial:
    """只记录帧，不打开或访问任何 COM 口。"""

    def __init__(self) -> None:
        self.frames: list[bytes] = []
        self.closed = False

    def write(self, data: bytes) -> int:
        assert len(data) == m.cfg.SERIAL_FRAME_BYTES
        self.frames.append(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


serial_sink = MemorySerial()
w.ser = serial_sink
w.show()
w.resize(1000, 650)
app.processEvents()
assert w.video.width() < m.cfg.DISP_W, "视频区应随小窗口收缩"
assert abs(w.video.width() / w.video.height() - 16.0 / 9.0) < 0.05
central_widget = w.centralWidget()
assert central_widget is not None
assert (
    w.control_scroll.geometry().right() <= central_widget.width()
), "控制面板不得被挤到窗口右侧之外"

# 模型严格校验通过，自动控制可用
assert w.model_ok, f"模型应可用: {w.model_err}"
assert w.solver.seg_pos.shape == (6, 30, 3)
assert w.spin_cmd_max.value() == m.cfg.DEFAULT_CMD_LIMIT
w.last_sent_cmd = [99, -99, 80, -80, 0, 0]
w.send_commands(w.last_sent_cmd)
assert max(abs(c) for c in w.last_sent_cmd) <= w.spin_cmd_max.value()
w.emergency_stop()

# 方向标定：已知模型→相机为 +90°，拟合应恢复该正交变换。
R90 = np.array([[0.0, -1.0], [1.0, 0.0]])
w.coil_test_records = []
for fxy in ([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]):
    f = np.array([fxy[0], fxy[1], 0.0])
    v = R90 @ f[:2]
    w.coil_test_records.append({"F_model": f, "velocity": v})
assert w._fit_force_frame_calibration()
assert np.allclose(w.force_model_to_camera, R90, atol=1e-8)
w.reset_force_frame_calibration()

# GUI 对齐场方向/模长可调，方向输入自动归一化并进入整体驱动伪逆。
w.spin_bdir_x.setValue(1.0)
w.spin_bdir_y.setValue(1.0)
w.spin_bdir_z.setValue(0.0)
w.spin_bmag.setValue(12.0)
rec_bf = w._solve_motion_target(np.zeros(3), np.array([10e-6, 0.0, 0.0]))
assert np.allclose(
    rec_bf["requested_B_direction"], np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
)
assert rec_bf["requested_B_magnitude_mT"] == 12.0
w.spin_bdir_x.setValue(m.cfg.CONTROL_FIELD_DIRECTION[0])
w.spin_bdir_y.setValue(m.cfg.CONTROL_FIELD_DIRECTION[1])
w.spin_bdir_z.setValue(m.cfg.CONTROL_FIELD_DIRECTION[2])
w.spin_bmag.setValue(m.cfg.CONTROL_FIELD_TARGET_MT)

# 规则路径
w.make_circle(400)
assert 100 <= len(w.path_px) <= 400, f"圆重采样点数 {len(w.path_px)}"
w.make_rect()
assert len(w.path_px) > 100
w.make_triangle(600)
assert len(w.path_px) > 100

# 模拟磁珠从中心偏左 2mm 开始追踪圆路径
w.frame_size = (1920, 1080)
w.bead = (760, 540, 300)
w.make_circle(400)

# 回归：手动"实时发送"开启时点开始追踪，必须自动退出手动模式，
# 否则 tick 的 elif 分派会让追踪分支失效（目标点冻结在起点、电流不动）
w.chk_cur_live.setChecked(True)
w.estimate_state(0.033, True)
w.start_tracking()
assert w.worker is not None and w.worker.is_alive()
assert not w.chk_cur_live.isChecked(), "开始追踪应自动退出手动电流实时发送"
assert not w.chk_force_live.isChecked()
w.estimate_state(0.033, True)  # 初始化状态估计器到磁珠当前位置
print("full_path 点数:", len(w.full_path))

# 闭环模拟：磁珠以 F_des/阻力系数 稳态响应（无相机，手动喂观测给估计器）
drag = w.solver.drag_uN_per_mm_s(1000.0)
for step in range(60):
    if w.tracking and w.last_solver_rec:
        v = np.clip(w.last_solver_rec["achieved_force"][:2] * 1e6 / drag, -30, 30)
        new_mm = w.state_pos_mm + v * 0.033
        px = w.world_mm_to_px(new_mm)
        w.bead = (px[0], px[1], 300)
        w.estimate_state(0.033, True)  # 无相机：显式喂入观测
    time.sleep(0.034)  # 给真实 10Hz worker 留出运行时间。
    w.tick()  # 完整主循环（视觉占位/控制/渲染/状态栏）
print("模拟 60 帧完成, 追踪中:", w.tracking, " 目标索引:", w.target_idx)
assert w.exp_log_mpc and len(w.traj_px) == len(w.traj_mm) > 0
assert all(len(row) == 24 for row in w.exp_log_mpc)
assert np.allclose(
    w.last_solver_rec["requested_B_direction"], m.cfg.CONTROL_FIELD_DIRECTION
)
assert w.last_solver_rec["solver_mode"] == "field-force-moore-penrose"
assert w.last_solver_rec["requested_B_magnitude_mT"] == 10.0
w.emergency_stop()

# 手动磁力（约束逆解 + F_act 按发送电流重算）
w.bead = (960, 540, 300)
w.spin_fx.setValue(4.0)
w.apply_manual_force()
rec = w.last_solver_rec
F_ref = rec["actuation_matrix"][3:, :] @ rec["currents"]
assert np.linalg.norm(rec["achieved_force_linear"] - F_ref) < 1e-15
F_actual_ref = w.solver.force_at(w._bead_pos_m(), rec["currents"])
assert np.linalg.norm(rec["achieved_force_model"] - F_actual_ref) < 1e-15
assert np.any(rec["commands"] != 0), "4µN 指令不应全零"
print(f"手动磁力: cmd={rec['commands']} 误差 {rec['force_error_percent']:.2f}%")

# 回归：方向测试的 GUI 输入已经是 µN，不得重复乘 1e-6
w.spin_ftx.setValue(30.0)
w.spin_fty.setValue(-10.0)
w.spin_ftz.setValue(5.0)
w.direction_test_step()
assert np.allclose(w.last_solver_rec["requested_force"], [30e-6, -10e-6, 5e-6])
assert w.last_solver_rec["requested_B_magnitude_mT"] == 10.0

# 回归：摩擦标定斜坡必须走磁力解算器，不能把三维力误发成六路电流
w.calib_fz_levels = [10.0]
w.calib_level = 0
w.calib_state = "ramp"
w.calib_Fx = 0.0
w.calib_hits = 0
w.state_vel_mm = np.zeros(2)
w.calib_step(0.1)
assert np.allclose(w.last_solver_rec["requested_force"], [2e-6, 0.0, 10e-6])
assert w.last_solver_rec["requested_B_magnitude_mT"] == 10.0

# 斜率安全层：帧间变化 ≤ 9 指令
MD = m.cfg.MAX_DELTA_CMD
prev = list(w.last_sent_cmd)
w.send_commands([99] * 6)
assert max(abs(a - b) for a, b in zip(w.last_sent_cmd, prev)) <= MD
print("斜率层步进:", [a - b for a, b in zip(w.last_sent_cmd, prev)])

# 运动控制必须同时满足目标力和 GUI 对齐场矢量，因此自动切到六路自由模式
w.chk_cur_live.setChecked(False)
w.combo_constraint.setCurrentIndex(2)  # 三路模式
w.bead = (760, 540, 300)
w.make_circle(400)
w.start_tracking()
assert w.combo_constraint.currentIndex() == 0
assert not w.combo_constraint.isEnabled()
for step in range(20):
    if w.tracking and w.last_solver_rec:
        v = np.clip(w.last_solver_rec["achieved_force"][:2] * 1e6 / drag, -20, 20)
        px = w.world_mm_to_px(w.state_pos_mm + v * 0.033)
        w.bead = (px[0], px[1], 300)
        w.estimate_state(0.033, True)
    time.sleep(0.034)
    w.tick()
assert w.shared.get_I_target()[1] > 0
print("[B;F] MPC 伪逆联合控制 20 帧 OK 末帧指令:", w.last_sent_cmd)
w.normal_stop()

# normal_stop 与 emergency_stop
w.normal_stop()
assert w.stopping and w.shared.stopped() and w.worker is None
for _ in range(8):
    previous = list(w.last_sent_cmd)
    w.tick()
    assert max(abs(a - b) for a, b in zip(w.last_sent_cmd, previous)) <= MD
assert w.last_sent_cmd == [0] * 6 and not w.stopping
w.send_commands([30] * 6)
w.emergency_stop()
assert w.last_sent_cmd == [0] * 6 and not w.stopping
assert serial_sink.frames[-1] == m.build_command([0] * 6)[0].encode("ascii")
print("急停帧:", m.build_command([0] * 6)[0].strip())

# 回归：到达最后一个路径点后必须结束，不能在零长度切线处生成标量 v_des
w.bead = (960, 540, 300)
w.state_pos_mm = np.zeros(2)
w.path_px = [(960, 540)]
w.start_tracking()  # worker is the sole finish authority, including zero length
finish_deadline = time.monotonic() + 1.0
while not w.shared.get_progress().finished and time.monotonic() < finish_deadline:
    w.shared.set_kalman(w.state_pos_mm, w.state_vel_mm, time.time())
    time.sleep(0.01)
assert w.shared.get_progress().finished
w.control_step(1.0 / 30.0)
assert not w.tracking and w.stopping
w.emergency_stop()

# MPC GUI 集成：主循环应发布状态/参数，工作线程输出安培目标，30Hz 层执行命令
w.spin_mpc_horizon.setValue(4)
w.spin_mpc_w_pos.setValue(2.5)
w.spin_mpc_w_vel.setValue(1.5)
w.spin_mpc_w_u.setValue(0.004)
w.spin_mpc_w_du.setValue(0.008)
w.spin_mpc_fmax.setValue(25.0)
w.bead = (800, 540, 300)
w.state_pos_mm = np.array(w.px_to_world_mm(w.bead[:2]))
w.state_vel_mm = np.zeros(2)
w.make_rect()
w.start_tracking()
assert w.worker is not None and w.worker.is_alive()
first_shared = w.shared
for _ in range(12):
    time.sleep(0.04)
    w.mpc_track_step(1.0 / 30.0)
target_A, seq, _, _ = w.shared.get_I_target()
assert seq > 0 and np.max(np.abs(target_A)) <= m.cfg.MAX_CURRENT_A + 1e-9
assert w.exp_log_mpc and all(len(row) == 24 for row in w.exp_log_mpc)
assert np.allclose(
    w.shared._I_target["rec"]["requested_B_direction"], m.cfg.CONTROL_FIELD_DIRECTION
)
assert w.shared._I_target["rec"]["solver_mode"] == "field-force-moore-penrose"
assert w.worker.mpc_x.N == 4 and w.worker.mpc_x.wp == 2.5
assert w.worker.mpc_x.fmax == 25.0
assert np.all(np.isfinite(w.shared._I_target["ref_target"]))
# 运行中修改参数，无需重启即可在下一 10Hz 周期生效。
w.spin_mpc_w_pos.setValue(3.5)
for _ in range(4):
    time.sleep(0.04)
    w.mpc_track_step(1.0 / 30.0)
assert w.worker.mpc_x.wp == 3.5
# 正式实验导出仅包含原有 24 列 MPC 数据，保留字段顺序和数据值。
csv_path = os.path.join(settings_dir.name, "experiment_mpc10hz.csv")
with patch.object(
    m.QFileDialog, "getSaveFileName", return_value=(csv_path, "CSV (*.csv)")
), patch.object(m.QMessageBox, "information"):
    w.save_traj()
with open(csv_path, encoding="utf-8", newline="") as f:
    rows = list(csv.reader(f))
assert len(rows[0]) == 24 and rows[0][:4] == [
    "timestamp",
    "mpc_ms",
    "solver_ms",
    "mpc_cost",
]
assert len(rows) == len(w.exp_log_mpc) + 1
assert rows[1] == [str(value) for value in w.exp_log_mpc[0]]
w.emergency_stop()
assert first_shared.stopped()
# stop Event 不可复用：第二次 MPC 启动必须创建新的 SharedState 和工作线程。
w.bead = (800, 540, 300)
w.state_pos_mm = np.array(w.px_to_world_mm(w.bead[:2]))
w.start_tracking()
assert w.shared is not first_shared and not w.shared.stopped()
assert w.worker is not None and w.worker.is_alive()
w.emergency_stop()


# 合成相机验证视觉主循环，不访问真实摄像头。
class SyntheticCamera:
    def __init__(self, frame: NDArray[np.uint8]) -> None:
        self.frame: NDArray[np.uint8] | None = frame

    def read(self) -> tuple[bool, NDArray[np.uint8] | None]:
        frame, self.frame = self.frame, None
        return frame is not None, frame

    def release(self) -> None:
        pass


image = np.full((1080, 1920, 3), 255, dtype=np.uint8)
cv2.circle(image, (960, 540), 15, (0, 0, 0), -1)
w.cap = SyntheticCamera(image)
w.tick()
assert w.bead is not None and np.allclose(w.bead[:2], [960, 540], atol=1)
assert w.kf.initialized and np.all(np.isfinite(w.state_pos_mm))
w.tick()
assert w.frame is None and w.bead is None
w.close_camera()

# 蓝色磁场箭头与红色受力箭头均能绘制
canvas = np.zeros((160, 240, 3), np.uint8)
w._draw_vector_arrow(canvas, (120, 80), [0, 0, -10e-3], 10e-3, (255, 0, 0), "B")
w._draw_vector_arrow(canvas, (120, 80), [30e-6, 0, 0], 100e-6, (0, 0, 255), "F")
assert np.any(canvas[:, :, 0] > 0) and np.any(canvas[:, :, 2] > 0)

# 路径截图和录像保存的都是最终标注画面，而不是无标注的相机原始帧。
w.frame_size = (640, 360)
w.video.frame_size = w.frame_size
w.frame = np.full((360, 640, 3), 35, np.uint8)
w.path_px = [(80, 180), (320, 80), (560, 180)]
w.bead = (320, 180, 500)
w.current_B_T = np.array([5e-3, 0.0, 5e-3])
w.current_F_N = np.array([10e-6, -5e-6, 0.0])
w.render(None)
snapshot_path = os.path.join(tempfile.gettempdir(), "磁控路径截图_smoke.png")
saved_snapshot = w._save_path_snapshot_to(snapshot_path)
snapshot = cv2.imdecode(np.fromfile(saved_snapshot, dtype=np.uint8), cv2.IMREAD_COLOR)
assert snapshot is not None and snapshot.shape[:2] == w.last_render_bgr.shape[:2]
assert np.any(snapshot[:, :, 1] > snapshot[:, :, 0] + 80), "截图应包含绿色路径"

video_path = os.path.join(tempfile.gettempdir(), "magnetic_record_smoke.avi")
if os.path.exists(video_path):
    os.remove(video_path)
w._start_video_recording_to(video_path)
for _ in range(4):
    w.render(None)
assert w.record_frame_count == 4 and not w.btn_record_start.isEnabled()
saved_video = w._finish_video_recording(show_message=False)
assert saved_video == video_path and os.path.getsize(video_path) > 0
recording = cv2.VideoCapture(video_path)
ok, recorded_frame = recording.read()
recording.release()
assert ok and recorded_frame is not None, "保存的视频应可被 OpenCV 重新读取"
assert w.btn_record_start.isEnabled() and not w.btn_record_stop.isEnabled()
os.remove(saved_snapshot)
os.remove(video_path)

# 鼠标绘制路径（模拟右键拖动）
w.toggle_drawing(True)
RIGHT = int(Qt.MouseButton.RightButton.value)
w.on_video_mouse(500, 500, RIGHT)
for p in [(510, 505), (520, 512), (530, 520), (540, 530), (550, 540), (560, 550)]:
    w.on_video_mouse(p[0], p[1], 0)
w.on_video_mouse(570, 560, RIGHT)
print("鼠标绘制路径点数:", len(w.path_px))
assert len(w.path_px) > 0

# 手动实时模式和诊断切换仍正确停止 MPC worker。
w.frame = None
w.frame_size = (1920, 1080)
w.bead = (800, 540, 300)
w.state_pos_mm = np.array(w.px_to_world_mm(w.bead[:2]))
w.make_rect()
w.start_tracking()
manual_shared = w.shared
w.chk_force_live.setChecked(True)
assert not w.tracking and w.worker is None and manual_shared.stopped()
w.chk_force_live.setChecked(False)
w.start_tracking()
w.toggle_dir_test(True)
assert not w.tracking and w.worker is None and w.dir_test_on
w.toggle_dir_test(False)
w.start_tracking()
w.toggle_calib(True)
assert not w.tracking and w.worker is None and w.calib_active
w.toggle_calib(False)
w.start_coil_scan()
assert w.coil_test_idx == 0 and not w.tracking and w.worker is None
w.normal_stop()
assert w.coil_test_idx is None
w.close()
assert serial_sink.closed
settings_dir.cleanup()
print("SMOKE OK")

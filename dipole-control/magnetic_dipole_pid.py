"""
基于 180 磁偶极子标定模型的视觉闭环磁控程序
=============================================================================
自动轨迹控制链（唯一控制器 MPC）：
    相机 → 磁珠检测 → Kalman/EMA/RAW 状态估计与 ESO 扰动估计
    → 路径参考 → SharedState → ControlWorker / ForceMPC（10Hz）
    → 180 偶极子 [B;F]=A·I 伪逆（目标场、坐标标定、Fz 减摩）
    → CurrentExecutor（30Hz，插值/斜率/整数化/估计电流）
    → send_commands() → STM32；MPC CSV 与 GUI 显示

力学与摩擦模型见 friction_model.py，状态估计见 estimators.py，
电流逆解见 dipole_solver.py（本次未改动其核心算法）。
"""

import json
import logging
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np

_serial: ModuleType | None
list_ports: ModuleType | None

try:
    import serial as _serial
    from serial.tools import list_ports
except ImportError:
    _serial = None
    list_ports = None

import config as cfg
import friction_model as fm
from adc_csv import ADCLogger
from curt_telemetry import ADCSnapshot, ADCStreamDecoder
from dipole_solver import DipoleSolver
from estimators import ESO1D, KalmanFilter2D
from multirate import ControlWorker, CurrentExecutor, SharedState
from PySide6.QtCore import QSize, Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

SETTINGS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "gui_settings.json"
)
logger = logging.getLogger(__name__)

ADC_RX_INTERVAL_MS = 15
ADC_RX_MAX_BYTES = 1024
ADC_STALE_S = 0.5
ADC_POLES = (1, 3, 5, 4, 6, 2)

# 历史公共 CSV 表头保留作为旧数据格式说明；主 GUI 仅导出 MPC 日志。
CSV_HEADER = [
    "timestamp",
    "x",
    "y",
    "vx",
    "vy",
    "xd",
    "yd",
    "vxd",
    "vyd",
    "ex",
    "ey",
    "Fx_pid",
    "Fy_pid",
    "Fx_ff",
    "Fy_ff",
    "Fx_drag",
    "Fy_drag",
    "Fx_fric",
    "Fy_fric",
    "Fz_lift",
    "ESO_x",
    "ESO_y",
    "Fx_cmd",
    "Fy_cmd",
    "Fz_cmd",
    "Fx_act",
    "Fy_act",
    "Fz_act",
    "a0_cmd",
    "a1_cmd",
    "a2_cmd",
    "a3_cmd",
    "a4_cmd",
    "a5_cmd",
    "a0_A",
    "a1_A",
    "a2_A",
    "a3_A",
    "a4_A",
    "a5_A",
    "I0_est",
    "I1_est",
    "I2_est",
    "I3_est",
    "I4_est",
    "I5_est",
    "Bx_mT",
    "By_mT",
    "Bz_mT",
    "B_magnitude_mT",
    "grad_absB_x",
    "grad_absB_y",
    "grad_absB_z",
    "magnetic_alignment_tau_ms",
    "magnetic_alignment_ratio",
    "sum_abs_current",
    "force_error_percent",
    "force_angle_error_deg",
    "normal_force_est",
    "friction_est",
    "solver_objective",
    "solver_elapsed_ms",
    "controller_mode",
    "vision_mode",
    "estimator_mode",
    "vision_detected",
    "vision_time_ms",
    "estimator_time_ms",
    "controller_time_ms",
    "serial_time_ms",
    "total_cycle_time_ms",
    "loop_jitter_ms",
]


def build_command(cmd_list, max_cmd=cfg.CMD_MAX):
    """构造 STM32 指令帧（固定 43 字节）:
    a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\\r\\n
    线圈顺序固定 a0~a5 = +X,+Y,+Z,−X,−Y,−Z。"""
    if len(cmd_list) != cfg.N_COILS:
        raise ValueError(
            f"电流指令必须恰好包含 {cfg.N_COILS} 路，实际 {len(cmd_list)} 路"
        )
    max_cmd = int(np.clip(max_cmd, 1, cfg.CMD_MAX))
    cur = [max(-max_cmd, min(max_cmd, round(x))) for x in cmd_list]
    frame = (
        f"a0:{cur[0]:+03d},a1:{cur[1]:+03d},a2:{cur[2]:+03d},"
        f"a3:{cur[3]:+03d},a4:{cur[4]:+03d},a5:{cur[5]:+03d}\r\n"
    )
    assert len(frame) == cfg.SERIAL_FRAME_BYTES
    return frame, cur


def apply_slew(target, last, max_delta=cfg.MAX_DELTA_CMD, max_cmd=cfg.CMD_MAX):
    """电流安全层：运行时幅值限幅 + 斜率限幅。

    最后再次做幅值钳位，确保用户在运行中降低上限时，下一帧立即满足新上限；
    这种安全约束优先于斜率约束。
    """
    if len(target) != cfg.N_COILS or len(last) != cfg.N_COILS:
        raise ValueError(f"目标和上一帧指令都必须为 {cfg.N_COILS} 路")
    max_cmd = int(np.clip(max_cmd, 1, cfg.CMD_MAX))
    out = []
    for t, l in zip(target, last):
        v = max(-max_cmd, min(max_cmd, round(t)))
        v = max(l - max_delta, min(l + max_delta, v))
        v = max(-max_cmd, min(max_cmd, v))
        out.append(int(v))
    return out


def resample_path(points_px, ds_px):
    """路径按弧长等距重采样（供目标速度切向计算与均匀到达判定）"""
    pts = np.asarray(points_px, dtype=float)
    if len(pts) < 2:
        return [tuple(p) for p in pts]
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total < 1e-9:
        return [tuple(pts[0])]
    n = max(2, round(total / max(ds_px, 1e-3)))
    s_new = np.linspace(0.0, total, n)
    x = np.interp(s_new, s, pts[:, 0])
    y = np.interp(s_new, s, pts[:, 1])
    return list(zip(x, y))


def detect_bead(frame, vp):
    """磁珠检测（GRAY/HSV + 形态学 + 面积/圆度/半径过滤）"""
    if vp["mode"] == "HSV":
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (int(vp["h_lo"]), int(vp["s_lo"]), int(vp["v_lo"]))
        upper = (int(vp["h_hi"]), 255, 255)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.bitwise_not(mask) if vp["invert"] else mask
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flags = cv2.THRESH_BINARY_INV if vp["invert"] else cv2.THRESH_BINARY
        _, mask = cv2.threshold(gray, int(vp["thresh"]), 255, flags)
    k = int(vp["morph_k"])
    if k >= 3 and vp["morph_open"]:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        )
    if k >= 3 and vp["morph_close"]:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        )
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_a = None, 0.0
    for c in cnts:
        a = cv2.contourArea(c)
        if not (vp["a_min"] <= a <= vp["a_max"]):
            continue
        r = math.sqrt(a / math.pi)
        if not (vp["r_min"] <= r <= vp["r_max"]):
            continue
        per = cv2.arcLength(c, True)
        if per > 0 and 4 * math.pi * a / (per * per) < vp["circ_min"]:
            continue
        if a > best_a:
            best, best_a = c, a
    if best is None:
        return None, None, None, mask
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None, None, None, mask
    return M["m10"] / M["m00"], M["m01"] / M["m00"], best_a, mask


class VideoLabel(QLabel):
    """显示视频并转发鼠标事件（坐标已换算到相机帧坐标系）"""

    mouseEvent = Signal(int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.frame_size = (cfg.FRAME_W, cfg.FRAME_H)
        self.setMinimumSize(480, 270)
        policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        policy.setHeightForWidth(True)
        self.setSizePolicy(policy)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #101010;")

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        fw, fh = self.frame_size
        return max(1, round(width * fh / max(fw, 1)))

    def sizeHint(self):
        return QSize(cfg.DISP_W, cfg.DISP_H)

    def _emit(self, event):
        pos = event.position()
        sx = self.frame_size[0] / self.width()
        sy = self.frame_size[1] / self.height()
        self.mouseEvent.emit(int(pos.x() * sx), int(pos.y() * sy), event.button().value)

    def mousePressEvent(self, e):
        self._emit(e)

    def mouseMoveEvent(self, e):
        self._emit(e)

    def mouseReleaseEvent(self, e):
        self._emit(e)


class MagneticDipoleControl(QMainWindow):
    adc_snapshot: ADCSnapshot | None
    adc_received_at: float | None
    adc_logger: ADCLogger | None
    spin_mpc_w_pos: QDoubleSpinBox
    spin_mpc_w_vel: QDoubleSpinBox
    spin_mpc_w_u: QDoubleSpinBox
    spin_mpc_w_du: QDoubleSpinBox

    def __init__(self):
        super().__init__()
        self.setWindowTitle("180 磁偶极子磁力解算 MPC 磁控系统")
        available = QApplication.primaryScreen().availableGeometry()
        self.resize(
            min(1520, int(available.width() * 0.96)),
            min(820, int(available.height() * 0.92)),
        )
        self.setMinimumSize(900, 600)
        self.n_coils_total = cfg.N_COILS

        # ---------- 相机 / 视觉 ----------
        self.cap = None
        self.frame = None
        self.frame_size = (cfg.FRAME_W, cfg.FRAME_H)
        self.bead = None
        self.traj_px, self.traj_mm = [], []
        # 截图/录像均保存 render() 生成的最终标注画面，因此会包含路径、
        # 目标点、磁珠轮廓以及 B/F 箭头。录像尺寸在开始时固定，窗口缩放后
        # 的画面会缩放到该尺寸，避免 VideoWriter 因帧尺寸变化而损坏文件。
        self.last_render_bgr = None
        self.video_writer = None
        self.record_path = None
        self.record_size = None
        self.record_frame_count = 0
        self.record_started_at = None

        # ---------- 状态估计 ----------
        self.kf = KalmanFilter2D(cfg.KALMAN_Q_POS, cfg.KALMAN_Q_VEL, cfg.KALMAN_R)
        self.ema_vel = np.zeros(2)
        self.state_pos_mm = np.zeros(2)  # 估计器输出（供 MPC/ESO）
        self.state_vel_mm = np.zeros(2)
        self.last_pos_mm = None

        # ---------- ESO ----------
        self.eso_x = None
        self.eso_y = None
        self.last_F_actual = np.zeros(3)  # 上一帧实际磁力（ESO 输入 u）
        self.current_B_T = np.zeros(3)  # 当前位置、最终指令电流对应的模型磁场
        self.current_F_N = np.zeros(3)  # 当前位置、最终指令电流对应的模型磁力
        self.z3 = np.zeros(2)

        # ---------- 路径与追踪 ----------
        self.path_px = []
        self.full_path = []
        self.drawing_mode = False
        self.is_drawing = False
        self.draw_pts = []
        self.tracking = False
        self.target_idx = 0
        self.lost_since = None

        # ---------- 控制状态 ----------
        self.last_time = time.time()
        self.last_sent_cmd = [0] * 6
        self.man_currents = [0] * 6
        self.stopping = False
        self.last_solver_rec = None
        self.cycle_over_cnt = 0
        # 多速率架构：10Hz MPC/MDM 工作线程 + 30Hz 电流执行层
        self.shared = SharedState()
        self.executor = CurrentExecutor()
        self.worker = None
        self.last_tick_time = None
        self.loop_jitter_ms = 0.0
        self.exp_log_mpc = []  # 10Hz MPC/MDM 日志（工作线程写入）
        self.last_diag = None
        self.mpc_meas_hz = 0.0
        self.vision_ms = 0.0
        self.estimator_ms = 0.0
        self.controller_ms = 0.0
        self.serial_ms = 0.0
        self.total_ms = 0.0

        # 模型 XY 坐标到相机/路径世界坐标的正交映射。它只修正安装旋转、
        # 相机镜像和通道命名差异，不替代 180 偶极子正向/逆向模型。
        self.force_model_to_camera = np.eye(2)
        self.force_frame_calibrated = False
        self.force_frame_rms_deg = float("nan")

        # ---------- 诊断模式状态 ----------
        self.mode = "IDLE"
        self.dir_test_on = False
        self.calib_active = False
        self.calib_level = 0
        self.calib_Fx = 0.0
        self.calib_state = "settle"
        self.calib_timer = 0.0
        self.calib_hits = 0
        self.calib_records = []
        self.coil_test_idx = None
        self.coil_test_frames = 0
        self.coil_test_vsum = np.zeros(2)
        self.coil_test_samples = 0
        self.coil_test_phase = "zero"
        self.coil_test_records = []
        self.coil_test_start = None

        # ---------- 串口 / 模型 ----------
        self.ser = None
        self.adc_decoder = ADCStreamDecoder()
        self.adc_snapshot = None
        self.adc_received_at = None
        self.adc_receive_time = ""
        self.adc_rx_bytes = 0
        self.adc_rx_frames = 0
        self.adc_rx_errors = 0
        self.adc_parser_errors = 0
        self.adc_monitor_error = ""
        self.adc_logger = None
        self.adc_rx_timer = QTimer(self)
        self.adc_rx_timer.setInterval(ADC_RX_INTERVAL_MS)
        self.adc_rx_timer.timeout.connect(self.poll_serial_rx)
        self.adc_status_timer = QTimer(self)
        self.adc_status_timer.setInterval(100)
        self.adc_status_timer.timeout.connect(self._refresh_adc_display)
        self.solver = None
        self.model_ok = False
        self.model_err = ""
        self._phys_params = {
            "MODEL": cfg.MODEL_PATH,
            "GAIN": cfg.CMD_TO_A,
            "D": cfg.BEAD_DIAMETER_MM,
            "BR": cfg.BEAD_BR_T,
        }
        self.viscosity_mPas = cfg.VISCOSITY_MPA_S

        self._build_ui()
        self.load_settings()
        # 设置文件会先更新控件；解算器必须使用控件中的持久化物理参数，不能继续
        # 使用构造函数里的默认快照。
        self._sync_physics_from_ui()
        self._reload_solver()

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(cfg.TICK_MS)
        self.adc_status_timer.start()

    # ================= 解算器加载（严格校验，无静默降级） =================
    def _reload_solver(self):
        p = self._phys_params
        try:
            self.solver = DipoleSolver.from_json(
                p["MODEL"],
                bead_diameter_mm=p["D"],
                bead_Br=p["BR"],
                current_gain=p["GAIN"],
            )
            self.solver.validate_model()
            self.model_ok = True
            self.model_err = ""
        except (OSError, ValueError, TypeError) as e:
            self.solver = None
            self.model_ok = False
            self.model_err = f"模型加载失败: {e}"
        except Exception as e:
            logger.exception("模型加载发生程序错误，禁用模型控制")
            self.solver = None
            self.model_ok = False
            self.model_err = f"模型加载失败: {e}"
            self._update_model_ui()
            raise
        self._update_model_ui()
        if hasattr(self, "spin_cmd_max"):
            self._on_current_limit_changed(self.spin_cmd_max.value())
        if hasattr(self, "spin_omega0"):
            self._rebuild_eso()

    def _update_model_ui(self):
        if self.model_ok:
            info = self.solver.model_info
            self.lbl_model.setText(
                f"✓ {info.get('model_name', '-')} | "
                f"{', '.join(info.get('coil_order', []))} | "
                f"{self.solver.seg_pos.shape[0] * self.solver.seg_pos.shape[1]} 偶极子"
            )
            self.lbl_model.setStyleSheet("color: #2e7d32; font-family: Consolas;")
        else:
            self.lbl_model.setText(
                f"✗ {self.model_err}\n（自动控制已禁用，仅手动电流可用）"
            )
            self.lbl_model.setStyleSheet("color: #c62828; font-family: Consolas;")
        self.btn_track.setEnabled(self.model_ok)
        self.btn_force_once.setEnabled(self.model_ok)
        self.chk_force_live.setEnabled(self.model_ok)
        self.btn_dir_test.setEnabled(self.model_ok)
        self.btn_calib.setEnabled(self.model_ok)
        self.btn_coilscan.setEnabled(self.model_ok)
        self._fill_coil_table_model()

    def _drag_c_uN(self):
        """斯托克斯阻力系数，µN/(mm/s)"""
        if getattr(self, "solver", None):
            return self.solver.drag_uN_per_mm_s(self.viscosity_mPas)
        return (
            6.0
            * math.pi
            * (self.viscosity_mPas * 1e-3)
            * (cfg.BEAD_DIAMETER_MM * 0.5e-3)
            * 1e3
        )

    # ================= UI =================
    def _build_ui(self):
        left = QVBoxLayout()
        self.video = VideoLabel()
        self.video.mouseEvent.connect(self.on_video_mouse)
        left.addWidget(self.video)
        self.status_lbl = QLabel("模式: 空闲")
        self.status_lbl.setStyleSheet("font-family: Consolas; font-size: 13px;")
        self.status_lbl.setWordWrap(True)
        left.addWidget(self.status_lbl)
        self.field_lbl = QLabel("当前位置场/力: -   蓝色箭头=B，红色箭头=F")
        self.field_lbl.setStyleSheet("font-family: Consolas; font-size: 12px;")
        self.field_lbl.setWordWrap(True)
        left.addWidget(self.field_lbl)
        left.addStretch(1)

        left_widget = QWidget()
        left_widget.setLayout(left)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._tab_manual_current(), "手动电流")
        self.tabs.addTab(self._tab_manual_force(), "手动磁力")
        self.tabs.addTab(self._tab_vision(), "识别参数")
        self.tabs.addTab(self._tab_path(), "路径与追踪")
        self.tabs.addTab(self._tab_advanced(), "高级控制")
        self.tabs.addTab(self._tab_diag(), "诊断")
        self.tabs.addTab(self._tab_physics(), "物理参数")
        self.tabs.addTab(self._tab_adc(), "CURT / ADC")
        self.tabs.setMinimumWidth(400)
        self.control_scroll = QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.control_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.control_scroll.setWidget(self.tabs)
        self.control_scroll.setMinimumWidth(410)

        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.addWidget(left_widget)
        self.main_splitter.addWidget(self.control_scroll)
        self.main_splitter.setStretchFactor(0, 3)
        self.main_splitter.setStretchFactor(1, 0)
        self.main_splitter.setSizes([960, 500])

        outer = QWidget()
        v = QVBoxLayout(outer)
        v.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._serial_bar())
        v.addWidget(self.main_splitter, stretch=1)
        self.setCentralWidget(outer)

    def _serial_bar(self):
        bar = QHBoxLayout()
        bar.addWidget(QLabel("串口:"))
        self.combo_port = QComboBox()
        self.combo_port.setFixedWidth(110)
        bar.addWidget(self.combo_port)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(btn_refresh)
        self.btn_serial = QPushButton("连接")
        self.btn_serial.clicked.connect(self.toggle_serial)
        bar.addWidget(self.btn_serial)
        self.lbl_serial = QLabel("未连接")
        bar.addWidget(self.lbl_serial)
        bar.addWidget(QLabel("电流上限:"))
        self.spin_cmd_max = QSpinBox()
        self.spin_cmd_max.setRange(1, cfg.CMD_MAX)
        self.spin_cmd_max.setValue(cfg.DEFAULT_CMD_LIMIT)
        self.spin_cmd_max.setSuffix(" /99")
        self.spin_cmd_max.setToolTip(
            "六路统一绝对上限。例：50 表示任一路串口指令均限制在 -50~+50。"
        )
        self.spin_cmd_max.valueChanged.connect(self._on_current_limit_changed)
        bar.addWidget(self.spin_cmd_max)
        self.lbl_cmd_limit = QLabel("")
        bar.addWidget(self.lbl_cmd_limit)
        btn_stop = QPushButton("停止")
        btn_stop.clicked.connect(self.normal_stop)
        bar.addWidget(btn_stop)
        btn_estop = QPushButton("急停")
        btn_estop.setStyleSheet("background-color: #b71c1c; color: white;")
        btn_estop.clicked.connect(self.emergency_stop)
        bar.addWidget(btn_estop)
        bar.addStretch(1)
        self.refresh_ports()
        self._on_current_limit_changed(self.spin_cmd_max.value())
        return bar

    def _current_cmd_limit(self):
        if hasattr(self, "spin_cmd_max"):
            return int(self.spin_cmd_max.value())
        return int(cfg.DEFAULT_CMD_LIMIT)

    def _on_current_limit_changed(self, value):
        """同步手动控件，并保证正在输出的六路指令立刻落入新上限。"""
        limit = int(np.clip(value, 1, cfg.CMD_MAX))
        gain = (
            self.solver.current_gain
            if getattr(self, "solver", None)
            else self._phys_params.get("GAIN", cfg.CMD_TO_A)
        )
        if hasattr(self, "lbl_cmd_limit"):
            self.lbl_cmd_limit.setText(f"(±{limit * gain:.3f} A)")
        for s in getattr(self, "cur_sliders", []):
            s.setRange(-limit, limit)
        for sp in getattr(self, "cur_spins", []):
            sp.setRange(-limit, limit)
        if hasattr(self, "man_currents"):
            self.man_currents = [
                int(np.clip(c, -limit, limit)) for c in self.man_currents
            ]
        if hasattr(self, "last_sent_cmd") and any(
            abs(c) > limit for c in self.last_sent_cmd
        ):
            self.send_commands(self.last_sent_cmd)
        if hasattr(self, "tbl_coil") and getattr(self, "model_ok", False):
            self._fill_coil_table_model()

    def _dspin(self, lo, hi, val, step, dec=2):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(dec)
        sp.setSingleStep(step)
        sp.setValue(val)
        return sp

    # ---------- 手动电流 ----------
    def _tab_manual_current(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("各路电流指令 a0~a5（名称来自模型；物理方向以扫描标定为准）")
        grid = QGridLayout(box)
        self.cur_sliders, self.cur_spins = [], []
        for j in range(6):
            grid.addWidget(QLabel(f"a{j} ({cfg.COIL_ORDER[j]})"), j, 0)
            s = QSlider(Qt.Horizontal)
            s.setRange(-cfg.CMD_MAX, cfg.CMD_MAX)
            sp = QSpinBox()
            sp.setRange(-cfg.CMD_MAX, cfg.CMD_MAX)
            s.valueChanged.connect(sp.setValue)
            sp.valueChanged.connect(s.setValue)
            sp.valueChanged.connect(lambda val, idx=j: self.on_manual_current(idx, val))
            grid.addWidget(s, j, 1)
            grid.addWidget(sp, j, 2)
            self.cur_sliders.append(s)
            self.cur_spins.append(sp)
        v.addWidget(box)
        note = QLabel(
            "实时发送、力解算、MPC 均经过顶部“电流上限”和统一安全层；\n"
            "斜率限制为 9 指令/帧（降低上限时安全限幅优先）。"
        )
        note.setWordWrap(True)
        v.addWidget(note)
        h = QHBoxLayout()
        self.chk_cur_live = QCheckBox("实时发送")
        self.chk_cur_live.toggled.connect(self._on_manual_current_live)
        btn_send_cur = QPushButton("发送")
        btn_zero_cur = QPushButton("全部清零(斜率限制)")
        btn_send_cur.clicked.connect(self.send_manual_currents)
        btn_zero_cur.clicked.connect(self.zero_manual_currents)
        h.addWidget(self.chk_cur_live)
        h.addWidget(btn_send_cur)
        h.addWidget(btn_zero_cur)
        v.addLayout(h)
        v.addStretch(1)
        return w

    def on_manual_current(self, idx, val):
        self.man_currents[idx] = val

    def _on_manual_current_live(self, on):
        """手动实时电流具有最高优先级；开启时明确退出其他控制模式。"""
        if not on:
            return
        self.tracking = False
        self.dir_test_on = False
        self.calib_active = False
        self.coil_test_idx = None
        self.stopping = False
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        self.chk_force_live.setChecked(False)
        self.combo_constraint.setEnabled(True)
        self._stop_worker()

    def _on_manual_force_live(self, on):
        if not on:
            return
        self.chk_cur_live.setChecked(False)
        self.tracking = False
        self.dir_test_on = False
        self.calib_active = False
        self.coil_test_idx = None
        self.stopping = False
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        self.combo_constraint.setEnabled(True)
        self._stop_worker()

    def send_manual_currents(self):
        self.send_commands(self.man_currents)

    def zero_manual_currents(self):
        for sp in self.cur_spins:
            sp.setValue(0)
        self.man_currents = [0] * 6

    # ---------- 手动磁力 ----------
    def _tab_manual_force(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("期望磁力 (µN，世界系: x右 / y上 / z向上为正)")
        grid = QGridLayout(box)
        grid.addWidget(QLabel("Fx"), 0, 0)
        self.spin_fx = self._dspin(-300, 300, 0.0, 5.0)
        grid.addWidget(self.spin_fx, 0, 1)
        grid.addWidget(QLabel("Fy"), 1, 0)
        self.spin_fy = self._dspin(-300, 300, 0.0, 5.0)
        grid.addWidget(self.spin_fy, 1, 1)
        grid.addWidget(QLabel("Fz"), 2, 0)
        self.spin_fz = self._dspin(-50, 50, 0.0, 1.0)
        grid.addWidget(self.spin_fz, 2, 1)
        v.addWidget(box)
        h = QHBoxLayout()
        self.chk_force_live = QCheckBox("实时解算发送")
        self.chk_force_live.toggled.connect(self._on_manual_force_live)
        self.btn_force_once = QPushButton("单次解算发送")
        btn_fz = QPushButton("清零")
        self.btn_force_once.clicked.connect(self.apply_manual_force)
        btn_fz.clicked.connect(self.zero_manual_force)
        h.addWidget(self.chk_force_live)
        h.addWidget(self.btn_force_once)
        h.addWidget(btn_fz)
        v.addLayout(h)
        self.lbl_force_out = QLabel("-")
        self.lbl_force_out.setStyleSheet("font-family: Consolas;")
        v.addWidget(self.lbl_force_out)
        v.addStretch(1)
        return w

    def apply_manual_force(self):
        if not self.model_ok:
            return
        self._solve_and_send(
            np.array([self.spin_fx.value(), self.spin_fy.value(), self.spin_fz.value()])
        )

    def zero_manual_force(self):
        self.spin_fx.setValue(0)
        self.spin_fy.setValue(0)
        self.spin_fz.setValue(0)

    def _current_max_active(self):
        """当前约束模式允许的最大同时工作电磁铁数"""
        return (self.n_coils_total, 1, 3)[self.combo_constraint.currentIndex()]

    def _sparse_transition_needed(self, max_active):
        """稀疏模式过渡判定：斜率箱内无法归零（|上一帧指令| > Δ）的通道数
        超过名额时，本帧无严格可行解，需先按斜率归零过渡。"""
        return (
            sum(1 for c in self.last_sent_cmd if abs(c) > cfg.MAX_DELTA_CMD)
            > max_active
        )

    def _solve_and_send(self, f_uN):
        """期望力(µN) → 力/10mT 场幅值联合逆解 → 最终整数电流。"""
        pos_m = self._bead_pos_m()
        force_N = np.asarray(f_uN, float) * 1e-6
        try:
            # 即使 F_des=0，也仍按 [B;F]=A·I 维持 GUI 指定的对齐场；停止/急停
            # 由 normal_stop/emergency_stop 独立发送零电流，不混淆“零力”和“断场”。
            if self.combo_constraint.currentIndex() != 0:
                self.combo_constraint.setCurrentIndex(0)
            rec = self._solve_motion_target(pos_m, force_N)
        except Exception as e:
            # 安全边界：任何求解故障都必须退出实时发送，并留下完整 traceback。
            logger.exception("手动磁力解算失败，进入安全归零")
            self.lbl_force_out.setText(
                f"解算失败，正在安全归零：{type(e).__name__}: {e}"
            )
            self.chk_force_live.setChecked(False)
            self.normal_stop()
            return
        self.last_solver_rec = rec
        t0 = time.perf_counter()
        self.send_commands([int(c) for c in rec["commands"]])
        rec["serial_ms"] = (time.perf_counter() - t0) * 1e3
        self._show_force_result(rec)

    def _solve_motion_target(self, pos_m, force_N):
        """按文献构造 [B;F]=A·I，并用 Moore–Penrose 伪逆求电流。

        路径与手动磁力使用相机世界坐标；逆解前通过单线圈视觉扫描得到的正交映射
        转回模型坐标。这样保留多偶极子物理模型，同时修正安装和相机坐标差异。
        """
        requested_camera = np.asarray(force_N, float).copy()
        force_model = requested_camera.copy()
        force_model[:2] = self.force_model_to_camera.T @ requested_camera[:2]
        field_camera, field_magnitude = self._field_target_camera()
        field_model = field_camera.copy()
        field_model[:2] = self.force_model_to_camera.T @ field_camera[:2]
        rec = self.solver.solve_field_force_pseudoinverse(
            pos_m,
            force_model,
            B_direction=field_model,
            B_magnitude_mT=field_magnitude,
            cmd_prev=self.last_sent_cmd,
            max_cmd=self._current_cmd_limit(),
        )
        return self._solution_to_camera(rec, requested_camera)

    def _solution_to_camera(self, rec, requested_force_camera=None):
        """把求解器的模型坐标诊断量转换到相机世界坐标（原位修改记录）。"""
        R = self.force_model_to_camera
        if "achieved_force" in rec:
            f_model = np.asarray(rec["achieved_force"], float).copy()
            rec["achieved_force_model"] = f_model.copy()
            f_camera = f_model.copy()
            f_camera[:2] = R @ f_model[:2]
            rec["achieved_force"] = f_camera
        if "achieved_force_nonlinear" in rec:
            f_nl_model = np.asarray(rec["achieved_force_nonlinear"], float).copy()
            rec["achieved_force_nonlinear_model"] = f_nl_model.copy()
            f_nl_camera = f_nl_model.copy()
            f_nl_camera[:2] = R @ f_nl_model[:2]
            rec["achieved_force_nonlinear"] = f_nl_camera
        if "achieved_force_linear" in rec:
            f_lin_model = np.asarray(rec["achieved_force_linear"], float).copy()
            rec["achieved_force_linear_model"] = f_lin_model.copy()
            f_lin_camera = f_lin_model.copy()
            f_lin_camera[:2] = R @ f_lin_model[:2]
            rec["achieved_force_linear"] = f_lin_camera
        if "B" in rec:
            b_model = np.asarray(rec["B"], float).copy()
            rec["B_model"] = b_model.copy()
            b_camera = b_model.copy()
            b_camera[:2] = R @ b_model[:2]
            rec["B"] = b_camera
        if rec.get("requested_B_direction") is not None:
            requested_b_model = np.asarray(rec["requested_B_direction"], float).copy()
            rec["requested_B_direction_model"] = requested_b_model.copy()
            requested_b_camera = requested_b_model.copy()
            requested_b_camera[:2] = R @ requested_b_model[:2]
            rec["requested_B_direction"] = requested_b_camera
        if requested_force_camera is not None:
            req = np.asarray(requested_force_camera, float).copy()
            rec["requested_force_camera"] = req
            if "achieved_force" in rec:
                denom = max(float(np.linalg.norm(req)), 1e-12)
                err = float(np.linalg.norm(rec["achieved_force"] - req))
                rec["force_error"] = err
                rec["force_error_percent"] = 100.0 * err / denom
        rec["force_frame_calibrated"] = bool(self.force_frame_calibrated)
        return rec

    def _diag_to_camera(self, diag):
        """将执行器正向模型诊断复制并转换为相机世界坐标。"""
        out = dict(diag)
        for key in ("B", "F_est", "grad_absB"):
            if key in out:
                vec = np.asarray(out[key], float).copy()
                if vec.size >= 2:
                    vec[:2] = self.force_model_to_camera @ vec[:2]
                out[key] = vec
        return out

    def _show_force_result(self, rec):
        if rec.get("sparse_infeasible"):
            constraint_txt = "约束不可行(衰减中)"
        elif rec["current_constraint_active"]:
            constraint_txt = "磁力不可达/受电流约束"
        else:
            constraint_txt = "✓" if rec["converged"] else "未收敛"
        active = [cfg.COIL_ORDER[i] for i in rec.get("active_coils", [])]
        active_txt = f" | 工作: {', '.join(active)}" if active else ""
        field_txt = ""
        if "B_magnitude_mT" in rec:
            field_txt = (
                f"\nB_act: [{rec['B'][0]*1e3:+.2f}, {rec['B'][1]*1e3:+.2f}, "
                f"{rec['B'][2]*1e3:+.2f}] mT | |B|={rec['B_magnitude_mT']:.2f} mT"
                f" | 矢量误差={rec.get('field_vector_error_percent', 0.0):.1f}%"
                f" | 方向误差={rec.get('B_direction_error_deg', 0.0):.1f}°"
            )
        self.lbl_force_out.setText(
            f"指令: [{', '.join(str(int(c)) for c in rec['commands'])}] "
            f"(实际最大 {np.max(np.abs(rec['currents'])):.3f} A){active_txt}\n"
            f"F_act(按发送电流): [{rec['achieved_force'][0] * 1e6:+.2f}, "
            f"{rec['achieved_force'][1] * 1e6:+.2f}, "
            f"{rec['achieved_force'][2] * 1e6:+.2f}] µN"
            f"{field_txt}\n"
            f"力误差 {rec['force_error_percent']:.2f}% | {constraint_txt} | "
            f"rank={rec.get('actuation_rank', '-')} "
            f"cond={rec.get('actuation_condition', float('nan')):.2e} | "
            f"{rec['elapsed_ms']:.1f} ms"
            + (" ⚠ 超过33ms" if rec["elapsed_ms"] > cfg.SOLVER_WARN_MS else "")
        )

    # ---------- 识别参数 ----------
    def _tab_vision(self):
        w = QWidget()
        v = QVBoxLayout(w)
        cam_box = QGroupBox("相机")
        ch = QHBoxLayout(cam_box)
        ch.addWidget(QLabel("索引"))
        self.spin_cam = QSpinBox()
        self.spin_cam.setRange(0, 8)
        self.spin_cam.setValue(cfg.CAM_INDEX)
        ch.addWidget(self.spin_cam)
        btn_cam = QPushButton("打开")
        btn_cam.clicked.connect(self.open_camera)
        ch.addWidget(btn_cam)
        btn_camoff = QPushButton("关闭")
        btn_camoff.clicked.connect(self.close_camera)
        ch.addWidget(btn_camoff)
        v.addWidget(cam_box)

        media_box = QGroupBox("截图与录像（包含路径和画面标注）")
        mg = QGridLayout(media_box)
        self.btn_path_snapshot = QPushButton("保存路径截图")
        self.btn_path_snapshot.clicked.connect(self.save_path_snapshot)
        mg.addWidget(self.btn_path_snapshot, 0, 0, 1, 2)
        self.btn_record_start = QPushButton("开始录制")
        self.btn_record_start.clicked.connect(self.start_video_recording)
        mg.addWidget(self.btn_record_start, 1, 0)
        self.btn_record_stop = QPushButton("结束录制并保存")
        self.btn_record_stop.clicked.connect(self.stop_video_recording)
        self.btn_record_stop.setEnabled(False)
        mg.addWidget(self.btn_record_stop, 1, 1)
        self.lbl_recording = QLabel("未录制")
        self.lbl_recording.setStyleSheet("color: #9e9e9e;")
        self.lbl_recording.setWordWrap(True)
        mg.addWidget(self.lbl_recording, 2, 0, 1, 2)
        v.addWidget(media_box)

        vis_box = QGroupBox("识别参数")
        vg = QGridLayout(vis_box)
        vg.addWidget(QLabel("识别模式"), 0, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["GRAY", "HSV"])
        self.combo_mode.setCurrentIndex(0)
        vg.addWidget(self.combo_mode, 0, 1)
        vg.addWidget(QLabel("灰度阈值"), 1, 0)
        self.sld_thresh = QSlider(Qt.Horizontal)
        self.sld_thresh.setRange(0, 255)
        self.sld_thresh.setValue(30)
        vg.addWidget(self.sld_thresh, 1, 1)
        self.chk_invert = QCheckBox("反相(暗磁珠)")
        self.chk_invert.setChecked(True)
        vg.addWidget(self.chk_invert, 1, 2)
        vg.addWidget(QLabel("HSV H lo/hi"), 2, 0)
        self.spin_hlo = QSpinBox()
        self.spin_hlo.setRange(0, 179)
        self.spin_hlo.setValue(0)
        self.spin_hhi = QSpinBox()
        self.spin_hhi.setRange(0, 179)
        self.spin_hhi.setValue(179)
        h2 = QHBoxLayout()
        h2.addWidget(self.spin_hlo)
        h2.addWidget(self.spin_hhi)
        vg.addLayout(h2, 2, 1)
        vg.addWidget(QLabel("S lo / V lo"), 3, 0)
        self.spin_slo = QSpinBox()
        self.spin_slo.setRange(0, 255)
        self.spin_slo.setValue(60)
        self.spin_vlo = QSpinBox()
        self.spin_vlo.setRange(0, 255)
        self.spin_vlo.setValue(40)
        h3 = QHBoxLayout()
        h3.addWidget(self.spin_slo)
        h3.addWidget(self.spin_vlo)
        vg.addLayout(h3, 3, 1)
        vg.addWidget(QLabel("形态学核(0=关)"), 4, 0)
        self.spin_morph_k = QSpinBox()
        self.spin_morph_k.setRange(0, 21)
        self.spin_morph_k.setValue(3)
        vg.addWidget(self.spin_morph_k, 4, 1)
        self.chk_morph_open = QCheckBox("开运算")
        self.chk_morph_open.setChecked(True)
        vg.addWidget(self.chk_morph_open, 4, 2)
        self.chk_morph_close = QCheckBox("闭运算")
        vg.addWidget(self.chk_morph_close, 5, 2)
        vg.addWidget(QLabel("面积(px²) min/max"), 6, 0)
        self.spin_amin = QSpinBox()
        self.spin_amin.setRange(1, 100000)
        self.spin_amin.setValue(50)
        self.spin_amax = QSpinBox()
        self.spin_amax.setRange(10, 2000000)
        self.spin_amax.setValue(50000)
        h4 = QHBoxLayout()
        h4.addWidget(self.spin_amin)
        h4.addWidget(self.spin_amax)
        vg.addLayout(h4, 6, 1)
        vg.addWidget(QLabel("最小圆度"), 7, 0)
        self.spin_circ = self._dspin(0.0, 1.0, 0.0, 0.05)
        vg.addWidget(self.spin_circ, 7, 1)
        vg.addWidget(QLabel("半径(px) min/max"), 8, 0)
        self.spin_rmin = QSpinBox()
        self.spin_rmin.setRange(1, 500)
        self.spin_rmin.setValue(2)
        self.spin_rmax = QSpinBox()
        self.spin_rmax.setRange(1, 1000)
        self.spin_rmax.setValue(200)
        h5 = QHBoxLayout()
        h5.addWidget(self.spin_rmin)
        h5.addWidget(self.spin_rmax)
        vg.addLayout(h5, 8, 1)
        vg.addWidget(QLabel("画面宽度(mm)"), 9, 0)
        self.spin_vieww = self._dspin(1.0, 200.0, cfg.VIEW_WIDTH_MM, 0.5)
        vg.addWidget(self.spin_vieww, 9, 1)
        self.chk_show_binary = QCheckBox("显示掩膜")
        vg.addWidget(self.chk_show_binary, 10, 0, 1, 2)
        v.addWidget(vis_box)
        v.addStretch(1)
        return w

    # ---------- 路径与追踪 ----------
    def _tab_path(self):
        w = QWidget()
        v = QVBoxLayout(w)

        shape_box = QGroupBox("规则形状路径（画面中心，单位像素）")
        sg = QGridLayout(shape_box)
        sg.addWidget(QLabel("圆半径"), 0, 0)
        self.spin_circ_r = QSpinBox()
        self.spin_circ_r.setRange(20, 2000)
        self.spin_circ_r.setValue(400)
        sg.addWidget(self.spin_circ_r, 0, 1)
        btn_circ = QPushButton("画圆")
        btn_circ.clicked.connect(lambda: self.make_circle(self.spin_circ_r.value()))
        sg.addWidget(btn_circ, 0, 2)
        sg.addWidget(QLabel("矩形宽/高"), 1, 0)
        self.spin_rw = QSpinBox()
        self.spin_rw.setRange(20, 3000)
        self.spin_rw.setValue(700)
        self.spin_rh = QSpinBox()
        self.spin_rh.setRange(20, 3000)
        self.spin_rh.setValue(450)
        h1 = QHBoxLayout()
        h1.addWidget(self.spin_rw)
        h1.addWidget(self.spin_rh)
        sg.addLayout(h1, 1, 1)
        btn_rect = QPushButton("画矩形")
        btn_rect.clicked.connect(lambda: self.make_rect())
        sg.addWidget(btn_rect, 1, 2)
        sg.addWidget(QLabel("三角形边长"), 2, 0)
        self.spin_tri = QSpinBox()
        self.spin_tri.setRange(20, 3000)
        self.spin_tri.setValue(600)
        sg.addWidget(self.spin_tri, 2, 1)
        btn_tri = QPushButton("画三角形")
        btn_tri.clicked.connect(lambda: self.make_triangle(self.spin_tri.value()))
        sg.addWidget(btn_tri, 2, 2)
        v.addWidget(shape_box)

        draw_box = QGroupBox("鼠标绘制")
        dh = QHBoxLayout(draw_box)
        self.btn_draw = QPushButton("开始绘制(右键拖动)")
        self.btn_draw.setCheckable(True)
        self.btn_draw.toggled.connect(self.toggle_drawing)
        dh.addWidget(self.btn_draw)
        btn_clr = QPushButton("清空路径")
        btn_clr.clicked.connect(self.clear_path)
        dh.addWidget(btn_clr)
        v.addWidget(draw_box)

        ff_box = QGroupBox("路径跟踪")
        fg = QGridLayout(ff_box)
        fg.addWidget(QLabel("路径速度(mm/s)"), 0, 0)
        self.spin_path_speed = self._dspin(0.05, 20.0, cfg.PATH_SPEED_MM_S, 0.1)
        fg.addWidget(self.spin_path_speed, 0, 1)
        fg.addWidget(QLabel("重采样间距(mm)"), 1, 0)
        self.spin_path_ds = self._dspin(0.05, 1.0, cfg.PATH_DS_MM, 0.05, 2)
        fg.addWidget(self.spin_path_ds, 1, 1)
        v.addWidget(ff_box)

        bf_box = QGroupBox("文献法 [B;F]=A·I 驱动目标（Moore–Penrose 伪逆）")
        bg = QGridLayout(bf_box)
        bg.addWidget(QLabel("对齐场方向 X/Y/Z"), 0, 0)
        self.spin_bdir_x = self._dspin(
            -1.0, 1.0, cfg.CONTROL_FIELD_DIRECTION[0], 0.1, 3
        )
        self.spin_bdir_y = self._dspin(
            -1.0, 1.0, cfg.CONTROL_FIELD_DIRECTION[1], 0.1, 3
        )
        self.spin_bdir_z = self._dspin(
            -1.0, 1.0, cfg.CONTROL_FIELD_DIRECTION[2], 0.1, 3
        )
        bd_h = QHBoxLayout()
        for sp in (self.spin_bdir_x, self.spin_bdir_y, self.spin_bdir_z):
            sp.setFixedWidth(75)
            sp.valueChanged.connect(self._update_field_target_label)
            bd_h.addWidget(sp)
        bg.addLayout(bd_h, 0, 1)
        bg.addWidget(QLabel("对齐场模长 (mT)"), 1, 0)
        self.spin_bmag = self._dspin(0.1, 30.0, cfg.CONTROL_FIELD_TARGET_MT, 0.5, 2)
        self.spin_bmag.valueChanged.connect(self._update_field_target_label)
        bg.addWidget(self.spin_bmag, 1, 1)
        self.lbl_field_target = QLabel("")
        self.lbl_field_target.setWordWrap(True)
        bg.addWidget(self.lbl_field_target, 2, 0, 1, 2)
        v.addWidget(bf_box)
        self._update_field_target_label()

        mpc_box = QGroupBox("MPC 参数（运行中修改，下一次 10 Hz 周期生效）")
        mg = QGridLayout(mpc_box)
        mg.addWidget(QLabel("预测步数 N"), 0, 0)
        self.spin_mpc_horizon = QSpinBox()
        # 当前活动集 QP 枚举 3^N 个状态，限制到 6 可保证 10Hz 实时性。
        self.spin_mpc_horizon.setRange(1, 6)
        self.spin_mpc_horizon.setValue(cfg.MPC_HORIZON)
        self.spin_mpc_horizon.setToolTip("预测时间=N/10秒；最大6步以保证实时求解")
        mg.addWidget(self.spin_mpc_horizon, 0, 1)
        mg.addWidget(QLabel("水平力上限(µN)"), 0, 2)
        self.spin_mpc_fmax = self._dspin(1.0, 500.0, cfg.MPC_FMAX_UN, 5.0, 1)
        mg.addWidget(self.spin_mpc_fmax, 0, 3)
        mpc_specs = [
            ("位置 Wpos", "spin_mpc_w_pos", cfg.MPC_W_POS, 0.1),
            ("速度 Wvel", "spin_mpc_w_vel", cfg.MPC_W_VEL, 0.1),
            ("用力 Wu", "spin_mpc_w_u", cfg.MPC_W_U, 0.001),
            ("变化 Wdu", "spin_mpc_w_du", cfg.MPC_W_DU, 0.001),
        ]
        for idx, (label, name, value, step) in enumerate(mpc_specs):
            row, col = 1 + idx // 2, (idx % 2) * 2
            mg.addWidget(QLabel(label), row, col)
            spin = self._dspin(0.0, 100.0, value, step, 6)
            spin.setToolTip("实时发布到 MPC 工作线程，无需停止追踪")
            setattr(self, name, spin)
            mg.addWidget(spin, row, col + 1)
        self.lbl_mpc_hint = QLabel(
            "调节规律：Wpos↑更积极；Wu↑力更小；Wdu↑更平滑但响应更慢"
        )
        self.lbl_mpc_hint.setWordWrap(True)
        mg.addWidget(self.lbl_mpc_hint, 3, 0, 1, 4)
        v.addWidget(mpc_box)

        h = QHBoxLayout()
        self.btn_track = QPushButton("开始追踪")
        self.btn_track.setStyleSheet("background-color: #2e7d32; color: white;")
        self.btn_track.clicked.connect(self.start_tracking)
        btn_csv = QPushButton("保存实验CSV")
        btn_csv.clicked.connect(self.save_traj)
        h.addWidget(self.btn_track)
        h.addWidget(btn_csv)
        v.addLayout(h)

        pg2 = QGridLayout()
        pg2.addWidget(QLabel("约束模式"), 0, 0)
        self.combo_constraint = QComboBox()
        self.combo_constraint.addItems(
            ["关闭（6 路自由）", "单极模式（最多 1 路）", "三路模式（最多 3 路）"]
        )
        self.combo_constraint.setCurrentIndex(0)
        pg2.addWidget(self.combo_constraint, 0, 1)
        pg2.addWidget(QLabel("控制器"), 1, 0)
        pg2.addWidget(QLabel("MPC (10 Hz 多速率)"), 1, 1)
        v.addLayout(pg2)
        v.addStretch(1)
        return w

    def _field_target_camera(self):
        """返回 GUI 世界坐标中的单位场方向和目标模长；零向量明确拒绝。"""
        direction = np.array(
            [
                self.spin_bdir_x.value(),
                self.spin_bdir_y.value(),
                self.spin_bdir_z.value(),
            ],
            dtype=float,
        )
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            raise ValueError("对齐场方向 X/Y/Z 不能同时为 0")
        return direction / norm, float(self.spin_bmag.value())

    def _update_field_target_label(self, *_):
        if not hasattr(self, "lbl_field_target"):
            return
        try:
            direction, magnitude = self._field_target_camera()
            self.lbl_field_target.setText(
                f"实际目标 B={np.round(direction * magnitude, 3).tolist()} mT；"
                "方向输入会自动归一化"
            )
            self.lbl_field_target.setStyleSheet("color: #2e7d32;")
        except ValueError as exc:
            self.lbl_field_target.setText(f"参数错误：{exc}")
            self.lbl_field_target.setStyleSheet("color: #c62828;")

    # ---------- 高级控制 ----------
    def _tab_advanced(self):
        w = QWidget()
        v = QVBoxLayout(w)

        fz_box = QGroupBox("Fz 减摩 / 底面摩擦")
        fg = QGridLayout(fz_box)
        self.chk_lift = QCheckBox("启用 Fz 减摩（适度减小法向压力，不做完全悬浮）")
        self.chk_lift.setChecked(True)
        fg.addWidget(self.chk_lift, 0, 0, 1, 2)
        fg.addWidget(QLabel("目标法向力比例"), 1, 0)
        self.spin_normal_ratio = self._dspin(0.0, 1.0, cfg.NORMAL_RATIO_DEFAULT, 0.05)
        fg.addWidget(self.spin_normal_ratio, 1, 1)
        fg.addWidget(QLabel("最大减摩力 Fz_max (µN)"), 2, 0)
        self.spin_fz_max = self._dspin(0.0, 100.0, cfg.FZ_MAX_UN, 5.0, 1)
        fg.addWidget(self.spin_fz_max, 2, 1)
        fg.addWidget(QLabel("最小安全法向力 N_min (µN)"), 3, 0)
        self.spin_n_min = self._dspin(0.0, 50.0, cfg.N_MIN_UN, 1.0, 1)
        fg.addWidget(self.spin_n_min, 3, 1)
        v.addWidget(fz_box)

        eso_box = QGroupBox("ESO 扰动观测器（动力学：c·v = F + d，b0 = 1/c）")
        eg = QGridLayout(eso_box)
        self.chk_eso = QCheckBox("启用 ESO 补偿 (F_cmd −= z3)")
        self.chk_eso.setChecked(True)
        eg.addWidget(self.chk_eso, 0, 0, 1, 2)
        eg.addWidget(QLabel("带宽 ω0 (rad/s)"), 1, 0)
        self.spin_omega0 = self._dspin(0.5, 20.0, cfg.ESO_OMEGA0, 0.5, 1)
        self.spin_omega0.setToolTip("30Hz 下建议 ≤6；过大离散不稳定")
        eg.addWidget(self.spin_omega0, 1, 1)
        eg.addWidget(QLabel("fal δ (mm)"), 2, 0)
        self.spin_fal_delta = self._dspin(0.001, 1.0, cfg.ESO_FAL_DELTA, 0.01, 3)
        eg.addWidget(self.spin_fal_delta, 2, 1)
        eg.addWidget(QLabel("扰动限幅 (µN)"), 3, 0)
        self.spin_dist_limit = self._dspin(1.0, 500.0, cfg.ESO_DIST_LIMIT_UN, 10.0, 1)
        eg.addWidget(self.spin_dist_limit, 3, 1)
        v.addWidget(eso_box)

        kal_box = QGroupBox("状态估计（视觉）")
        kg = QGridLayout(kal_box)
        kg.addWidget(QLabel("估计器"), 0, 0)
        self.combo_estimator = QComboBox()
        self.combo_estimator.addItems(["RAW", "EMA", "KALMAN"])
        self.combo_estimator.setCurrentIndex(2)
        kg.addWidget(self.combo_estimator, 0, 1)
        kg.addWidget(QLabel("Q 位置 (mm²)"), 1, 0)
        self.spin_q_pos = self._dspin(1e-6, 1.0, cfg.KALMAN_Q_POS, 1e-4, 6)
        kg.addWidget(self.spin_q_pos, 1, 1)
        kg.addWidget(QLabel("Q 速度 (mm²/s²)"), 2, 0)
        self.spin_q_vel = self._dspin(1e-5, 10.0, cfg.KALMAN_Q_VEL, 1e-3, 5)
        kg.addWidget(self.spin_q_vel, 2, 1)
        kg.addWidget(QLabel("R 观测 (mm²)"), 3, 0)
        self.spin_r_meas = self._dspin(1e-5, 1.0, cfg.KALMAN_R, 0.001, 5)
        kg.addWidget(self.spin_r_meas, 3, 1)
        v.addWidget(kal_box)

        self.lbl_adv = QLabel("")
        self.lbl_adv.setStyleSheet("font-family: Consolas;")
        v.addWidget(self.lbl_adv)
        v.addStretch(1)
        return w

    # ---------- 诊断 ----------
    def _tab_diag(self):
        w = QWidget()
        v = QVBoxLayout(w)

        dir_box = QGroupBox("方向测试（恒定测试力，检验逆解/摩擦/自动控制归属）")
        dg = QGridLayout(dir_box)
        dg.addWidget(QLabel("F_test_x (µN)"), 0, 0)
        self.spin_ftx = self._dspin(-300, 300, 30.0, 5.0)
        dg.addWidget(self.spin_ftx, 0, 1)
        dg.addWidget(QLabel("F_test_y (µN)"), 1, 0)
        self.spin_fty = self._dspin(-300, 300, 0.0, 5.0)
        dg.addWidget(self.spin_fty, 1, 1)
        dg.addWidget(QLabel("F_test_z (µN)"), 2, 0)
        self.spin_ftz = self._dspin(-50, 50, 0.0, 5.0)
        dg.addWidget(self.spin_ftz, 2, 1)
        self.btn_dir_test = QPushButton("开始方向测试")
        self.btn_dir_test.setCheckable(True)
        self.btn_dir_test.toggled.connect(self.toggle_dir_test)
        dg.addWidget(self.btn_dir_test, 3, 0, 1, 2)
        self.lbl_dir = QLabel("-")
        self.lbl_dir.setStyleSheet("font-family: Consolas;")
        self.lbl_dir.setWordWrap(True)
        dg.addWidget(self.lbl_dir, 4, 0, 1, 2)
        v.addWidget(dir_box)

        cal_box = QGroupBox("摩擦标定（+X 方向逐级 Fx 斜坡，记录启动力 F_start(Fz)）")
        cg = QGridLayout(cal_box)
        cg.addWidget(QLabel("Fz 档位 (µN, 逗号分隔)"), 0, 0)
        self.edit_calib_fz = QLineEdit("0,10,20,30,40")
        cg.addWidget(self.edit_calib_fz, 0, 1)
        cg.addWidget(QLabel("Fx 斜率 (µN/s)"), 1, 0)
        self.spin_calib_rate = self._dspin(1.0, 200.0, 20.0, 5.0, 1)
        cg.addWidget(self.spin_calib_rate, 1, 1)
        cg.addWidget(QLabel("启动判定速度 (mm/s)"), 2, 0)
        self.spin_calib_vth = self._dspin(0.05, 5.0, 0.3, 0.05)
        cg.addWidget(self.spin_calib_vth, 2, 1)
        self.btn_calib = QPushButton("开始摩擦标定")
        self.btn_calib.setCheckable(True)
        self.btn_calib.toggled.connect(self.toggle_calib)
        cg.addWidget(self.btn_calib, 3, 0)
        btn_calib_save = QPushButton("保存标定CSV")
        btn_calib_save.clicked.connect(self.save_calib)
        cg.addWidget(btn_calib_save, 3, 1)
        self.lbl_calib = QLabel("-")
        self.lbl_calib.setStyleSheet("font-family: Consolas;")
        self.lbl_calib.setWordWrap(True)
        cg.addWidget(self.lbl_calib, 4, 0, 1, 2)
        v.addWidget(cal_box)

        mr_box = QGroupBox("多速率控制状态")
        mv = QVBoxLayout(mr_box)
        self.lbl_multirate = QLabel("-")
        self.lbl_multirate.setStyleSheet("font-family: Consolas; font-size: 12px;")
        self.lbl_multirate.setWordWrap(True)
        mv.addWidget(self.lbl_multirate)
        v.addWidget(mr_box)

        coil_box = QGroupBox("线圈方向/坐标标定（单线圈扫描：模型力 vs 实验视觉速度）")
        cv = QVBoxLayout(coil_box)
        self.lbl_force_frame = QLabel("未标定：暂按模型 XY 与相机世界 XY 一致")
        self.lbl_force_frame.setWordWrap(True)
        cv.addWidget(self.lbl_force_frame)
        bh = QHBoxLayout()
        btn_scan = QPushButton("扫描六路并自动标定")
        btn_scan.clicked.connect(self.start_coil_scan)
        bh.addWidget(btn_scan)
        btn_reset_frame = QPushButton("重置坐标标定")
        btn_reset_frame.clicked.connect(self.reset_force_frame_calibration)
        bh.addWidget(btn_reset_frame)
        cv.addLayout(bh)
        self.btn_coilscan = btn_scan
        self.tbl_coil = QTableWidget(6, 7)
        self.tbl_coil.setHorizontalHeaderLabels(
            [
                "coil",
                "I(A)",
                "Fx_model(µN)",
                "Fy_model(µN)",
                "Fz_model(µN)",
                "vx_exp(mm/s)",
                "vy_exp(mm/s)",
            ]
        )
        self.tbl_coil.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_coil.setEditTriggers(QTableWidget.NoEditTriggers)
        cv.addWidget(self.tbl_coil)
        v.addWidget(coil_box)
        v.addStretch(1)
        return w

    def _fill_coil_table_model(self):
        """线圈诊断表：模型力列（原点处，受当前指令上限约束）。"""
        for j in range(6):
            if self.model_ok:
                test_A = min(1.0, self._current_cmd_limit() * self.solver.current_gain)
                I = np.zeros(6)
                I[j] = test_A
                F = self.solver.force_at(np.zeros(3), I) * 1e6
                vals = [
                    cfg.COIL_ORDER[j],
                    f"{test_A:.3f}",
                    f"{F[0]:+.1f}",
                    f"{F[1]:+.1f}",
                    f"{F[2]:+.1f}",
                    "-",
                    "-",
                ]
            else:
                vals = [cfg.COIL_ORDER[j], "-", "-", "-", "-", "-", "-"]
            for col, val in enumerate(vals):
                self.tbl_coil.setItem(j, col, QTableWidgetItem(str(val)))

    def _set_coil_table_exp(self, j, vx, vy):
        self.tbl_coil.setItem(j, 5, QTableWidgetItem(f"{vx:+.3f}"))
        self.tbl_coil.setItem(j, 6, QTableWidgetItem(f"{vy:+.3f}"))

    def _update_force_frame_label(self):
        if not hasattr(self, "lbl_force_frame"):
            return
        if not self.force_frame_calibrated:
            self.lbl_force_frame.setText("未标定：暂按模型 XY 与相机世界 XY 一致")
            self.lbl_force_frame.setStyleSheet("color: #b26a00;")
            return
        R = self.force_model_to_camera
        self.lbl_force_frame.setText(
            "已标定模型→相机坐标："
            f"[[{R[0,0]:+.3f},{R[0,1]:+.3f}],"
            f"[{R[1,0]:+.3f},{R[1,1]:+.3f}]]；"
            f"方向 RMS={self.force_frame_rms_deg:.1f}°"
        )
        self.lbl_force_frame.setStyleSheet("color: #2e7d32;")

    def reset_force_frame_calibration(self):
        self.force_model_to_camera = np.eye(2)
        self.force_frame_calibrated = False
        self.force_frame_rms_deg = float("nan")
        self._update_force_frame_label()

    def _fit_force_frame_calibration(self):
        """由单线圈模型力方向与视觉速度方向拟合正交映射。

        只使用方向，不把速度大小错误地当作力标定；允许 det(R)=-1，以覆盖
        相机镜像。至少需要三个有效且不共线的运动样本。
        """
        model_dirs, camera_dirs = [], []
        v_threshold = max(0.03, 0.2 * self.spin_calib_vth.value())
        for item in self.coil_test_records:
            f = np.asarray(item["F_model"], float)[:2]
            v = np.asarray(item["velocity"], float)[:2]
            if np.linalg.norm(f) < 1e-12 or np.linalg.norm(v) < v_threshold:
                continue
            model_dirs.append(f / np.linalg.norm(f))
            camera_dirs.append(v / np.linalg.norm(v))
        if len(model_dirs) < 3:
            self.reset_force_frame_calibration()
            self.lbl_force_frame.setText(
                f"标定失败：仅 {len(model_dirs)} 路产生可辨识运动；请检查电流上限、"
                "磁珠识别、摩擦和接线后重试"
            )
            self.lbl_force_frame.setStyleSheet("color: #c62828;")
            return False
        X = np.asarray(model_dirs).T
        Y = np.asarray(camera_dirs).T
        if np.linalg.matrix_rank(X) < 2:
            self.reset_force_frame_calibration()
            self.lbl_force_frame.setText(
                "标定失败：有效运动方向共线，无法确定二维坐标映射"
            )
            self.lbl_force_frame.setStyleSheet("color: #c62828;")
            return False
        U, _, Vt = np.linalg.svd(Y @ X.T)
        R = U @ Vt
        predicted = (R @ X).T
        observed = Y.T
        dots = np.clip(np.sum(predicted * observed, axis=1), -1.0, 1.0)
        errors = np.degrees(np.arccos(dots))
        rms_deg = float(np.sqrt(np.mean(errors**2)))
        if rms_deg > 30.0:
            self.reset_force_frame_calibration()
            self.lbl_force_frame.setText(
                f"标定失败：六路方向不能由同一坐标旋转/镜像解释（RMS={rms_deg:.1f}°）；"
                "请核对模型通道顺序与 STM32 a0~a5 接线"
            )
            self.lbl_force_frame.setStyleSheet("color: #c62828;")
            return False
        self.force_model_to_camera = R
        self.force_frame_calibrated = True
        self.force_frame_rms_deg = rms_deg
        self._update_force_frame_label()
        return True

    # ---------- 物理参数 ----------
    def _tab_physics(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("模型与磁参数（应用后重建解算器）")
        g = QGridLayout(box)
        mh = QHBoxLayout()
        self.edit_model = QLineEdit(cfg.MODEL_PATH)
        mh.addWidget(self.edit_model)
        btn_browse = QPushButton("浏览")
        btn_browse.clicked.connect(self.browse_model)
        mh.addWidget(btn_browse)
        g.addLayout(mh, 0, 0, 1, 2)

        def add(row, name, lo, hi, val, step=1.0, dec=1):
            g.addWidget(QLabel(name), row, 0)
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(dec)
            sp.setSingleStep(step)
            sp.setValue(val)
            g.addWidget(sp, row, 1)
            return sp

        self.spin_gain = add(
            1, "单位电流放大倍数 (指令→A, 99↔2A)", 0.0001, 100, cfg.CMD_TO_A, 0.001, 4
        )
        self.spin_beadD = add(2, "磁珠直径 (mm)", 0.1, 10, cfg.BEAD_DIAMETER_MM, 0.1)
        self.spin_br = add(3, "磁珠剩磁 Br (T)", 0.1, 2.0, cfg.BEAD_BR_T, 0.01, 2)
        self.spin_visc = add(
            4, "液体黏度 (mPa·s)", 0.1, 10000, cfg.VISCOSITY_MPA_S, 100.0
        )
        self.spin_rho_b = add(
            5, "磁珠密度 (kg/m³)", 1000, 20000, cfg.RHO_BEAD, 100.0, 0
        )
        self.spin_rho_f = add(6, "液体密度 (kg/m³)", 500, 5000, cfg.RHO_FLUID, 50.0, 0)
        v.addWidget(box)
        btn_apply = QPushButton("应用")
        btn_apply.clicked.connect(self.apply_physics)
        v.addWidget(btn_apply)
        self.lbl_model = QLabel()
        self.lbl_model.setStyleSheet("font-family: Consolas;")
        self.lbl_model.setWordWrap(True)
        v.addWidget(self.lbl_model)
        note = QLabel(
            "模型 JSON 严格校验：6 线圈 × 30 = 180 偶极子，\n"
            "coil_order = +X,+Y,+Z,−X,−Y,−Z。校验失败不回退错误模型。\n"
            "磁珠受力 F = m·∇|B|（解析磁场梯度张量）；\n"
            "有效重力 W_eff = (ρ珠−ρ液)·V·g 用于法向力/摩擦模型。"
        )
        note.setWordWrap(True)
        v.addWidget(note)
        v.addStretch(1)
        return w

    def browse_model(self):
        fn, _ = QFileDialog.getOpenFileName(
            self, "选择偶极子模型", "", "JSON (*.json);;All (*)"
        )
        if fn:
            self.edit_model.setText(fn)
            self.apply_physics()

    def apply_physics(self):
        was_active = (
            self.tracking
            or self.dir_test_on
            or self.calib_active
            or self.coil_test_idx is not None
        )
        if was_active:
            self.normal_stop()
        self._sync_physics_from_ui()
        self._reload_solver()
        self._rebuild_eso()

    def _sync_physics_from_ui(self):
        """把 GUI（含已载入的持久化设置）同步到运行时物理参数。"""
        self._phys_params = {
            "MODEL": self.edit_model.text().strip(),
            "GAIN": self.spin_gain.value(),
            "D": self.spin_beadD.value(),
            "BR": self.spin_br.value(),
        }
        self.viscosity_mPas = self.spin_visc.value()

    def _rebuild_eso(self):
        """参数变化后重建 ESO（b0=1/c 随黏度/珠径变化）"""
        c = self._drag_c_uN()
        self.eso_x = ESO1D(
            1.0 / c,
            self.spin_omega0.value(),
            self.spin_fal_delta.value(),
            self.spin_dist_limit.value(),
        )
        self.eso_y = ESO1D(
            1.0 / c,
            self.spin_omega0.value(),
            self.spin_fal_delta.value(),
            self.spin_dist_limit.value(),
        )
        self.kf = KalmanFilter2D(
            self.spin_q_pos.value(), self.spin_q_vel.value(), self.spin_r_meas.value()
        )

    # ================= 相机 =================
    def open_camera(self):
        self.close_camera()
        idx = self.spin_cam.value()
        cap = cv2.VideoCapture(idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.FRAME_H)
        ret, frame = cap.read()
        if not ret:
            cap.release()
            QMessageBox.warning(self, "相机", f"无法打开相机 {idx}")
            return
        self.cap = cap
        h, w = frame.shape[:2]
        self.frame_size = (w, h)
        self.video.frame_size = self.frame_size

    def close_camera(self):
        # 切换/关闭相机时也要正确写完视频尾部，否则容器可能无法播放。
        self._finish_video_recording(show_message=False)
        if self.cap:
            self.cap.release()
        self.cap = None
        self.frame = None
        self.bead = None
        self.last_render_bgr = None

    @staticmethod
    def _write_image_unicode(path, image):
        """用 imencode + tofile 支持 Windows 中文保存路径。"""
        ext = os.path.splitext(path)[1].lower()
        if ext not in (".png", ".jpg", ".jpeg", ".bmp"):
            path += ".png"
            ext = ".png"
        ok, encoded = cv2.imencode(ext, image)
        if not ok:
            raise OSError("OpenCV 图像编码失败")
        encoded.tofile(path)
        return path

    def _save_path_snapshot_to(self, path):
        """保存当前最终标注画面；独立 helper 便于无相机自动测试。"""
        if self.last_render_bgr is None:
            raise RuntimeError("当前还没有可保存的相机画面")
        return self._write_image_unicode(path, self.last_render_bgr)

    def save_path_snapshot(self):
        if self.frame is None or self.last_render_bgr is None:
            QMessageBox.information(self, "保存截图", "请先打开相机并等待画面显示。")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存带路径截图",
            f"path_snapshot_{stamp}.png",
            "PNG 图像 (*.png);;JPEG 图像 (*.jpg *.jpeg);;BMP 图像 (*.bmp)",
        )
        if not path:
            return
        try:
            saved = self._save_path_snapshot_to(path)
            self.lbl_recording.setText(f"截图已保存：{saved}")
            self.lbl_recording.setStyleSheet("color: #43a047;")
        except (OSError, cv2.error) as exc:
            QMessageBox.warning(self, "保存截图", f"截图保存失败：\n{exc}")

    def _start_video_recording_to(self, path):
        """创建录像文件并固定帧尺寸；返回实际保存路径。"""
        if self.video_writer is not None:
            return self.record_path
        if self.last_render_bgr is None:
            raise RuntimeError("当前还没有可录制的相机画面")

        _, ext = os.path.splitext(path)
        ext = ext.lower()
        if ext not in (".mp4", ".avi"):
            path += ".mp4"
            ext = ".mp4"
        h, w = self.last_render_bgr.shape[:2]
        # 常用 MP4/AVI 编码器要求宽高为偶数。
        w = max(2, w - w % 2)
        h = max(2, h - h % 2)
        codecs = ("mp4v", "avc1") if ext == ".mp4" else ("MJPG", "XVID")
        writer = None
        for codec in codecs:
            candidate = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*codec), float(cfg.VISION_HZ), (w, h)
            )
            if candidate.isOpened():
                writer = candidate
                break
            candidate.release()
        if writer is None:
            raise OSError("无法创建视频文件，请尝试选择 AVI 格式或检查保存路径")

        self.video_writer = writer
        self.record_path = path
        self.record_size = (w, h)
        self.record_frame_count = 0
        self.record_started_at = time.monotonic()
        self.btn_record_start.setEnabled(False)
        self.btn_record_stop.setEnabled(True)
        self.lbl_recording.setText(f"● 正在录制：{os.path.basename(path)}")
        self.lbl_recording.setStyleSheet("color: #e53935; font-weight: bold;")
        return path

    def start_video_recording(self):
        if self.video_writer is not None:
            return
        if self.frame is None or self.last_render_bgr is None:
            QMessageBox.information(self, "开始录制", "请先打开相机并等待画面显示。")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "选择录像保存位置",
            f"camera_record_{stamp}.mp4",
            "MP4 视频 (*.mp4);;AVI 视频 (*.avi)",
        )
        if not path:
            return
        try:
            self._start_video_recording_to(path)
        except (OSError, cv2.error) as exc:
            QMessageBox.warning(self, "开始录制", f"录像启动失败：\n{exc}")

    def _finish_video_recording(self, show_message=False):
        if self.video_writer is None:
            return None
        writer = self.video_writer
        path = self.record_path
        frames = self.record_frame_count
        elapsed = (
            time.monotonic() - self.record_started_at
            if self.record_started_at is not None
            else 0.0
        )
        self.video_writer = None
        self.record_path = None
        self.record_size = None
        self.record_started_at = None
        writer.release()
        if hasattr(self, "btn_record_start"):
            self.btn_record_start.setEnabled(True)
            self.btn_record_stop.setEnabled(False)
            self.lbl_recording.setText(
                f"录像已保存：{path}（{frames} 帧，{elapsed:.1f} 秒）"
            )
            self.lbl_recording.setStyleSheet("color: #43a047;")
        if show_message:
            QMessageBox.information(
                self,
                "录像已保存",
                f"已保存到：\n{path}\n\n共 {frames} 帧，时长 {elapsed:.1f} 秒",
            )
        return path

    def stop_video_recording(self):
        if self.video_writer is None:
            return
        self._finish_video_recording(show_message=True)

    def _record_rendered_frame(self, image):
        if self.video_writer is None:
            return
        frame = image
        if (frame.shape[1], frame.shape[0]) != self.record_size:
            frame = cv2.resize(frame, self.record_size, interpolation=cv2.INTER_AREA)
        self.video_writer.write(frame)
        self.record_frame_count += 1
        if self.record_frame_count % 10 == 0:
            elapsed = max(0.0, time.monotonic() - self.record_started_at)
            self.lbl_recording.setText(
                f"● 正在录制：{os.path.basename(self.record_path)} | "
                f"{elapsed:.1f} 秒 / {self.record_frame_count} 帧"
            )

    # ================= CURT 只读监视（不进入控制状态或反馈） =================
    def _tab_adc(self) -> QWidget:
        """Build raw ADC and health displays in unchanged logical channel order."""
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel(
            "CURT 仅用于观察和记录。ADC raw 尚未标定为 Ampere，"
            "不参与 MPC/PWM 反馈。\ncommand 是当前 GUI 指令；"
            "30 Hz 指令与约 10 Hz ADC 快照并不严格同步。"
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        self.lbl_adc_health = QLabel("DISCONNECTED")
        layout.addWidget(self.lbl_adc_health)
        self.tbl_adc = QTableWidget(6, 3)
        self.tbl_adc.setHorizontalHeaderLabels(
            ["逻辑通道 / 物理极", "command", "ADC raw（未标定）"]
        )
        self.tbl_adc.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.tbl_adc.verticalHeader().setVisible(False)
        self.tbl_adc.setFixedHeight(210)
        self.adc_command_cells: list[QTableWidgetItem] = []
        self.adc_raw_cells: list[QTableWidgetItem] = []
        for index, pole in enumerate(ADC_POLES):
            cells = [
                QTableWidgetItem(f"a{index} / Pole{pole}"),
                QTableWidgetItem("+00"),
                QTableWidgetItem("—"),
            ]
            for column, cell in enumerate(cells):
                cell.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
                self.tbl_adc.setItem(index, column, cell)
            self.adc_command_cells.append(cells[1])
            self.adc_raw_cells.append(cells[2])
        layout.addWidget(self.tbl_adc)
        info = QGridLayout()
        self.adc_info_labels: dict[str, QLabel] = {}
        fields = (
            ("frame_count", "frame_count"),
            ("timestamp_mcu", "MCU timestamp_ms"),
            ("receive_time", "PC receive time（UTC）"),
            ("age", "data age"),
            ("valid", "valid"),
            ("running", "running"),
            ("error_flags", "error_flags"),
            ("overrun_count", "overrun_count"),
            ("dma_error_count", "dma_error_count"),
            ("dma_late_count", "dma_late_count"),
            ("received", "本连接已收 ADC 帧 / 字节"),
            ("rejected", "拒绝的 ADC 帧"),
            ("rx_errors", "RX / parser 异常次数"),
        )
        for row, (key, title) in enumerate(fields):
            info.addWidget(QLabel(title), row, 0)
            label = QLabel("—")
            label.setWordWrap(True)
            info.addWidget(label, row, 1)
            self.adc_info_labels[key] = label
        layout.addLayout(info)
        self.lbl_adc_error = QLabel("")
        self.lbl_adc_error.setWordWrap(True)
        layout.addWidget(self.lbl_adc_error)
        self.edit_adc_csv = QLineEdit()
        self.edit_adc_csv.setReadOnly(True)
        self.edit_adc_csv.setPlaceholderText("选择新的 ADC CSV 文件")
        layout.addWidget(self.edit_adc_csv)
        buttons = QHBoxLayout()
        self.btn_adc_path = QPushButton("选择 CSV 路径")
        self.btn_adc_path.clicked.connect(self.choose_adc_csv)
        self.btn_adc_log_start = QPushButton("开始 ADC 记录")
        self.btn_adc_log_start.clicked.connect(self.start_adc_logging)
        self.btn_adc_log_stop = QPushButton("停止 ADC 记录")
        self.btn_adc_log_stop.clicked.connect(self.stop_adc_logging)
        buttons.addWidget(self.btn_adc_path)
        buttons.addWidget(self.btn_adc_log_start)
        buttons.addWidget(self.btn_adc_log_stop)
        layout.addLayout(buttons)
        self.lbl_adc_logging = QLabel("ADC CSV 未记录")
        self.lbl_adc_logging.setWordWrap(True)
        layout.addWidget(self.lbl_adc_logging)
        layout.addStretch(1)
        self._refresh_adc_display()
        return page

    def _reset_adc_session(self) -> None:
        """Discard previous connection fragments, snapshots and monitor counters."""
        self.adc_decoder = ADCStreamDecoder()
        self.adc_snapshot = None
        self.adc_received_at = None
        self.adc_receive_time = ""
        self.adc_rx_bytes = self.adc_rx_frames = 0
        self.adc_rx_errors = self.adc_parser_errors = 0
        self.adc_monitor_error = ""

    def _stop_adc_rx(self) -> None:
        """Stop receiving and finish this connection's independent ADC CSV."""
        self.adc_rx_timer.stop()
        self.stop_adc_logging()
        self.adc_decoder.buffer.clear()
        self.adc_snapshot = None
        self.adc_received_at = None
        self.adc_receive_time = ""
        self._refresh_adc_display()

    def poll_serial_rx(self) -> None:
        """Read only available bytes from the GUI-owned, nonblocking serial port."""
        port = self.ser
        if port is None:
            self._stop_adc_rx()
            return
        try:
            waiting = port.in_waiting
            if waiting <= 0:
                return
            chunk = port.read(min(waiting, ADC_RX_MAX_BYTES))
        except Exception as error:  # noqa: BLE001 - isolate monitor faults from control
            self.adc_rx_errors += 1
            self.adc_monitor_error = f"RX: {error}"
            self._refresh_adc_display()
            return
        self.adc_rx_bytes += len(chunk)
        try:
            snapshots = self.adc_decoder.feed(chunk)
        except Exception as error:  # noqa: BLE001 - isolate decoder faults from control
            self.adc_parser_errors += 1
            self.adc_monitor_error = f"ADC parser: {error}"
            self.adc_decoder.buffer.clear()
            self._refresh_adc_display()
            return
        self.adc_monitor_error = ""
        for snapshot in snapshots:
            timestamp_pc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self.adc_snapshot = snapshot
            self.adc_received_at = time.monotonic()
            self.adc_receive_time = timestamp_pc
            self.adc_rx_frames += 1
            if self.adc_logger is not None:
                self.adc_logger.enqueue(snapshot, timestamp_pc)
        self._refresh_adc_display()

    def adc_health(self, now: float | None = None) -> str:
        """Describe observation freshness and validity without changing control."""
        if self.ser is None or not self.adc_rx_timer.isActive():
            return "DISCONNECTED"
        if self.adc_monitor_error:
            return "INVALID"
        if self.adc_snapshot is None or self.adc_received_at is None:
            return "STALE"
        now = time.monotonic() if now is None else now
        if max(0.0, now - self.adc_received_at) > ADC_STALE_S:
            return "STALE"
        snapshot = self.adc_snapshot
        if not snapshot.valid or not snapshot.running or snapshot.error_flags:
            return "INVALID"
        return "LIVE"

    def _refresh_adc_display(self) -> None:
        """Refresh health, current command and logging progress independently."""
        health = self.adc_health()
        colors = {
            "LIVE": "#2e7d32",
            "STALE": "#b26a00",
            "INVALID": "#c62828",
            "DISCONNECTED": "#666666",
        }
        self.lbl_adc_health.setText(health)
        self.lbl_adc_health.setStyleSheet(
            f"font-weight: bold; color: {colors[health]};"
        )
        snapshot = self.adc_snapshot
        for index in range(6):
            self.adc_command_cells[index].setText(f"{self.last_sent_cmd[index]:+03d}")
            self.adc_raw_cells[index].setText(
                str(snapshot.raw[index]) if snapshot is not None else "—"
            )
        for key in (
            "frame_count",
            "timestamp_mcu",
            "valid",
            "running",
            "overrun_count",
            "dma_error_count",
            "dma_late_count",
        ):
            self.adc_info_labels[key].setText(
                str(getattr(snapshot, key)) if snapshot is not None else "—"
            )
        self.adc_info_labels["error_flags"].setText(
            f"0x{snapshot.error_flags:08X}" if snapshot is not None else "—"
        )
        self.adc_info_labels["receive_time"].setText(self.adc_receive_time or "—")
        age = (
            max(0.0, time.monotonic() - self.adc_received_at)
            if self.adc_received_at is not None
            else None
        )
        self.adc_info_labels["age"].setText(
            f"{age * 1000:.0f} ms" if age is not None else "—"
        )
        self.adc_info_labels["received"].setText(
            f"{self.adc_rx_frames} / {self.adc_rx_bytes}"
        )
        self.adc_info_labels["rejected"].setText(str(self.adc_decoder.rejected))
        self.adc_info_labels["rx_errors"].setText(
            f"{self.adc_rx_errors} / {self.adc_parser_errors}"
        )
        self.lbl_adc_error.setText(self.adc_monitor_error)
        logger = self.adc_logger
        busy = logger is not None and not logger.finished.is_set()
        self.btn_adc_path.setEnabled(not busy)
        self.btn_adc_log_start.setEnabled(not busy and self.adc_rx_timer.isActive())
        self.btn_adc_log_stop.setEnabled(busy)
        if logger is not None:
            if logger.error:
                state = logger.error
            elif logger.finished.is_set():
                state = "ADC CSV 已保存"
            elif logger.opened.is_set():
                state = "ADC CSV 记录中 / 正在完成已接收记录"
            else:
                state = "ADC CSV 正在打开"
            self.lbl_adc_logging.setText(
                f"{state}：{logger.path}（已写入 {logger.rows_written} 帧）"
            )

    def choose_adc_csv(self) -> None:
        """Choose a new standalone ADC CSV; keep MPC experiment export separate."""
        name = datetime.now(timezone.utc).strftime("curt_adc_%Y%m%d_%H%M%S.csv")
        path, _ = QFileDialog.getSaveFileName(
            self, "选择 ADC CSV（不覆盖已有文件）", name, "CSV (*.csv)"
        )
        if path:
            self.edit_adc_csv.setText(path)

    def start_adc_logging(self) -> None:
        """Start asynchronous recording of future complete telemetry snapshots."""
        if not self.adc_rx_timer.isActive() or self.ser is None:
            return
        if self.adc_logger is not None and not self.adc_logger.finished.is_set():
            return
        if not self.edit_adc_csv.text():
            self.choose_adc_csv()
        if not self.edit_adc_csv.text():
            return
        self.adc_logger = ADCLogger(Path(self.edit_adc_csv.text()))
        self._refresh_adc_display()

    def stop_adc_logging(self) -> None:
        """Request asynchronous drain/close; telemetry observation keeps running."""
        if self.adc_logger is not None:
            self.adc_logger.request_stop()
        self._refresh_adc_display()

    # ================= 串口 =================
    def refresh_ports(self):
        self.combo_port.clear()
        if list_ports:
            for p in list_ports.comports():
                self.combo_port.addItem(p.device)

    def toggle_serial(self):
        if self.ser:
            self._stop_adc_rx()
            # 断开前先发硬急停帧，避免 MCU 保持最后一条非零指令。
            self.emergency_stop()
            self.ser.close()
            self.ser = None
            self.btn_serial.setText("连接")
            self.lbl_serial.setText("未连接")
            return
        if not _serial:
            QMessageBox.warning(self, "串口", "未安装 pyserial")
            return
        port = self.combo_port.currentText()
        if not port:
            QMessageBox.warning(self, "串口", "没有可用串口")
            return
        try:
            self.ser = _serial.Serial(port, cfg.BAUDRATE, timeout=0)
            # 新连接从已知的零输出状态开始，不能沿用上次会话的斜率历史。
            self.ser.write(b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n")
            self.last_sent_cmd = [0] * 6
            self.btn_serial.setText("断开")
            self.lbl_serial.setText(f"已连接 {port}")
            self._reset_adc_session()
            self.adc_rx_timer.start()
            self._refresh_adc_display()
        except (OSError, ValueError) as e:
            self._stop_adc_rx()
            if self.ser:
                try:
                    self.ser.close()
                except (OSError, ValueError) as close_error:
                    self.adc_monitor_error = f"串口关闭: {close_error}"
                self.ser = None
            QMessageBox.warning(self, "串口", f"打开失败: {e}")
        except Exception:
            logger.exception("串口连接发生程序错误，清理连接")
            self._stop_adc_rx()
            try:
                if self.ser:
                    self.ser.close()
            finally:
                self.ser = None
                self.btn_serial.setText("连接")
            raise

    def send_commands(self, cmd_list):
        """统一发送安全层：幅值限幅 + 斜率限幅，再按 43 字节协议发送。
        所有模式都经过；电流约束的正式实现位于 Solver（箱约束），此处为
        最后一道独立安全层。"""
        t0 = time.perf_counter()
        limit = self._current_cmd_limit()
        cmd = apply_slew(cmd_list, self.last_sent_cmd, cfg.MAX_DELTA_CMD, max_cmd=limit)
        frame, cmd = build_command(cmd, max_cmd=limit)
        if self.ser:
            try:
                self.ser.write(frame.encode("ascii"))
            except Exception as e:
                # TX 是安全边界，程序错误也要先断开端口；日志保留完整堆栈。
                logger.exception("串口发送失败，断开端口")
                self.lbl_serial.setText(f"串口错误: {e}")
                try:
                    self.ser.close()
                except (OSError, ValueError) as close_error:
                    logger.warning("发送失败后串口关闭失败: %s", close_error)
                    self.lbl_serial.setText(f"串口错误: {e}; 关闭失败: {close_error}")
                finally:
                    self.ser = None
                    self.btn_serial.setText("连接")
        self.last_sent_cmd = list(cmd)
        self._update_local_field_force()
        self.serial_ms = (time.perf_counter() - t0) * 1e3

    def _update_local_field_force(self):
        """按当前位置与最终整数指令，更新用于状态栏和画面箭头的 B/F 模型值。"""
        if not self.model_ok or self.solver is None:
            self.current_B_T = np.zeros(3)
            self.current_F_N = np.zeros(3)
            return
        try:
            currents = np.asarray(self.last_sent_cmd, float) * self.solver.current_gain
            out = self.solver.forward_model(self._bead_pos_m(), currents)
            b_model = np.asarray(out["B"], float).copy()
            f_model = np.atleast_1d(out["F"]).astype(float, copy=True)
            self.current_B_T = b_model.copy()
            self.current_F_N = f_model.copy()
            self.current_B_T[:2] = self.force_model_to_camera @ b_model[:2]
            self.current_F_N[:2] = self.force_model_to_camera @ f_model[:2]
        except Exception:
            # 可视化诊断不得中断安全控制链；求解异常仍由正式控制分支处理。
            logger.exception("本地磁场/磁力诊断失败，显示值归零")
            self.current_B_T = np.zeros(3)
            self.current_F_N = np.zeros(3)

    # ================= 停止 =================
    def normal_stop(self):
        self.tracking = False
        self.stopping = True
        self.dir_test_on = False
        self.calib_active = False
        self.coil_test_idx = None
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        self.chk_cur_live.setChecked(False)
        self.chk_force_live.setChecked(False)
        self.mode = "STOPPING"
        self.combo_constraint.setEnabled(True)
        self._stop_worker()

    def emergency_stop(self):
        self.tracking = False
        self.stopping = False
        self.dir_test_on = False
        self.calib_active = False
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        self.chk_cur_live.setChecked(False)
        self.chk_force_live.setChecked(False)
        if self.ser:
            try:
                self.ser.write(b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n")
            except Exception:
                # 急停写入失败也必须继续清零本地状态并停止工作线程。
                logger.exception("急停零指令发送失败，继续停止本地控制")
        self.last_sent_cmd = [0] * 6
        self.mode = "IDLE"
        self.combo_constraint.setEnabled(True)
        self._stop_worker()

    def _stop_worker(self):
        """安全停止 10Hz MPC/MDM 工作线程（退出/停止时调用）"""
        self.shared.stop()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        self.worker = None

    # ================= 坐标 =================
    def mm_per_px(self):
        return self.spin_vieww.value() / self.frame_size[0]

    def px_to_world_mm(self, p):
        w, h = self.frame_size
        return (
            (p[0] - w / 2.0) * self.mm_per_px(),
            (h / 2.0 - p[1]) * self.mm_per_px(),
        )

    def world_mm_to_px(self, p):
        w, h = self.frame_size
        mpp = self.mm_per_px()
        return (p[0] / mpp + w / 2.0, h / 2.0 - p[1] / mpp)

    def _bead_pos_m(self):
        return np.array([self.state_pos_mm[0], self.state_pos_mm[1], 0.0]) * 1e-3

    # ================= 路径 =================
    def _resampled(self, pts):
        ds_px = max(self.spin_path_ds.value() / self.mm_per_px(), 0.5)
        return resample_path(pts, ds_px)

    def make_circle(self, r):
        w, h = self.frame_size
        cx, cy = w / 2.0, h / 2.0
        raw = [
            (
                cx + r * math.cos(2 * math.pi * i / 240),
                cy + r * math.sin(2 * math.pi * i / 240),
            )
            for i in range(240)
        ]
        self.path_px = self._resampled(raw)

    def make_rect(self):
        rw, rh = self.spin_rw.value(), self.spin_rh.value()
        w, h = self.frame_size
        x0, y0 = w / 2.0 - rw / 2.0, h / 2.0 - rh / 2.0
        x1, y1 = w / 2.0 + rw / 2.0, h / 2.0 + rh / 2.0
        pts = []
        step = max(3, int((1.0 / self.mm_per_px()) / 10))
        for x in np.arange(x0, x1, step):
            pts.append((float(x), y0))
        for y in np.arange(y0, y1, step):
            pts.append((x1, float(y)))
        for x in np.arange(x1, x0, -step):
            pts.append((float(x), y1))
        for y in np.arange(y1, y0, -step):
            pts.append((x0, float(y)))
        self.path_px = self._resampled(pts)

    def make_triangle(self, side):
        w, h = self.frame_size
        cx, cy = w / 2.0, h / 2.0
        R = side / math.sqrt(3)
        verts = [
            (
                cx + R * math.cos(math.pi / 2 + 2 * math.pi * k / 3),
                cy - R * math.sin(math.pi / 2 + 2 * math.pi * k / 3),
            )
            for k in range(3)
        ]
        pts = []
        step = max(3, int((1.0 / self.mm_per_px()) / 10))
        for k in range(3):
            a, b = verts[k], verts[(k + 1) % 3]
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            for i in range(n):
                t = i / n
                pts.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        self.path_px = self._resampled(pts)

    def toggle_drawing(self, on):
        self.drawing_mode = on
        self.btn_draw.setText("绘制中…(右键拖动)" if on else "开始绘制(右键拖动)")

    def clear_path(self):
        self.path_px = []
        self.draw_pts = []

    def on_video_mouse(self, x, y, button):
        if not self.drawing_mode:
            return
        RIGHT = int(Qt.MouseButton.RightButton.value)
        if button == RIGHT and not self.is_drawing:
            self.is_drawing = True
            self.draw_pts = [(x, y)]
        elif button == 0 and self.is_drawing:
            self.draw_pts.append((x, y))
        elif button == RIGHT and self.is_drawing:
            self.draw_pts.append((x, y))
            self.is_drawing = False
            self._finish_draw()

    def _finish_draw(self):
        pts = self.draw_pts
        if len(pts) < 5:
            return
        arr = np.array(pts, float)
        k = 7
        ker = np.ones(k) / k
        for c in range(2):
            padded = np.pad(arr[:, c], (k // 2, k // 2), mode="edge")
            arr[:, c] = np.convolve(padded, ker, mode="valid")
        arr[0] = pts[0]
        arr[-1] = pts[-1]
        self.path_px = self._resampled([tuple(p) for p in arr[::3]])

    # ================= 追踪 =================
    def start_tracking(self):
        if not self.model_ok:
            QMessageBox.warning(
                self, "追踪", f"模型不可用，禁止自动控制。\n{self.model_err}"
            )
            return
        if not self.path_px:
            QMessageBox.information(self, "追踪", "请先生成或绘制路径")
            return
        if self.bead is None:
            QMessageBox.information(
                self, "追踪", "尚未检测到磁珠，请先打开相机并调整识别参数"
            )
            return
        try:
            self._field_target_camera()
        except ValueError as exc:
            QMessageBox.warning(self, "追踪", str(exc))
            return
        start = (
            self.bead[:2]
            if self.bead
            else (self.frame_size[0] / 2.0, self.frame_size[1] / 2.0)
        )
        p0 = np.array(start, float)
        p1 = np.array(self.path_px[0], float)
        n = max(
            2,
            int(
                np.linalg.norm(p1 - p0)
                * self.mm_per_px()
                / max(self.spin_path_ds.value(), 0.05)
            ),
        )
        lead = [tuple(p0 + (p1 - p0) * i / n) for i in range(n)]
        self.full_path = lead + list(self.path_px)
        self.target_idx = 0
        self.traj_px = []
        self.traj_mm = []
        self.exp_log_mpc = []
        self.last_time = time.time()
        self.tracking = True
        self.stopping = False
        self.lost_since = None
        self._reset_controllers()
        self.chk_cur_live.setChecked(False)
        self.chk_force_live.setChecked(False)
        self.dir_test_on = False
        self.calib_active = False
        self.coil_test_idx = None
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        # 目标力与 10mT 场幅值联合优化，运动期间固定使用 6 路自由模式。
        self.combo_constraint.setCurrentIndex(0)
        self.combo_constraint.setEnabled(False)
        # 多速率模式：发布参考路径（世界系）并启动 10Hz MPC/MDM 工作线程
        self._stop_worker()
        # stop() 后的 Event 不能复用；每次启动建立一份干净的共享状态。
        self.shared = SharedState()
        self.shared.set_reference(
            [np.array(self.px_to_world_mm(p)) for p in self.full_path],
            self.spin_path_speed.value(),
            time.time(),
        )
        self.shared.set_kalman(self.state_pos_mm, self.state_vel_mm, time.time())
        self.shared.set_last_sent(self.last_sent_cmd)
        self._publish_mpc_params(time.time())
        self.shared.set_solver_error(None)
        self.executor = CurrentExecutor()
        self.worker = ControlWorker(self.shared, self.solver)
        self.worker.start()

    def _reset_controllers(self):
        """模式切换/启动时复位 ESO（KF 保持连续）"""
        c = self._drag_c_uN()
        self.eso_x = ESO1D(
            1.0 / c,
            self.spin_omega0.value(),
            self.spin_fal_delta.value(),
            self.spin_dist_limit.value(),
        )
        self.eso_y = ESO1D(
            1.0 / c,
            self.spin_omega0.value(),
            self.spin_fal_delta.value(),
            self.spin_dist_limit.value(),
        )
        self.z3 = np.zeros(2)

    def save_traj(self) -> None:
        if not self.exp_log_mpc:
            QMessageBox.information(self, "保存", "实验日志为空（请先运行自动追踪）")
            return
        fn, _ = QFileDialog.getSaveFileName(
            self, "保存 MPC 实验CSV", "experiment_mpc10hz.csv", "CSV (*.csv)"
        )
        if not fn:
            return
        with open(fn, "w", encoding="utf-8", newline="") as f:
            f.write(
                "timestamp,mpc_ms,solver_ms,mpc_cost,ref_x,ref_y,"
                "vref_x,vref_y,Fx_target,Fy_target,force_error,converged,"
                "horizon,W_pos,W_vel,W_u,W_du,Fmax,a0_cmd,a1_cmd,a2_cmd,"
                "a3_cmd,a4_cmd,a5_cmd\n"
            )
            f.writelines(
                ",".join(str(x) for x in row) + "\n" for row in self.exp_log_mpc
            )
        QMessageBox.information(
            self, "保存", f"已保存 {len(self.exp_log_mpc)} 行 MPC 10Hz 日志"
        )

    # ================= 诊断模式 =================
    def toggle_dir_test(self, on):
        if on and not self.model_ok:
            self.btn_dir_test.setChecked(False)
            return
        self.dir_test_on = on
        if on:
            self.tracking = False
            self.chk_cur_live.setChecked(False)
            self.chk_force_live.setChecked(False)
            self.calib_active = False
            self.coil_test_idx = None
            self.btn_calib.setChecked(False)
            self.combo_constraint.setEnabled(True)
            self._stop_worker()
            self._reset_controllers()

    def toggle_calib(self, on):
        if on and not self.model_ok:
            self.btn_calib.setChecked(False)
            return
        self.calib_active = on
        if on:
            self.tracking = False
            self.chk_cur_live.setChecked(False)
            self.chk_force_live.setChecked(False)
            self.dir_test_on = False
            self.coil_test_idx = None
            self.btn_dir_test.setChecked(False)
            self.combo_constraint.setEnabled(True)
            self._stop_worker()
            try:
                self.calib_fz_levels = [
                    float(t) for t in self.edit_calib_fz.text().split(",") if t.strip()
                ]
            except ValueError:
                self.calib_fz_levels = [0.0, 10.0, 20.0, 30.0, 40.0]
            if not self.calib_fz_levels:
                self.calib_fz_levels = [0.0]
            fz_limit = self.spin_fz_max.value()
            self.calib_fz_levels = [
                float(np.clip(x, -fz_limit, fz_limit))
                for x in self.calib_fz_levels
                if math.isfinite(x)
            ]
            if not self.calib_fz_levels:
                self.calib_fz_levels = [0.0]
            self.calib_level = 0
            self.calib_Fx = 0.0
            self.calib_state = "settle"
            self.calib_timer = 0.0
            self.calib_hits = 0
            self.calib_records = []
            self._reset_controllers()

    def save_calib(self):
        if not self.calib_records:
            QMessageBox.information(self, "标定", "暂无标定数据")
            return
        fn, _ = QFileDialog.getSaveFileName(
            self, "保存摩擦标定CSV", "friction_calib.csv", "CSV (*.csv)"
        )
        if not fn:
            return
        with open(fn, "w", encoding="utf-8", newline="") as f:
            f.write("Fz_uN,N_est_uN,F_start_uN,velocity_mm_s\n")
            f.writelines(
                f"{r['Fz']:.2f},{r['N']:.2f},{r['F_start']:.2f},{r['v']:.3f}\n"
                for r in self.calib_records
            )
        QMessageBox.information(self, "保存", f"已保存 {len(self.calib_records)} 行")

    def start_coil_scan(self):
        if not self.model_ok:
            return
        if self.bead is None:
            QMessageBox.information(
                self, "线圈标定", "请先打开相机并稳定识别磁珠，再开始扫描。"
            )
            return
        self.coil_test_idx = 0
        self.coil_test_frames = 0
        self.coil_test_vsum = np.zeros(2)
        self.coil_test_samples = 0
        self.coil_test_phase = "zero"
        self.coil_test_records = []
        self.coil_test_start = time.time()
        self.tracking = False
        self.chk_cur_live.setChecked(False)
        self.chk_force_live.setChecked(False)
        self.dir_test_on = False
        self.calib_active = False
        self.btn_dir_test.setChecked(False)
        self.btn_calib.setChecked(False)
        self.combo_constraint.setEnabled(True)
        self._stop_worker()
        self.mode = "COIL_TEST"

    # ================= 状态估计 =================
    def estimate_state(self, dt, detected):
        """视觉 → 状态估计（RAW/EMA/KALMAN），输出位置与速度 (mm, mm/s)"""
        if detected and self.bead:
            z = np.array(self.px_to_world_mm(self.bead[:2]))
        else:
            z = None
        est = self.combo_estimator.currentText()
        if est == "KALMAN":
            p, v = self.kf.step(dt, z)
        elif est == "EMA":
            if z is not None:
                if self.last_pos_mm is not None:
                    v_new = (z - self.last_pos_mm) / dt
                    self.ema_vel = (
                        cfg.VEL_LPF_ALPHA * v_new
                        + (1 - cfg.VEL_LPF_ALPHA) * self.ema_vel
                    )
                self.last_pos_mm = z.copy()
                p, v = z, self.ema_vel
            else:
                p, v = self.state_pos_mm, self.ema_vel
        else:  # RAW
            if z is not None:
                if self.last_pos_mm is not None:
                    self.ema_vel = (z - self.last_pos_mm) / dt
                self.last_pos_mm = z.copy()
                p, v = z, self.ema_vel
            else:
                p, v = self.state_pos_mm, self.ema_vel
        self.state_pos_mm = np.asarray(p, float)
        self.state_vel_mm = np.asarray(v, float)

    # ================= 主循环（≈30Hz, dt 实测） =================
    def tick(self):
        t_cycle = time.perf_counter()
        if self.last_tick_time is not None:
            actual_period = t_cycle - self.last_tick_time
            self.loop_jitter_ms = abs(actual_period - 1.0 / cfg.CONTROL_HZ) * 1e3
        self.last_tick_time = t_cycle
        now = time.time()
        dt = min(0.2, max(0.005, now - self.last_time))
        self.last_time = now

        # 1-2. 相机 + 检测
        t0 = time.perf_counter()
        mask = None
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
            else:
                # 不得继续识别上一张旧图，否则相机掉线后 bead 会永久保持“可见”。
                self.frame = None
                self.bead = None
        detected = False
        if self.frame is not None:
            vp = {
                "mode": self.combo_mode.currentText(),
                "thresh": self.sld_thresh.value(),
                "invert": self.chk_invert.isChecked(),
                "h_lo": self.spin_hlo.value(),
                "h_hi": self.spin_hhi.value(),
                "s_lo": self.spin_slo.value(),
                "v_lo": self.spin_vlo.value(),
                "morph_k": self.spin_morph_k.value(),
                "morph_open": self.chk_morph_open.isChecked(),
                "morph_close": self.chk_morph_close.isChecked(),
                "a_min": self.spin_amin.value(),
                "a_max": self.spin_amax.value(),
                "circ_min": self.spin_circ.value(),
                "r_min": self.spin_rmin.value(),
                "r_max": self.spin_rmax.value(),
            }
            cx, cy, area, mask = detect_bead(self.frame, vp)
            self.bead = None if cx is None else (cx, cy, area)
            detected = cx is not None
        self.vision_ms = (time.perf_counter() - t0) * 1e3

        # 3. 状态估计
        t1 = time.perf_counter()
        self.estimate_state(dt, detected)
        self.estimator_ms = (time.perf_counter() - t1) * 1e3
        self._update_local_field_force()

        # 4-17. 控制分支
        t2 = time.perf_counter()
        if self.stopping:
            if any(c != 0 for c in self.last_sent_cmd):
                self.send_commands([0] * 6)
            else:
                self.stopping = False
                self.mode = "IDLE"
        elif self.chk_cur_live.isChecked():
            self.mode = "MANUAL_CURRENT"
            self.send_commands(self.man_currents)
        elif self.dir_test_on:
            self.mode = "FORCE_DIRECTION_TEST"
            self.direction_test_step()
        elif self.calib_active:
            self.mode = "FRICTION_CALIBRATION"
            self.calib_step(dt)
        elif self.coil_test_idx is not None:
            self.mode = "COIL_TEST"
            self.coil_test_step(dt)
        elif self.tracking:
            self.mode = "AUTO_TRACK"
            self.control_step(dt)
        elif self.chk_force_live.isChecked() and self.model_ok:
            self.mode = "MANUAL_FORCE"
            self._solve_and_send(
                np.array(
                    [self.spin_fx.value(), self.spin_fy.value(), self.spin_fz.value()]
                )
            )
        else:
            self.mode = "IDLE"
        self.controller_ms = (time.perf_counter() - t2) * 1e3

        total = (time.perf_counter() - t_cycle) * 1e3
        self.total_ms = total
        self.cycle_over_cnt = (
            self.cycle_over_cnt + 1 if total > 1000.0 / cfg.CONTROL_HZ else 0
        )

        self.render(mask)
        self.update_status(dt)

    # ================= 方向测试 =================
    def direction_test_step(self):
        f = np.array(
            [self.spin_ftx.value(), self.spin_fty.value(), self.spin_ftz.value()]
        )
        self._solve_and_send(f)
        rec = self.last_solver_rec
        if rec is None:
            return
        F_act = rec["achieved_force"] * 1e6
        th_d = (
            math.degrees(math.atan2(f[1], f[0]))
            if abs(f[0]) + abs(f[1]) > 1e-12
            else 0.0
        )
        th_a = (
            math.degrees(math.atan2(F_act[1], F_act[0]))
            if abs(F_act[0]) + abs(F_act[1]) > 1e-9
            else 0.0
        )
        th_err = (th_a - th_d + 180.0) % 360.0 - 180.0
        self.last_angle_err = th_err
        v = self.state_vel_mm
        comp = {0: "+X 方向", 90: "+Y 方向", 180: "−X 方向", -90: "−Y 方向"}

        def dir_name(a):
            for k, name in comp.items():
                if abs((a - k + 180) % 360 - 180) < 30:
                    return name
            return f"{a:.0f}°"

        self.lbl_dir.setText(
            f"F_des: [{f[0]:+.1f}, {f[1]:+.1f}, {f[2]:+.1f}] µN\n"
            f"F_act: [{F_act[0]:+.1f}, {F_act[1]:+.1f}, {F_act[2]:+.1f}] µN\n"
            f"幅值误差: {rec['force_error_percent']:.1f}%\n"
            f"目标方向: {dir_name(th_d)} ({th_d:.0f}°) | "
            f"实际方向: {dir_name(th_a)} ({th_a:.0f}°)\n"
            f"方向误差: {th_err:+.1f}° | 视觉速度: ({v[0]:+.2f},{v[1]:+.2f}) mm/s"
        )

    # ================= 摩擦标定 =================
    def calib_step(self, dt):
        fz = self.calib_fz_levels[self.calib_level]
        w_eff = fm.effective_weight_uN(
            self.spin_rho_b.value(),
            self.spin_rho_f.value(),
            self.spin_beadD.value() * 0.5e-3,
        )
        n_est = fm.normal_force_uN(w_eff, fz, self.spin_n_min.value())
        v = float(np.linalg.norm(self.state_vel_mm))
        if self.calib_state == "settle":
            self.calib_timer += dt
            self.send_commands([0] * 6)
            if self.calib_timer > 0.8:
                self.calib_state = "ramp"
                self.calib_Fx = 0.0
                self.calib_hits = 0
            self.lbl_calib.setText(
                f"档位 {self.calib_level+1}/{len(self.calib_fz_levels)}: "
                f"Fz={fz:.0f}µN 静置中… N_est={n_est:.1f}µN"
            )
            return
        # 斜坡
        self.calib_Fx += self.spin_calib_rate.value() * dt
        self._solve_and_send(np.array([self.calib_Fx, 0.0, fz]))
        if v > self.spin_calib_vth.value():
            self.calib_hits += 1
        else:
            self.calib_hits = 0
        self.lbl_calib.setText(
            f"档位 {self.calib_level+1}/{len(self.calib_fz_levels)}: Fz={fz:.0f}µN "
            f"Fx={self.calib_Fx:.1f}µN |v|={v:.2f}mm/s N_est={n_est:.1f}µN"
        )
        if self.calib_hits >= 3:
            self.calib_records.append(
                {"Fz": fz, "N": n_est, "F_start": self.calib_Fx, "v": v}
            )
            self.calib_level += 1
            self.calib_state = "settle"
            self.calib_timer = 0.0
            self.send_commands([0] * 6)
            if self.calib_level >= len(self.calib_fz_levels):
                self.calib_active = False
                self.btn_calib.setChecked(False)
                rows = "\n".join(
                    f"Fz={r['Fz']:.0f}: F_start={r['F_start']:.1f}µN "
                    f"(N={r['N']:.1f})"
                    for r in self.calib_records
                )
                self.lbl_calib.setText("标定完成：\n" + rows)
                self.send_commands([0] * 6)

    # ================= 线圈扫描 =================
    def coil_test_step(self, dt):
        j = self.coil_test_idx
        if j is None:
            return
        if self.bead is None:
            self.normal_stop()
            self.lbl_force_frame.setText("标定中止：磁珠识别丢失")
            self.lbl_force_frame.setStyleSheet("color: #c62828;")
            return
        # 目标约 1A，但永远不超过用户设置的统一上限。
        gain = self.solver.current_gain
        test_cmd = min(self._current_cmd_limit(), max(1, round(1.0 / gain)))
        if self.coil_test_phase == "zero":
            self.send_commands([0] * 6)
            if all(c == 0 for c in self.last_sent_cmd):
                self.coil_test_frames += 1
            else:
                self.coil_test_frames = 0
            if self.coil_test_frames >= 8:  # 静置约 0.27s
                self.coil_test_phase = "drive"
                self.coil_test_frames = 0
                self.coil_test_samples = 0
                self.coil_test_vsum = np.zeros(2)
            return

        target = [0] * 6
        target[j] = test_cmd
        self.send_commands(target)
        if self.last_sent_cmd == target:
            self.coil_test_frames += 1
            # 到达目标后先等待约 0.33s，再对后续约 0.67s 求平均。
            if self.coil_test_frames > 10:
                self.coil_test_vsum += self.state_vel_mm
                self.coil_test_samples += 1
        else:
            self.coil_test_frames = 0
        if self.coil_test_samples < 20:
            return

        v_mean = self.coil_test_vsum / max(self.coil_test_samples, 1)
        currents = np.zeros(6)
        currents[j] = test_cmd * gain
        f_model = self.solver.force_at(self._bead_pos_m(), currents)
        self.coil_test_records.append(
            {"coil": j, "velocity": v_mean.copy(), "F_model": f_model.copy()}
        )
        self.tbl_coil.setItem(j, 1, QTableWidgetItem(f"{currents[j]:.3f}"))
        for col, value in enumerate(f_model * 1e6, start=2):
            self.tbl_coil.setItem(j, col, QTableWidgetItem(f"{value:+.1f}"))
        self._set_coil_table_exp(j, v_mean[0], v_mean[1])
        self.send_commands([0] * 6)
        if j + 1 < 6:
            self.coil_test_idx = j + 1
            self.coil_test_phase = "zero"
            self.coil_test_frames = 0
            self.coil_test_samples = 0
            self.coil_test_vsum = np.zeros(2)
            return

        self.coil_test_idx = None
        self._fit_force_frame_calibration()
        self.save_settings()
        # 最后一路也必须按斜率逐帧归零。
        self.stopping = True
        self.mode = "STOPPING"

    # ================= 自动追踪控制器 =================
    def control_step(self, dt: float) -> None:
        """实机自动追踪的薄入口，唯一调用 MPC 多速率控制链。"""
        self.mpc_track_step(dt)

    def spin_tol_default(self):
        return 0.5

    # ================= MPC 模式（30Hz 电流执行，10Hz MPC/MDM 在工作线程） =================
    def mpc_track_step(self, dt):
        if self.bead is None:
            if self.lost_since is None:
                self.lost_since = time.time()
            elif time.time() - self.lost_since > 1.0:
                self.normal_stop()  # 视觉丢失 >1s 正常停止
            return
        self.lost_since = None
        pos = self.state_pos_mm
        while self.target_idx < len(self.full_path):
            target = np.array(self.px_to_world_mm(self.full_path[self.target_idx]))
            if np.linalg.norm(target - pos) >= self.spin_tol_default():
                break
            self.target_idx += 1
        if self.target_idx >= len(self.full_path):
            self.normal_stop()
            return
        now = time.time()
        # 用上一帧实际执行器估计磁力更新 z3，再把最新扰动力发布给 10Hz MPC。
        if self.chk_eso.isChecked():
            u_xy_uN = np.asarray(self.last_F_actual[:2], float) * 1e6
            self.z3[0] = self.eso_x.step(dt, pos[0], u_xy_uN[0])
            self.z3[1] = self.eso_y.step(dt, pos[1], u_xy_uN[1])
        else:
            self.z3[:] = 0.0
        self.shared.set_kalman(pos, self.state_vel_mm, now)
        self.shared.set_last_sent(self.last_sent_cmd)
        self._publish_mpc_params(now)
        err = self.shared.get_solver_error()
        if err:
            self.lbl_dir.setText(f"⚠ 工作线程异常，已安全停止：{err}")
            self.normal_stop()
            return
        # 30Hz 电流执行层：插值 → 斜率限幅 → 量化 → 发送
        t0 = time.perf_counter()
        diag = self.executor.step(
            self.shared,
            dt,
            self._bead_pos_m(),
            self.solver,
            self.last_sent_cmd,
            max_cmd=self._current_cmd_limit(),
        )
        diag = self._diag_to_camera(diag)
        self.send_commands(diag["cmd"])
        self.shared.set_last_sent(self.last_sent_cmd)
        self.last_F_actual = diag["F_est"].copy()
        self.last_diag = diag
        _, _, _, target_snapshot = self.shared.get_I_target()
        if target_snapshot.get("rec") is not None:
            self.last_solver_rec = target_snapshot["rec"]
        self.controller_ms = (time.perf_counter() - t0) * 1e3
        # 工作线程排空 10Hz 日志
        self.exp_log_mpc.extend(self.shared.drain_mpc_log())
        # 轨迹
        self.traj_px.append(tuple(self.bead[:2]))
        self.traj_mm.append(tuple(self.state_pos_mm))

    def _publish_mpc_params(self, now):
        """把主线程可调参数发布给 10Hz MPC/MDM 工作线程。"""
        w_eff = fm.effective_weight_uN(
            self.spin_rho_b.value(),
            self.spin_rho_f.value(),
            self.spin_beadD.value() * 0.5e-3,
        )
        fz_lift = (
            fm.lift_force_uN(
                w_eff,
                self.spin_normal_ratio.value(),
                self.spin_fz_max.value(),
                self.spin_n_min.value(),
            )
            if self.chk_lift.isChecked()
            else 0.0
        )
        self.shared.set_params(
            {
                "fmax": self.spin_mpc_fmax.value(),
                "mpc_horizon": self.spin_mpc_horizon.value(),
                "mpc_w_pos": self.spin_mpc_w_pos.value(),
                "mpc_w_vel": self.spin_mpc_w_vel.value(),
                "mpc_w_u": self.spin_mpc_w_u.value(),
                "mpc_w_du": self.spin_mpc_w_du.value(),
                "max_active": self.n_coils_total,
                "mpc_on": self.tracking,
                "fz_lift": fz_lift,
                "eso_d": self.z3.copy() if self.chk_eso.isChecked() else np.zeros(2),
                "viscosity_mPas": self.viscosity_mPas,
                "bead_radius_m": self.spin_beadD.value() * 0.5e-3,
                "max_cmd": self._current_cmd_limit(),
                "force_model_to_camera": self.force_model_to_camera.copy(),
                "field_direction": self._field_target_camera()[0],
                "field_magnitude_mT": self._field_target_camera()[1],
            },
            now,
        )

    # ================= 渲染 =================
    @staticmethod
    def _draw_vector_arrow(image, origin, vector, reference_magnitude, color, label):
        """把三维矢量以轻微等轴投影画到二维相机画面；Z 分量也保持可见。"""
        vec = np.asarray(vector, float)
        magnitude = float(np.linalg.norm(vec))
        if magnitude <= 1e-15:
            return
        projected = np.array([vec[0] + 0.35 * vec[2], -(vec[1] + 0.35 * vec[2])], float)
        pn = float(np.linalg.norm(projected))
        if pn <= 1e-15:
            projected = np.array([0.0, -1.0 if vec[2] >= 0 else 1.0])
            pn = 1.0
        length = float(
            np.clip(70.0 * magnitude / max(reference_magnitude, 1e-15), 18.0, 95.0)
        )
        end = (
            round(origin[0] + projected[0] / pn * length),
            round(origin[1] + projected[1] / pn * length),
        )
        cv2.arrowedLine(image, origin, end, color, 3, cv2.LINE_AA, tipLength=0.25)
        cv2.putText(
            image,
            label,
            (end[0] + 4, end[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    def render(self, mask):
        disp_w = max(1, self.video.width())
        disp_h = max(1, self.video.height())
        if self.chk_show_binary.isChecked() and mask is not None:
            disp_src = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            disp_src = (
                self.frame
                if self.frame is not None
                else np.zeros((self.frame_size[1], self.frame_size[0], 3), np.uint8)
            )
        disp = cv2.resize(disp_src, (disp_w, disp_h))
        sx = disp.shape[1] / self.frame_size[0]
        sy = disp.shape[0] / self.frame_size[1]
        for i in range(1, len(self.path_px)):
            p0 = (int(self.path_px[i - 1][0] * sx), int(self.path_px[i - 1][1] * sy))
            p1 = (int(self.path_px[i][0] * sx), int(self.path_px[i][1] * sy))
            cv2.line(disp, p0, p1, (0, 255, 0), 1)
        for i in range(1, len(self.traj_px)):
            p0 = (int(self.traj_px[i - 1][0] * sx), int(self.traj_px[i - 1][1] * sy))
            p1 = (int(self.traj_px[i][0] * sx), int(self.traj_px[i][1] * sy))
            cv2.line(disp, p0, p1, (0, 0, 255), 1)
        if self.tracking and self.target_idx < len(self.full_path):
            t = self.full_path[self.target_idx]
            cv2.circle(disp, (int(t[0] * sx), int(t[1] * sy)), 6, (0, 255, 255), 2)
        mpc_snapshot = None
        if self.tracking:
            _, seq, _, mpc_snapshot = self.shared.get_I_target()
            if seq > 0:
                ref_mm = np.asarray(mpc_snapshot.get("ref_target", np.zeros(2)), float)
                ref_px = self.world_mm_to_px(ref_mm)
                ref_draw = (int(ref_px[0] * sx), int(ref_px[1] * sy))
                cv2.circle(disp, ref_draw, 7, (255, 0, 255), 2)
                cv2.putText(
                    disp,
                    "MPC ref",
                    (ref_draw[0] + 7, ref_draw[1] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
        if self.bead:
            cx, cy, area = self.bead
            center = (int(cx * sx), int(cy * sy))
            r = max(3, int(math.sqrt(area) * (sx + sy) / 4))
            cv2.circle(disp, center, r, (255, 255, 0), 2)
            cv2.circle(disp, center, 2, (255, 255, 255), -1)
            x_mm, y_mm = self.px_to_world_mm((cx, cy))
            cv2.putText(
                disp,
                f"({x_mm:+.2f},{y_mm:+.2f})mm",
                (int(cx * sx) + 10, int(cy * sy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )
            # OpenCV 为 BGR：磁场蓝色，受力红色。箭头从识别目标中心出发。
            self._draw_vector_arrow(
                disp,
                center,
                self.current_B_T,
                self.spin_bmag.value() * 1e-3,
                (255, 0, 0),
                "B",
            )
            self._draw_vector_arrow(
                disp,
                center,
                self.current_F_N,
                max(self.spin_mpc_fmax.value(), 1.0) * 1e-6,
                (0, 0, 255),
                "F act",
            )
            if mpc_snapshot is not None:
                f_target = np.asarray(mpc_snapshot.get("F_target", np.zeros(2)), float)
                self._draw_vector_arrow(
                    disp,
                    center,
                    np.array([f_target[0], f_target[1], 0.0]) * 1e-6,
                    max(self.spin_mpc_fmax.value(), 1.0) * 1e-6,
                    (0, 165, 255),
                    "F mpc",
                )
        bar_px = int(5.0 / self.mm_per_px() * sx)
        cv2.line(
            disp, (15, disp_h - 20), (15 + bar_px, disp_h - 20), (255, 255, 255), 2
        )
        cv2.putText(
            disp,
            "5 mm",
            (15, disp_h - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )
        if self.video_writer is not None:
            cv2.circle(disp, (18, 20), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                disp,
                "REC",
                (30, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        # 保留 BGR 副本供截图使用，并把同一张最终标注画面写入录像。
        self.last_render_bgr = disp.copy()
        self._record_rendered_frame(self.last_render_bgr)
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        img = QImage(rgb.data, disp_w, disp_h, 3 * disp_w, QImage.Format_RGB888)
        self.video.setPixmap(QPixmap.fromImage(img.copy()))

    def update_status(self, dt):
        if self.stopping:
            mode = "停止归零中(斜率限制)"
        else:
            mode = self.mode
        pos = (
            f"({self.state_pos_mm[0]:+.2f},{self.state_pos_mm[1]:+.2f})mm"
            if self.last_pos_mm is not None or self.kf.initialized
            else "-"
        )
        vel = f"v=({self.state_vel_mm[0]:+.2f},{self.state_vel_mm[1]:+.2f})"
        rec = self.last_solver_rec
        extra = ""
        if rec is not None:
            warn = " ⚠求解>33ms" if rec["elapsed_ms"] > cfg.SOLVER_WARN_MS else ""
            cons = (
                " | 磁力不可达/受电流约束" if rec["current_constraint_active"] else ""
            )
            extra = (
                f" | F_act:[{rec['achieved_force'][0] * 1e6:+.0f},"
                f"{rec['achieved_force'][1] * 1e6:+.0f},"
                f"{rec['achieved_force'][2] * 1e6:+.0f}]µN"
                f" | err{rec['force_error_percent']:.0f}%{cons}{warn}"
            )
        if self.last_diag is not None:
            d = self.last_diag
            align_warn = " ⚠对齐慢" if d["ratio"] < cfg.ALIGNMENT_RATIO_MIN else ""
            low_warn = " ⚠低场" if d["low_field"] else ""
            _, seq, _, snap = self.shared.get_I_target()
            f_target = np.asarray(snap.get("F_target", np.zeros(2)), float)
            ref_target = np.asarray(snap.get("ref_target", np.zeros(2)), float)
            mpc_line = (
                f"\nMPC#{seq}: F_target="
                f"[{f_target[0]:+.2f},{f_target[1]:+.2f}]µN | "
                f"ref=[{ref_target[0]:+.2f},{ref_target[1]:+.2f}]mm | "
                f"J={float(snap.get('mpc_cost', 0.0)):.3f}"
            )
            self.lbl_multirate.setText(
                f"视觉/Kalman: {cfg.VISION_HZ:.0f}/{cfg.KALMAN_HZ:.0f} Hz | "
                f"MPC/MDM: {cfg.MPC_HZ:.0f}/{cfg.SOLVER_HZ:.0f} Hz | "
                f"电流: {cfg.CURRENT_HZ:.0f} Hz | PWM: {cfg.PWM_HZ/1000:.0f} kHz\n"
                f"B={d['Bmag_mT']:.3f} mT | τ_align={d['tau_ms']:.2f} ms | "
                f"T/τ={d['ratio']:.2f}{align_warn}{low_warn}\n"
                f"I_est(A)=[{', '.join(f'{x:+.3f}' for x in d['I_est'])}]"
                f"{mpc_line}"
            )
        B_mT = self.current_B_T * 1e3
        F_uN = self.current_F_N * 1e6
        bmag = float(np.linalg.norm(B_mT))
        fmag = float(np.linalg.norm(F_uN))
        try:
            target_dir, target_mag = self._field_target_camera()
            b_vec_target = target_dir * target_mag
            field_vec_err = float(np.linalg.norm(B_mT - b_vec_target))
            align_text = (
                f" | B目标={np.round(b_vec_target, 1).tolist()}mT"
                f" 矢量误差={field_vec_err:.2f}mT"
                if self.tracking
                else ""
            )
        except ValueError:
            align_text = " | ⚠对齐场方向无效" if self.tracking else ""
        self.field_lbl.setText(
            f"当前位置模型值  B=[{B_mT[0]:+.2f},{B_mT[1]:+.2f},{B_mT[2]:+.2f}]mT "
            f"|B|={bmag:.2f}mT{align_text}\n"
            f"F=[{F_uN[0]:+.2f},{F_uN[1]:+.2f},{F_uN[2]:+.2f}]µN "
            f"|F|={fmag:.2f}µN   蓝=B，红=实际力，橙=MPC目标力，品红=MPC参考点"
        )
        timeout = " | ⚠控制周期超时" if self.cycle_over_cnt >= 5 else ""
        self.status_lbl.setText(
            f"模式: {mode} | 磁珠: {pos} {vel} | "
            f"电流: [{', '.join(f'{c:+03d}' for c in self.last_sent_cmd)}]{extra} | "
            f"Cycle {self.total_ms:.1f}ms ({1.0 / dt:.0f}Hz){timeout}"
        )

    # ================= 参数持久化 =================
    def _settings_items(self):
        return {
            "port": self.combo_port,
            "cam_index": self.spin_cam,
            "current_max": self.spin_cmd_max,
            "view_width": self.spin_vieww,
            "show_binary": self.chk_show_binary,
            "vis_mode": self.combo_mode,
            "thresh": self.sld_thresh,
            "invert": self.chk_invert,
            "h_lo": self.spin_hlo,
            "h_hi": self.spin_hhi,
            "s_lo": self.spin_slo,
            "v_lo": self.spin_vlo,
            "morph_k": self.spin_morph_k,
            "morph_open": self.chk_morph_open,
            "morph_close": self.chk_morph_close,
            "a_min": self.spin_amin,
            "a_max": self.spin_amax,
            "circ_min": self.spin_circ,
            "r_min": self.spin_rmin,
            "r_max": self.spin_rmax,
            "circ_r": self.spin_circ_r,
            "rect_w": self.spin_rw,
            "rect_h": self.spin_rh,
            "tri_side": self.spin_tri,
            "path_speed": self.spin_path_speed,
            "path_ds": self.spin_path_ds,
            "field_dir_x": self.spin_bdir_x,
            "field_dir_y": self.spin_bdir_y,
            "field_dir_z": self.spin_bdir_z,
            "field_magnitude_mT": self.spin_bmag,
            "mpc_horizon": self.spin_mpc_horizon,
            "mpc_fmax": self.spin_mpc_fmax,
            "mpc_w_pos": self.spin_mpc_w_pos,
            "mpc_w_vel": self.spin_mpc_w_vel,
            "mpc_w_u": self.spin_mpc_w_u,
            "mpc_w_du": self.spin_mpc_w_du,
            "constraint_mode": self.combo_constraint,
            "man_currents": list(self.cur_spins),
            "fx": self.spin_fx,
            "fy": self.spin_fy,
            "fz": self.spin_fz,
            "cur_live": self.chk_cur_live,
            "force_live": self.chk_force_live,
            "model_path": self.edit_model,
            "gain": self.spin_gain,
            "bead_d": self.spin_beadD,
            "bead_br": self.spin_br,
            "viscosity": self.spin_visc,
            "rho_bead": self.spin_rho_b,
            "rho_fluid": self.spin_rho_f,
            "lift_enable": self.chk_lift,
            "normal_ratio": self.spin_normal_ratio,
            "fz_max": self.spin_fz_max,
            "n_min": self.spin_n_min,
            "eso_enable": self.chk_eso,
            "eso_omega0": self.spin_omega0,
            "eso_delta": self.spin_fal_delta,
            "eso_limit": self.spin_dist_limit,
            "estimator": self.combo_estimator,
            "q_pos": self.spin_q_pos,
            "q_vel": self.spin_q_vel,
            "r_meas": self.spin_r_meas,
            "calib_fz": self.edit_calib_fz,
            "calib_rate": self.spin_calib_rate,
            "calib_vth": self.spin_calib_vth,
        }

    @staticmethod
    def _widget_get(w):
        if isinstance(w, (QSpinBox, QDoubleSpinBox, QSlider)):
            return w.value()
        if isinstance(w, QCheckBox):
            return w.isChecked()
        if isinstance(w, QComboBox):
            return w.currentText()
        if isinstance(w, QLineEdit):
            return w.text()
        return None

    @staticmethod
    def _widget_set(w, val):
        try:
            if isinstance(w, (QSpinBox, QDoubleSpinBox, QSlider)):
                w.setValue(type(w.value())(val))
            elif isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            elif isinstance(w, QComboBox):
                idx = w.findText(str(val))
                if idx >= 0:
                    w.setCurrentIndex(idx)
            elif isinstance(w, QLineEdit):
                w.setText(str(val))
        except (TypeError, ValueError, OverflowError) as exc:
            logger.warning("忽略无效控件设置 %r: %s", val, exc)

    def save_settings(self):
        data = {}
        for key, item in self._settings_items().items():
            if isinstance(item, list):
                data[key] = [self._widget_get(w) for w in item]
            else:
                data[key] = self._widget_get(item)
        data["force_model_to_camera"] = self.force_model_to_camera.tolist()
        data["force_frame_calibrated"] = bool(self.force_frame_calibrated)
        data["force_frame_rms_deg"] = (
            self.force_frame_rms_deg if np.isfinite(self.force_frame_rms_deg) else None
        )
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
        except (OSError, UnicodeError) as e:
            print(f"参数保存失败: {e}")
        except Exception:
            # closeEvent 会保存设置；程序错误也不能阻止随后执行急停。
            logger.exception("参数保存发生程序错误")

    def load_settings(self):
        if not os.path.exists(SETTINGS_FILE):
            return
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            print(f"参数载入失败，使用默认值: {e}")
            return
        if not isinstance(data, dict):
            print("参数载入失败，使用默认值: 设置根节点必须是 JSON 对象")
            return
        for key, item in self._settings_items().items():
            if key not in data:
                continue
            val = data[key]
            if isinstance(item, list) and isinstance(val, list):
                for w, v in zip(item, val):
                    self._widget_set(w, v)
            elif not isinstance(item, list):
                self._widget_set(item, val)
        if "constraint_mode" not in data and data.get("unipolar"):
            self.combo_constraint.setCurrentIndex(1)
        try:
            R = np.asarray(data.get("force_model_to_camera", np.eye(2)), float)
            if R.shape == (2, 2) and np.all(np.isfinite(R)):
                should_be_I = R.T @ R
                if np.allclose(should_be_I, np.eye(2), atol=0.05):
                    self.force_model_to_camera = R
                    self.force_frame_calibrated = bool(
                        data.get("force_frame_calibrated", False)
                    )
                    rms = data.get("force_frame_rms_deg")
                    self.force_frame_rms_deg = (
                        float(rms) if rms is not None else float("nan")
                    )
        except (TypeError, ValueError, OverflowError) as exc:
            logger.warning("方向标定设置无效，恢复默认值: %s", exc)
            self.reset_force_frame_calibration()
        self._update_force_frame_label()

    # ================= 退出清理 =================
    def closeEvent(self, e):
        self._stop_adc_rx()
        self.adc_status_timer.stop()
        self.timer.stop()
        self.save_settings()
        self.tracking = False
        self.stopping = False
        self.emergency_stop()
        self._stop_worker()  # 安全停止 10Hz 工作线程
        self.close_camera()
        if self.ser:
            self.ser.close()
        if self.adc_logger is not None:
            self.adc_logger.wait_closed()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MagneticDipoleControl()
    win.show()
    sys.exit(app.exec())

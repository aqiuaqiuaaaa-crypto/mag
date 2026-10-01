# -*- coding: utf-8 -*-
"""
强化学习路径控制程序（RL 版，与 PID 版并行的另一套控制程序）
============================================================
- 控制器：ES 训练的 numpy MLP 策略（models/rl_policy.json，train_rl.py 训练），
  输入视觉追踪得到的状态（目标相对位置/速度/当前指令），输出每帧指令增量：
      Δcmd = clip(action × 9, ±9)，cmd = clip(round(cmd_prev + Δcmd), ±99)
  斜率（≤0.1818A/帧）与幅值（±99 ↔ ±2A）约束由该映射结构性保证，
  且所有发送仍经过统一安全层（与 PID 版一致）。
- 视觉/串口/安全层/路径功能复用 magnetic_dipole_pid 的实现。
- 使用前先运行 `py train_rl.py` 生成策略文件。
"""

import sys
import os
import json
import math
import time

import numpy as np
import cv2

try:
    import serial as _serial
    from serial.tools import list_ports
except ImportError:
    _serial = None
    list_ports = None

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QGroupBox, QTabWidget, QSlider, QDoubleSpinBox,
    QSpinBox, QCheckBox, QComboBox, QFileDialog, QMessageBox, QLineEdit,
)

import config as cfg
from dipole_solver import DipoleSolver
from rl_policy import MLPPolicy, make_state, action_to_cmd

from magnetic_dipole_pid import (
    VideoLabel, detect_bead, build_command, apply_slew, CSV_HEADER,
)

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "rl_gui_settings.json")
DEFAULT_POLICY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "models", "rl_policy.json")


class RLPathControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("强化学习路径控制（180 磁偶极子视觉追踪）")
        self.resize(1500, 780)

        # 相机 / 视觉
        self.cap = None
        self.frame = None
        self.frame_size = (cfg.FRAME_W, cfg.FRAME_H)
        self.bead = None
        self.traj_px, self.traj_mm = [], []

        # 路径
        self.path_px = []
        self.full_path = []
        self.drawing_mode = False
        self.is_drawing = False
        self.draw_pts = []
        self.tracking = False
        self.target_idx = 0
        self.lost_since = None

        # 控制
        self.last_time = time.time()
        self.last_pos_mm = None
        self.vel_mm = np.zeros(2)
        self.last_sent_cmd = [0] * 6
        self.man_currents = [0] * 6
        self.stopping = False
        self.exp_log = []
        self.log_t0 = None
        self.last_rl_ms = 0.0

        # 串口 / 模型 / 策略
        self.ser = None
        self.solver = None
        self.model_ok = False
        self.model_err = ""
        self.policy = None
        self.policy_err = ""
        self.viscosity_mPas = cfg.VISCOSITY_MPA_S

        self._build_ui()
        self.load_settings()
        self._reload_solver()
        self._reload_policy()

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(cfg.TICK_MS)

    # ================= 模型与策略加载 =================
    def _reload_solver(self):
        try:
            self.solver = DipoleSolver.from_json(self.edit_model.text().strip())
            self.solver.validate_model()
            self.model_ok, self.model_err = True, ""
        except Exception as e:
            self.solver, self.model_ok = None, False
            self.model_err = f"模型加载失败: {e}"
        self._update_status_labels()

    def _reload_policy(self):
        try:
            self.policy = MLPPolicy.load(self.edit_policy.text().strip())
            self.policy_err = ""
        except Exception as e:
            self.policy, self.policy_err = None, f"策略加载失败: {e}"
        self._update_status_labels()

    def _update_status_labels(self):
        if self.model_ok:
            self.lbl_model.setText("✓ " + self.solver.model_info.get("model_name", ""))
            self.lbl_model.setStyleSheet("color: #2e7d32; font-family: Consolas;")
        else:
            self.lbl_model.setText("✗ " + self.model_err)
            self.lbl_model.setStyleSheet("color: #c62828; font-family: Consolas;")
        if self.policy is not None:
            meta = getattr(self.policy, "meta", {})
            self.lbl_policy.setText(
                f"✓ ES-MLP ({self.policy.state_dim}→{self.policy.hidden}→"
                f"{self.policy.act_dim})  训练完成率 "
                f"{meta.get('eval_completion', float('nan')) * 100:.1f}%  "
                f"终点距 {meta.get('eval_final_dist_mm', float('nan')):.2f}mm")
            self.lbl_policy.setStyleSheet("color: #2e7d32; font-family: Consolas;")
        else:
            self.lbl_policy.setText("✗ " + self.policy_err +
                                    "\n请先运行: py train_rl.py")
            self.lbl_policy.setStyleSheet("color: #c62828; font-family: Consolas;")
        ready = self.model_ok and self.policy is not None
        self.btn_rl_start.setEnabled(ready)

    # ================= UI =================
    def _build_ui(self):
        left = QVBoxLayout()
        self.video = VideoLabel()
        self.video.mouseEvent.connect(self.on_video_mouse)
        left.addWidget(self.video)
        self.status_lbl = QLabel("模式: 空闲")
        self.status_lbl.setStyleSheet("font-family: Consolas; font-size: 13px;")
        left.addWidget(self.status_lbl)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._tab_manual_current(), "手动电流")
        self.tabs.addTab(self._tab_rl(), "RL 控制")
        self.tabs.addTab(self._tab_vision(), "识别参数")
        self.tabs.addTab(self._tab_path(), "路径")
        self.tabs.setFixedWidth(460)

        root = QHBoxLayout()
        root.addLayout(left, stretch=3)
        root.addWidget(self.tabs, stretch=0)
        outer = QWidget()
        v = QVBoxLayout(outer)
        v.setContentsMargins(0, 0, 0, 0)
        v.addLayout(self._serial_bar())
        v.addLayout(root, stretch=1)
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
        btn_stop = QPushButton("停止")
        btn_stop.clicked.connect(self.normal_stop)
        bar.addWidget(btn_stop)
        btn_estop = QPushButton("急停")
        btn_estop.setStyleSheet("background-color: #b71c1c; color: white;")
        btn_estop.clicked.connect(self.emergency_stop)
        bar.addWidget(btn_estop)
        bar.addStretch(1)
        self.refresh_ports()
        return bar

    def _dspin(self, lo, hi, val, step, dec=2):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi); sp.setDecimals(dec)
        sp.setSingleStep(step); sp.setValue(val)
        return sp

    # ---------- 手动电流（复用 PID 版逻辑，安全层一致） ----------
    def _tab_manual_current(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("各路电流指令 a0~a5 = +X,+Y,+Z,−X,−Y,−Z (±99 ↔ ±2A)")
        grid = QGridLayout(box)
        self.cur_spins = []
        for j in range(6):
            grid.addWidget(QLabel(f"a{j} ({cfg.COIL_ORDER[j]})"), j, 0)
            s = QSlider(Qt.Horizontal)
            s.setRange(-cfg.CMD_MAX, cfg.CMD_MAX)
            sp = QSpinBox()
            sp.setRange(-cfg.CMD_MAX, cfg.CMD_MAX)
            s.valueChanged.connect(sp.setValue)
            sp.valueChanged.connect(s.setValue)
            sp.valueChanged.connect(lambda val, idx=j: self.man_currents.__setitem__(idx, val))
            grid.addWidget(s, j, 1)
            grid.addWidget(sp, j, 2)
            self.cur_spins.append(sp)
        v.addWidget(box)
        h = QHBoxLayout()
        self.chk_cur_live = QCheckBox("实时发送(经安全层)")
        btn_send = QPushButton("发送")
        btn_zero = QPushButton("清零(斜率限制)")
        btn_send.clicked.connect(lambda: self.send_commands(self.man_currents))
        btn_zero.clicked.connect(lambda: self.send_commands([0] * 6))
        h.addWidget(self.chk_cur_live)
        h.addWidget(btn_send)
        h.addWidget(btn_zero)
        v.addLayout(h)
        v.addStretch(1)
        return w

    # ---------- RL 控制 ----------
    def _tab_rl(self):
        w = QWidget()
        v = QVBoxLayout(w)

        box = QGroupBox("策略与模型")
        g = QGridLayout(box)
        g.addWidget(QLabel("策略 JSON"), 0, 0)
        self.edit_policy = QLineEdit(DEFAULT_POLICY)
        g.addWidget(self.edit_policy, 0, 1)
        btn_pol = QPushButton("加载")
        btn_pol.clicked.connect(self._reload_policy)
        g.addWidget(btn_pol, 0, 2)
        g.addWidget(QLabel("模型 JSON"), 1, 0)
        self.edit_model = QLineEdit(cfg.MODEL_PATH)
        g.addWidget(self.edit_model, 1, 1)
        btn_model = QPushButton("加载")
        btn_model.clicked.connect(self._reload_solver)
        g.addWidget(btn_model, 1, 2)
        g.addWidget(QLabel("粘度(mPa·s)"), 2, 0)
        self.spin_visc = self._dspin(0.1, 10000, cfg.VISCOSITY_MPA_S, 100.0)
        g.addWidget(self.spin_visc, 2, 1)
        g.addWidget(QLabel("路点容差(mm)"), 3, 0)
        self.spin_tol = self._dspin(0.05, 5.0, 0.5, 0.05)
        g.addWidget(self.spin_tol, 3, 1)
        v.addWidget(box)

        self.lbl_policy = QLabel("-")
        self.lbl_policy.setWordWrap(True)
        v.addWidget(self.lbl_policy)
        self.lbl_model = QLabel("-")
        self.lbl_model.setWordWrap(True)
        v.addWidget(self.lbl_model)

        h = QHBoxLayout()
        self.btn_rl_start = QPushButton("开始 RL 追踪")
        self.btn_rl_start.setStyleSheet("background-color: #2e7d32; color: white;")
        self.btn_rl_start.clicked.connect(self.start_tracking)
        btn_csv = QPushButton("保存实验CSV")
        btn_csv.clicked.connect(self.save_csv)
        h.addWidget(self.btn_rl_start)
        h.addWidget(btn_csv)
        v.addLayout(h)

        note = QLabel("策略动作 = 每帧指令增量 Δcmd = action×9（斜率 ≤0.1818A/帧，\n"
                      "幅值 ±99 ↔ ±2A 结构性保证），训练环境与实机同一映射。\n"
                      "训练: py train_rl.py")
        note.setWordWrap(True)
        v.addWidget(note)
        v.addStretch(1)
        return w

    # ---------- 识别参数（与 PID 版一致） ----------
    def _tab_vision(self):
        w = QWidget()
        v = QVBoxLayout(w)
        cam_box = QGroupBox("相机")
        ch = QHBoxLayout(cam_box)
        ch.addWidget(QLabel("索引"))
        self.spin_cam = QSpinBox()
        self.spin_cam.setRange(0, 8); self.spin_cam.setValue(cfg.CAM_INDEX)
        ch.addWidget(self.spin_cam)
        btn_cam = QPushButton("打开")
        btn_cam.clicked.connect(self.open_camera)
        ch.addWidget(btn_cam)
        btn_camoff = QPushButton("关闭")
        btn_camoff.clicked.connect(self.close_camera)
        ch.addWidget(btn_camoff)
        v.addWidget(cam_box)

        vis_box = QGroupBox("识别参数")
        vg = QGridLayout(vis_box)
        vg.addWidget(QLabel("识别模式"), 0, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["GRAY", "HSV"])
        vg.addWidget(self.combo_mode, 0, 1)
        vg.addWidget(QLabel("灰度阈值"), 1, 0)
        self.sld_thresh = QSlider(Qt.Horizontal)
        self.sld_thresh.setRange(0, 255); self.sld_thresh.setValue(30)
        vg.addWidget(self.sld_thresh, 1, 1)
        self.chk_invert = QCheckBox("反相(暗磁珠)")
        self.chk_invert.setChecked(True)
        vg.addWidget(self.chk_invert, 1, 2)
        vg.addWidget(QLabel("HSV H lo/hi"), 2, 0)
        self.spin_hlo = QSpinBox(); self.spin_hlo.setRange(0, 179)
        self.spin_hhi = QSpinBox(); self.spin_hhi.setRange(0, 179); self.spin_hhi.setValue(179)
        h2 = QHBoxLayout(); h2.addWidget(self.spin_hlo); h2.addWidget(self.spin_hhi)
        vg.addLayout(h2, 2, 1)
        vg.addWidget(QLabel("S lo / V lo"), 3, 0)
        self.spin_slo = QSpinBox(); self.spin_slo.setRange(0, 255); self.spin_slo.setValue(60)
        self.spin_vlo = QSpinBox(); self.spin_vlo.setRange(0, 255); self.spin_vlo.setValue(40)
        h3 = QHBoxLayout(); h3.addWidget(self.spin_slo); h3.addWidget(self.spin_vlo)
        vg.addLayout(h3, 3, 1)
        vg.addWidget(QLabel("形态学核(0=关)"), 4, 0)
        self.spin_morph_k = QSpinBox(); self.spin_morph_k.setRange(0, 21); self.spin_morph_k.setValue(3)
        vg.addWidget(self.spin_morph_k, 4, 1)
        self.chk_morph_open = QCheckBox("开运算"); self.chk_morph_open.setChecked(True)
        vg.addWidget(self.chk_morph_open, 4, 2)
        self.chk_morph_close = QCheckBox("闭运算")
        vg.addWidget(self.chk_morph_close, 5, 2)
        vg.addWidget(QLabel("面积(px²) min/max"), 6, 0)
        self.spin_amin = QSpinBox(); self.spin_amin.setRange(1, 100000); self.spin_amin.setValue(50)
        self.spin_amax = QSpinBox(); self.spin_amax.setRange(10, 2000000); self.spin_amax.setValue(50000)
        h4 = QHBoxLayout(); h4.addWidget(self.spin_amin); h4.addWidget(self.spin_amax)
        vg.addLayout(h4, 6, 1)
        vg.addWidget(QLabel("最小圆度"), 7, 0)
        self.spin_circ = self._dspin(0.0, 1.0, 0.0, 0.05)
        vg.addWidget(self.spin_circ, 7, 1)
        vg.addWidget(QLabel("半径(px) min/max"), 8, 0)
        self.spin_rmin = QSpinBox(); self.spin_rmin.setRange(1, 500); self.spin_rmin.setValue(2)
        self.spin_rmax = QSpinBox(); self.spin_rmax.setRange(1, 1000); self.spin_rmax.setValue(200)
        h5 = QHBoxLayout(); h5.addWidget(self.spin_rmin); h5.addWidget(self.spin_rmax)
        vg.addLayout(h5, 8, 1)
        vg.addWidget(QLabel("画面宽度(mm)"), 9, 0)
        self.spin_vieww = self._dspin(1.0, 200.0, cfg.VIEW_WIDTH_MM, 0.5)
        vg.addWidget(self.spin_vieww, 9, 1)
        self.chk_show_binary = QCheckBox("显示掩膜")
        vg.addWidget(self.chk_show_binary, 10, 0, 1, 2)
        v.addWidget(vis_box)
        v.addStretch(1)
        return w

    # ---------- 路径 ----------
    def _tab_path(self):
        w = QWidget()
        v = QVBoxLayout(w)
        shape_box = QGroupBox("规则形状路径（画面中心，单位像素）")
        sg = QGridLayout(shape_box)
        sg.addWidget(QLabel("圆半径"), 0, 0)
        self.spin_circ_r = QSpinBox(); self.spin_circ_r.setRange(20, 2000); self.spin_circ_r.setValue(400)
        sg.addWidget(self.spin_circ_r, 0, 1)
        btn_circ = QPushButton("画圆")
        btn_circ.clicked.connect(lambda: self.make_circle(self.spin_circ_r.value()))
        sg.addWidget(btn_circ, 0, 2)
        sg.addWidget(QLabel("矩形宽/高"), 1, 0)
        self.spin_rw = QSpinBox(); self.spin_rw.setRange(20, 3000); self.spin_rw.setValue(700)
        self.spin_rh = QSpinBox(); self.spin_rh.setRange(20, 3000); self.spin_rh.setValue(450)
        h1 = QHBoxLayout(); h1.addWidget(self.spin_rw); h1.addWidget(self.spin_rh)
        sg.addLayout(h1, 1, 1)
        btn_rect = QPushButton("画矩形")
        btn_rect.clicked.connect(lambda: self.make_rect())
        sg.addWidget(btn_rect, 1, 2)
        sg.addWidget(QLabel("三角形边长"), 2, 0)
        self.spin_tri = QSpinBox(); self.spin_tri.setRange(20, 3000); self.spin_tri.setValue(600)
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
        btn_clr.clicked.connect(lambda: setattr(self, "path_px", []))
        dh.addWidget(btn_clr)
        v.addWidget(draw_box)
        v.addStretch(1)
        return w

    # ================= 相机/串口（复用 PID 版逻辑） =================
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
        self.video.frame_size = self.frame_size   # 鼠标映射基准同步

    def close_camera(self):
        if self.cap:
            self.cap.release()
        self.cap = None
        self.frame = None

    def refresh_ports(self):
        self.combo_port.clear()
        if list_ports:
            for p in list_ports.comports():
                self.combo_port.addItem(p.device)

    def toggle_serial(self):
        if self.ser:
            self.ser.close(); self.ser = None
            self.btn_serial.setText("连接"); self.lbl_serial.setText("未连接")
            return
        if not _serial:
            QMessageBox.warning(self, "串口", "未安装 pyserial")
            return
        port = self.combo_port.currentText()
        if not port:
            QMessageBox.warning(self, "串口", "没有可用串口")
            return
        try:
            self.ser = _serial.Serial(port, cfg.BAUDRATE, timeout=0.1)
            self.btn_serial.setText("断开"); self.lbl_serial.setText(f"已连接 {port}")
        except Exception as e:
            QMessageBox.warning(self, "串口", f"打开失败: {e}")

    def send_commands(self, cmd_list):
        """统一安全层：幅值限幅 + 斜率限幅 + 43 字节协议（与 PID 版一致）"""
        cmd = apply_slew(cmd_list, self.last_sent_cmd, cfg.MAX_DELTA_CMD)
        frame, cmd = build_command(cmd)
        if self.ser:
            try:
                self.ser.write(frame.encode("ascii"))
            except Exception as e:
                self.lbl_serial.setText(f"串口错误: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.btn_serial.setText("连接")
        self.last_sent_cmd = list(cmd)

    def normal_stop(self):
        self.tracking = False
        self.stopping = True

    def emergency_stop(self):
        self.tracking = False
        self.stopping = False
        if self.ser:
            try:
                self.ser.write(b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n")
            except Exception:
                pass
        self.last_sent_cmd = [0] * 6

    # ================= 坐标 =================
    def mm_per_px(self):
        return self.spin_vieww.value() / self.frame_size[0]

    def px_to_world_mm(self, p):
        w, h = self.frame_size
        return ((p[0] - w / 2.0) * self.mm_per_px(), (h / 2.0 - p[1]) * self.mm_per_px())

    def world_mm_to_px(self, p):
        w, h = self.frame_size
        mpp = self.mm_per_px()
        return (p[0] / mpp + w / 2.0, h / 2.0 - p[1] / mpp)

    # ================= 路径 =================
    def make_circle(self, r):
        w, h = self.frame_size
        cx, cy = w / 2.0, h / 2.0
        self.path_px = [(cx + r * math.cos(2 * math.pi * i / 240),
                         cy + r * math.sin(2 * math.pi * i / 240)) for i in range(240)]

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
        self.path_px = pts

    def make_triangle(self, side):
        w, h = self.frame_size
        cx, cy = w / 2.0, h / 2.0
        R = side / math.sqrt(3)
        verts = [(cx + R * math.cos(math.pi / 2 + 2 * math.pi * k / 3),
                  cy - R * math.sin(math.pi / 2 + 2 * math.pi * k / 3)) for k in range(3)]
        pts = []
        step = max(3, int((1.0 / self.mm_per_px()) / 10))
        for k in range(3):
            a, b = verts[k], verts[(k + 1) % 3]
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            for i in range(n):
                t = i / n
                pts.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        self.path_px = pts

    def toggle_drawing(self, on):
        self.drawing_mode = on
        self.btn_draw.setText("绘制中…(右键拖动)" if on else "开始绘制(右键拖动)")

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
        arr[0], arr[-1] = pts[0], pts[-1]
        self.path_px = [tuple(p) for p in arr[::3]]

    # ================= RL 追踪 =================
    def start_tracking(self):
        if self.policy is None:
            QMessageBox.warning(self, "RL", f"策略不可用。\n{self.policy_err}")
            return
        if not self.path_px:
            QMessageBox.information(self, "RL", "请先生成或绘制路径")
            return
        start = self.bead[:2] if self.bead \
            else (self.frame_size[0] / 2.0, self.frame_size[1] / 2.0)
        p0 = np.array(start, float)
        p1 = np.array(self.path_px[0], float)
        n = max(2, int(np.linalg.norm(p1 - p0) * self.mm_per_px()))
        lead = [tuple(p0 + (p1 - p0) * i / n) for i in range(n)]
        self.full_path = lead + list(self.path_px)
        self.target_idx = 0
        self.last_pos_mm = None
        self.vel_mm = np.zeros(2)
        self.traj_px = []
        self.traj_mm = []
        self.exp_log = []
        self.log_t0 = time.time()
        self.last_time = time.time()
        self.tracking = True
        self.stopping = False
        self.lost_since = None
        # 模式互斥（同 PID 版修复）
        self.chk_cur_live.setChecked(False)

    def save_csv(self):
        if not self.exp_log:
            QMessageBox.information(self, "保存", "实验日志为空")
            return
        fn, _ = QFileDialog.getSaveFileName(self, "保存实验CSV", "rl_experiment.csv",
                                            "CSV (*.csv)")
        if not fn:
            return
        with open(fn, "w", encoding="utf-8", newline="") as f:
            f.write(",".join(CSV_HEADER) + "\n")
            for row in self.exp_log:
                f.write(",".join(str(x) for x in row) + "\n")
        QMessageBox.information(self, "保存", f"已保存 {len(self.exp_log)} 行")

    # ================= 主循环 =================
    def tick(self):
        now = time.time()
        dt = min(0.2, max(0.005, now - self.last_time))
        self.last_time = now

        mask = None
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
        if self.frame is not None:
            vp = {
                "mode": self.combo_mode.currentText(),
                "thresh": self.sld_thresh.value(),
                "invert": self.chk_invert.isChecked(),
                "h_lo": self.spin_hlo.value(), "h_hi": self.spin_hhi.value(),
                "s_lo": self.spin_slo.value(), "v_lo": self.spin_vlo.value(),
                "morph_k": self.spin_morph_k.value(),
                "morph_open": self.chk_morph_open.isChecked(),
                "morph_close": self.chk_morph_close.isChecked(),
                "a_min": self.spin_amin.value(), "a_max": self.spin_amax.value(),
                "circ_min": self.spin_circ.value(),
                "r_min": self.spin_rmin.value(), "r_max": self.spin_rmax.value(),
            }
            cx, cy, area, mask = detect_bead(self.frame, vp)
            self.bead = None if cx is None else (cx, cy, area)

        if self.stopping:
            if any(c != 0 for c in self.last_sent_cmd):
                self.send_commands([0] * 6)
            else:
                self.stopping = False
        elif self.chk_cur_live.isChecked():
            self.send_commands(self.man_currents)
        elif self.tracking:
            self.rl_step(dt)

        self.render(mask)
        self.update_status(dt)

    def rl_step(self, dt):
        """RL 控制步：视觉状态 → 策略 → 指令增量 → 安全层发送"""
        if self.bead is None:
            if self.lost_since is None:
                self.lost_since = time.time()
            elif time.time() - self.lost_since > 1.0:
                self.normal_stop()
            return
        self.lost_since = None

        b_mm = np.array(self.px_to_world_mm(self.bead[:2]))
        tgt = self.full_path[self.target_idx]
        t_mm = np.array(self.px_to_world_mm(tgt))
        e_raw = t_mm - b_mm

        # 路点推进（与训练环境相同的 0.5mm 容差，可调）
        while np.linalg.norm(e_raw) < self.spin_tol.value():
            self.target_idx += 1
            if self.target_idx >= len(self.full_path):
                self.normal_stop()
                return
            tgt = self.full_path[self.target_idx]
            t_mm = np.array(self.px_to_world_mm(tgt))
            e_raw = t_mm - b_mm

        # 速度估计（EMA）
        if self.last_pos_mm is not None:
            v_new = (b_mm - self.last_pos_mm) / dt
            self.vel_mm = 0.6 * v_new + 0.4 * self.vel_mm
        self.last_pos_mm = b_mm

        # 状态 → 策略 → 指令增量（约束结构性保证）
        state = make_state(e_raw[0], e_raw[1], self.vel_mm[0], self.vel_mm[1],
                           self.last_sent_cmd)
        t0 = time.perf_counter()
        action = self.policy.act(state)
        self.last_rl_ms = (time.perf_counter() - t0) * 1e3
        cmd = action_to_cmd(action, self.last_sent_cmd)
        self.send_commands([int(c) for c in cmd])   # 安全部（应为 no-op）

        # 轨迹
        self.traj_px.append(tuple(self.bead[:2]))
        self.traj_mm.append(tuple(b_mm))

        # 实际力（按发送电流重算，模型可用时）
        F_act = np.zeros(3)
        if self.model_ok:
            pos_m = np.array([b_mm[0], b_mm[1], 0.0]) * 1e-3
            F_act = self.solver.force_at(pos_m, cmd * cfg.CMD_TO_A)

        # CSV 行（RL 无显式 F_des，记 0；F_act 为发送电流对应的真实力）
        I_A = [c * cfg.CMD_TO_A for c in cmd]
        row = [round(time.time() - self.log_t0, 4),
               round(b_mm[0], 4), round(b_mm[1], 4),
               round(t_mm[0], 4), round(t_mm[1], 4),
               round(e_raw[0], 4), round(e_raw[1], 4),
               round(float(np.linalg.norm(e_raw)), 4),
               round(float(self.vel_mm[0]), 4), round(float(self.vel_mm[1]), 4),
               0.0, 0.0,
               round(float(F_act[0] * 1e6), 3), round(float(F_act[1] * 1e6), 3),
               *[int(c) for c in cmd],
               *[round(a, 5) for a in I_A],
               round(float(np.abs(I_A).sum()), 5),
               "0.0", "0.0",
               1, 0,
               int(self.bead is not None),
               "rl_track",
               round(self.last_rl_ms, 3)]
        self.exp_log.append(row)

    # ================= 显示 =================
    def render(self, mask):
        if self.chk_show_binary.isChecked() and mask is not None:
            disp_src = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            disp_src = self.frame if self.frame is not None \
                else np.zeros((cfg.DISP_H, cfg.DISP_W, 3), np.uint8)
        disp = cv2.resize(disp_src, (cfg.DISP_W, cfg.DISP_H))
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
        if self.bead:
            cx, cy, area = self.bead
            r = max(3, int(math.sqrt(area) * sx / 2))
            cv2.circle(disp, (int(cx * sx), int(cy * sy)), r, (255, 0, 0), 2)
        bar_px = int(5.0 / self.mm_per_px() * sx)
        cv2.line(disp, (15, cfg.DISP_H - 20), (15 + bar_px, cfg.DISP_H - 20),
                 (255, 255, 255), 2)
        cv2.putText(disp, "5 mm", (15, cfg.DISP_H - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        img = QImage(rgb.data, cfg.DISP_W, cfg.DISP_H, 3 * cfg.DISP_W,
                     QImage.Format_RGB888)
        self.video.setPixmap(QPixmap.fromImage(img.copy()))

    def update_status(self, dt):
        if self.stopping:
            mode = "停止归零中"
        elif self.tracking:
            mode = "RL 追踪中"
        elif self.chk_cur_live.isChecked():
            mode = "手动电流"
        else:
            mode = "空闲"
        pos = f"({self.last_pos_mm[0]:+.2f},{self.last_pos_mm[1]:+.2f})mm" \
            if self.last_pos_mm is not None else "-"
        self.status_lbl.setText(
            f"模式: {mode} | 磁珠: {pos} | 路点 {self.target_idx}/{len(self.full_path)} | "
            f"电流: [{', '.join(f'{c:+03d}' for c in self.last_sent_cmd)}] | "
            f"策略 {self.last_rl_ms:.2f}ms | {1.0 / dt:.0f}fps")

    # ================= 参数持久化 =================
    def _settings_items(self):
        return {
            "port": self.combo_port, "cam_index": self.spin_cam,
            "view_width": self.spin_vieww, "show_binary": self.chk_show_binary,
            "vis_mode": self.combo_mode, "thresh": self.sld_thresh,
            "invert": self.chk_invert,
            "h_lo": self.spin_hlo, "h_hi": self.spin_hhi,
            "s_lo": self.spin_slo, "v_lo": self.spin_vlo,
            "morph_k": self.spin_morph_k,
            "morph_open": self.chk_morph_open, "morph_close": self.chk_morph_close,
            "a_min": self.spin_amin, "a_max": self.spin_amax,
            "circ_min": self.spin_circ,
            "r_min": self.spin_rmin, "r_max": self.spin_rmax,
            "circ_r": self.spin_circ_r, "rect_w": self.spin_rw,
            "rect_h": self.spin_rh, "tri_side": self.spin_tri,
            "man_currents": list(self.cur_spins),
            "model_path": self.edit_model, "policy_path": self.edit_policy,
            "viscosity": self.spin_visc, "tol": self.spin_tol,
        }

    @staticmethod
    def _widget_get(w):
        from PySide6.QtWidgets import QSpinBox as _QS, QDoubleSpinBox as _QD, QSlider as _QSL
        if isinstance(w, (_QS, _QD, _QSL)):
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
            from PySide6.QtWidgets import QSpinBox as _QS, QDoubleSpinBox as _QD, QSlider as _QSL
            if isinstance(w, (_QS, _QD, _QSL)):
                w.setValue(type(w.value())(val))
            elif isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            elif isinstance(w, QComboBox):
                idx = w.findText(str(val))
                if idx >= 0:
                    w.setCurrentIndex(idx)
            elif isinstance(w, QLineEdit):
                w.setText(str(val))
        except Exception:
            pass

    def save_settings(self):
        data = {k: ([self._widget_get(x) for x in it] if isinstance(it, list)
                    else self._widget_get(it))
                for k, it in self._settings_items().items()}
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
        except Exception as e:
            print(f"参数保存失败: {e}")

    def load_settings(self):
        if not os.path.exists(SETTINGS_FILE):
            return
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"参数载入失败: {e}")
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

    def closeEvent(self, e):
        self.save_settings()
        self.tracking = False
        self.stopping = False
        self.emergency_stop()
        self.close_camera()
        if self.ser:
            self.ser.close()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RLPathControl()
    win.show()
    sys.exit(app.exec())

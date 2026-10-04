"""
磁驱运动仿真器 GUI（PySide6 + Matplotlib）
==========================================
参数调节 → 单步/连续仿真 → 轨迹跟踪闭环 → 参数扫描 → 实验数据拟合 → CSV 导出
磁场/磁力/逆解全部来自现有 180 磁偶极子模型（DipoleSolver），无第二套磁场模型。
运行：py bead_sim_gui.py
"""

import csv
import math
import os
import sys

import matplotlib
import numpy as np

matplotlib.use("QtAgg")
import config as cfg
from bead_sim import (
    BeadSimulator,
    ClosedLoopRunner,
    ExperimentFitter,
    FluidModel,
    FrictionModel,
    MagnetModel,
    ParameterSweep,
    PositionController,
    TrajectoryModel,
    save_history_csv,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

CTRL_HZ = 30.0
SIM_SUBSTEPS = 10  # 每控制周期 10 个仿真子步
DT_SIM = 1.0 / (CTRL_HZ * SIM_SUBSTEPS)


class BeadSimGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("1mm N38 自由对齐磁珠磁驱运动仿真器")
        self.resize(1500, 860)

        # 仿真对象（参数变化时重建）
        self.sim = None
        self.ctrl = None
        self.runner = None
        self.traj = None
        self.running = False

        self._build_ui()
        self.apply_and_reset()  # 初始构建

        self.timer = QTimer()
        self.timer.timeout.connect(self.on_tick)

    # ================= UI =================
    def _build_ui(self):
        left = self._panel_params()
        center = self._panel_canvas()
        right = self._panel_telemetry()

        root = QHBoxLayout()
        root.addWidget(left, stretch=0)
        root.addWidget(center, stretch=5)
        root.addWidget(right, stretch=0)

        outer = QWidget()
        v = QVBoxLayout(outer)
        v.addLayout(self._control_bar())
        v.addLayout(root, stretch=1)
        self.setCentralWidget(outer)

    def _control_bar(self):
        bar = QHBoxLayout()
        self.btn_run = QPushButton("连续运行")
        self.btn_run.setCheckable(True)
        self.btn_run.toggled.connect(self.toggle_run)
        self.btn_run.setStyleSheet("background-color: #2e7d32; color: white;")
        bar.addWidget(self.btn_run)
        self.btn_step = QPushButton("单步(1 控制周期)")
        self.btn_step.clicked.connect(self.on_single_step)
        bar.addWidget(self.btn_step)
        btn_reset = QPushButton("复位")
        btn_reset.clicked.connect(self.apply_and_reset)
        bar.addWidget(btn_reset)
        btn_csv = QPushButton("导出 CSV")
        btn_csv.clicked.connect(self.export_csv)
        bar.addWidget(btn_csv)
        btn_sweep = QPushButton("参数扫描")
        btn_sweep.clicked.connect(self.run_sweep)
        bar.addWidget(btn_sweep)
        btn_fit = QPushButton("实验拟合")
        btn_fit.clicked.connect(self.run_fit)
        bar.addWidget(btn_fit)
        bar.addStretch(1)
        return bar

    def _panel_params(self):
        w = QWidget()
        v = QVBoxLayout(w)
        tabs = QTabWidget()
        tabs.addTab(self._tab_bead(), "磁珠/环境")
        tabs.addTab(self._tab_mode(), "模式")
        tabs.addTab(self._tab_ctrl(), "控制器")
        tabs.setFixedWidth(360)
        v.addWidget(tabs)

        note = QLabel(
            "磁场/磁力/逆解全部来自 180 磁偶极子标定模型\n"
            "（DipoleSolver，解析梯度，无第二套磁场模型）。\n"
            "内部 SI (m/N/T/A)，显示 mm/µN/mT。"
        )
        note.setWordWrap(True)
        v.addWidget(note)
        return w

    def _dspin(self, lo, hi, val, step, dec=2):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(dec)
        sp.setSingleStep(step)
        sp.setValue(val)
        return sp

    def _tab_bead(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("磁珠 / 液体 / 摩擦（改变后点“复位”生效）")
        g = QGridLayout(box)

        def add(row, name, lo, hi, val, step=0.05, dec=2, suffix=""):
            g.addWidget(QLabel(f"{name} {suffix}"), row, 0)
            sp = self._dspin(lo, hi, val, step, dec)
            g.addWidget(sp, row, 1)
            return sp

        self.spin_d = add(0, "直径", 0.1, 5.0, 1.0, 0.05, 2, "(mm)")
        self.spin_br = add(1, "剩磁 Br", 0.5, 1.4, 1.20, 0.01, 2, "(T)")
        self.spin_rho = add(2, "密度", 1000, 10000, 7500, 100, 0, "(kg/m³)")
        self.spin_eta = add(3, "黏度", 1.0, 10000.0, 1000.0, 50.0, 1, "(mPa·s)")
        self.sld_mu = QSlider(Qt.Horizontal)
        self.sld_mu.setRange(0, 100)
        self.sld_mu.setValue(10)
        self.sld_mu.valueChanged.connect(
            lambda v: self.lbl_mu_val.setText(f"{v / 100:.2f}")
        )
        g.addWidget(QLabel("摩擦系数 μ"), 4, 0)
        g.addWidget(self.sld_mu, 4, 1)
        self.lbl_mu_val = QLabel("0.10")
        g.addWidget(self.lbl_mu_val, 4, 2)
        v.addWidget(box)

        th_box = QGroupBox("理论性能（实时）")
        tv = QVBoxLayout(th_box)
        self.lbl_theory = QLabel("-")
        self.lbl_theory.setStyleSheet("font-family: Consolas;")
        tv.addWidget(self.lbl_theory)
        v.addWidget(th_box)
        v.addStretch(1)
        return w

    def _tab_mode(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("模式")
        mv = QVBoxLayout(box)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Mode A：给定电流（开环）", "Mode B：轨迹跟踪闭环"])
        mv.addWidget(self.combo_mode)

        # Mode A 电流滑条
        cur_box = QGroupBox("六路电流指令（±99 ↔ ±2A）")
        cg = QGridLayout(cur_box)
        self.cur_sliders = []
        for j in range(6):
            cg.addWidget(QLabel(f"a{j} ({cfg.COIL_ORDER[j]})"), j, 0)
            s = QSlider(Qt.Horizontal)
            s.setRange(-cfg.CMD_MAX, cfg.CMD_MAX)
            lab = QLabel("0")
            s.valueChanged.connect(lambda val, lb=lab: lb.setText(f"{val:+d}"))
            cg.addWidget(s, j, 1)
            cg.addWidget(lab, j, 2)
            self.cur_sliders.append(s)
        mv.addWidget(cur_box)

        # Mode B 轨迹
        trj_box = QGroupBox("目标轨迹（世界系 mm，工作区中心）")
        tg = QGridLayout(trj_box)
        self.combo_traj = QComboBox()
        self.combo_traj.addItems(
            ["圆 (r=3mm)", "矩形 (6×4mm)", "三角形 (边5mm)", "直线 (+3mm)"]
        )
        tg.addWidget(self.combo_traj, 0, 0)
        tg.addWidget(QLabel("轨迹速度 (mm/s)"), 1, 0)
        self.spin_speed = self._dspin(0.1, 10.0, 1.0, 0.1)
        tg.addWidget(self.spin_speed, 1, 1)
        tg.addWidget(QLabel("视觉噪声 σ (mm)"), 2, 0)
        self.spin_noise = self._dspin(0.0, 0.5, 0.0, 0.01, 2)
        tg.addWidget(self.spin_noise, 2, 1)
        mv.addWidget(trj_box)
        v.addWidget(box)
        v.addStretch(1)
        return w

    def _tab_ctrl(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("位置控制器（X/Y 独立 PID + 速度前馈，输出 µN）")
        pg = QGridLayout(box)
        pg.addWidget(QLabel("X:"), 0, 0)
        pg.addWidget(QLabel("Y:"), 1, 0)
        self.pid_spins = {}
        for col, stem in enumerate(["Kp", "Ki", "Kd"]):
            pg.addWidget(QLabel(stem), 0, col * 2 + 1)
            for r, ax in enumerate(["x", "y"]):
                sp = self._dspin(
                    0, 500, cfg.PID[f"{stem}{ax}"], 1.0 if stem == "Kp" else 0.5
                )
                self.pid_spins[f"{stem}{ax}"] = sp
                pg.addWidget(sp, r, col * 2 + 2)
        pg.addWidget(QLabel("力限幅(µN)"), 2, 0)
        self.spin_flim = self._dspin(1, 600, cfg.PID_FMAX_UN, 10.0, 1)
        pg.addWidget(self.spin_flim, 2, 1)
        v.addWidget(box)
        v.addStretch(1)
        return w

    def _panel_canvas(self):
        self.fig = Figure(figsize=(7.2, 6.4))
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111)
        return self.canvas

    def _panel_telemetry(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox("实时遥测")
        tv = QVBoxLayout(box)
        self.lbl_telemetry = QLabel("-")
        self.lbl_telemetry.setStyleSheet("font-family: Consolas; font-size: 12px;")
        tv.addWidget(self.lbl_telemetry)
        v.addWidget(box)

        vec_box = QGroupBox("矢量（磁珠局部）")
        vv = QVBoxLayout(vec_box)
        self.lbl_vectors = QLabel("-")
        self.lbl_vectors.setStyleSheet("font-family: Consolas; font-size: 12px;")
        vv.addWidget(self.lbl_vectors)
        v.addWidget(vec_box)

        # 梯度张量
        g_box = QGroupBox("梯度张量 G (T/m)")
        gv = QVBoxLayout(g_box)
        self.lbl_grad = QLabel("-")
        self.lbl_grad.setStyleSheet("font-family: Consolas; font-size: 12px;")
        gv.addWidget(self.lbl_grad)
        v.addWidget(g_box)

        # 对齐角
        self.lbl_align = QLabel("Alignment angle = 0.00° (自由对齐)")
        self.lbl_align.setStyleSheet("font-family: Consolas;")
        v.addWidget(self.lbl_align)
        v.addStretch(1)
        return w

    # ================= 仿真生命周期 =================
    def _collect(self):
        mag = MagnetModel(
            self.spin_d.value(), self.spin_br.value(), self.spin_rho.value()
        )
        fluid = FluidModel(self.spin_eta.value())
        mu = self.sld_mu.value() / 100.0
        fric = FrictionModel(mu)
        return mag, fluid, fric

    def apply_and_reset(self):
        mag, fluid, fric = self._collect()
        self.sim = BeadSimulator(mag, fluid, fric)
        self.sim.reset((0.0, 0.0))
        self.ctrl = PositionController(
            self.sim.c_v,
            (self.pid_spins["Kpx"].value(), self.pid_spins["Kpy"].value()),
            (self.pid_spins["Kix"].value(), self.pid_spins["Kiy"].value()),
            (self.pid_spins["Kdx"].value(), self.pid_spins["Kdy"].value()),
            self.spin_flim.value() * 1e-6,
        )
        self._make_traj()
        self.runner = ClosedLoopRunner(
            self.sim,
            self.traj,
            self.ctrl,
            self.spin_speed.value(),
            self.spin_noise.value(),
        )
        self.running = False
        self.btn_run.setChecked(False)
        self._update_theory()
        self.redraw()

    def _make_traj(self):
        name = self.combo_traj.currentText()
        if name.startswith("圆"):
            self.traj = TrajectoryModel.circle(r_mm=3.0)
        elif name.startswith("矩形"):
            self.traj = TrajectoryModel.rectangle()
        elif name.startswith("三角"):
            self.traj = TrajectoryModel.triangle()
        else:
            self.traj = TrajectoryModel.from_points([(0.0, 0.0), (3.0, 0.0)])

    def toggle_run(self, on):
        self.running = on
        self.btn_run.setText("暂停" if on else "连续运行")
        if on:
            self.timer.start(cfg.TICK_MS)
        else:
            self.timer.stop()

    def on_single_step(self):
        self.do_cycle()

    def on_tick(self):
        self.do_cycle()

    def do_cycle(self):
        """一个控制周期：30Hz 控制更新 + 10 个仿真子步"""
        if self.combo_mode.currentText().startswith("Mode B"):
            rec, p_ref, _ = self.runner.step_ctrl(1.0 / CTRL_HZ)
            self.last_rec = rec
            self.last_ref = p_ref
        else:
            self.last_rec = None
            self.last_ref = None
        I_A = self._current_I(
            rec if self.combo_mode.currentText().startswith("Mode B") else None
        )
        for _ in range(SIM_SUBSTEPS):
            row = self.sim.step(I_A, DT_SIM)
        self.last_row = row
        self.redraw()
        self.update_telemetry(row)

    def _current_I(self, rec_b):
        if rec_b is not None:
            return rec_b["currents"]
        return [sl.value() * cfg.CMD_TO_A for sl in self.cur_sliders]

    def _update_theory(self):
        th = self.sim.theory(np.array([30e-6, 0, 0]))
        self.lbl_theory.setText(
            f"c_v = {th['c_v_uN_per_mm_s']:.3f} µN/(mm/s)\n"
            f"F_start = μmg = {th['F_start_uN']:.2f} µN\n"
            f"v_ss@30µN = {th['v_ss_mm_s']:.3f} mm/s"
        )

    def update_telemetry(self, row):
        d = row
        B_txt = (
            f"B  = ({d['Bx_mT']:+.3f}, {d['By_mT']:+.3f}, "
            f"{d['Bz_mT']:+.3f}) mT\n|B| = {d['Bmag_mT']:.3f} mT"
        )
        F_txt = (
            f"F_m = ({d['Fx_uN']:+.2f}, {d['Fy_uN']:+.2f}, "
            f"{d['Fz_uN']:+.2f}) µN\n|F_m| = {d['Fmag_uN']:.2f} µN\n"
            f"F_drag = ({d['Fdrag_x_uN']:+.2f}, {d['Fdrag_y_uN']:+.2f}) µN "
            f"|F_drag| = {math.hypot(d['Fdrag_x_uN'], d['Fdrag_y_uN']):.2f}\n"
            f"F_fric = ({d['Ffric_x_uN']:+.2f}, {d['Ffric_y_uN']:+.2f}) µN "
            f"|F_fric| = {math.hypot(d['Ffric_x_uN'], d['Ffric_y_uN']):.2f}"
        )
        m_txt = (
            f"pos = ({d['x']:+.3f}, {d['y']:+.3f}) mm\n"
            f"vel = ({d['vx']:+.4f}, {d['vy']:+.4f}) mm/s\n"
            f"speed = {d['speed']:.4f} mm/s\n"
            f"state = {d['state']}\n"
            f"t = {d['t']:.2f} s"
        )
        self.lbl_telemetry.setText(f"{B_txt}\n\n{F_txt}\n\n{m_txt}")
        G = d["G"]
        self.lbl_grad.setText(
            f"Gxx {G[0,0]:+.4f}  Gxy {G[0,1]:+.4f}  Gxz {G[0,2]:+.4f}\n"
            f"Gyx {G[1,0]:+.4f}  Gyy {G[1,1]:+.4f}  Gyz {G[1,2]:+.4f}\n"
            f"Gzx {G[2,0]:+.4f}  Gzy {G[2,1]:+.4f}  Gzz {G[2,2]:+.4f}"
        )
        # 磁矩方向 = B̂（自由对齐），对齐角恒 0
        if d["Bmag_mT"] > 1e-9:
            b_hat = np.array([d["Bx_mT"], d["By_mT"], d["Bz_mT"]]) / d["Bmag_mT"]
            self.lbl_align.setText(
                f"Alignment angle = 0.00° (自由对齐; B̂ = m̂ = "
                f"{np.round(b_hat, 3)})"
            )
        else:
            self.lbl_align.setText("Alignment angle: undefined (B ≈ 0)")

    # ================= 绘制 =================
    def redraw(self):
        ax = self.ax
        ax.clear()
        half = 12.0
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        # 工作区边界与线圈标记
        ax.add_patch(
            matplotlib.patches.Rectangle(
                (-half, -half), 2 * half, 2 * half, fill=False, ec="gray"
            )
        )
        for name, ang in zip(cfg.COIL_ORDER, [0, 90, 90, 180, 270, 270]):
            pass
        coil_marks = [
            ("+X", -half * 0.92, 0),
            ("+Y", 0, half * 0.92),
            ("+Z", 0, half * 0.55),
            ("-X", half * 0.92, 0),
            ("-Y", 0, -half * 0.92),
            ("-Z", 0, -half * 0.55),
        ]
        for name, mx, my in coil_marks:
            ax.annotate(
                name, (mx, my), color="#888888", ha="center", va="center", fontsize=9
            )
        # 目标轨迹
        if self.combo_mode.currentText().startswith("Mode B") and self.traj:
            pts = self.traj.pts
            ax.plot(pts[:, 0], pts[:, 1], "g--", lw=1, alpha=0.8, label="目标轨迹")
        # 实际轨迹
        h = self.sim.history
        if len(h) > 1:
            xs = [r["x"] for r in h]
            ys = [r["y"] for r in h]
            ax.plot(xs, ys, "r-", lw=1, alpha=0.85, label="实际轨迹")
        # 磁珠 + 矢量
        row = self.sim.history[-1] if h else None
        if row:
            bx, by = row["x"], row["y"]
            r_bead = self.spin_d.value() * 0.5
            ax.add_patch(
                matplotlib.patches.Circle(
                    (bx, by), r_bead, fc="#ffd54f", ec="#555555", zorder=5
                )
            )
            # 磁矩/磁场方向箭头（自由对齐下重合）
            if row["Bmag_mT"] > 1e-9:
                u = np.array([row["Bx_mT"], row["By_mT"]])
                u = u / max(np.linalg.norm(u), 1e-12)
                ax.arrow(
                    bx,
                    by,
                    1.6 * u[0],
                    1.6 * u[1],
                    width=0.04,
                    head_width=0.35,
                    fc="#1565c0",
                    ec="#1565c0",
                    zorder=6,
                    label="m̂ = B̂",
                )
            # 磁力矢量（尺度 /10µN）
            F = np.array([row["Fx_uN"], row["Fy_uN"]])
            fn = np.linalg.norm(F)
            if fn > 0.05:
                us = F / fn * min(fn / 10.0, 3.0)
                ax.arrow(
                    bx,
                    by,
                    us[0],
                    us[1],
                    width=0.04,
                    head_width=0.35,
                    fc="#c62828",
                    ec="#c62828",
                    zorder=6,
                    label="F",
                )
            # 参考点
            if getattr(self, "last_ref", None) is not None:
                ax.plot(*self.last_ref, "g+", ms=12, mew=2, zorder=7)
        ax.legend(loc="upper right", fontsize=8)
        self.canvas.draw_idle()

    # ================= 导出 / 扫描 / 拟合 =================
    def export_csv(self):
        if not self.sim.history:
            QMessageBox.information(self, "导出", "无历史数据（请先运行）")
            return
        fn, _ = QFileDialog.getSaveFileName(
            self, "导出仿真 CSV", "bead_sim.csv", "CSV (*.csv)"
        )
        if not fn:
            return
        save_history_csv(self.sim.history, fn)
        QMessageBox.information(self, "导出", f"已导出 {len(self.sim.history)} 行")

    def run_sweep(self):
        try:
            QApplication.setOverrideCursor(Qt.WaitCursor)
            rows = ParameterSweep.run_ofat()
            base, _ = os.path.splitext("bead_sweep")
            csv_path = base + ".csv"
            png_path = base + ".png"
            ParameterSweep.save_csv(rows, csv_path)
            ParameterSweep.plot_results(rows, png_path)
        finally:
            QApplication.restoreOverrideCursor()
        QMessageBox.information(
            self, "参数扫描", f"完成 {len(rows)} 组。\n{csv_path}\n{png_path}"
        )

    def run_fit(self):
        fn, _ = QFileDialog.getOpenFileName(
            self, "选择实验 CSV（t,x,y[,I0..I5]）", "", "CSV (*.csv);;All (*)"
        )
        if not fn:
            return
        try:
            t_exp, xy_exp, I_of_t = ExperimentFitter.load_experiment_csv(fn)
        except (OSError, ValueError, csv.Error, IndexError, StopIteration) as e:
            QMessageBox.warning(self, "拟合", f"读取失败: {e}")
            return
        if I_of_t is None:
            QMessageBox.warning(
                self,
                "拟合",
                "CSV 缺少 I0..I5 电流列，" "无法重建驱动（当前要求包含电流）",
            )
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            fitter = ExperimentFitter(
                self.spin_d.value(), self.spin_br.value(), self.spin_rho.value()
            )
            res = fitter.fit(t_exp, xy_exp, I_of_t)
            png = fn.rsplit(".", 1)[0] + "_fit.png"
            ExperimentFitter.save_comparison_plot(
                t_exp, xy_exp, res["xs_fit"], res["ys_fit"], png
            )
        finally:
            QApplication.restoreOverrideCursor()
        QMessageBox.information(
            self,
            "拟合结果",
            f"η_eff = {res['eta_eff_mPas']:.1f} mPa·s\n"
            f"μ_eff = {res['mu_eff']:.3f}\n"
            f"轨迹 RMS 误差 = {res['rms_mm']:.3f} mm\n对比图: {png}",
        )

    def closeEvent(self, e):
        self.timer.stop()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    win = BeadSimGUI()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

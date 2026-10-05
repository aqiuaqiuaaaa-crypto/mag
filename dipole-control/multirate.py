"""
多速率调度与电流执行层
======================
频率结构（PWM 在 STM32/H 桥固件侧，Python 仅记录 PWM_HZ 常量）：
    VISION/KALMAN 30 Hz（主线程 QTimer）
    MPC + MDM 逆解 10 Hz（本模块 ControlWorker 后台线程，绝不阻塞视觉）
    电流执行 30 Hz（主线程：目标插值 → 斜率限幅 → 量化 → 串口；RL 线圈估计）
    PWM 20 kHz（固件）

线程安全：所有跨线程数据经 SharedState（threading.Lock 保护的时间戳快照），
主线程 30Hz 写入 Kalman 状态/参考/参数并读取 I_target；工作线程 10Hz 读取状态、
计算 MPC→MDM、写回 I_target。退出通过 stop 事件安全 join。

电流执行层（每 30Hz 帧）：
    1. 取最新 I_target（10Hz 写入，带序号）
    2. 100ms 窗口内线性插值 alpha = 1/3, 2/3, 1（不在 100ms 边界突跳）
    3. 斜率限幅 |Δcmd| ≤ 9（叠加在插值结果上）
    4. 整数量化 → 43 字节协议发送
    5. RL 线圈模型更新 I_est：L dI/dt = V − R·I，V = (cmd/99)·V_supply
       （稳态 I_est → cmd×2/99，与指令-电流映射自洽；I_est 是估计值非测量值）
    6. 用 I_est 走 MDM 正向模型得 B/G/∇|B|/F_est（而非 I_target——不忽略线圈动态）
    7. 磁矩对齐诊断 τ_r = ζ_r/(m_b|B|)、ratio = T_current/τ_r、低场告警
"""

import copy
import math
import threading
import time
from collections.abc import Sequence
from typing import TYPE_CHECKING, Any, cast

import config as cfg
import numpy as np
from control_log import ControlLogger, Scalar
from frames import cam_to_model_pos, cam_to_model_vec, model_to_cam_vec
from numpy.typing import NDArray

if TYPE_CHECKING:
    from mpc import ForceMPC


class SharedState:
    """30Hz 主线程 ↔ 10Hz 工作线程 的线程安全共享快照"""

    def __init__(self):
        self._lock = threading.Lock()
        # 主线程 → 工作线程
        self._kalman = {"pos": np.zeros(2), "vel": np.zeros(2), "t": 0.0}
        self._ref = {"path": [], "speed": 1.0, "t": 0.0}  # 世界系 mm 路径点
        self._params = {
            "fmax": cfg.MPC_FMAX_UN,
            "max_active": 6,
            "mpc_on": False,
            "mpc_horizon": cfg.MPC_HORIZON,
            "mpc_w_pos": cfg.MPC_W_POS,
            "mpc_w_vel": cfg.MPC_W_VEL,
            "mpc_w_u": cfg.MPC_W_U,
            "mpc_w_du": cfg.MPC_W_DU,
            "fz_lift": 0.0,
            "eso_d": np.zeros(2),
            "t": 0.0,
        }
        self._last_sent = [0] * 6
        # 工作线程 → 主线程
        # currents 始终使用安培；串口整数指令只存在 rec["commands"] 中。
        self._I_target = {
            "currents": np.zeros(6),
            "seq": 0,
            "t": 0.0,
            "rec": None,
            "mpc_time_ms": 0.0,
            "solver_time_ms": 0.0,
            "mpc_cost": 0.0,
            "F_target": np.zeros(2),
            "ref_target": np.zeros(2),
            "ref_velocity": np.zeros(2),
        }
        self._mpc_log = []  # 10Hz 事件日志
        self._solver_error = None
        self._stop = threading.Event()
        # Optional observation only; never consumed by a control calculation.
        self.control_logger: ControlLogger | None = None
        self.control_log_run = 0

    def attach_control_log(self, sink: ControlLogger | None, run: int) -> None:
        with self._lock:
            self.control_logger = sink
            self.control_log_run = run
            if sink is not None:
                sink.remember_target(run, self._I_target["seq"], self._I_target)

    # ---- 主线程 → 工作线程 ----
    def set_kalman(self, pos, vel, t):
        with self._lock:
            self._kalman = {
                "pos": np.array(pos, float).copy(),
                "vel": np.array(vel, float).copy(),
                "t": t,
            }

    def set_reference(self, path_world_mm, speed, t):
        with self._lock:
            self._ref = {
                "path": [np.array(p, float) for p in path_world_mm],
                "speed": float(speed),
                "t": t,
            }

    def set_params(self, params: dict, t):
        with self._lock:
            self._params = dict(params)
            self._params["t"] = t

    def set_last_sent(self, cmd_list):
        with self._lock:
            self._last_sent = list(cmd_list)

    def get_kalman(self):
        with self._lock:
            k = self._kalman
            return k["pos"].copy(), k["vel"].copy(), k["t"]

    def get_reference(self):
        with self._lock:
            return self._ref["path"], self._ref["speed"]

    def get_params(self):
        with self._lock:
            p = dict(self._params)
            p["eso_d"] = np.array(self._params.get("eso_d", np.zeros(2)), float)
            p["force_model_to_camera"] = np.array(
                self._params.get("force_model_to_camera", np.eye(2)), float
            ).copy()
            p["frame_offset_mm"] = np.array(
                self._params.get("frame_offset_mm", cfg.FRAME_OFFSET_MM), float
            ).copy()
            p["field_direction"] = np.array(
                self._params.get("field_direction", cfg.CONTROL_FIELD_DIRECTION), float
            ).copy()
            return p

    def get_last_sent(self):
        with self._lock:
            return list(self._last_sent)

    # ---- 工作线程 → 主线程 ----
    def set_I_target(
        self,
        currents,
        rec,
        f_target,
        mpc_ms,
        solver_ms,
        cost,
        ref_target=None,
        ref_velocity=None,
    ):
        with self._lock:
            seq = self._I_target["seq"] + 1
            self._I_target = {
                "currents": np.asarray(currents, float).copy(),
                "seq": seq,
                "t": time.time(),
                "rec": rec,
                "mpc_time_ms": mpc_ms,
                "solver_time_ms": solver_ms,
                "mpc_cost": cost,
                "F_target": np.asarray(f_target, float).copy(),
                "ref_target": np.asarray(
                    np.zeros(2) if ref_target is None else ref_target, float
                ).copy(),
                "ref_velocity": np.asarray(
                    np.zeros(2) if ref_velocity is None else ref_velocity, float
                ).copy(),
            }
            if self.control_logger is not None:
                self.control_logger.remember_target(
                    self.control_log_run, seq, self._I_target
                )

    def get_I_target(self):
        with self._lock:
            d = self._I_target
            return d["currents"].copy(), d["seq"], d["t"], d

    def get_solver_error(self):
        with self._lock:
            return self._solver_error

    def set_solver_error(self, err):
        with self._lock:
            self._solver_error = err

    def append_mpc_log(self, row):
        with self._lock:
            self._mpc_log.append(tuple(row))
            if len(self._mpc_log) > 5000:
                del self._mpc_log[:1000]

    def drain_mpc_log(self):
        with self._lock:
            rows = self._mpc_log
            self._mpc_log = []
            return rows

    # ---- 生命周期 ----
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.is_set()


class ControlWorker(threading.Thread):
    """10Hz 工作线程：读 Kalman 快照 → MPC → MDM 逆解 → 写 I_target。
    deadline 调度（perf_counter 基准 + 失步重同步）；异常不外泄，转安全停止标志。
    MDM 偶发 15~30ms 不会影响 30Hz 主循环（视觉/Kalman/电流执行在主线程）。"""

    def __init__(self, shared: SharedState, solver, period=None):
        super().__init__(daemon=True, name="mpc-mdm-worker")
        self.shared = shared
        # DipoleSolver 有位置相关的 Bc/Gc 可变缓存。GUI 主线程还会同时做正向
        # 诊断，因此工作线程必须持有独立副本，避免两个线程交叉覆盖同一缓存。
        self.solver = copy.deepcopy(solver)
        self.period = period or (1.0 / cfg.MPC_HZ)
        self.mpc_x = None
        self.mpc_y = None
        self._ref_arc = 0.0  # 当前实际位置在路径上的投影弧长（诊断）
        self._c_drag = 9.42
        self._mpc_signature = None

    def _refresh_mpc(self, params, c_drag):
        from mpc import ForceMPC

        signature = self._controller_signature(params, c_drag)
        horizon, wp, wv, wu, wd, fmax, _ = signature
        self.mpc_x = ForceMPC(self.period, horizon, c_drag, wp, wv, wu, fmax, wd)
        self.mpc_y = ForceMPC(self.period, horizon, c_drag, wp, wv, wu, fmax, wd)
        self._c_drag = c_drag
        self._mpc_signature = signature

    @staticmethod
    def _controller_signature(params, c_drag):
        """GUI 参数快照；任一项变化都会在下一次 10Hz 周期重建 MPC。"""
        return (
            int(np.clip(params.get("mpc_horizon", cfg.MPC_HORIZON), 1, 6)),
            max(float(params.get("mpc_w_pos", cfg.MPC_W_POS)), 0.0),
            max(float(params.get("mpc_w_vel", cfg.MPC_W_VEL)), 0.0),
            max(float(params.get("mpc_w_u", cfg.MPC_W_U)), 0.0),
            max(float(params.get("mpc_w_du", cfg.MPC_W_DU)), 0.0),
            max(float(params.get("fmax", cfg.MPC_FMAX_UN)), 1.0),
            float(c_drag),
        )

    def _reference_window(self, path, speed, pos, horizon=None):
        """由实际位置在路径上的最近投影生成 MPC 参考，而非按时间开环推进。"""
        horizon = int(
            horizon or (self.mpc_x.N if self.mpc_x is not None else cfg.MPC_HORIZON)
        )
        if len(path) < 2:
            z = [np.zeros(2) for _ in range(horizon)]
            return z, [p.copy() for p in z]
        path_arr = np.vstack(path).astype(float)
        seg_vec = np.diff(path_arr, axis=0)
        seg = np.linalg.norm(seg_vec, axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = s[-1]
        if total <= 1e-12:
            p = path_arr[-1].copy()
            return [p.copy() for _ in range(horizon)], [
                np.zeros(2) for _ in range(horizon)
            ]

        # 当前磁珠到各路径线段的最近投影，并换算为路径弧长 s_near。
        pos = np.asarray(pos, float)
        best_d2, s_near = float("inf"), 0.0
        for i, vec in enumerate(seg_vec):
            length2 = float(vec @ vec)
            if length2 <= 1e-12:
                continue
            t = float(np.clip(((pos - path_arr[i]) @ vec) / length2, 0.0, 1.0))
            projection = path_arr[i] + t * vec
            d2 = float(np.sum((pos - projection) ** 2))
            if d2 < best_d2:
                best_d2 = d2
                s_near = float(s[i] + t * seg[i])
        self._ref_arc = s_near
        s0 = min(s_near + max(float(speed), 0.0) * self.period, total)
        pts, velocities = [], []
        for k in range(horizon):
            sk = min(s0 + max(float(speed), 0.0) * self.period * k, total)
            i = int(np.searchsorted(s, sk, side="right") - 1)
            i = min(max(i, 0), len(seg) - 1)
            t = (sk - s[i]) / max(seg[i], 1e-9)
            pts.append(path_arr[i] + t * seg_vec[i])
            if sk >= total - 1e-9 or seg[i] <= 1e-9:
                velocities.append(np.zeros(2))
            else:
                velocities.append(float(speed) * seg_vec[i] / seg[i])
        return pts, velocities

    def run(self):
        next_t = time.perf_counter()
        _c_drag = None
        while not self.shared.stopped():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, 0.02))
                continue
            next_t += self.period
            if now - next_t > self.period:  # 失步重同步（不追帧）
                next_t = now + self.period
            try:
                self._step()
            except Exception as e:  # noqa
                self.shared.set_solver_error(f"{type(e).__name__}: {e}")
                time.sleep(self.period)

    def _step(self):
        params = self.shared.get_params()
        if not params.get("mpc_on", False):
            time.sleep(self.period)  # 非 MPC 模式挂起
            return
        c_drag = self._drag_c(params)
        signature = self._controller_signature(params, c_drag)
        if self.mpc_x is None or signature != self._mpc_signature:
            self._refresh_mpc(params, c_drag)
        fmax = self.mpc_x.fmax

        pos, _vel, kalman_t = self.shared.get_kalman()
        path, speed = self.shared.get_reference()
        eso_d = params.get("eso_d", np.zeros(2))
        R = np.asarray(params.get("force_model_to_camera", np.eye(2)), float)
        last_sent = self.shared.get_last_sent()
        pos_m = cam_to_model_pos(pos, R, params["frame_offset_mm"])
        # ΔF 项从实际发送整数指令对应的磁力开始，而不是从上一次理想 MPC
        # 目标开始；这样 10Hz 预测与 30Hz 插值/斜率执行层保持一致。
        F_prev_model = (
            np.atleast_1d(
                self.solver.force_at(
                    pos_m, np.asarray(last_sent, float) * self.solver.current_gain
                )
            )
            * 1e6
        )
        F_prev_camera = model_to_cam_vec(F_prev_model, R)

        # MPC（输出水平力目标）
        t0 = time.perf_counter()
        ref, vref = self._reference_window(path, speed, pos, self.mpc_x.N)
        rx = [p[0] for p in ref]
        ry = [p[1] for p in ref]
        vrx = [v[0] for v in vref]
        vry = [v[1] for v in vref]
        out_x = self.mpc_x.compute(pos[0], eso_d[0], rx, vrx, f_prev=F_prev_camera[0])
        out_y = self.mpc_y.compute(pos[1], eso_d[1], ry, vry, f_prev=F_prev_camera[1])
        # 始终三维力（Fz 来自减摩控制器，可为 0）
        F_target = np.array([out_x["F0"], out_y["F0"], params.get("fz_lift", 0.0)])
        # 两轴 MPC 分别有箱约束，再施加 GUI“最大水平磁力”的圆形总幅值约束。
        fxy_norm = float(np.linalg.norm(F_target[:2]))
        if fxy_norm > fmax:
            F_target[:2] *= fmax / fxy_norm
        F_target_model = cam_to_model_vec(F_target, R)
        B_direction_camera = np.asarray(
            params.get("field_direction", cfg.CONTROL_FIELD_DIRECTION), float
        )
        B_direction_model = cam_to_model_vec(B_direction_camera, R)
        mpc_ms = (time.perf_counter() - t0) * 1e3

        # MDM 逆解（约束在解算器内部；cmd_prev 用 30Hz 层实际发送指令）
        t1 = time.perf_counter()
        rec = self.solver.solve_field_force_pseudoinverse(
            pos_m,
            F_target_model * 1e-6,
            B_direction=B_direction_model,
            B_magnitude_mT=float(
                params.get("field_magnitude_mT", cfg.CONTROL_FIELD_TARGET_MT)
            ),
            cmd_prev=last_sent,
            max_cmd=int(params.get("max_cmd", cfg.CMD_MAX)),
        )
        solver_ms = (time.perf_counter() - t1) * 1e3

        # 求解器记录为模型坐标；外部控制和日志统一使用相机世界坐标。
        for key in (
            "achieved_force",
            "achieved_force_nonlinear",
            "achieved_force_linear",
        ):
            if key not in rec:
                continue
            f_model = np.asarray(rec[key], float).copy()
            rec[key + "_model"] = f_model.copy()
            rec[key] = model_to_cam_vec(f_model, R)
        if "B" in rec:
            b_model = np.asarray(rec["B"], float).copy()
            rec["B_model"] = b_model.copy()
            rec["B"] = model_to_cam_vec(b_model, R)
        if rec.get("requested_B_direction") is not None:
            bd_model = np.asarray(rec["requested_B_direction"], float).copy()
            rec["requested_B_direction_model"] = bd_model.copy()
            rec["requested_B_direction"] = model_to_cam_vec(bd_model, R)
        rec["requested_force_camera"] = F_target * 1e-6

        # 力回代验证（对 I_target 而非内部候选）
        F_re = np.asarray(rec["achieved_force"], float) * 1e6
        ferr = float(np.linalg.norm(F_re - F_target))

        target_currents = (
            np.zeros(self.solver.n_coils)
            if rec.get("sparse_infeasible", False)
            else rec["currents"]
        )
        self.shared.set_I_target(
            target_currents,
            rec,
            F_target[:2],
            mpc_ms,
            solver_ms,
            out_x["cost"] + out_y["cost"],
            ref_target=ref[0],
            ref_velocity=vref[0],
        )
        self.shared.append_mpc_log(
            (
                time.time(),
                round(mpc_ms, 3),
                round(solver_ms, 3),
                round(out_x["cost"] + out_y["cost"], 6),
                round(ref[0][0], 4),
                round(ref[0][1], 4),
                round(vref[0][0], 4),
                round(vref[0][1], 4),
                round(F_target[0], 3),
                round(F_target[1], 3),
                round(ferr, 3),
                int(rec["converged"]),
                int(self.mpc_x.N),
                self.mpc_x.wp,
                self.mpc_x.wv,
                self.mpc_x.wu,
                self.mpc_x.wd,
                self.mpc_x.fmax,
                *(int(c) for c in rec["commands"]),
            )
        )
        if self.shared.control_logger is not None:
            self._log_control_worker(
                kalman_t,
                pos,
                eso_d,
                F_prev_camera,
                F_target,
                ref,
                vref,
                mpc_ms,
                solver_ms,
                out_x,
                out_y,
                ferr,
                rec,
                params,
            )

    def _log_control_worker(
        self,
        kalman_t: float,
        pos: NDArray[np.float64],
        eso_d: NDArray[np.float64],
        f_prev: NDArray[np.float64],
        f_target: NDArray[np.float64],
        ref: Sequence[NDArray[np.float64]],
        vref: Sequence[NDArray[np.float64]],
        mpc_ms: float,
        solver_ms: float,
        out_x: dict[str, Any],
        out_y: dict[str, Any],
        ferr: float,
        rec: dict[str, Any],
        params: dict[str, Any],
    ) -> None:
        """Copy already computed values; diagnostics faults cannot stop the worker."""
        sink = self.shared.control_logger
        if sink is None or not sink.recording:
            return
        try:
            mpc = cast("ForceMPC", self.mpc_x)
            _, seq, _, _ = self.shared.get_I_target()
            run = self.shared.control_log_run
            row: dict[str, Scalar] = {
                "t_mono": time.monotonic(),
                "t_wall": time.time(),
                "run": run,
                "seq": seq,
                "kalman_age_ms": (time.time() - kalman_t) * 1e3 if kalman_t else None,
                "s_progress": float(self._ref_arc),
                "mpc_ms": float(mpc_ms),
                "solver_ms": float(solver_ms),
                "mpc_cost": float(out_x["cost"] + out_y["cost"]),
                "horizon": int(mpc.N),
                "w_pos": float(mpc.wp),
                "w_vel": float(mpc.wv),
                "w_u": float(mpc.wu),
                "w_du": float(mpc.wd),
                "fmax": float(mpc.fmax),
                "max_cmd": int(params.get("max_cmd", cfg.CMD_MAX)),
                "force_error_uN": float(ferr),
            }
            for prefix, vector in (
                ("x0", pos),
                ("d_used", eso_d),
                ("F_prev", f_prev),
                ("F_target", f_target),
                ("ref", ref[0]),
                ("vref", vref[0]),
            ):
                for axis, value in zip(("x", "y", "z"), vector):
                    row[f"{prefix}_{axis}"] = float(value)
            for key in (
                "converged",
                "current_constraint_active",
                "field_constraint_active",
                "direction_constraint_active",
                "sparse_infeasible",
                "current_bound_ok",
                "slew_ok",
            ):
                row[key] = bool(rec[key]) if key in rec else None
            if "actuation_condition" in rec:
                row["actuation_condition"] = float(rec["actuation_condition"])
            row.update(sink.target(run, seq))
            sink.remember_progress(run, seq, float(self._ref_arc))
            sink.enqueue("worker", row)
        except Exception as exc:  # noqa: BLE001 -- diagnostics cannot stop the worker
            sink.fail(exc)

    def _drag_c(self, params):
        """µN/(mm/s)——由黏度与珠径计算（与 dipole_solver.drag_uN_per_mm_s 同式）"""
        eta = params.get("viscosity_mPas", 1000.0)
        r = params.get("bead_radius_m", cfg.BEAD_DIAMETER_MM * 0.5e-3)
        return 6.0 * math.pi * eta * 1e-3 * r * 1e3


class CurrentExecutor:
    """30Hz 电流执行层：目标插值 → 斜率限幅 → 量化 → RL 估计 → 正向模型 → 诊断"""

    def __init__(self):
        self.seq = -1
        self.I_from = np.zeros(6)  # 上一目标（A）
        self.I_to = np.zeros(6)  # 当前目标（A）
        self.frames_since = 0
        self.I_est = np.zeros(6)  # RL 模型估计电流（非测量值）
        self.low_field = False
        self.align_tau_ms = float("inf")
        self.align_ratio = float("inf")
        self.last_B = np.zeros(3)
        self.last_grad = np.zeros(3)
        self.last_F_est = np.zeros(3)

    def step(
        self, shared: SharedState, dt, pos_m, solver, last_sent, max_cmd=cfg.CMD_MAX
    ):
        """每 30Hz 帧调用。返回 (cmd_sent, diag dict)。"""
        tgt, seq, _, _ = shared.get_I_target()
        if seq != self.seq:
            self.I_from = np.array(last_sent, float) * solver.current_gain
            self.I_to = np.asarray(tgt, float)
            self.seq = seq
            self.frames_since = 0
        else:
            self.frames_since += 1
        # 1) 窗口内插值（α = 1/3, 2/3, 1，之后保持）
        alpha = min((self.frames_since + 1) / 3.0, 1.0)
        I_interp = self.I_from + alpha * (self.I_to - self.I_from)
        # 2) 斜率限幅 + 幅值限幅 + 量化（安全层）
        cmd = apply_slew_cmd(
            I_interp, last_sent, max_cmd=max_cmd, current_gain=solver.current_gain
        )
        # 3)-5) RL 估计 + 正向模型 + 诊断
        diag = self.update_est(dt, pos_m, solver, cmd)
        diag["cmd"] = cmd
        diag["interp_alpha"] = alpha
        return diag

    def update_est(self, dt, pos_m, solver, cmd):
        """RL 线圈估计 + MDM 正向模型 + 磁矩对齐/低场诊断。
        所有模式（PID/MPC）每 30Hz 帧调用；cmd 为实际发送的整数指令。"""
        # 一阶线圈动态。稳态电流使用当前解算器的已标定“指令→A”增益，
        # 避免 GUI 修改增益后执行器估计仍偷偷使用固定 2/99 A。
        I_steady = np.asarray(cmd, float) * solver.current_gain
        # 一阶 RL 的精确离散解。GUI 偶发卡顿时 dt 可能大于 2L/R，显式欧拉会
        # 数值发散；精确解对任意正 dt 都稳定。
        dt = max(0.0, float(dt))
        a = math.exp(-cfg.R_COIL_OHM * dt / cfg.L_COIL_H)
        self.I_est = a * self.I_est + (1.0 - a) * I_steady
        # MDM 正向模型（用 I_est 而非 I_target——不忽略线圈动态）
        fm_out = solver.forward_model(pos_m, self.I_est)
        B = fm_out["B"]
        bmag = float(fm_out["B_magnitude_mT"]) * 1e-3
        self.last_B, self.last_grad = B.copy(), fm_out["G"].copy()
        self.last_F_est = np.atleast_1d(fm_out["F"]).copy()
        # 磁矩对齐与低场诊断
        tau = cfg.ZETA_R / (solver.bead_moment * max(bmag, 1e-12))
        self.align_tau_ms = tau * 1e3
        self.align_ratio = dt / max(tau, 1e-9)
        self.low_field = bmag < cfg.B_MAG_THRESHOLD_T
        diag = {
            "cmd": list(cmd),
            "I_est": self.I_est.copy(),
            "B": B.copy(),
            "G": fm_out["G"].copy(),
            "grad_absB": fm_out["grad_absB"],
            "Bmag_mT": float(fm_out["B_magnitude_mT"]),
            "F_est": self.last_F_est.copy(),
            "tau_ms": self.align_tau_ms,
            "ratio": self.align_ratio,
            "low_field": self.low_field,
        }
        return diag


def apply_slew_cmd(
    target_currents,
    last_cmd,
    max_delta=cfg.MAX_DELTA_CMD,
    max_cmd=cfg.CMD_MAX,
    current_gain=cfg.CMD_TO_A,
):
    """电流目标 → 指令域安全层（运行时幅值上限 + 斜率 ≤9/帧）。"""
    max_cmd = int(np.clip(max_cmd, 1, cfg.CMD_MAX))
    out = []
    for t, l in zip(target_currents, last_cmd):
        c = round(t / current_gain)
        c = max(-max_cmd, min(max_cmd, c))
        c = max(l - max_delta, min(l + max_delta, c))
        c = max(-max_cmd, min(max_cmd, c))
        out.append(int(c))
    return out

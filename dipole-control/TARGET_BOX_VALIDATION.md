# 10 Hz solver target box 时间尺度验证（2026-10-06）

本 checkpoint 只扩 tracking worker 的目标箱：默认 **27**，GUI/config 保留 **9**。30 Hz executor、1/3→2/3→1 interpolation、两层逐帧 slew、物理参数和控制数学保持原样。未操作真实 COM、相机或实板，未 push。

## 实际基线与原 ±9 来源

开始时 HEAD=`9d0479e8655a1602bc45dd961f066be59dd94ad6`，message=`fix: align eso and mpc steady-state dynamics`；status 仅 `?? artifacts/`。已读 PROJECT_MEMORY 和 ESO_MPC_VALIDATION，并核对 L3（FirstOrderESO+steady_state）、freshness、原子 path progress。

旧 `dipole_solver.py` 模块常数 MAX_DELTA_CMD=9；`solve_field_force_pseudoinverse(max_delta_cmd=MAX_DELTA_CMD)` 的 guard 只允许0…9。`cmd_prev` 非None时，整数下/上界为 `max(-max_cmd,ceil(cp−delta))` / `min(max_cmd,floor(cp+delta))`，再乘 current_gain 成电流箱；连续有界伪逆与整数邻域都用这组边界。worker 原来未指定 delta，故每100ms仍只允许9。

**中心必须准确描述**：worker 传的是 `SharedState.get_last_sent()` 快照，不是上一理想 target；GUI在当前 executor 之前发布 last_sent，因此该快照还可能落后当前成功命令一帧。本次保留此中心和时序，修复半宽；没有改成 previous_target。两次理想 target 的差分在提前更新/快照滞后时并非普遍受同一半宽约束。

## 生产修改范围与回退

新增 `config.WORKER_TARGET_DELTA_MAX=27`；SharedState params→GUI `_publish_mpc_params`→worker 的显式 `target_box_delta` 传递。solver 仅在正式 Moore–Penrose 方法末尾新增 keyword-only 参数，None 保留直接调用旧9，显式值仅允许9/27；旧 max_delta_cmd guard 保留，其他solver接口/物理矩阵/有界算法完全不变。box=9分支运算顺序与父代码相同。

GUI路径页“10 Hz target box (cmd)”可选27/9，按既有settings机制持久化，下一worker周期读取；旧 gui_settings.json 缺少该键时默认27，保存文件原字节未改。回退仅切该项9或设置 WORKER_TARGET_DELTA_MAX=9；L3/权重无需切换。

| 路径 | 当前调度/求解方式 | 本次 box / 安全 |
|---|---|---|
| tracking worker | 约10Hz，MPC+MDM target | 新默认27，显式可选9 |
| manual force | live每GUI tick求解；按钮一次求解 | solver默认9，最终发送9 |
| direction test | 每GUI tick `_solve_and_send` | solver默认9，最终发送9 |
| friction calibration | ramp每tick求解，settle直接零 | solver默认9，最终发送9 |
| manual current | 每tick直接发送指定命令 | 不使用solver box；最终发送9 |
| coil scan | 每tick单通道命令 | 不使用solver box；最终发送9 |
| normal_stop / freshness timeout | 原逐帧归零 | 原每帧9 |
| emergency_stop | 原hard-zero直接帧 | 保留既有绕过slew的急停例外 |

## 名义可达集合与最终发送证明

30/10=3，3×9=27。CurrentExecutor接受新seq时 **I_from=当时last_sent×gain**、I_to=target、frames_since=0；三个有效帧alpha=1/3、2/3、1。中心与接受时命令一致、六通道目标差不超过27时，连续插值每步不超过9 command；整数round后的差也不超过9。例如target=27得到真实帧9→18→27。

实际是QTimer33ms和worker perf_counter100ms独立调度，worker迟到时重同步，**不是严格每seq三帧**。新seq即使提前到来，也从当时last_sent重新锚定、phase归零，不能承诺所有目标都三帧到达；共享center滞后时目标相对接受命令甚至可超过27。这些原时序未改。生产保护链仍为 worker currents→SharedState→CurrentExecutor interpolation→`apply_slew_cmd(default MAX_DELTA_CMD=9)`→GUI `send_commands` / `apply_slew(MAX_DELTA_CMD=9)`→43-byte UART。两层普通发送均做幅值/斜率/量化，未把box27传给任何发送层。固定运行幅值上限下正常执行/normal_stop始终≤9/帧；原急停直接零例外和运行中收紧幅值上限时优先硬clip的既有语义保留。

## 专项、t90与随机证据

新 target-box **33/33**、L3 A/B **16/16**，共49项。覆盖双符号/混合六路/单路/全部通道、prev±45接近max50、27基本截断、无约束分支全记录逐位等价（正交矩阵+真实JSON20个reachable案例）、非法box guard、worker actual快照中心、过期输入/慢求解拒绝时seq/progress不变、未接受过的过期27target冻结phase、timeout请求normal_stop、真实tracking的normal/emergency stop、GUI持久化/手动path9/按实际seq日志。

t90使用真实solver的正交测试驱动矩阵生成S=±27目标，然后通过原SharedState、executor、GUI双安全层和实际43-byte内存UART帧测量。参考起点为step后的第一个完整30Hz frame：

| box | 实际正/负对称响应的幅值 | 实测 t90 ms |
|---:|---|---:|
| 9 | 3,6,9,12,15,18,21,24,27 | 300.000000 |
| 27 | 9,18,27 | 100.000000 |

以首次≥0.9×27的帧定义t90，测试只断言实测27更快、单调无overshoot，未硬编码期望t90。正交矩阵是隔离box的数学夹具，非实机磁场。

| max_cmd | 随机target组 | 真实UART帧 | max Δsent | 结果 |
|---:|---:|---:|---:|---|
| 50 | 10000 | 20056 | 9 | PASS |
| 99 | 10000 | 20012 | 9 | PASS |

随机固定seed，每组相对实际命令±27且幅值合法，随机仅1/2/3帧即更新；检验每帧UART反解析与last_sent一致、幅值/斜率界、alpha phase、位于新锚点至新目标间、正常三帧到达。这里只stub forward/R-L诊断的磁模型返回值，生产executor/slew/量化/GUI发送/frame builder全部实际调用。另有每100ms重复目标和提前反向目标回归，重锚后不向旧target回弹。

## L3正式闭环A/B（只变box）

复用上一checkpoint框架：真实GUI start/estimate/mpc_track_step、Kalman、FirstOrderESO、steady_state MPC、path/freshness、180偶极子逆解、CurrentExecutor，固定240帧/8s、80次worker发布；独立BeadSimulator和150Hz精确R-L积分，seed=20261006、视觉sigma=sqrt(.005)mm。friction=现有mu .10，drag=c_true=1.5c，仅为合成不确定性，不改生产配置。

两套原配置分开：defaults GUI为原config权重1/2/.005/.01、Fmax40、Br1.2、gain widget .0202、−Z；saved为40/.8/.02/.001、Fmax50、Br.35、gain.02、+Z、lift off。MPC c、R/L、Q/R等全部相同于父。旧L0–L3的33项历史矩阵显式保持box9，新增这16项独立比较box9/27，未削弱旧门槛。

mean speed/F为后4s均值；tracking RMS是全8s相对1mm/s时间轨迹的二维RMS；target Δmax是相邻发布target差，sent Δmax是相邻实际发送帧差。cmd上限占比与MPC force饱和不同。progress包括原lead；max ahead是plant相对时间参考最大x领先，不等于endpoint overshoot（8s尚未跑完整折线）。

### 当前保存GUI配置

| 场景 | box | mean speed mm/s | tracking RMS mm | target Δmax | sent Δmax | cmd上限% | solver mean/p95 ms | Ftarget/Fest µN | final progress mm | speed std mm/s | max ahead mm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 9 | 0.998480 | 0.318129 | 9 | 3 | 0.00 | 8.571/69.359 | 9.345/9.401 | 7.800446 | 0.061939 | -0.008093 |
| nominal | 27 | 0.995153 | 0.316207 | 26 | 9 | 0.00 | 6.414/57.770 | 9.334/9.371 | 7.813888 | 0.064587 | 0.023892 |
| friction | 9 | 1.001563 | 0.332992 | 9 | 3 | 0.00 | 10.275/70.273 | 13.013/13.048 | 7.509326 | 0.050920 | -0.033333 |
| friction | 27 | 1.005440 | 0.294389 | 19 | 7 | 0.00 | 7.851/65.074 | 13.013/13.089 | 7.566371 | 0.065315 | -0.033333 |
| drag | 9 | 0.998158 | 0.357076 | 9 | 3 | 0.00 | 10.935/74.049 | 14.071/14.095 | 7.463465 | 0.031146 | -0.030395 |
| drag | 27 | 1.009007 | 0.335328 | 22 | 8 | 0.00 | 7.304/61.673 | 14.119/14.248 | 7.515617 | 0.028911 | -0.028900 |
| friction_drag | 9 | 1.010385 | 0.542260 | 9 | 3 | 0.00 | 9.922/64.894 | 17.957/18.109 | 7.277724 | 0.024371 | -0.033333 |
| friction_drag | 27 | 1.011237 | 0.528649 | 24 | 8 | 0.00 | 7.466/61.686 | 17.961/18.121 | 7.294585 | 0.024375 | -0.033333 |

### config默认GUI配置

| 场景 | box | mean speed mm/s | tracking RMS mm | target Δmax | sent Δmax | cmd上限% | solver mean/p95 ms | Ftarget/Fest µN | final progress mm | speed std mm/s | max ahead mm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 9 | 0.937304 | 0.419263 | 9 | 3 | 92.92 | 63.256/70.453 | 9.164/8.724 | 7.955552 | 0.085506 | 0.381335 |
| nominal | 27 | 0.939894 | 0.418314 | 25 | 9 | 97.92 | 64.801/75.028 | 9.163/8.753 | 7.959048 | 0.099643 | 0.382789 |
| friction | 9 | 1.019453 | 0.358283 | 9 | 3 | 92.92 | 64.323/76.105 | 13.044/13.136 | 8.076795 | 0.069202 | 0.258814 |
| friction | 27 | 0.988636 | 0.301993 | 25 | 9 | 95.42 | 65.164/74.000 | 12.948/12.865 | 7.929635 | 0.092604 | 0.202678 |
| drag | 9 | 1.009286 | 0.283320 | 9 | 3 | 92.92 | 63.019/71.558 | 14.106/14.165 | 7.831911 | 0.059503 | 0.045584 |
| drag | 27 | 1.012427 | 0.279263 | 25 | 9 | 97.92 | 62.775/69.032 | 14.126/14.211 | 7.831468 | 0.059762 | 0.057746 |
| friction_drag | 9 | 0.996682 | 0.439580 | 9 | 3 | 92.92 | 61.856/69.867 | 17.783/17.706 | 7.613754 | 0.054209 | -0.033333 |
| friction_drag | 27 | 1.031113 | 0.293985 | 25 | 9 | 97.92 | 62.602/69.718 | 18.026/18.192 | 7.873268 | 0.052200 | 0.046282 |

16组全部未见明显振荡或发散，沿用原RMS<5mm、cross-track<1mm、speed std<.8mm/s等门槛，最大实际变化≤9；全部MPC Fmax饱和率0，逐tick solver/freshness/force/progress/command trace保留。噪声带来小幅command reversal与速度波动，表中std/max ahead及validation-results中的reversal/cross-track可审计。27修复了周期级额外限速；这组软件数据不证明实机必然更好、完整路径终点无overshoot，亦未重调任何参数。

## 隔离solver性能（真实JSON/正式solver）

每套配置box9/27各1000次，共4000；相同seed/输入配对交替顺序，排除100次warmup、BLAS/OMP单线程、没有并行QA。500个GUI 10mT/原方向启动目标（水平力±20µN、position±5mm）+500个从真实forward构造的reachable稳态，用于覆盖active/inactive，未把所有任意[B;F]请求当可行。defaults是库默认gain=2/99、Br1.2；saved直接复用原保存gain=.02、Br=.35、直径1mm与+Z；JSON SHA256=9bda6fa6e52d7bf91ef9aa111b36ddf51b258de3336ff6808f35dc9c399ff747。

| 配置 | box | 分支 | n | mean ms | median ms | p95 ms | p99 ms | max ms |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| defaults | 9 | all | 1000 | 22.507 | 17.432 | 55.127 | 64.406 | 82.235 |
| defaults | 9 | bounded | 500 | 44.275 | 42.427 | 58.817 | 65.672 | 82.235 |
| defaults | 9 | inactive | 500 | 0.739 | 0.687 | 1.050 | 1.158 | 1.631 |
| defaults | 27 | all | 1000 | 0.950 | 0.819 | 1.219 | 1.385 | 45.870 |
| defaults | 27 | bounded | 2 | 43.015 | 43.015 | 45.585 | 45.813 | 45.870 |
| defaults | 27 | inactive | 998 | 0.866 | 0.818 | 1.214 | 1.374 | 2.334 |
| saved | 9 | all | 1000 | 31.739 | 24.896 | 69.088 | 80.794 | 99.286 |
| saved | 9 | bounded | 500 | 62.461 | 61.890 | 73.416 | 85.457 | 99.286 |
| saved | 9 | inactive | 500 | 1.016 | 1.002 | 1.311 | 1.518 | 1.715 |
| saved | 27 | all | 1000 | 1.429 | 1.173 | 1.500 | 1.738 | 69.836 |
| saved | 27 | bounded | 4 | 63.333 | 63.306 | 68.953 | 69.659 | 69.836 |
| saved | 27 | inactive | 996 | 1.180 | 1.171 | 1.494 | 1.669 | 2.350 |

bounded表示连续箱约束分支（不是最终cmd是否恰好等于幅值上限）；对应target delta active统计保存于JSON。放宽box主要改变进入分支的比例，不能把总体均值下降误读为有界算法本身被优化；defaults的27 bounded只有2个样本，样本小。两profile均无异常非有限数、异常退出或安全越界，有界分支仍存在约几十ms耗时和原33ms告警条件。

defaults: nonconverged 9/27=500/448；saved: nonconverged 9/27=500/295。这里nonconverged是原2% force / 5% field等converged旗标，受可达性、箱/幅值约束和整数残差影响，**不是NaN/数值失败**；保留原门槛和worker处理方式，未把这些请求包装成全部精确跟踪成功。

## 父checkpoint逐位回放与代码保护

从固定Git对象9d0479e加载真实父config/solver/ESO/MPC/worker/GUI，当前选L3且box9，分别defaults/saved。每套8场景×normal/emergency stop，3840 active ticks + 960 stop ticks，共 **4856条43-byte帧逐位一致**；worker目标/I_target、seq、executor内部所有状态/R-L数组、cmd_sent、KF/ESO、路径进度/参考、父solver每个非计时字段以float64字节/struct.pack检查。只排除不可复现计时、clock callback身份和新增target_box纯诊断。原8549 L0回放也保留通过，4856帧；没有仅以帧数相同充当逐位证据。

151固件文件和10个禁止修改的PC文件原字节一致（ESO/MPC/frame/path/friction/CURT/ADC/仿真物理/保存设置）；CurrentExecutor原source及AST一致，SharedState guards与apply_slew_cmd一致，GUI除4个配置/metadata函数外118方法AST一致（包括send_commands/apply_slew调用、stop、freshness、路径流程），solver另39方法完全相同，所有既有config值相同。本次没有改current PI、R/L/c/Fmax/权重、offset/z/Fz、CURT或其他数学。

## 日志、全部回归及checkpoint

unified control仅104→105列、worker55→56列，各加target_box_delta。原run+seq缓存保证control记录执行器所用目标；GUI切换但未发布时仍是旧值。原round(I_target/gain)给target command，cmd_exec/cmd_sent、solver_ms/status/current_constraint_active复用；metadata limits记初始选择，settings记GUI选择。未重复增加六路target列或改24列CSV/写盘机制，详见CONTROL_LOG.md。

Ruff PASS；Black PASS（40文件）；mypy PASS（40文件，原untyped GUI提示保持，不声称全工程strict）。

| 检查 | 最终结果 |
|---|---|
| frames | PASS 57/57 |
| mpc | PASS 6/6 |
| solver-protocol-safety | PASS 25/25 |
| shared-control | PASS 6/6 |
| bead-simulator | PASS 8/8 |
| curt | PASS 132/132 |
| stop-modes | PASS 21/21 |
| error-paths | PASS 46/46 |
| serial-safety | PASS 46/46 |
| firmware-watchdog-integration | PASS 7/7 |
| firmware-watchdog-c | PASS 29/29 |
| freshness | PASS 25/25 |
| gui-smoke | PASS SMOKE OK |
| control-log | PASS 25/25 |
| path-progress | PASS 34/34 |
| eso-mpc-unit-gui | PASS 42/42 |
| closed-loop | PASS 33/33 |
| l0-checkpoint-replay | PASS 2/2 |
| multirate | PASS 18/18 |
| target-box | PASS 33/33 |
| target-box-ab | PASS 16/16 |

本轮首次全量multirate通过18/18，无需重试/放宽原60ms门槛；freshness含10000tick jitter。error paths的10条既有绘图warning保留。开发首轮专项24失败源于新测试误用了solver构造器和GUI控件名，修正为from_json/spin_cmd_max后通过；随后修正新增trace字段名、静态类型/格式问题，未改变生产数学/门槛。初轮记录及最终stdout/stderr/quality/protection/replay/AB/performance evidence位于artifacts/target-box-20261006，不提交artifacts。

全部验证完成后更新PROJECT_MEMORY并创建独立checkpoint `fix: align solver target box with control period`，不push。最终parent/commit、diff --stat、status、阶段验证在同目录checkpoint-results.json/diff-stat.txt/status-final.txt。最终受跟踪工作区干净，只余原有artifacts未跟踪。

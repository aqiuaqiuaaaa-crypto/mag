# FirstOrderESO + MPC steady-state effort 验证（2026-10-06）

已按第二层第一个问题实施闭环一致性修复。应用新默认为 **L3 = FirstOrderESO + steady_state**；原 ESO1D、ESO off、absolute 模式保留，GUI/config 可切回 L0。所有新结构证据是软件数学/仿真证据。

## 首先确认的实际基线

- HEAD：`8549352e492e4d9cde2cc4142001fcada7ff994a`，`fix: unify path progress tracking`；初始 tracked worktree clean，仅 artifacts/ 未跟踪，未 push。
- PROJECT_MEMORY 最新摘要与 Git 对应。第一层的统一日志、stop 清理、PC serial fault/write timeout/IDLE heartbeat、STM32 300ms watchdog + CTRL_SD、frame、freshness、path progress 已存在；本次未改变这些控制/安全逻辑。
- 实际链：30Hz camera → Kalman/EMA/RAW → ESO → SharedState → 10Hz worker（路径局部投影/reference → ForceMPC → DipoleSolver 逆解）→ 30Hz executor（插值 → slew → 量化 → UART → R-L/MDM F_est）。
- **两套现存配置必须区分**：config 默认 MPC 权重 1/2/0.005/0.01、Fmax=40；已跟踪 gui_settings.json 启动覆盖为 40/0.8/0.02/0.001、Fmax=50、gain=.02、Br=.35、场方向+Z、lift off。保存文件未改；新的 mode 键缺省时采用 L3。

## 当前源代码的 legacy ESO 复现

MPC 正式模型是 `c*x_dot=F+d`。legacy ESO1D 用 `z1_dot=z2-β1*e`、`z2_dot=b0*(u+z3)-β2*fal(e)`，其对象为二阶。恒定 u/d 的一阶对象匀速运动时，z2 恒定意味着 `u+z3→0`，故 `z3→−u`，不取决于真实 d。使用当前模块、c=9.42477796076938、omega=4、dt=1/30、200s 得到：

| u (µN) | true d (µN) | legacy z3 (µN) |
|---:|---:|---:|
| 8 | 3 | -7.999999999999 |
| -8 | 3 | 8.000000000000 |
| 8 | -3 | -8.000000000000 |
| -8 | -3 | 7.999999999999 |
| 0 | 6 | -0.000000000000 |

新正式回归保存此错误作为对照；FirstOrderESO 在同组输入上收敛到真实 d（误差<1e−9µN）。legacy ESO1D/fal 的计算没有修改。首轮新对照测试只运行40s，legacy剩余误差约5.5e−5µN未达到1e−8阈值；延长到200s后通过，未降低精度门槛。

## 一阶 observer 的数学、单位和时序

`x_dot=b0*(u+d)`, `b0=1/c`；z1 为位置 mm，z2 为扰动力 µN。c 为 µN*s/mm，b0 为 mm/(µN*s)，l1=2ω 为 1/s，l2=ω²/b0=cω² 为 µN/(mm*s)。

每个有效子步 T：`z1_pred=z1+T*b0*(u_prev+z2)`，`e=y−z1_pred`，`z1=z1_pred+T*l1*e`，`z2=clip(z2+T*l2*e, ±d_max)`。mm↔m、µN↔N 换算回归验证了同一物理响应。

u 使用上一执行帧的 camera-world `last_F_actual[:2]*1e6`。last_F_actual 在 executor.step 后从 R-L I_est 经 MDM forward 的 F_est 写入，本次 ESO 先 step，再发布参数，再生成/执行新命令。保持历史 endpoint 模型估计语义；它不是电流/力实测，也不是未来 F_target，更不是未知缺口内力的重建。

令 h=Tω，未限幅的齐次误差矩阵为：

```text
M = [[1−2h,  (1−2h)*T*b0],
     [−T*l2, 1−h²]]
trace(M)=2−2h−h², det(M)=1−2h
Schur/Jury 条件：0<h<sqrt(8)−2≈0.828427124746
```

有效更新按 ceil(dt*ω/0.5) 个子步，保证 h≤0.5；量测在上次有效位置与本次量测之间线性插值，u保持上一执行模型值。网格覆盖 GUI允许 omega=.5…20、dt=.005… .2（40×40）；dt≤.15 的1200组合验证矩阵谱半径<1与恒定d收敛，其余400组合验证重置。视觉缺口 mark_gap，或 dt>.15（复用 input freshness 门槛）时，只重置位置、清零扰动，不把巨大恢复创新灌入d。下一有效帧正常恢复；ESO off→on也重新锚定。原 freshness/停止流程未改变。omega=4沿用软件默认，未实机标定。

## MPC H/g 推导与当前 κ

令 L=(T/c)*tril(1)，p0=x0+(T/c)*[1…N]*d，D*F−b=[F0−F_prev,F1−F0,…]。

```text
J = wp*||p0+L*F−r||² + (wv/c²)*||F+d−c*vr||²
    + wu*||F−F_center||² + wd*||D*F−b||²
H = wp*LᵀL + (wu+wv/c²)*I + wd*DᵀD
g_abs = wp*Lᵀ*(p0−r) + (wv/c²)*(d−c*vr) − wd*Dᵀb
F_ss = c*vr−d
g_ss = g_abs − wu*F_ss
```

QP 使用 `FᵀHF+2gᵀF`，所以不额外乘2。H、活动集 solver、Fmax 箱约束、总水平幅值约束和wd不变，wu数值不变。测试捕获实际 H/g，比较两种模式并用直接四项cost与有限差分梯度独立核对；正确d且可行时F_ss固定点cost=0。F_ss超限不提前clip中心，QP在原箱约束内正确饱和且无NaN。

对直线几何 lookahead `r=x0+T*v*[1…N]`、d=0、理想执行、未饱和，首步 `F0=A*v+B*F_prev`。令 F_prev=F0，则 κ=A/[c*(1−B)]。该比例不是完整非线性执行链的实机速度比例。

| 配置 | wp/wv/wu/wd | A | B | 重算 κ absolute | vref=1所需力 / absolute稳态力 (µN) |
|---|---|---:|---:|---:|---:|
| config默认 | 1/2/.005/.01 | 6.056941702460568 | 0.218581845338490 | 0.822429669454520 | 9.424777960769 / 7.751217022958 |
| 已保存GUI | 40/.8/.02/.001 | 6.104470759314133 | 0.023534330761064 | 0.663315142157544 | 9.424777960769 / 6.251597932851 |

两种配置 steady_state 固定点 κ=1（回归误差<1e−13），消除了absolute effort对必要稳态力的系统性压低；不声称消除未知d、参考曲率、KF滞后、执行约束等误差。

## 正式闭环仿真矩阵

每组240个30Hz帧/8s，80次10Hz MPC发布。真实GUI start_tracking/estimate_state/mpc_track_step、SharedState freshness/path reference、现有非线性180偶极子solver及CurrentExecutor全部使用生产代码；仅由测试确定worker调度、提供视觉位置和内存UART。独立一阶plant使用现有BeadSimulator库仑摩擦，150Hz积分当前R-L lag和位置相关磁力。视觉噪声sigma=sqrt(.005)mm，固定seed，原Kalman Q/R不变。摩擦mu=.10为现有仿真器默认，drag案例c_true=1.5*c是声明的合成不确定性，不写入生产参数。

路径包含原lead及直线/折线reference。平均速度为后4s沿当前直线路径x分量；tracking RMS为全8s相对1mm/s时间参考的二维RMS，另保存几何横向RMS。d_true相对nominal模型为c*v−F，表中d/F是后4s均值。完整逐tick trace在artifacts的pytest输出中；不是硬件保证。

L0=legacy+absolute，L1=off+absolute，L2=off+steady_state，L3=first_order+steady_state。

### 当前保存的GUI配置

| 场景 | 模式 | mean speed mm/s | tracking RMS mm | d_hat µN | d_true µN | F_target µN | F_est µN | command上限占比 | speed std mm/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | L0 | 1.109435 | 0.836995 | -6.201221 | -0.005331 | 10.319889 | 10.428439 | 0.00% | 0.099834 |
| nominal | L1 | 0.673286 | 1.560132 | 0.000000 | -0.003158 | 6.253409 | 6.336407 | 0.00% | 0.022686 |
| nominal | L2 | 1.011418 | 0.061824 | 0.000000 | -0.004778 | 9.426737 | 9.517411 | 0.00% | 0.021652 |
| nominal | L3 | 0.998480 | 0.318129 | 0.075364 | -0.004877 | 9.345313 | 9.401327 | 0.00% | 0.061939 |
| friction | L0 | 0.727968 | 2.385025 | -6.692773 | -3.848763 | 10.645893 | 10.686405 | 0.00% | 0.098710 |
| friction | L1 | 0.334957 | 3.119311 | 0.000000 | -3.134580 | 6.252320 | 6.287603 | 0.00% | 0.025791 |
| friction | L2 | 0.602360 | 1.907132 | 0.000000 | -3.847446 | 9.426961 | 9.514137 | 0.00% | 0.027521 |
| friction | L3 | 1.001563 | 0.332992 | -3.589644 | -3.629049 | 13.013409 | 13.048237 | 0.00% | 0.050920 |
| drag | L0 | 0.761771 | 1.957256 | -6.800439 | -3.593420 | 10.715025 | 10.751752 | 0.00% | 0.069470 |
| drag | L1 | 0.442163 | 2.595285 | 0.000000 | -2.085639 | 6.251501 | 6.246391 | 0.00% | 0.016269 |
| drag | L2 | 0.673181 | 1.572968 | 0.000000 | -3.175471 | 9.426456 | 9.508200 | 0.00% | 0.013804 |
| drag | L3 | 0.998158 | 0.357076 | -4.649420 | -4.708750 | 14.070886 | 14.094940 | 0.00% | 0.031146 |
| friction_drag | L0 | 0.504747 | 3.053620 | -7.171395 | -6.225092 | 10.961733 | 10.967720 | 0.00% | 0.066781 |
| friction_drag | L1 | 0.219501 | 3.630854 | 0.000000 | -4.143452 | 6.250638 | 6.210772 | 0.00% | 0.015458 |
| friction_drag | L2 | 0.397564 | 2.827053 | 0.000000 | -5.718593 | 9.425399 | 9.460126 | 0.00% | 0.015201 |
| friction_drag | L3 | 1.010385 | 0.542260 | -8.536333 | -8.609363 | 17.956958 | 18.108564 | 0.00% | 0.024371 |

### config默认配置

| 场景 | 模式 | mean speed mm/s | tracking RMS mm | d_hat µN | d_true µN | F_target µN | F_est µN | command上限占比 | speed std mm/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | L0 | 1.542463 | 1.104363 | -8.606409 | -0.033609 | 14.621253 | 14.274352 | 83.33% | 0.159372 |
| nominal | L1 | 0.874679 | 0.412387 | 0.000000 | -0.017395 | 7.844230 | 8.166289 | 80.00% | 0.077300 |
| nominal | L2 | 0.981221 | 0.445429 | 0.000000 | -0.020131 | 9.359645 | 9.129798 | 92.92% | 0.080140 |
| nominal | L3 | 0.937304 | 0.419263 | 0.141746 | -0.018213 | 9.164206 | 8.724007 | 92.92% | 0.085506 |
| friction | L0 | 1.296146 | 0.696540 | -9.715542 | -3.825288 | 15.680847 | 15.853153 | 80.83% | 0.163730 |
| friction | L1 | 0.499968 | 2.024290 | 0.000000 | -3.841311 | 7.920094 | 8.527109 | 82.08% | 0.080380 |
| friction | L2 | 0.623516 | 1.446972 | 0.000000 | -3.818597 | 9.472780 | 9.654245 | 81.67% | 0.077870 |
| friction | L3 | 1.019453 | 0.358283 | -3.602817 | -3.665867 | 13.043541 | 13.136481 | 92.92% | 0.069202 |
| drag | L0 | 1.135374 | 0.744474 | -9.925530 | -5.369435 | 15.825348 | 15.924746 | 80.83% | 0.105611 |
| drag | L1 | 0.584438 | 1.669392 | 0.000000 | -2.762997 | 7.863711 | 8.245265 | 71.25% | 0.071068 |
| drag | L2 | 0.679510 | 1.286961 | 0.000000 | -3.212031 | 9.458992 | 9.566828 | 85.00% | 0.056010 |
| drag | L3 | 1.009286 | 0.283320 | -4.674319 | -4.774455 | 14.106430 | 14.164804 | 92.92% | 0.059503 |
| friction_drag | L0 | 0.870321 | 1.765610 | -10.492457 | -7.950909 | 16.230458 | 16.068802 | 82.50% | 0.085895 |
| friction_drag | L1 | 0.338070 | 2.821780 | 0.000000 | -5.440460 | 7.947797 | 8.624107 | 92.92% | 0.052563 |
| friction_drag | L2 | 0.420219 | 2.412369 | 0.000000 | -5.816204 | 9.501743 | 9.763390 | 92.92% | 0.060571 |
| friction_drag | L3 | 0.996682 | 0.439580 | -8.372412 | -8.416167 | 17.782957 | 17.705633 | 92.92% | 0.054209 |

全部32组无明显发散，最大command变化≤9/帧、每组240条43-byte帧、Fmax force饱和率0。command reversal、横向RMS、solver状态、各tick模式和freshness保存在simulation-results/trace中；config默认场方向−Z，保存配置为+Z，Br/gain/lift也不同，不能混用两表解释。config默认测试的command到±50比例较高（71%–93%），与MPC Fmax饱和不同；保存配置这组仿真未到command上限。未为此改场目标、电流限制、增益、Fz或任何物理值。

L0 nominal 的legacy d_hat明显非真实d，速度可超过目标，是错误observer与absolute偏置耦合的补偿，不能据其速度视作正确物理模型。L1/L2在摩擦/增阻下不估d；L3给出一致的扰动力与必要力。L3也不保证每个指标都优于所有其他组：噪声、执行器与有限视界仍会产生tracking误差。额外L3 friction+drag视觉9帧缺口回归产生STALE_INPUT、停止新目标发布；第69帧恢复d_hat=[0,0]，后一帧未出现大扰动脉冲，后段速度恢复且无发散。

## 父checkpoint逐位等价与保护

45组MPC absolute golden取自父Git源码，覆盖允许的N=1…6，包含当前保存权重、符号零、参考补齐、无速度参考及饱和：H、g、F0/F_seq/x_pred/v_pred/delta_F/cost/fmax全部以float64字节比较。新absolute分支保持原运算顺序。

L0真实父GUI/ESO/MPC/worker从固定Git对象加载，分别用config默认与已保存GUI配置回放。每套8种场景×normal/emergency stop，共3840 active tick + 960 stop tick、**4856条43-byte帧逐位一致**；包括转角折线、视觉缺口和其他五种主动模式。reference/progress、seq、target、KF/legacyESO、R-L/executor、所有父非计时solver字段逐位相同。仅排除不可重现的耗时、callback身份和新增纯mode/F_ss诊断；不是只比总帧数。

保护检查：151个固件文件、9个禁止修改的上位机/保存设置文件原字节保持；ESO1D/Kalman/整个CurrentExecutor原文保持，QP solver、SharedState guards、所有既有config赋值数学值相同；GUI其他111个既有函数AST不变。未改串口/watchdog、frame、path progress、freshness、R/L/c、摩擦/CURT/Kalman参数、offset/z/Fz/current PI。

## 日志、回退和测试

control log仅由101增至104列（effort_mode/F_ss_x/y）；worker由51增至55列（eso_mode/effort_mode/F_ss_x/y）。已有d_hat/u_eso/F_target/F_est/vref/solver字段复用。目标的effort/F_ss按run+seq关联，保证切换期间日志表示实际采用目标；未改队列/磁盘线程/原24列CSV格式。详细语义见CONTROL_LOG.md。

GUI高级控制中选择legacy且启用ESO、路径页选择absolute即可回到L0；关闭ESO则L1/L2。配置ESO_MODE=first_order、MPC_EFFORT_MODE=steady_state为新默认；保存设置缺少新键时也进入L3，已有物理/权重覆盖仍保留。直接ForceMPC构造器为兼容历史调用继续默认absolute，应用显式传入配置模式。

| 检查 | 最终结果 |
|---|---|
| ruff | PASS |
| black | PASS，38文件 |
| mypy | PASS，38文件 |
| mypy-strict-new-observer | PASS，新observer AST提取检查 |
| frames | PASS 57/57 |
| mpc | PASS 6/6 |
| solver-protocol-safety | PASS 25/25 |
| shared-control | PASS 6/6 |
| bead-simulator | PASS 8/8 |
| curt | PASS 132/132 |
| stop-modes | PASS 21/21 |
| error-paths | PASS 46/46，10条既有绘图warning |
| serial-safety | PASS 46/46 |
| firmware-watchdog-integration | PASS 7/7 |
| firmware-watchdog-c | PASS 29/29 host生产场景 |
| freshness | PASS 25/25，含10000tick jitter |
| gui-smoke | PASS SMOKE OK |
| control-log | PASS 25/25 |
| path-progress | PASS 34/34 |
| eso-mpc-unit-gui | PASS 42/42 |
| closed-loop | PASS 33/33（32矩阵+1 gap） |
| l0-checkpoint-replay | PASS 2/2（两套配置） |
| multirate | PASS 18/18 |

FirstOrderESO可执行行跟踪：44/44（100.00%）；新结构/模式正式回归77项（unit/GUI42 + simulation33 + replay2）。旧pytest装饰器测试继续使用原有外置adapter/独立入口，没有改测试阈值。

首轮全量multirate中test_solver_no_block测得60.5ms，超过原60ms门槛（17/18）；initial-final-multirate日志和initial-regressions.json保存。独立复测18/18，最终整组18/18；未改调度或阈值，不把偶发失败隐藏为首测通过。首次单元对照收敛时间和测试导入/类型/格式问题已修正；未改变legacy数学。静态检查保留原untyped GUI方法提示；新增observer单独strict通过，不能称全工程strict。

全部验证通过后独立checkpoint `fix: align eso and mpc steady-state dynamics`，不push。最终diff/stat/status和commit结果保存在artifacts/eso-mpc-20261006；artifacts不提交。未操作真实COM/相机/实板，没有构建或烧录新固件。

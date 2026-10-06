# CurrentExecutor 三帧插值评估（2026-10-06）

本次从 `2a1be54b5e1e668963f4ec756aa44b6084008a26` 开始。先核对 status/log、PROJECT_MEMORY、ESO_MPC_VALIDATION、TARGET_BOX_VALIDATION 和执行链；tracked clean、仅 artifacts 未跟踪，默认 L3、tracking box27、非 tracking solver9、两层逐帧9均成立。先保持 ON，再登记判据、增加明确模式、运行专项与闭环；未操作真实 COM/相机/实板，不 push。

**最终决策：保留 `legacy_three_frame` 默认，提供 `direct` 供 GUI/config 切换。** Direct 的小中阶跃延迟更低，正常发送保护通过，但两套配置的闭环表现不同，部分工况未通过预登记的性能门槛。不存在“为删插值而改参数”的操作；配置、物理、MPC、observer 与保护数值均保留。

## 真实生产语义

worker 成功发布 → SharedState 原子更新 seq/target/progress → executor 接受 fresh seq → 构造候选 → 原 `apply_slew_cmd` → 原 GUI `send_commands/apply_slew` → 串口写入。

- anchor 是 **接受新 seq 时传入的 last_sent × solver.current_gain**，即 GUI 最后成功写入的整数命令状态；不是 previous target。新 seq 不论旧 phase 是否完成，都重新锚定、frames_since=0。
- legacy 的 alpha=min((frames_since+1)/3,1)。只有一次 fresh executor.step 才推进 phase；30Hz GUI tick 失检而提前返回时不推进。第三帧后保持 target。新 seq 激活时序没有改变。
- direct 同样只接受 fresh seq；I_from/I_to/seq/frames_since 的接受与推进保持，仅候选改为 I_to、alpha=1。它依然经过两层 ±9/frame 与幅值限幅。
- stale >0.30s 不接受未见的 seq、不追旧目标、不推进 phase；保持最后命令并按它更新原 R-L 诊断。>1s 仍请求原 normal_stop，随后逐帧9归零。normal_stop 不继续插值；emergency_stop 原样立即硬零，是现存 slew 例外。restart 重新创建 SharedState 和 executor，旧 phase/I_est 不继承。
- 生产代码未发现 activation reward；目标激活是 seq 接受，不引入新的 reward 或 acceptance 时间规则。
- worker box 的中心是它读取的 SharedState.last_sent 快照。重新核查 GUI：executor 前和成功发送后**均更新**该快照，因此旧记忆中“固定落后一帧”的简化不成立；异步求解仍可能消费较早快照，本次未改时序。

GUI 模式选择写入 settings，**下次 start_tracking 生效**；一次追踪内固定。这样切换设置不会中途重新构造 anchor/phase。config `EXECUTOR_INTERPOLATION_MODE` 默认 legacy，库构造器也保持 legacy 以兼容旧调用。现存 gui_settings.json 原字节不变；无新键时使用 config。

## 解析与时间定义

legacy 候选电流 `I_j=I_anchor+alpha_j*(I_target-I_anchor)`；direct 为已接受 I_target。安全层按现有顺序 round(I/current_gain)、幅值 clip、相对上一实际命令 clip9、最终幅值 clip；GUI 再执行相同保护。下表由真实 executor、GUI sender、43-byte 帧解析取得，并逐帧保存 target/candidate/post-slew/UART。

时间表使用**累计执行帧预算**：第一帧33.333ms、第二帧66.667ms、第三帧100ms，便于与父报告比较；这不是测量串口时延。注入 clock 下发布/接受/首帧写入同一 tick0，实际调度发送 tick 时间须从表中减33.333ms。真实 worker/QTimer 相位、jitter、求解及串口耗时尚需实机测量。每个原始阶跃记录同时给出 scheduled_send_time_ms 和 *_send_tick_ms。

| 区间 | S绝对值 | ON 前三帧 | OFF 前三帧 | 首响应预算 ON/OFF ms | t50 ON/OFF ms | t90=t100 ON/OFF ms | 最大单帧 ON/OFF |
|---|---|---|---|---|---|---|---|
| 小 | 1 | 0,1,1 | 1,1,1 | 66.667/33.333 | 66.667/33.333 | 66.667/33.333 | 1/1 |
| 小 | 3 | 1,2,3 | 3,3,3 | 33.333/33.333 | 66.667/33.333 | 100/33.333 | 1/3 |
| 小 | 6 | 2,4,6 | 6,6,6 | 33.333/33.333 | 66.667/33.333 | 100/33.333 | 2/6 |
| 小 | 9 | 3,6,9 | 9,9,9 | 33.333/33.333 | 66.667/33.333 | 100/33.333 | 3/9 |
| 中 | 12 | 4,8,12 | 9,12,12 | 33.333/33.333 | 66.667/33.333 | 100/66.667 | 4/9 |
| 中 | 18 | 6,12,18 | 9,18,18 | 33.333/33.333 | 66.667/33.333 | 100/66.667 | 6/9 |
| 大 | 27 | 9,18,27 | 9,18,27 | 33.333/33.333 | 66.667/66.667 | 100/100 | 9/9 |

全部负阶跃逐帧呈对应反号；不是假设对称后省略执行。±1 的额外首响应帧来自原 round。对于大于27的输入变化，三帧并不保证到达，两种模式都由实际 slew 限制继续约束。

## 独立作用与安全含义

27 是 nominal100ms内3×9的目标可达范围，9给正常发送的幅值变化硬上限；**二者不约束 Δ²cmd 或高频能量**。插值对小中阶跃提供额外平滑、也增加执行滞后；27阶跃才在这组构造中完全重叠。MPC w_delta 约束的是力变化，经非线性/有界逆解后不能直接视作 command jerk 上限；R-L 过滤电流而不会让不同 command 序列变成同一电流轨迹。

因此软件证据支持“直接路径保持现有硬保护”，不支持“插值没有任何独立平滑作用”。尚无实机电压、机械振动、PI饱和、热或EMI的量化限制来证明这种平滑是否物理上必需。本次保留默认的理由是性能门槛未全通过，也不是已经证明 ON 对所有实机条件必要。

## 实验设计、指标与边界

`artifacts/interpolation-20261006/study-design.json` 在试验前登记：两 profile（config默认与原保存GUI覆盖）×7工况×3 seed×2模式=84组，加4组300Hz积分复核。基础四工况 nominal/现有Coulomb mu=.10/true drag1.5c/组合，附加无噪声、9tick视觉缺口、三段6mm直角转弯及方向反转。基础仍用父路径(-4,0)→(9,0)→(9,4)，8s窗口；短转角路径(-4,0)→(-2,0)→(-2,2)→(-4,2)运行到原0.5mm finish判据。仅合成 plant uncertainty 属于工况输入，没有写入生产参数。

固定 L3、steady_state、box27、c/weights/Fmax、R/L、Kalman Q/R、场/增益/摩擦与路径机制；同 profile/case/seed 的 ON/OFF 只改插值。无噪声三 seed 得到相同确定性轨迹，不能当作三份独立随机证据。噪声工况 sigma=sqrt(.005)mm；seed=20261006/07/08。

预登记组均值门槛：OFF RMS 增量≤max(.05mm,.1×ON)，max误差/overshoot增量≤.10mm，沿路径均速变化≤.10mm/s，完成时间增加≤.50s，发散/安全失败0。它们是本轮软件比较准则，不是硬件规格，也没有实验后放宽。

- tracking RMS/max：位置相对1mm/s时间参考。直线沿用父定义；短路径用相同折线弧长参数的时间参考。转角速度沿实际 committed progress 对应的切向投影。基础8s尚未到终点，completion记不可用。
- 直线 overshoot 为 max(0,时间参考最大有符号纵向超前)，每个 seed 分别算后汇总；原有符号 max_ahead_reference_mm 也保留。短路径为最后线段方向的终点正向越界，统计到原 finish 请求时。完成时间是 GUI 观察到 worker.finished 后 normal_stop 的 tick；不把0.5mm软件完成等同于精确停在终点，也未声称覆盖停止后的实物滑行。
- command/Δ/Δ² RMS 对所有控制时间样本和六轴取均方根，初始差分前置零。HF 为30Hz采样、去均值、Hann窗、FFT频点≥8Hz的功率占比；不是PWM/连续时间频谱。速度相邻差 RMS、原 reversal ratio/std用于观察波动，不等同于证明全局稳定。
- **target_to_sent_error=六轴 ||target_cmd-cmd_sent||₂**（command单位）。有效executor帧报告mean/median/p95/max，排除无视觉而未调用executor的tick；JSON也保留包括held tick的lag。目标来自未取整I_target/gain。达到每轴±1cmd才计首次到达帧预算；被替换或未到达的目标单列censored，禁止填一个虚假延迟；调度tick延迟=帧预算−33.333ms。量化浮点~1e−15差值保留。
- I_est、独立plant电流、采样差分dI/dt、frame开始瞬时模型导数 `(I_steady−I_previous)/tau`、F_est/F_target、command与力饱和、solver mean/p95完整保存；力饱和与command上限事件不混用。solver是真实墙钟耗时，控制时序由注入clock提供，不假设实际系统能同步执行零耗时求解。

生产 start_tracking/estimate_state/SharedState/worker/MPC/180偶极子solver/executor/send均沿用正式框架；仅独立时间、量测和UART为harness。plant为原BeadSimulator、150Hz R-L/非线性力积分；300Hz复核只改变plant数值步长。离散硬clip/量化会放大轨迹差异，4个refinement不是完整收敛证明；不把代码验证当作物理模型验证。

完整机器证据、逐帧trace、stdout/stderr、预登记、源码hash、软件版本及PNG/SVG/PDF见 `artifacts/interpolation-20261006/`。artifacts不提交；下面的汇总表由保存trace重新计算，不重新执行控制器或挑选seed。

<!-- Generated evidence tables follow. -->

## 连续 target 与提前更新

每100ms发布一个target，M=99以覆盖81级序列。A/B为周期级可达案例；C的36级反转、E的部分变化超过nominal27，是用户要求的执行器构造/压力案例，不声称它们都能由同一实际worker box发布。D为初始0后±3交替100个目标，303帧。单元格ON / OFF。

|序列|Δcmd RMS|Δ²cmd RMS|HF %|lag mean L2|峰值Δ|
|---|---|---|---|---|---|
|A|1.6833 / 2.9155|0.5270 / 4.1231|0.0095 / 10.0918|3.6742 / 0.0000|2.0000 / 6.0000|
|B|7.7942 / 7.7942|2.5981 / 2.5981|0.0583 / 0.0583|16.5341 / 16.5341|9.0000 / 9.0000|
|C|7.0356 / 7.3485|6.9821 / 6.8739|3.7628 / 1.4471|20.2083 / 18.3712|9.0000 / 9.0000|
|D|1.9826 / 3.4340|2.2821 / 4.8564|3.0302 / 11.1111|4.8262 / 0.0000|2.0000 / 6.0000|
|E|4.9777 / 5.4772|4.7799 / 5.7009|2.4061 / 2.9986|20.0051 / 19.2101|9.0000 / 9.0000|

A小步lag降低，B输出逐帧完全一致；C反转峰值均9，OFF的HF更低，不能泛化ON总更平滑；D的Δ/Δ²/HF明显增加，但lag为0；E六轴混合也有残余差异。首帧提前换seq单独测1帧重锚与后续收敛，均无旧目标回弹/残余phase。target/candidate/actual逐帧与全部正负阶跃见evidence文件。

## Random property 与兼容

|mode|max_cmd|目标组|实际UART帧|stale检查|CAS拒绝|stop/restart|违规|
|---|---|---|---|---|---|---|---|
|direct|50|20000|40476|190|20000|21|0|
|direct|99|20000|40422|190|20000|21|0|
|legacy_three_frame|50|20000|40476|190|20000|21|0|
|legacy_three_frame|99|20000|40422|190|20000|21|0|

共80,000组、161,796条正常/held/归零UART帧。两mode各40,000组，每个limit各20,000；slew/amplitude/NaN-inf/stale/seq倒退/stop后非零重启/overwrite违规全部0。随机生成的是有限合法目标，NaN-inf=0表示观测输出非有限数计数，并非宣称新增了非法输入净化。seq单调按一次运行统计，restart合法复位。

模式专项覆盖stale input、过期未见seq、hold/timeout、slow solve提交复核、CAS/newer seq/stop拒绝、read期间发布、normal/emergency stop、PATH_DEVIATION、restart、新设置下次运行生效、持久化与模式日志。原freshness边界/stop/error/serial回归保留；原紧急硬零与动态收紧幅值上限优先clip的例外不冒充正常slew≤9。

## Legacy ON golden

父Git对象加载config/solver/estimators/MPC/multirate/GUI，当前固定L3+box27+legacy；两profile、8运行场景×normal/emergency，共3840 active+960 stop ticks、4856条43-byteUART帧。target/I_target/seq、接受anchor/phase、实际interpolated candidate（在生产apply_slew_cmd入口捕获）、post-slew与UART、F_target/模型力、KF/ESO/R-L状态、MPC状态/父非计时solver字段、path progress全部逐位一致。仅排除耗时、clock callback身份、新增模式/纯诊断。原L0父回放与9d0479e父box9回放也经历史回归通过；REPLAY_EVIDENCE保存在stdout。

## R-L解释

同一安全command sequence输入两mode的update_est，2000帧全部输出/I_est/F_est float64字节相同；因此同dt的采样dI/dt也相同，mode没有改变R-L。tau=L/R=22.750ms，a30Hz=.23103144199243125，fc=6.995821674Hz。连续H(s)=1/(1+s*tau)，端点离散H(z)=(1-a)/(1-a*z^-1)，已存在低通，但不是把不同command完全等同。

固定模型位置(1,-2,0)mm的step电流模式为[S,0,-S,S,-S,0]，另有原D/E序列；仅用于结构对照，未经过MPC force target约束，不把它的独立MDM力幅值当作闭环Fmax违规或实机结果。下方R-L表说明6/18级残余电流和force差异不为零，27级逐帧完全一致；小反转的采样dI RMS为.993392/1.624857 A/s、瞬态峰值2.267044/5.295154 A/s。闭环另有各工况自身I/F指标。

## 闭环：三 seed 均值，单元格 ON / OFF

|profile/case|速度 mm/s|RMS mm|最大误差 mm|超前/终点越界 mm|完成 s|
|---|---|---|---|---|---|
|defaults/corner_reversal_finish|1.042 / 1.032|0.674 / 0.764|1.309 / 1.504|0.008 / 0.002|4.700 / 4.500|
|defaults/drag|1.010 / 0.989|0.299 / 0.312|0.734 / 0.681|0.079 / 0.124|未跑终点|
|defaults/friction|1.003 / 0.985|0.294 / 0.318|0.813 / 0.847|0.159 / 0.231|未跑终点|
|defaults/friction_drag|1.033 / 1.020|0.314 / 0.320|0.609 / 0.689|0.015 / 0.087|未跑终点|
|defaults/nominal|0.975 / 0.952|0.436 / 0.519|0.994 / 1.033|0.396 / 0.499|未跑终点|
|defaults/nominal_no_noise|0.974 / 0.955|0.467 / 0.554|1.084 / 1.113|0.407 / 0.526|未跑终点|
|defaults/vision_gap|0.966 / 0.965|0.483 / 0.592|1.059 / 1.160|0.455 / 0.571|未跑终点|
|saved/corner_reversal_finish|1.451 / 1.282|0.327 / 0.297|0.771 / 0.665|0.000 / 0.004|5.133 / 5.133|
|saved/drag|1.011 / 1.007|0.338 / 0.339|0.385 / 0.383|0.000 / 0.000|未跑终点|
|saved/friction|1.011 / 1.006|0.292 / 0.283|0.351 / 0.332|0.000 / 0.000|未跑终点|
|saved/friction_drag|1.012 / 1.009|0.551 / 0.525|0.605 / 0.582|0.000 / 0.000|未跑终点|
|saved/nominal|1.002 / 1.005|0.203 / 0.156|0.526 / 0.312|0.019 / 0.006|未跑终点|
|saved/nominal_no_noise|1.013 / 1.010|0.032 / 0.032|0.058 / 0.063|0.046 / 0.017|未跑终点|
|saved/vision_gap|1.002 / 1.004|0.299 / 0.240|0.810 / 0.519|0.025 / 0.004|未跑终点|

## command：三 seed 均值，ON / OFF

|profile/case|cmd RMS|Δ RMS|Δ² RMS|HF %|速度变差 RMS mm/s|上限 %|
|---|---|---|---|---|---|---|
|defaults/corner_reversal_finish|36.743 / 36.726|1.480 / 1.572|1.084 / 1.243|0.564 / 0.623|0.098 / 0.133|96.45 / 96.29|
|defaults/drag|37.809 / 37.806|1.129 / 1.173|0.816 / 0.910|0.208 / 0.229|0.039 / 0.052|97.92 / 97.92|
|defaults/friction|37.698 / 37.729|1.353 / 1.373|1.091 / 1.136|0.421 / 0.388|0.052 / 0.074|95.69 / 96.39|
|defaults/friction_drag|37.748 / 37.703|1.127 / 1.165|0.812 / 0.897|0.249 / 0.268|0.037 / 0.051|97.92 / 97.92|
|defaults/nominal|37.582 / 37.310|1.130 / 1.651|0.822 / 1.089|0.176 / 0.142|0.058 / 0.078|97.92 / 95.28|
|defaults/nominal_no_noise|37.585 / 37.564|1.141 / 1.180|0.848 / 0.940|0.282 / 0.305|0.066 / 0.077|96.67 / 96.67|
|defaults/vision_gap|37.494 / 37.254|1.357 / 1.657|1.095 / 1.094|0.375 / 0.162|0.061 / 0.079|97.22 / 95.69|
|saved/corner_reversal_finish|13.257 / 13.187|0.908 / 1.230|0.945 / 1.599|0.331 / 1.216|0.110 / 0.138|0.00 / 0.00|
|saved/drag|14.213 / 14.348|1.486 / 2.099|1.196 / 2.492|0.170 / 1.358|0.029 / 0.036|0.00 / 0.00|
|saved/friction|14.228 / 14.345|1.595 / 2.062|1.267 / 2.321|0.167 / 1.080|0.039 / 0.032|0.00 / 0.00|
|saved/friction_drag|14.418 / 14.348|1.531 / 1.834|1.346 / 2.388|0.228 / 1.934|0.023 / 0.020|0.14 / 0.00|
|saved/nominal|14.438 / 14.445|1.821 / 2.210|1.367 / 2.191|0.166 / 0.641|0.071 / 0.075|0.14 / 0.00|
|saved/nominal_no_noise|15.037 / 15.035|1.928 / 2.278|1.268 / 2.369|0.097 / 0.683|0.028 / 0.043|0.00 / 0.00|
|saved/vision_gap|14.438 / 14.482|1.766 / 2.257|1.357 / 2.352|0.205 / 0.848|0.095 / 0.107|0.00 / 0.28|

## lag：有效 executor 帧 L2 command，ON / OFF

|profile/case|mean|median|p95|max|到达帧预算 ms|未到达目标|
|---|---|---|---|---|---|---|
|defaults/corner_reversal_finish|1.143 / 0.659|0.000 / 0.000|1.847 / 0.000|38.820 / 36.373|36.399 / 36.299|0.000 / 0.000|
|defaults/drag|0.629 / 0.370|0.000 / 0.000|1.732 / 0.000|38.820 / 36.373|35.278 / 35.000|0.000 / 0.000|
|defaults/friction|0.818 / 0.530|0.000 / 0.000|1.900 / 0.000|40.577 / 37.727|35.833 / 35.556|0.000 / 0.000|
|defaults/friction_drag|0.617 / 0.370|0.000 / 0.000|1.732 / 0.000|38.820 / 36.373|35.139 / 35.000|0.000 / 0.000|
|defaults/nominal|0.645 / 0.819|0.000 / 0.000|1.732 / 4.883|38.820 / 38.946|35.000 / 36.667|0.000 / 0.000|
|defaults/nominal_no_noise|0.715 / 0.370|0.000 / 0.000|1.732 / 0.000|38.820 / 36.373|35.417 / 35.000|0.000 / 0.000|
|defaults/vision_gap|0.862 / 0.858|0.000 / 0.000|1.732 / 6.072|40.577 / 38.946|35.613 / 36.752|0.000 / 0.000|
|saved/corner_reversal_finish|1.356 / 0.082|1.000 / 0.000|3.819 / 0.000|21.817 / 12.649|52.807 / 33.983|0.000 / 0.000|
|saved/drag|1.905 / 0.367|0.000 / 0.000|9.473 / 0.000|28.728 / 21.784|52.639 / 36.111|0.000 / 0.000|
|saved/friction|1.922 / 0.457|0.167 / 0.000|10.275 / 0.000|34.990 / 32.539|50.139 / 36.250|0.000 / 0.000|
|saved/friction_drag|1.874 / 0.152|0.000 / 0.000|8.221 / 0.000|38.791 / 14.148|55.139 / 34.861|0.000 / 0.000|
|saved/nominal|2.294 / 0.732|1.138 / 0.000|10.043 / 0.000|41.250 / 41.947|56.528 / 36.806|0.000 / 0.000|
|saved/nominal_no_noise|2.044 / 0.713|0.000 / 0.000|11.105 / 0.000|44.091 / 44.091|50.417 / 36.667|0.000 / 0.000|
|saved/vision_gap|2.340 / 0.734|1.276 / 0.000|9.179 / 0.764|41.808 / 37.647|60.656 / 38.203|0.333 / 0.333|

## 闭环 R-L/力/性能：ON / OFF

|profile/case|I RMS A|采样 dI RMS A/s|瞬态 dI peak A/s|F_target_x µN|F_est_x µN|F_est RMS µN|solver mean/p95 ms|
|---|---|---|---|---|---|---|---|
|defaults/corner_reversal_finish|0.741 / 0.741|0.839 / 0.882|10.264 / 10.385|-5.997 / -5.911|-6.947 / -6.918|7.591 / 7.905|69.670 / 67.335 ; 83.088 / 79.667|
|defaults/drag|0.763 / 0.763|0.641 / 0.660|10.264 / 10.385|14.131 / 14.015|14.173 / 13.883|10.307 / 10.327|65.759 / 67.160 ; 73.221 / 77.211|
|defaults/friction|0.761 / 0.762|0.754 / 0.762|10.265 / 10.385|13.022 / 12.999|13.022 / 12.854|9.633 / 9.675|67.861 / 65.318 ; 81.236 / 75.786|
|defaults/friction_drag|0.762 / 0.761|0.640 / 0.656|10.264 / 10.385|18.054 / 17.958|18.210 / 18.019|12.984 / 13.092|68.898 / 71.818 ; 83.554 / 84.291|
|defaults/nominal|0.759 / 0.753|0.642 / 0.947|10.264 / 10.388|9.270 / 9.241|9.087 / 8.877|7.085 / 7.151|77.260 / 70.813 ; 99.065 / 84.638|
|defaults/nominal_no_noise|0.759 / 0.758|0.646 / 0.661|10.264 / 10.385|9.287 / 9.249|9.056 / 8.885|7.138 / 7.211|66.917 / 69.081 ; 76.771 / 84.335|
|defaults/vision_gap|0.757 / 0.752|0.756 / 0.950|10.264 / 10.388|9.220 / 9.266|9.017 / 9.000|7.103 / 7.262|67.413 / 68.025 ; 81.031 / 80.885|
|saved/corner_reversal_finish|0.265 / 0.263|0.481 / 0.604|6.572 / 8.861|-5.942 / -5.306|-5.991 / -5.329|9.749 / 8.997|2.156 / 2.126 ; 2.998 / 2.605|
|saved/drag|0.284 / 0.286|0.827 / 1.069|7.185 / 9.773|14.156 / 14.132|14.270 / 14.217|9.820 / 9.799|8.881 / 8.471 ; 72.253 / 67.325|
|saved/friction|0.284 / 0.286|0.888 / 1.067|8.491 / 9.898|13.121 / 13.169|13.234 / 13.245|9.401 / 9.318|8.676 / 8.591 ; 70.742 / 71.276|
|saved/friction_drag|0.288 / 0.286|0.837 / 0.905|9.160 / 8.861|17.991 / 17.984|18.132 / 18.086|12.190 / 12.214|9.548 / 9.160 ; 80.600 / 71.604|
|saved/nominal|0.288 / 0.288|1.022 / 1.181|10.041 / 10.221|9.385 / 9.387|9.436 / 9.460|8.100 / 7.486|8.430 / 9.220 ; 74.163 / 76.989|
|saved/nominal_no_noise|0.300 / 0.300|1.098 / 1.211|10.221 / 10.170|9.423 / 9.417|9.531 / 9.500|6.716 / 6.713|6.938 / 7.316 ; 63.189 / 66.442|
|saved/vision_gap|0.288 / 0.288|0.988 / 1.194|10.117 / 10.046|9.368 / 9.372|9.438 / 9.451|9.595 / 8.571|7.229 / 6.843 ; 65.923 / 63.549|

## 预登记失败组

|profile/case|失败判据|Δ RMS mm|Δ max mm|Δ overshoot mm|三个 paired ΔRMS 范围|
|---|---|---|---|---|---|
|defaults/corner_reversal_finish|tracking_rms,max_tracking_error|0.090743|0.194803|-0.006403|[-0.003378203139275948, 0.14813906802429921]|
|defaults/nominal|tracking_rms,overshoot|0.082337|0.038558|0.103021|[0.025475470488699525, 0.17777174134379087]|
|defaults/nominal_no_noise|tracking_rms,overshoot|0.086316|0.028783|0.119636|[0.08631566730292867, 0.08631566730292867]|
|defaults/vision_gap|tracking_rms,max_tracking_error,overshoot|0.109055|0.101904|0.116477|[0.0489600794050048, 0.20316079618080407]|
|saved/corner_reversal_finish|mean_speed|-0.030415|-0.105980|0.003803|[-0.042629855516950144, -0.017468375720444074]|

## 150→300 Hz 物理积分复核

|profile/mode|RMS150|RMS300|ΔRMS mm|Δ速度 mm/s|
|---|---|---|---|---|
|defaults/direct|0.443789|0.442683|-0.001107|-0.005339|
|defaults/legacy_three_frame|0.418314|0.419266|0.000952|-0.000099|
|saved/direct|0.192029|0.198773|0.006744|-0.007894|
|saved/legacy_three_frame|0.316207|0.331986|0.015779|0.006732|

## 固定位置实际非线性模型 R-L

|case|I RMS A ON/OFF|dI RMS A/s ON/OFF|瞬态峰值 ON/OFF|F RMS µN ON/OFF|ON/OFF I 差 RMS A|F 差 RMS µN|
|---|---|---|---|---|---|---|
|step6|0.091002 / 0.096698|0.453345 / 0.677409|2.281109 / 5.328005|20.340974 / 21.614017|0.018320|4.094860|
|step18|0.273007 / 0.281663|1.360036 / 1.594379|6.843327 / 9.838413|61.022923 / 62.957614|0.027480|6.142291|
|step27|0.409511 / 0.409511|2.040055 / 2.040055|10.264991 / 10.264991|91.534384 / 91.534384|0.000000|0.000000|
|alternating3|0.032479 / 0.049871|0.993392 / 1.624857|2.267044 / 5.295154|0.015781 / 0.024231|0.043183|0.012950|
|mixed|0.127012 / 0.137414|2.598670 / 2.836409|10.264991 / 10.264991|28.593156 / 31.065760|0.037693|12.103906|


## 默认决策与未验证事项

5/14个profile/case组违反预登记门槛，明细见上表。config默认nominal/no-noise/vision-gap与转角表现给出保留ON的直接软件依据；保存GUI的straight nominal RMS与lag改善，并不推翻另一套合法配置的劣化。保存GUI转角RMS也改善，其沿路径尾段均速1.451→1.282更接近1mm/s，但绝对变化.169超过预登记的.10“保持一致”门槛；不将这个第五项表述成OFF危险。

本轮84组无发散，最大正常Δcmd均≤9，MPC水平力饱和率均0；部分默认配置command上限事件接近98%，不是force饱和。波动指标变化因配置/序列而异，不能从有限软件模拟证明所有硬件稳定性或不存在任何振荡。150→300Hz nominal RMS差最大.015779mm，两种mode的该seed结论方向保持；这只是四个solution verification样本，不是模型的实物validation。

最终config/library默认legacy_three_frame，GUI可保存direct，下次追踪生效。保留ON用于已有行为复现、降低部分command高频、比较不同工作条件；direct作为明确可审查的备选，无重调参数来使它胜出。

实机A/B仍需测真实worker/QTimer相位与jitter、求解/发送延迟、校准CURT后的电流/电压及PI饱和、R-L模型偏差、路径误差/反转/停止滑行、机械振动与热条件。首次比较继续固定现有控制/物理参数，不能用本轮软件表代替这些测量。

## 回归、失败记录与保护

新增专项87/87，成对闭环/积分复核88/88，总175；Ruff/Black/mypy全工程43文件通过。历史相关回归各项如下，完整原始记录见regressions-initial/final、isolated-control-log-result、quality-initial/final。

|suite|结果|
|---|---|
|frames|PASS (exit0)：.........................................................                [100%] ; 57 passed in 7.45s|
|mpc|PASS (exit0)：......                                                                   [100%] ; 6 passed, 12 deselected in 1.86s|
|solver-protocol-safety|PASS (exit0)：---------------------------------------------------------------- ; 总计 25 项, 通过 25, 失败 0|
|shared-control|PASS (exit0)：---------------------------------------------------------------- ; 总计 6 项, 通过 6, 失败 0|
|bead-simulator|PASS (exit0)：---------------------------------------------------------------- ; 总计 8 项, 通过 8, 失败 0|
|curt|PASS (exit0)：............................................................             [100%] ; 132 passed in 2.96s|
|stop-modes|PASS (exit0)：.....................                                                    [100%] ; 21 passed in 6.57s|
|error-paths|PASS (exit0)：-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html ; 46 passed, 10 warnings in 7.36s|
|serial-safety|PASS (exit0)：..............................................                           [100%] ; 46 passed in 11.86s|
|firmware-watchdog-integration|PASS (exit0)： ; OK|
|firmware-watchdog-c|PASS (exit0)：PASS: trace ; 29/29 production control scenarios PASS|
|freshness|PASS (exit0)：...... ; 25 passed in 28.63s|
|gui-smoke|PASS (exit0)：鼠标绘制路径点数: 15 ; SMOKE OK|
|control-log|PASS (exit0)：... ; 25 passed in 6.29s|
|path-progress|PASS (exit0)：.......................... ; 34 passed in 7.03s|
|eso-mpc-unit-gui|PASS (exit0)：..........................................                               [100%] ; 42 passed in 14.38s|
|closed-loop|PASS (exit0)：.................................                                        [100%] ; 33 passed in 123.34s (0:02:03)|
|l0-checkpoint-replay|PASS (exit0)：. ; 2 passed in 73.56s (0:01:13)|
|multirate|PASS (exit0)：---------------------------------------------------------------- ; 总计 18 项, 通过 18, 失败 0|
|target-box|PASS (exit0)：. ; 33 passed in 78.51s (0:01:18)|
|target-box-ab|PASS (exit0)：. ; 16 passed in 37.85s|

首轮control-log为23 pass/2 fail：新增mode字符串漏入旧类型检查；另一个原宣称确定性的双窗口对照只冻结time.time，仍依赖真实monotonic freshness，串行求解时可触发0.15s检查。已补合法mode枚举检查，并使用原clock注入接口使对照两边共享时间；真实QTimer/worker测试和逐位/开销断言全部保留。原始失败stdout/stderr及初轮JSON保存，隔离全suite25/25复核；未放宽生产/测试门槛。首轮Black仅要求新branch排版，修正后通过。multirate原18/18首测通过，本轮未出现其60ms门槛失败。

保护证明：151固件文件与11个受保护PC文件逐字节相同；整个SharedState/ControlWorker、update_est、apply_slew_cmd原source相同；移除仅有的mode选择后legacy executor AST相同；GUI其余116方法及模块安全函数AST相同；所有旧config值相同；control_log除一个CONTROL_FIELDS字符串外整棵AST相同。未修改solver/observer/MPC/weights/Fmax/c/RL/freshness/frame/path/serial/firmware/CURT/offset/z/Fz/PI。

只提交模式/config/GUI/一个log字段、专项及测试框架观察扩展、两项日志harness修正、文档。独立checkpoint message=`refactor: make executor interpolation explicit`；Git最终diff/status/checkpoint原文另存artifacts，不提交artifacts、不push。

## 软件与流程来源

Python3.14.4、NumPy2.4.4、SciPy1.17.1、Matplotlib3.10.9、pytest9.1.0、PySide6 6.11.2、OpenCV4.13.0.92；BLAS/OMP每进程1线程，完整源码hash/预登记hash见provenance.json。图形为Matplotlib OO独立PNG300dpi/SVG/PDF，`interpolation-study.*`；统计来自原始trace。

绘图流程采用本地matplotlib技能，按其引用要求列入：Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026), [Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents](https://doi.org/10.48550/arXiv.2609.00065)。2026-10-06核验arXiv当前版本v2，链接不固定版本；此引用仅说明流程来源，控制结论来自本项目代码和试验。

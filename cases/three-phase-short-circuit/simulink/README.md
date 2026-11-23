# Simulink 搭建步骤 · three-phase-short-circuit

参数和案例卡第 3 节一致。本目录只放步骤，不提交二进制 `.slx`。先在 MATLAB 命令行定义参数（搭法 A、B 都用得到）：

```matlab
UN = 10.5e3; f = 50; w = 2*pi*f; Em = sqrt(2)*UN/sqrt(3);
X = 0.5; Ta = 0.05; L = X/w; R = L/Ta;          % R = 0.031831 Ω，L = 1.5915 mH
Rl = 13.6; Ll = 6.6/w;                          % 负荷 Ll = 21.0085 mH
tf = 0.04;                                      % 短路时刻（先跑 2 个周期）
alpha = deg2rad(177.98);                        % 工况 A；B 用 87.983，C 用 176.357（且去掉负荷）
ph = alpha + deg2rad([0; -120; 120]);           % 三相短路瞬间电压相位
z0 = complex(R + Rl, w*(L + Ll));
i0 = Em/abs(z0) * sin(ph - w*tf - angle(z0));   % t = 0 稳态初值（工况 C 改成 zeros(3,1)）
Ipm = Em/abs(complex(R, X));                    % 17.1118 kA
```

## 搭法 A：基础库（与 Python / MATLAB 脚本同一方程）

三相互不耦合，用向量信号一次画三相（所有模块都能处理 3×1 向量）：

```text
Sine Wave(3 相) ─►(+)Sum(−)─► Divide(÷ l(t)) ─► Integrator(i0) ─┬─► Scope / To Workspace
       e                                                        │ i
                    ▲                                           │
                    └──── Product( r(t) × i ) ◄─────────────────┘
r(t) = Step(tf: R+Rl → R)        l(t) = Step(tf: L+Ll → L)
```

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Sine Wave | Simulink / Sources | Amplitude = `Em`，Frequency = `w`（rad/s），Phase = `ph - w*tf`（**rad**，3×1 向量），Sample time = `0` |
| Sum | Simulink / Math Operations | 符号 `+-` |
| Step（r） | Simulink / Sources | Step time = `tf`，Initial = `R+Rl`，Final = `R` |
| Step（l） | Simulink / Sources | Step time = `tf`，Initial = `L+Ll`，Final = `L` |
| Product ×2 | Simulink / Math Operations | 一个 `**`（r × i），一个 `*/`（(e − r·i) ÷ l） |
| Integrator | Simulink / Continuous | Initial condition = `i0` |
| To Workspace | Simulink / Sinks | 变量 `i_sim`，Save format = Timeseries |

工况 C（空载）最简单的做法：把 Sine Wave 输出经 `Switch`（控制端接 Clock，阈值 `tf`，`u2 >= Threshold`）送进 Sum，`tf` 之前送 0；Integrator 初值 `zeros(3,1)`。`tf` 之前电压和电流都是 0，r、l 取什么值都不影响。

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `0.34`（短路后 15 个周期） |
| Solver | `ode4`（Fixed-step），Fixed-step size `1e-5`；或 `ode45` 变步长 + Max step `1e-5` |

Step 块在 `tf` 处是跳变，变步长求解器会自动在该点过零检测，定步长 `1e-5` 刚好落在 0.04 s 上，两种都能与脚本一致。

## 搭法 B：Simscape Electrical（Specialized Power Systems）

| 模块 | 参数 |
|------|------|
| powergui | Simulation type = Continuous |
| Three-Phase Source | Phase-to-phase voltage = `UN`，Phase angle of phase A = `rad2deg(alpha)`（deg），Frequency = `50`，Internal connection = Yg，取消 *Specify impedance using short-circuit level*，Source resistance = `0`，Source inductance = `0`（内阻抗放在下一个支路，便于读数） |
| Three-Phase Series RLC Branch（线路） | Branch type = RL，R = `R`，L = `L` |
| Three-Phase V-I Measurement | 放在线路之后、短路点之前，测线路电流 |
| Three-Phase Fault | Phase A/B/C Fault 全勾，Ground Fault 勾；Transition times = `tf`，初始状态 0（断开）；Fault resistance = `1e-4` Ω，Ground resistance = `1e-4` Ω；Snubber Rs = `1e6` Ω、Cs = `inf` |
| Three-Phase Series RLC Load / Branch（负荷） | Branch type = RL，R = `Rl`，L = `Ll`，Yg 接地 |

说明：
- SPS 交流源按 `sin` 定义，相角单位是**度**；`tf = 0.04` 是整 2 个周期，所以短路瞬间 a 相相位就是所填相角。
- SPS 从 powergui 稳态初始化，等价于脚本的「从稳态起算」。
- 故障电阻 1e-4 Ω 远小于 R = 0.0318 Ω，对峰值的影响在 0.5% 以内。

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `0.34` |
| Solver | `ode23tb`（Variable-step），Max step size `1e-5`，Relative tolerance `1e-4` |

## 读数（对照案例卡第 4 节，时间从短路时刻算起）

| 工况 | 应看到 |
|------|--------|
| A：α = 177.98°，带负荷 | a 相首峰约 −30.94 kA（约 9.75 ms），b、c 相约 +24.50 / +23.57 kA；a 相曲线在约 150 ms 后基本对称 |
| B：α = 87.98°，带负荷 | a 相一开始就对称（峰值 17.11 kA = Ipm），但 c 相 −29.49 kA、b 相 +28.69 kA |
| C：α = 176.36°，空载 | a 相首峰约 −31.15 kA，Kim ≈ 1.820 |

搭法 A 应与脚本一致到小数点后 3~4 位。搭法 B 多了故障电阻和缓冲电路，数值会略有出入（本仓库自动化环境没有 MATLAB / Simulink，搭法 B 未实测），但「哪一相最大、约几倍 Ipm」应相同。

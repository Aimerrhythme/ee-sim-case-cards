# Simulink 搭建步骤 · pll-grid-sync

参数和案例卡第 3 节一致。本目录只放步骤，不提交二进制 `.slx`。先在 MATLAB 命令行定义参数（搭法 A、B 都用得到）：

```matlab
Vm = sqrt(2)*380/sqrt(3);  f0 = 50;  w0 = 2*pi*f0;      % Vm = 310.269 V
fn = 20;  zeta = 1/sqrt(2);  wn = 2*pi*fn;
Kp = 2*zeta*wn/Vm;   Ki = wn^2/Vm;                      % 0.57278、50.8958
tev = 0.1;                                              % 扰动时刻
jump = deg2rad(30);  df = 0;  ramp = 0;  tramp = 0.1;   % 工况 A；B：jump = 0, df = -0.5；C：jump = 0, ramp = -5
sag = 1;  k2 = 0;  k5 = 0;                              % D：jump = deg2rad(30), sag = 0.5；E：k2 = 0.1, k5 = 0.05
```

## 搭法 A：基础库（与 Python / MATLAB 脚本同一方程）

```text
 电网相位 θg：Constant(w0) + Step(df·2π) + [Ramp(ramp·2π) − Ramp 延迟 tramp] ─► Integrator ─(+)─ Step(jump) ─► θg
 三相电压：θg ─► Fcn×3 ─► Mux ─► × 幅值 Step(1 → sag) ─► vabc
 Clarke：vabc ─► Gain（矩阵 Tαβ，Matrix(K*u)）─► [vα; vβ]
 Park：  vq = −vα·sin θ̂ + vβ·cos θ̂，vd = vα·cos θ̂ + vβ·sin θ̂
 PI：    vq ─┬─► Gain(Kp) ──────────────(+)
             └─► Integrator ─► Gain(Ki) ─(+)─(+ Constant w0)─► ω̂ ─► Integrator(Wrap 0~2π) ─► θ̂ ─┐
                                                              └─► Gain(1/(2π)) ─► f̂                │
                       θ̂ 反馈到 Park ◄──────────────────────────────────────────────────────────────┘
```

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Constant（w0） | Simulink / Sources | `w0` |
| Step（频率阶跃） | Simulink / Sources | Step time = `tev`，Initial = 0，Final = `2*pi*df` |
| Ramp ×2（频率斜坡） | Simulink / Sources | ① Slope = `2*pi*ramp`，Start time = `tev`；② Slope = `-2*pi*ramp`，Start time = `tev+tramp`（两者相加 = 限时斜坡） |
| Sum + Integrator（θg） | Simulink / Math Operations、Continuous | 对 ωg 积分，初值 0 |
| Step（相位跳变） | Simulink / Sources | Step time = `tev`，Final = `jump`（rad），与积分器输出相加得 θg |
| Fcn ×3 | Simulink / User-Defined Functions | `cos(u)+k2*cos(u)+k5*cos(5*u)`、`cos(u-2*pi/3)+k2*cos(u+2*pi/3)+k5*cos(5*(u-2*pi/3))`、`cos(u+2*pi/3)+k2*cos(u-2*pi/3)+k5*cos(5*(u+2*pi/3))`（k2、k5 直接写数值或用 MATLAB Function） |
| Step（幅值） + Product | Simulink / Sources、Math Operations | Initial = `Vm`，Final = `Vm*sag`，Step time = `tev`；与 Mux 后的 3×1 向量相乘 |
| Gain（Clarke） | Simulink / Math Operations | Gain = `(2/3)*[1 -1/2 -1/2; 0 sqrt(3)/2 -sqrt(3)/2]`，Multiplication = `Matrix(K*u)` |
| Trigonometric Function ×2 | Simulink / Math Operations | `sin`、`cos`，输入 θ̂ |
| Product ×4 + Sum ×2 | Simulink / Math Operations | 组成 vd、vq（Sum 符号 vq：`-+`，vd：`++`） |
| （可选）幅值归一化 | Math Operations：`Sqrt`、`Divide` | `vq*Vm/sqrt(vα²+vβ²)`，对应工况 D 的「归一化」 |
| Gain（Kp）、Gain（Ki）、Integrator（∫vq） | Math Operations、Continuous | `Kp`、`Ki`；积分初值 0 |
| Sum（+ w0）+ Integrator（θ̂） | Continuous | 积分初值 0；勾选 **Wrap state**，Upper = `2*pi`，Lower = `0`（R2016a 以上有此选项；没有时用 `mod`，并在 Park 中照常用 sin/cos） |
| Scope / To Workspace | Simulink / Sinks | 记录 θg、θ̂、vd、vq、f̂；相位误差用 `angle(exp(1j*(θg − θ̂)))` 事后计算 |

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `0.4` |
| Solver | `ode4`（Fixed-step），Fixed-step size `1e-5`（与脚本 RK4 步长相同） |

Step 块切换时刻 0.1 s 是步长的整数倍，定步长 `ode4` 与脚本一致到小数点后 2~3 位。若改用 `ode45` 变步长，设 Max step `1e-5`。

## 搭法 B：Simscape Electrical（Specialized Power Systems）

| 模块 | 参数 |
|------|------|
| powergui | Simulation type = Discrete，Sample time `Ts = 1e-5` |
| Three-Phase Programmable Voltage Source | Positive-sequence `[380  0  50]`（V rms 线电压、相角 deg、Hz）；Time variation of = `Phase` / `Frequency` / `Amplitude`，Type = Step；Step magnitude = 30（°）/ −0.5（Hz）/ 0.5（pu），Variation timing = `[0.1  0.4]`；勾选 *Fundamental and/or Harmonic generation* 可加 5 次谐波（Seq = 2 负序）；负序可在 Positive-sequence 外再叠一个反相序源或用不平衡设置 |
| Three-Phase V-I Measurement | Voltage measurement = phase-to-ground，*in pu* 不勾（或勾选后 Vm 取 1） |
| PLL (3ph) | Control & Measurements / PLL 库。Minimum frequency = 45 Hz；Initial inputs = `[0  50]`；Regulator gains = `[Kp*Vm  Ki*Vm  0]` = `[177.7  15791  0]`（模块内部先把输入归一化到 1 pu，所以增益要乘 Vm）；**Automatic gain control** 勾选 = 本卡归一化版；Maximum rate of change of frequency 调大（如 `1e4` Hz/s）以免限幅掩盖动态 |

说明（与搭法 A 的差异）：
- PLL (3ph) 的鉴相器后有一个按实测频率计算的**一周期滑动平均**，能滤掉 100 / 300 Hz 脉动，但带来约半个周期的延时，相位跳变后的锁定时间会比本卡长、超调也不同。
- 模块默认的频率变化率限幅（几 Hz/s 到十几 Hz/s 量级）会让工况 A 的频率尖峰被削平、锁定明显变慢。
- 本仓库自动化环境没有 MATLAB / Simulink，搭法 B 未实测；只需定性对照：频率阶跃无静差、斜坡有小静差、归一化后暂降不影响动态、不平衡时脉动被滑动平均压低。

## 读数（对照案例卡第 4 节，时间从 t_ev = 0.1 s 算起）

| 工况 | 应看到（搭法 A） |
|------|--------|
| A：+30° 跳变 | vq 跳到约 155 V 后回零；相位误差 32.73 ms 进入 ±2°，反向最大约 −6.2°（20.77%）；f̂ 冲到约 64.13 Hz，47.69 ms 回到 ±0.1 Hz |
| B：50 → 49.5 Hz | 相位误差最大 −0.653°，最终 0；f̂ 20.04 ms 进入 49.5 ± 0.1 Hz，下冲 0.104 Hz |
| C：−5 Hz/s 斜坡 100 ms | 斜坡期间相位误差约 −0.114°，结束后回零 |
| D：0.5 pu + 30° | 不归一化：96.25 ms、超调 29.82%；归一化：与 A 相同 |
| E：10% 负序 + 5% 5 次 | vq 峰峰 77.56 V（100 Hz + 300 Hz），相位误差峰峰 3.795°，f̂ 峰峰 6.657 Hz |

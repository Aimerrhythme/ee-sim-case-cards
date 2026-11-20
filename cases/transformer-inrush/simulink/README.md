# Simulink 搭建步骤 · transformer-inrush

参数和案例卡第 3 节一致。本目录只放步骤，不提交二进制 `.slx`。先在 MATLAB 命令行定义参数（搭法 A、B 都用得到）：

```matlab
SN = 10e3; UN = 220; f = 50; w = 2*pi*f; Zb = UN^2/SN; IN = SN/UN;
Vm = sqrt(2)*UN; lamN = Vm/w;
R = 0.01*Zb; Ls = 0.02*Zb/w; Lm = 50*Zb/w; Lair = 0.13*Zb/w;
lamk = 1.2*lamN; ik = lamk/Lm; psik = lamk + Ls*ik;
lr = 0.8*lamN;                 % 剩磁，工况 B/D 改成 0，C 改成 -0.8*lamN
alpha = 0;                     % rad：0 / pi/2 / acos(-0.8)
psi0 = lr*(1 + Ls/Lm);
psiMax = 4*lamN;               % 查表范围，够覆盖 2.8 pu
iMax = ik + (psiMax - psik)/(Lair + Ls);
psiBp = [-psiMax, -psik, 0, psik, psiMax];
iTab  = [-iMax,   -ik,   0, ik,   iMax];
```

## 搭法 A：基础库（与 Python / MATLAB 脚本同一方程）

信号流：

```text
Sine Wave ─►(+)Sum(−)─► Integrator(ψ0) ─► 1-D Lookup Table(ψ→i) ─┬─► Scope / To Workspace
   v                          ψ                                  │ i
             ▲                                                   │
             └────────────── Gain R ◄────────────────────────────┘
```

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Sine Wave | Simulink / Sources | Amplitude = `Vm`，Frequency = `w`（rad/s），Phase = `alpha`（**rad**），Sample time = `0` |
| Sum | Simulink / Math Operations | 符号 `+-` |
| Integrator | Simulink / Continuous | Initial condition = `psi0` |
| 1-D Lookup Table | Simulink / Lookup Tables | Breakpoints = `psiBp`，Table data = `iTab`，Interpolation = Linear，Extrapolation = Linear |
| Gain | Simulink / Math Operations | `R` |
| To Workspace | Simulink / Sinks | 变量 `i_sim`，Save format = Timeseries |
| Scope | Simulink / Sinks | 看 i |

查表在 `±ψk` 处有拐点，中间一段斜率 `1/(Lm+Ls)`，两边斜率 `1/(Lair+Ls)`，就是案例卡里的 i(ψ)。

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `2` |
| Type | Fixed-step（与脚本逐点对比）或 Variable-step |
| Solver | `ode4`（Fixed-step size `1e-5`）或 `ode45`（Max step `1e-5`，Rel tol `1e-8`） |

二次谐波：把 `i_sim` 按 2000 点一行 reshape 后逐行 `fft`，取第 2、3 个点，和 MATLAB 脚本同法。

## 搭法 B：Simscape Electrical（Specialized Power Systems）

库路径随版本不同：旧版在 *Specialized Power Systems / Fundamental Blocks / Elements*，新版在 *Specialized Power Systems / Passives*、*Sources*、*Switches and Breakers*、*Sensors and Measurements*；找不到时在 Library Browser 里直接搜块名。

| 模块 | 参数 |
|------|------|
| powergui | Simulation type = Continuous |
| AC Voltage Source | Peak amplitude = `Vm`，Phase = `0` deg，Frequency = `50` |
| Breaker | Initial status = `0`（断开），Switching times = `[t_close]`，缓冲 Rs = `1e6` Ω、Cs = `inf`（纯电阻缓冲，泄漏电流约 0.2 mA，对剩磁影响可忽略） |
| Saturable Transformer | Units = pu；[Pn fn] = `[10e3 50]`；Winding 1 [V1 R1 L1] = `[220 0.01 0.02]`；Winding 2 [V2 R2 L2] = `[110 0.01 0.02]`（二次侧开路，取值不影响结果）；Rm = `1e4` pu；Saturation characteristic = `[0 0; 0.024 1.2; 13.8702 3.0]`；勾 *Specify initial flux*，phi0 = `0.8`（B/D 填 0，C 填 −0.8）；不勾 *Simulate hysteresis* |
| Current Measurement | 串在 Breaker 与一次绕组之间 |
| 二次侧 | 两端悬空即可；若报浮空节点，接一个 `1e6` Ω 电阻 |

饱和特性是按块帮助的 pu 基值换算的（磁链基值 = 额定磁链峰值 λN，电流基值 = 额定电流峰值）：拐点 `ik/(√2·IN) = 0.024`、`λk/λN = 1.2`；第三点在 3.0 pu 处，`0.024 + 1.8/0.13 = 13.8702`，即饱和段斜率等于空心电抗 0.13 pu。不同版本的基值定义如有差别，以所装版本的块帮助为准。

合闸角用断路器动作时刻设定（电源相位保持 0，先空跑两个周期）：

| 工况 | α | t_close |
|------|---|---------|
| A / B / C | 0° | `0.04` |
| D / E | 90° | `0.045` |
| F（选相） | 143.13° | `0.04795` |

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `2.04` |
| Solver | `ode23tb`（Variable-step） |
| Max step size | `1e-5` |
| Relative tolerance | `1e-4` |

## 读数（对照案例卡第 4 节，时间从合闸时刻算起）

| 工况 | 应看到 |
|------|--------|
| A：α = 0°，phi0 = 0.8 | 首峰约 634 A（约 9.7 ms），首周期 I2/I1 约 17.8%，约 0.56 s 后峰值低于 64.3 A |
| B：α = 0°，phi0 = 0 | 首峰约 325 A，首周期 I2/I1 约 55% |
| C：α = 0°，phi0 = −0.8 | 无涌流，峰值约 1.5 A |
| D：α = 90°，phi0 = 0 | 无涌流，峰值约 1.29 A（空载电流） |
| E：α = 90°，phi0 = 0.8 | 首峰约 249 A |
| F：α = 143.13°，phi0 = 0.8 | 无涌流，峰值约 1.3 A |

搭法 A 应与脚本一致到小数点后 2~3 位。搭法 B 多了 Rm、二次绕组参数和断路器缓冲，数值会略有出入（本仓库自动化环境没有 MATLAB / Simulink，搭法 B 未实测），但「谁有涌流、谁最大」的排序应相同。

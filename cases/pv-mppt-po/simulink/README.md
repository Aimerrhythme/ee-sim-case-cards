# Simulink 搭建步骤 · pv-mppt-po

参数和案例卡第 3 节一致。本目录只放步骤，不提交二进制 `.slx`。先在 MATLAB 命令行定义参数：

```matlab
% PV 阵列：3 块 × 60 片串联（教学示例参数）
Ns = 60; Nser = 3; n = 1.3; Isc = 8.5; VocMod = 37.5; RsMod = 0.3; RshMod = 250;
Vt = 1.380649e-23*298.15/1.602176634e-19;          % 25.693 mV
a  = Nser*Ns*n*Vt;                                  % 6.0121 V
Rs = Nser*RsMod;  Rsh = Nser*RshMod;                % 0.9 Ω、750 Ω
I0 = Isc/(exp(Nser*VocMod/a) - 1);                  % 6.3495e-8 A
% Boost 与母线
Vbus = 200; L = 3e-3; Cin = 47e-6; RL = 0.2; fs = 20e3;
% MPPT
Tmppt = 0.02; dD = 0.005; D0 = 0.45; Dmin = 0.05; Dmax = 0.90;   % 对比时 dD 改 0.002 / 0.02
Tstep = 1.5; G1 = 1000; G2 = 600;
```

## 总体结构

```text
 辐照 G：Step(1.5 s, 1000 → 600) ─────────────┐
                                             ▼
             ┌──────────── PV Array（Ir 输入）──+── Cin ──( L )──┬──|>|──┬── + 200 V（DC Voltage Source / Battery）
             │                                                  [S]      │
             │                                                   │       │
             └──────────────────────────────────────────────────┴───────┴── −
  Voltage Measurement → Vpv ─┐
  Current Measurement → Ipv ─┤─► Zero-Order Hold(Tmppt) ─► MATLAB Function「P&O」─► D ─► PWM Generator (DC-DC) ─► S 的 g 端
                             └─► Product → Ppv ─► Scope
```

## 搭法 A：Simscape Electrical（Specialized Power Systems，开关模型）

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| powergui | Specialized Power Systems | Simulation type = Discrete，Sample time `Ts = 1e-6`（fs = 20 kHz 时每周期 50 点） |
| PV Array | Specialized Power Systems（在库浏览器搜索 `PV Array`，不同版本归在 Sources 或 Renewables 子库） | Module = **User-defined**；Maximum Power = 696.17/3 = 232.06 W；Parallel strings = 1，Series-connected modules per string = 3；Cells per module = 60，Voc = 37.5 V，Isc = 8.5 A；Vmp、Imp 填单块值（Vmp = 89.158/3 = 29.72 V，Imp = 7.808 A），模块会自行拟合 I0、n、Rs、Rsh——拟合结果与本卡略有差异，对照时以模块显示的 MPP 为准；输入 Ir 接辐照，T 接 Constant 25 |
| Step（辐照） | Simulink / Sources | Step time = `Tstep`，Initial = `G1`，Final = `G2`；斜坡工况用 `Repeating Sequence Interpolated`（时间 `[0 1.5 2.5 3.5]`、值 `[600 600 1000 1000]`） |
| Capacitor（Cin） | Series RLC Branch，Branch type = C | `Cin`，初始电压可设 112（≈ Voc） |
| Series RLC Branch（L + RL） | Elements | Branch type = RL，`R = RL`，`L = L` |
| IGBT/Diode 或 Mosfet | Power Electronics | 开关 S，默认参数；栅极接 PWM |
| Diode | Power Electronics | 默认参数（Ron 0.001 Ω、Vf 0） |
| DC Voltage Source（或 Battery） | Electrical Sources | `Vbus`；Battery 时标称电压 200 V、初始 SOC 50% |
| PWM Generator (DC-DC) | Control / Pulse & Signal Generators | Switching frequency = `fs`，Sample time = `Ts`；输入 D |
| Voltage / Current Measurement | Measurements | 测 Vpv（阵列两端）、Ipv（阵列出线） |
| Zero-Order Hold ×2 | Simulink / Discrete | Sample time = `Tmppt`（采样 Vpv、Ipv） |
| （可选）Mean | Control / Measurements | 采样前对 Vpv、Ipv 取一个开关周期平均（`1/fs`），去掉开关纹波 |
| MATLAB Function「P&O」 | Simulink / User-Defined Functions | 见下方代码；在 *Edit Data* 里把块采样时间设成 `Tmppt` |
| Product、Scope / To Workspace | Simulink | 记录 Vpv、Ipv、Ppv = Vpv·Ipv、D、G |

### P&O 的 MATLAB Function

```matlab
function D = po_mppt(V, I)
% 扰动观察法：Boost 接恒压母线，D 增大 → Vpv 降低
dD = 0.005; D0 = 0.45; Dmin = 0.05; Dmax = 0.90;
persistent Vp Pp Dk first
if isempty(Dk)
    Dk = D0; Vp = 0; Pp = 0; first = true;
end
P = V * I;
if first
    Dk = Dk + dD;  first = false;         % 第一次先往降电压方向扰动
else
    dP = P - Pp;  dV = V - Vp;
    if dP ~= 0
        if dP * dV > 0
            Dk = Dk - dD;                  % 在 MPP 左侧 → 升电压
        else
            Dk = Dk + dD;                  % 在 MPP 右侧 → 降电压
        end
    end
end
Dk = min(max(Dk, Dmin), Dmax);
Vp = V;  Pp = P;
D = Dk;
end
```

`persistent` 变量在两次调用之间保持；块每 `Tmppt` 执行一次，相当于脚本里每 400 个 RK4 步执行一次 P&O。要做「dP = 0 也继续扰动」的改进版，把 `if dP ~= 0` 改成按上一次方向继续走。

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `3` |
| powergui | Discrete，`Ts = 1e-6` |
| 运行时间 | 3 s × 1 MHz 步长，普通笔记本约 1~3 分钟；课堂演示可把 `Tstep` 提前、Stop time 缩短 |

## 搭法 B：平均值模型（与 Python / MATLAB 脚本同一方程，运行快）

用 Simscape Electrical 的 `Controlled Voltage Source` 代替开关：Boost 开关节点电压平均值 = `(1 − D)·Vbus`。

```text
 PV Array(+) ── Cin ── (L, RL) ──► Controlled Voltage Source[ (1 − D)·Vbus ] ── PV Array(−)
                                     ▲
 D（MATLAB Function 输出）─► Fcn: (1-u)*Vbus
```

- 电感电流不能为负（二极管阻断）：在 L 支路串一个 `Diode`（Vf = 0、Ron 很小），对应脚本里的 `iL ≥ 0`。
- 若想与脚本数值完全一致，PV 也不要用 PV Array 模块，而用 `Controlled Current Source` + `MATLAB Function` 实现本卡的单二极管方程（以端电压为输入、牛顿迭代求电流），求解器 `ode4` 固定步长 `5e-5`（与脚本 RK4 步长相同）。
- 平均值模型没有开关纹波，P&O 不需要先滤波。

## 读数（对照案例卡第 4 节）

| 项 | 应看到（搭法 B，脚本同参数） | 搭法 A 的差异 |
|----|------|------|
| 1000 W/m² MPP | Vmp = 89.158 V、Pmp = 696.168 W、D_mpp = 0.5464 | PV Array 模块自拟合参数，MPP 略有不同 |
| 启动跟踪到 99% Pmpp | ΔD = 0.002 / 0.005 / 0.02：0.961 / 0.400 / 0.101 s | 接近 |
| 稳态功率峰峰（1000 W/m²） | 0.17 / 1.75 / 26.99 W，D 只有 3 个取值 | 再叠加 20 kHz 开关纹波 |
| 1.5 s 降到 600 W/m² | 功率瞬间跌到新 Pmpp 的 74~80%，约 1.1 ms 回到 99%；Pmp = 412.515 W | 暂态形状受开关纹波影响 |
| 600 → 1000 W/m² 斜坡（ΔD = 0.002，D0 = 0.50） | 斜坡期间全是降电压动作，Vpv 偏离 Vmp 最多 19.80 V | 定性一致 |

说明：本仓库自动化环境没有 MATLAB / Simulink，以上两种搭法均未实测；表中数字来自 Python 脚本（MATLAB 脚本计算部分已用 GNU Octave 核对一致）。上课前请先跑一遍并对照。

# Simulink 搭建步骤 · three-phase-inverter-spwm

物理电路优先用 Simscape Electrical。没有该工具箱时，用后面的基础模块搭「正弦比三角波」和星形 RL。本目录只放步骤，不提交二进制 `.slx`。

参数与案例卡一致：`Vdc = 200 V`，`m = 0.8`，`f1 = 50 Hz`，`mf = 21`（`fc = 1050 Hz`），每相 `R = 10 Ω`、`L = 0.02 H`。理想互补开关，死区为 0。星形中性点悬浮。

线电压基波峰值要对 `138.564 V`，不要对斩波电平 `200 V`。

## 1. Simscape Electrical

新建空白模型，保存为 `three_phase_inverter_spwm.slx`。

一条桥臂是直流正极、上管、交流输出、下管、直流负极。B、C 两相照这张图再做两份。

```text
 Vdc+ ---- S1 ----+---- A ---- 电感 ---- 电阻 ----+---- N（悬空，不接地）
                  |                               |
 Vdc- ---- S4 ----+                               |
                  |                               |
               参考地                          B、C 相同样接过来
                  |
         Solver Configuration
```

门极不在这张电气图里：正弦与三角波比较得到 `q`，`q` 驱动上管，`NOT q` 驱动下管。`v_AB` 经 PS-Simulink Converter 进 Scope。

### 模块与参数

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Solver Configuration | Simscape / Utilities | 默认即可，连到参考地 |
| DC Voltage Source | Simscape / Foundation Library / Electrical / Electrical Sources / DC Voltage Source | Constant voltage = `200` V |
| Ideal Semiconductor Switch ×6 | Simscape / Electrical / Semiconductors & Converters / Ideal Semiconductor Switch | On-state resistance = `0.001` Ω，Off-state conductance = `1e-6` 1/Ω，Threshold voltage = `0.5` V |
| Inductor ×3 | Simscape / Foundation Library / Electrical / Electrical Elements / Inductor | Inductance = `0.02` H，Series resistance = `0`，Initial current = `0` |
| Resistor ×3 | 同上目录 / Resistor | Resistance = `10` Ω |
| Electrical Reference | 同上目录 / Electrical Reference | 全图只有一个参考地，接在直流负极。不要接中性点 N |
| Voltage Sensor | Simscape / Foundation Library / Electrical / Electrical Sensors / Voltage Sensor | 正端接 A，负端接 B，读 `v_AB` |
| Current Sensor | 同上目录 / Current Sensor | 可选。串在 A 相电感支路，箭头指向负载 |
| Sine Wave ×3 | Simulink / Sources / Sine Wave | 见下表 |
| Repeating Sequence | Simulink / Sources / Repeating Sequence | Time values = `[0 1/(2*1050) 1/1050]`，Output values = `[-1 1 -1]` |
| Relational Operator ×3 | Simulink / Logic and Bit Operations | 关系运算符 `>=` |
| Logical Operator ×3 | 同上 | Operator = `NOT` |
| Data Type Conversion ×3 | Simulink / Signal Attributes | Output data type = `double`。接在比较器后面，避免布尔量进不了后级 |
| Simulink-PS Converter ×6 | Simscape / Utilities | Input signal unit = `1`，Filtering and derivatives = `Provide signals` |
| PS-Simulink Converter | Simscape / Utilities | Output signal unit = `V` |
| Scope | Simulink / Sinks | 看 `v_AB` |

三路 Sine Wave（Time based）：

| 相 | Amplitude | Frequency | Phase | Bias | Sample time |
|----|-----------|-----------|-------|------|-------------|
| A | `0.8` | `100*pi` | `0` | `0` | `0` |
| B | `0.8` | `100*pi` | `-2*pi/3` | `0` | `0` |
| C | `0.8` | `100*pi` | `2*pi/3` | `0` | `0` |

Frequency 的单位是 rad/s。填 `50` 会得到大约 126 ms 的基波。Amplitude 是调制比，填 `0.8`，不要填 `80`。

Repeating Sequence 的时间向量是一个载波周期：起点 −1，半周期处 +1，周期结束回到 −1，然后自己重复。不要把 Time values 写成 `[0 0.5 1]`，那是 1 秒一个载波。

比较器：调制波接第一个输入，载波接第二个，运算符 `>=`。结果为真时上管导通。下管接 `NOT`。两路不要接成同一个信号，否则母线经上下管短路。

开关的 Ron 不要填 0。`0.001 Ω` 对这组电流只掉几毫伏，基波仍在 138.6 V 附近。

电感若还有 Parallel conductance，先留着模块默认的很小电导；改成 0 以后若求解器报电路奇异，再改回默认值。串联电阻必须是 0。

### 连线顺序

1. 放下直流源、六只开关、三组 LR、参考地、Solver Configuration、电压传感器。
2. 直流正极同时接到 S1、S3、S5 的上端。这三只是上管。
3. S1 的下端接 A，再接到 S4 的上端。S4 的下端接直流负极。S3/S6 对 B，S5/S2 对 C，同样接。
4. 直流负极接 Electrical Reference，Solver Configuration 也接在这里。
5. A 经电感、电阻接到中性点 N。B、C 同样接到 N。N 上不要再拖一根地线。
6. Voltage Sensor 跨在 A 和 B 上。物理信号输出接 PS-Simulink Converter，再接 Scope。
7. 三路正弦分别和同一路 Repeating Sequence 比较。比较结果转成 double，再接 Simulink-PS Converter，送到对应上管的门极。`NOT` 之后的信号送到同一桥臂的下管。Simulink-PS Converter 不要打开输入滤波，否则门极变成斜坡，脉宽会漂。

### 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.06` |
| Type | Fixed-step |
| Solver | `ode4` |
| Fixed-step size | `1e-6` |

载波周期大约 0.95 ms。步长若到 `1e-4`，一个载波里只有大约 10 步，基波会偏 1 V 这一档。`1e-6` 和案例卡脚本的网格同一量级，FFT 读数应离 138.564 V 不到 0.1 V。

Simscape 若拒绝定步长，改回 **Variable-step / ode23t**，Max step size 仍填 `1e-6`，Relative tolerance = `1e-4`。这种采样不均匀，不要直接拿下面的求和公式；先用第 2 节的方框图对 138.564 V。

### 从波形读基波

示波器上的峰值是 200 V，那是母线电压。基波要用最后一个基波周期做一次傅里叶系数。Fixed-step 的记录是均匀的。To Workspace 用 Array，记下 `vab` 和 `tout`。

```matlab
t0 = 0.04;
m = tout >= t0 & tout < 0.06;   % 不要把 0.06 再算进去
x = vab(m);
N = numel(x);
k = (0:N-1)';
th = 2*pi*k/N;                  % 这一窗正好 20 ms，第 1 次谐波就是 50 Hz
c = (2/N) * sum(x .* cos(th));
s = (2/N) * sum(x .* sin(th));
Vll1_peak = hypot(c, s)
```

`Vll1_peak` 应在 138.56 V 附近。若得到大约 69 V，窗口不是整数个周期，或者把峰值和 `t = 0` 的瞬时值搞混了。若得到大约 200 V，求和时没有乘 `2/N`，读到的是直流附近的斩波电平，不是基波。

电流可选。稳态峰值约 6.77 A，落后 `v_AN` 大约 32°。0–10 ms 的电流还在按 `τ = 2 ms` 爬，不要在第一周读基波。

过调制对照：三路正弦的 Amplitude 改成 `1.2`，基波应落到 191 V 附近，而不是 `1.2/0.8*138.6 ≈ 208 V`。

## 2. 只有基础库时

不安装 Simscape 也能把调制和基波做出来。开关函数 `q` 是 0/1，相电压和线电压是代数式，电流是三条 RL。

```text
v_AO = (2 q_a − 1) · (Vdc/2)
v_AB = v_AO − v_BO
v_AN = (2 v_AO − v_BO − v_CO) / 3
di_A/dt = (v_AN − R i_A) / L
```

`Vdc/2 = 100`，`1/L = 50`，`R = 10`。

```text
Sine(m_a) --+
            |-- >= -- q_a -- Gain(2) -- Sum(+-) -- Gain(100) -- v_AO --+
Repeating --+                              (−)                          |
            ^                                |                          |
            |                             Constant(1)                   |
            |                                                           v
            +------------------ 同一路载波 -------- q_b、q_c         v_AB = v_AO − v_BO

v_AN = (2 v_AO − v_BO − v_CO) / 3
v_AN --> Sum(+-) --> Gain(1/L) --> Integrator --> i_A --> Scope
           (−)
            ^
            +---- Gain(R) <---- i_A
```

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Sine Wave ×3 | Simulink / Sources | 与上一节相同。Amplitude = `0.8`，Frequency = `100*pi` |
| Repeating Sequence | Simulink / Sources | 与上一节相同 |
| Relational Operator ×3 | Simulink / Logic and Bit Operations | `>=` |
| Data Type Conversion ×3 | Simulink / Signal Attributes | `double` |
| Gain `2` ×3 | Simulink / Math Operations | Gain = `2` |
| Constant | Simulink / Sources | `1` |
| Sum ×3 | Simulink / Math Operations | 符号 `+-`。正端是 `2 q`，负端是 `1`，得到 ±1 |
| Gain `Vdc/2` ×3 | Simulink / Math Operations | Gain = `100`，输出 `v_AO`、`v_BO`、`v_CO` |
| Sum | 同上 | 符号 `+-`。正端 `v_AO`，负端 `v_BO`，输出 `v_AB` |
| Sum | 同上 | 用系数或三输入把 `2 v_AO − v_BO − v_CO` 加出来，再 Gain `1/3`，得到 `v_AN` |
| Sum | 同上 | 符号 `+-`。正端 `v_AN`，负端 `R i_A` |
| Gain `R` | 同上 | Gain = `10` |
| Gain `1/L` | 同上 | Gain = `50` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出 `i_A` |
| Scope ×2 | Simulink / Sinks | 一个看 `v_AB`，一个看 `i_A` |

B、C 两相的电流用同样的积分器各做一条，电压用 `v_BN`、`v_CN`。只想看线电压基波时，电流可以先做 A 相。

求解器：**Fixed-step / ode4**，Fixed-step size = `1e-6`，Stop time = `0.06`。

读数用上一节的求和脚本。`Vll1_peak` 应在 138.56 V 附近，`i_A` 的基波峰值约 6.77 A。`v_AO` 在 ±100 V 之间跳，基波是 `80·sin(ωt)`。

Sine Wave 的 Frequency 填 `50` 时，基波周期变成大约 126 ms，20 ms 的窗口里几乎看不到一个完整正弦。要 50 Hz，填 `100*pi`。

## 3. 可选：Specialized Power Systems

仅当已安装该工具箱、课堂又习惯 `powergui` 时使用。FFT 可以直接在 powergui 里读，不必再写求和。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 200 V。负极是直流母线的参考。
3. `Universal Bridge`：Number of bridge arms = `3`，Power Electronic device = `Ideal Switches`。Ron = `0.001` Ω，Snubber resistance `Rs = 1e6`（或 `inf`），Snubber capacitance `Cs = 0`。
4. 桥的 A、B、C 各接一段 `Series RLC Branch`：Resistance = `10`，Inductance = `0.02`，Capacitance = `inf`。三段的另一端接在一起当作 N，不要接地。
5. `Voltage Measurement` 接在 A、B 之间。
6. `PWM Generator (2-Level)`：
   - Generator type = `Three-phase bridge (6 pulses)`
   - Mode of operation = `Unsynchronized`
   - Carrier frequency = `1050` Hz
   - 调制波选内部生成
   - Modulation index = `0.8`（分数，不是 80，也不是百分数）
   - Frequency of output voltage = `50` Hz
   - Phase = `0` degrees
   - 若有 Blanking time / dead time，填 `0`
7. 发生器的脉冲向量直接接 Universal Bridge 的 pulses 端口，不要自己重排顺序。
8. 求解器 `ode23tb`，Stop time = `0.06`，Max step size = `1e-6`。

powergui → FFT Analysis：输入选 `v_AB`，Start time = `0.04`，Number of cycles = `1`，Fundamental frequency = `50` Hz，Max frequency = `2500` Hz。显示选 **peak** 时，基波约 138.6 V；显示 **RMS** 时约 98.0 V。基波旁边不应再有 5 次、7 次的柱子。1050 Hz 和 2100 Hz 附近应有成对的边带。

不同版本里调制比那个框可能写成 Modulation index 或 m。填 `0.8`。发生器若改成外部调制波，把第 2 节的三路正弦接进去，幅值仍然是 0.8。

把调制比改成 `1.2` 再做一次 FFT。基波应在 191 V 附近，并且 5 次谐波变得看得见（脚本里是 6.22 V 峰值）。这是过调制，不是 FFT 设置错了。

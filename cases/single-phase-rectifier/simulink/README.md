# Simulink 搭建步骤 · single-phase-rectifier

物理电路优先用 Simscape Electrical。没有该工具箱时，用后面的基础模块搭 `Rs = 1 Ω` 的状态方程。本目录只放步骤，不提交二进制 `.slx`。

参数与案例卡一致：`Vm = 100 V`，`f = 50 Hz`，`R = 100 Ω`，`C = 1000e-6 F`，电容初值 0。主工况二极管前向压降为 0、合闸角为 0，交流支路不另串电阻。峰值合闸的对照再串 `1 Ω`、初相改成 90°。

## 1. Simscape Electrical

新建空白模型，保存为 `single_phase_rectifier.slx`。

电路：

```text
                 D1                         D2
        +-------|>|-------+------- +vo ----|<|-------+
        |                 |                |         |
        |                 |                |         |
        |                === C             |         |
   vs ~ +                 |                |         + ~ vs
        |                [ R ]             |         |
        |                 |                |         |
        +-------|<|-------+------- -vo ----|>|-------+
                 D4                         D3
                          |
                       参考地
                          |
                 Solver Configuration
```

电流传感器不画在桥臂上：它串在 `+vo` 和「电容、电阻的公共上端」之间，所以读到的是整桥送出的 `i_D`，不是电容支路自己的电流。左端为正时走 D1 和 D3，右端为正时走 D2 和 D4。阴极（横杠）都朝向 `+vo`。

电压、电流经 PS-Simulink Converter 进 Scope。主工况没有栅极，也不要加 Pulse Generator。

### 模块与参数

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Solver Configuration | Simscape / Utilities | 默认即可，连到参考地或任一电气节点 |
| AC Voltage Source | Simscape / Foundation Library / Electrical / Electrical Sources / AC Voltage Source | Peak amplitude = `100` V，Frequency = `50` Hz，Phase = `0`。相位单位选 `deg` |
| Diode ×4 | Simscape / Electrical / Semiconductors & Converters / Diode | Diode model = `Piecewise linear`，Forward voltage = `0` V，On resistance = `0.001` Ω，Off conductance = `1e-6` 1/Ω |
| Capacitor | Simscape / Foundation Library / Electrical / Electrical Elements / Capacitor | Capacitance = `1000e-6` F，Series resistance = `0`，Initial voltage = `0` |
| Resistor | 同上目录 / Resistor | Resistance = `100` Ω |
| Electrical Reference | 同上目录 / Electrical Reference | 全图只有一个参考地，接在 `−vo` |
| Current Sensor | Simscape / Foundation Library / Electrical / Electrical Sensors / Current Sensor | 串在桥的 `+vo` 和电容、电阻的公共端之间，箭头指向电容。读到的是整流后的 `i_D`（电容电流加负载电流） |
| Voltage Sensor | 同上目录 / Voltage Sensor | 正端接 `+vo`，负端接参考地 |
| PS-Simulink Converter ×2 | Simscape / Utilities | Output signal unit 分别填 `A`、`V` |
| Scope ×2 | Simulink / Sinks | 一个看 `v_C`，一个看 `i_D` |

二极管若还有 Reverse recovery、Junction capacitance 之类的选项，保持关闭或 None。

电容若还有 Parallel conductance，先留着模块默认的很小电导；改成 0 以后若求解器报电路奇异，再改回默认值。串联电阻必须是 0，否则纹波里会多出 `i·ESR` 的台阶。

四只二极管的 On resistance 不要填 0。两只串联是 `0.002 Ω`，对 6–13 A 的脉冲只掉几十毫伏，平均电压会比案例卡里严格的 96.007 V 略低。要和 `Rs = 0` 的脚本逐位去对，用 MATLAB / Python，不用这个有通态电阻的电路。

交流源的 Peak amplitude 填峰值 `100`，不要填有效值 `70.7`，也不要填 `220`。Frequency 这里的单位是 Hz，填 `50`。这和基础库 Sine Wave 不一样，见第 2 节。

### 连线顺序

1. 放下 AC Voltage Source、四只二极管、电流传感器、电容、电阻、Electrical Reference、Solver Configuration。
2. 交流源的两个端子就是桥的两个交流端，先不要接地。单相桥的交流侧没有「地」。
3. D1：阳极接交流源左端，阴极接 `+vo`。D2：阳极接交流源右端，阴极也接 `+vo`。两只阴极连在一起。
4. D4：阴极接交流源左端，阳极接 `−vo`。D3：阴极接交流源右端，阳极也接 `−vo`。两只阳极连在一起。
5. `+vo` 经电流传感器接到电容和电阻的上端。电容和电阻的下端都接 `−vo`，再接 Electrical Reference。Solver Configuration 也接参考地。
6. Voltage Sensor 的 `+` 接 `+vo`，`−` 接参考地。两个传感器的物理信号输出各接一个 PS-Simulink Converter，再接 Scope。

方向反了一只，常见的结果是直流侧接近 0，或者正弦直接穿过去。先数阴极是不是都朝向 `+vo`。

### 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.2` |
| Type | Variable-step |
| Solver | `ode23t` |
| Max step size | `1e-5` |
| Relative tolerance | `1e-4` |

导通窗口只有大约 1.4 ms。最大步长若到 `1e-4`，脉冲会被削成几级台阶，峰值对不上十几安。

运行后：

- 0–10 ms：电压从 0 跟着第一座正弦爬到 100 V 附近，电流冲到 30 A 这一量级，然后二极管截止
- 10 ms 以后：每 10 ms 在正弦顶点补一次电。电压在大约 92 V 和 100 V 之间，不是一条直线
- 160–200 ms：把横轴放大。电压峰峰值约 8.3 V；电流是窄脉冲，其余时间为 0。手算的 10 V 比这个大，是估算把放电时间当成了整整 10 ms

电容改成 `2000e-6` 再跑一次。纹波大约减半，落在 4.4 V 附近，平均值更靠近 100 V。

### 峰值合闸

这一组专门看电流，不再和 96 V、8.3 V 对读数。

1. 在交流源支路串一只 Resistor，Resistance = `1` Ω。
2. AC Voltage Source 的 Phase 改为 `90`，单位仍是 `deg`。
3. Max step size 改为 `1e-6`。100 A 的尖峰比 1.4 ms 的稳态脉冲窄得多，`1e-5` 会把它吃掉。
4. Stop time 可以仍是 `0.2`。

读数：`t = 0` 电流到 100 A；约 2 ms 时电压只有 78 V 左右，电流已经回到 0，因为正弦自己掉下去了；约 9.6 ms 电压才过 90 V，10 ms 附近到 93 V。稳态平均约 93.0 V，纹波约 7.2 V。

不串这只 1 Ω 就把相位设成 90°，理想二极管和空电容之间没有电阻，求解器会报代数环，或在 0 时刻画出一条没有宽度的针。这是模型的限制，不是桥接反了。

## 2. 只有基础库时（Rs = 1 Ω）

不安装 Simscape 也能看滤波的形状。`Rs = 0` 在导通时是代数约束，基础库会绕进代数环，所以这一节固定串 `1 Ω`，读数对案例卡里的 `Vavg_Rs_meas_V` 和 `dV_Rs_meas_V`，不要去对 96.007 V。

```text
i_D = max(|Vm sin(ωt + φ)| − v_C, 0) / Rs
dv_C/dt = (i_D − v_C/R) / C
```

`Rs = 1`，`1/R = 0.01`，`1/C = 1000`。主对照用 `φ = 0`；看冲击时把 Sine Wave 的 Phase 改成 `pi/2`。

```text
Sine Wave --> Abs --> (+) Sum(+-) --> Gain(1/Rs) --> Saturation(下限 0) --> i_D
                           (−)                                              |
                            ^                                               v
                            |                                          (+) Sum(+-)
                            |                                           (−)  |
                            |                                            ^   +--> Gain(1/C) --> Integrator --> v_C --> Scope
                            |                                            |
                            +---------------------- v_C                  +------ Gain(1/R) <--- v_C
```

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Sine Wave | Simulink / Sources | Amplitude = `100`，Bias = `0`，Frequency = `100*pi`，Phase = `0`，Sample time = `0` |
| Abs | Simulink / Math Operations | 默认 |
| Sum | 同上 | 符号 `+-`。正端是 `\|v_s\|`，负端是 `v_C` |
| Gain `1/Rs` | 同上 | Gain = `1` |
| Saturation | Simulink / Discontinuities | Upper limit = `inf`，Lower limit = `0`。输出就是 `i_D` |
| Sum | Simulink / Math Operations | 符号 `+-`。正端 `i_D`，负端 `v_C/R` |
| Gain `1/R` | 同上 | Gain = `0.01` |
| Gain `1/C` | 同上 | Gain = `1000` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `v_C` |
| Scope ×2 | Simulink / Sinks | 一个看 `v_C`，一个看 `i_D` |

Sine Wave 的 Frequency 单位是 **rad/s**。填 `50` 会得到大约 126 ms 的周期，纹波看起来像慢放。要 50 Hz，填 `100*pi`。

求解器：**Variable-step / ode45**，Stop time = `0.2`，Max step size = `1e-5`，Relative tolerance = `1e-4`。

读数（Phase = `0`）：稳态平均约 93.0 V，纹波约 7.23 V，峰值约 96.6 V，谷值约 89.4 V。过零合闸的第一下电流约 24.8 A，比 `Rs = 0` 的 31.4 A 小。

把 Phase 改成 `pi/2`、Max step size 改成 `1e-6`：`t = 0` 的电流应到 100 A，2 ms 时电压约 77.6 V，10 ms 时约 92.7 V。稳态平均仍是约 93.0 V，和过零合闸相同。

## 3. 可选：Specialized Power Systems

仅当已安装该工具箱、课堂又习惯 `powergui` 时使用。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `AC Voltage Source`：Peak amplitude = `100` V，Phase = `0`（度），Frequency = `50` Hz。交流侧两端接到 Universal Bridge 的 A、B，不要把交流端接地。
3. `Universal Bridge`：Number of bridge arms = `2`，Power Electronic device = `Diodes`。Ron = `0.001` Ω，Lon = `0`，Forward voltage Vf = `0`，Snubber resistance `Rs = inf`（或 `1e6`），Snubber capacitance `Cs = 0`。
4. 桥的 `+`、`−` 之间并两段 `Series RLC Branch`：
   - 电容：Resistance = `0`，Inductance = `0`，Capacitance = `1000e-6`，初始电压 0
   - 负载：Resistance = `100`，Inductance = `0`，Capacitance = `inf`
5. `Current Measurement` 串在桥的直流正输出。`Voltage Measurement` 并在负载两端。
6. 求解器改为 `ode23tb`，Stop time = `0.2`，Max step size = `1e-5`。

读数与第 1 节相同：稳态电压大约 92–100 V，纹波约 8.3 V，电流为 10 ms 一次的窄脉冲。

峰值合闸：在交流源支路再串一段 Series RLC Branch，Resistance = `1`，Inductance = `0`，Capacitance = `inf`；交流源 Phase = `90`；Max step size = `1e-6`。`t = 0` 的电流应到 100 A。

# Simulink 搭建步骤 · pi-speed-loop

参数和案例卡第 3 节一致。电机部分沿用 [`dc-motor-open-loop` 的基础库搭法 A](../../dc-motor-open-loop/simulink/README.md)，这里只写新增的控制部分。本目录只放步骤，不提交二进制 `.slx`。

## 1. 先复现开环

把上一张卡的模型另存为 `pi_speed_loop.slx`，先不改任何东西跑一次：峰值约 367.3 A、空载约 2095.6 r/min。对上了再往下做。被控对象错了，调 PI 没有意义。

## 2. 加转速 PI

信号流：

```text
Ramp ─► Saturation ─► (+)Sum0(−) ─► PID Controller ─► [原来 Va 的 Step 位置] ─► 电机模型 ─┬─► ω
  (ω_ref, rad/s)          ▲            (PI, 0…220 V,                                 │
                          │             clamping)                                     │
                          └──────────────────────────────────────────────────────────┘
```

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Ramp `ω_ref` | Simulink / Sources | Slope = `1500*2*pi/60`，Start time = `0`，Initial output = `0` |
| Saturation | Simulink / Discontinuities | Upper = `1500*2*pi/60`，Lower = `0` |
| Sum0 | Simulink / Math Operations | 符号 `+-`（给定减反馈） |
| PID Controller | Simulink / Continuous | Controller = `PI`，Form = `Parallel`，Time domain = Continuous-time，P = `4`，I = `45` |
| （PID 块 Output Saturation 页） | | 勾 *Limit output*，Upper = `220`，Lower = `0`；*Anti-windup method* = `clamping` |
| Step `T_L` | Simulink / Sources | Step time = `1.5`，Final = `20`（上一张卡是 0.8，要改） |
| 删除 | | 原来的 Va Step 删掉，PID 输出接到电气环 Sum1 的 `+` 口 |
| Gain `rpm` + Scope | | 转速 `60/(2*pi)` 后看；另一个 Scope 看 `ia`，第三个看 PID 输出 `Va` |

Ramp + Saturation 就是「1 s 斜坡后保持」。阶跃工况：把这两个块换成 Step（Step time `0`，Final `1500*2*pi/60`）。

### 求解器

| 项 | 取值 |
|----|------|
| Stop time | `3` |
| Type | Variable-step |
| Solver | `ode45` |
| Max step size | `1e-4` |
| Relative tolerance | `1e-6` |

## 3. 读数（对照案例卡第 4 节）

| 改动 | 应看到 |
|------|--------|
| I = `0`（纯 P） | 1.5 s 前约 1199.4 r/min，加载后约 1180.3 r/min |
| I = `45` | 稳在 1500 r/min；加载跌落约 18.6 r/min，约 0.23 s 回到 ±2 r/min；带载 Va 约 167.5 V |
| I = `200` | 斜坡结束超调约 20 r/min，电流振荡到约 47 A |
| 阶跃给定，I = `45`，clamping | 电流峰值约 367.3 A，转速不超调 |
| 阶跃给定，I = `45`，Anti-windup = `none` | 转速冲到约 1835 r/min |

## 4. 可选：Simscape Electrical

在上一张卡搭法 B 的基础上，把 `DC Voltage Source` 换成 `Controlled Voltage Source`，控制端经 `Simulink-PS Converter` 接 PID 输出；`Ideal Rotational Motion Sensor` 的 W 端口经 `PS-Simulink Converter`（单位 `rad/s`，不要选 rpm，否则增益要换算）回到 Sum0 的减号口。求解器 `ode23t`，Max step `1e-4`。读数应与上表一致。

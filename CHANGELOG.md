# CHANGELOG

格式：`YYYY-MM-DD` · 新增/修改 · slug · 简述

## 2026-11-23
- 新增 · `three-phase-short-circuit` · 三相短路电流暂态案例卡（10.5 kV 无限大电源经 X = 0.5 Ω、R = 0.031831 Ω（X/R = 15.708，Ta = 50 ms）线路供电，短路前负荷 13.6 + j6.6 Ω/相（394.416 A，cos φ0 = 0.887），t_f = 40 ms 三相金属性接地短路；Ipm = 17.112 kA、Ip = 12.100 kA、Sk = 220.05 MVA；A（α = 177.98°，a 相非周期分量最大）i_ap0 = −16.830 kA、冲击电流 30.939 kA（Kim = 1.808）；B（α = 87.98°，a 相无非周期分量）c 相仍 29.490 kA（1.723 Ipm）；C（空载 α = φk + 90°）31.145 kA，Kim = 1.820 vs 教材 1 + e^(−0.01/Ta) = 1.8187；α 扫描三相最大瞬时值 29.103~30.967 kA（≥ 1.70 Ipm）；三相 i_ap0 之和 ~1e-12 A；每周期平均值拟合 Ta = 50.000 ms；X/R 扫描 Kim 精确值 vs 近似（3 → 1.3786 / 1.3509，50 → 1.9395 / 1.9391）；RK4 与解析解最大偏差 1.3e-13 Ipm；Simulink 基础库 / Simscape Electrical Three-Phase Fault 步骤，Python 可运行脚本，MATLAB 脚本）
- 修改 · `transformer-inrush` · 相关链接指向已完成的 `three-phase-short-circuit`

## 2026-11-20
- 新增 · `transformer-inrush` · 变压器励磁涌流案例卡（单相 10 kVA、220 V / 50 Hz 空载合闸；R = 0.01 pu（0.0484 Ω）、Lσ = 0.02 pu（0.3081 mH）、Lm = 50 pu（0.7703 H，空载电流 2%）、两段线性 λ-i 饱和拐点 1.2 pu、空心电感 0.13 pu（2.0028 mH）、剩磁 ±0.8 pu；α = 0° + 剩磁 0.8 峰值 634.052 A（13.949 倍额定，忽略 R 粗估 687.222 A），α = 0° 无剩磁 325.036 A，α = 90° 无剩磁无涌流（1.285 A），α = 90° + 剩磁 249.301 A，选相 α = 143.130° 无涌流（1.295 A）；首周期二次谐波 17.771% / 55.430% / 65.146%（A 落在 15%~20% 制动整定之间）；A 等效衰减时间常数 171.228 ms、28 个周期后低于额定电流峰值；Simulink 基础库 / Simscape Electrical Saturable Transformer 步骤，Python 可运行脚本（RK4 + Radau 核对），MATLAB 脚本）
- 修改 · `pi-speed-loop` · 相关链接指向已完成的 `transformer-inrush`

## 2026-11-18
- 新增 · `pi-speed-loop` · 直流电机 PI 转速环案例卡（电机同 `dc-motor-open-loop`；理想电压源 0–220 V 限幅、钳位抗饱和；给定 1500 r/min、1 s 斜坡，1.5 s 加 20 N·m；P（Kp = 4）静差：空载 1199.400、带载 1180.311 r/min；PI-A（4, 45，零点约消电机慢极点，ζ = 0.702）跌落 18.608 r/min、231.9 ms 回到 ±2 r/min、带载 Va = 167.472 V；PI-B（4, 200，ζ = 0.285）超调 20.286 r/min、电流 46.782 A；阶跃给定电流峰值仍 367.288 A，无抗饱和超调 334.696 r/min；Simulink PID Controller 步骤，Python 可运行脚本（RK45 + Radau 核对），MATLAB 脚本）
- 修改 · `dc-motor-open-loop` · 相关链接指向已完成的 `pi-speed-loop`

## 2026-11-16
- 新增 · `dc-motor-open-loop` · 直流电机开环启动案例卡（他励励磁恒定 / 永磁示例，Va = 220 V、Ra = 0.5 Ω、La = 5 mH、Ke = Kt = 1 V·s/rad、J = 0.2 kg·m²、B = 0.005 N·m·s；堵转电流 Va/Ra = 440 A（22 倍额定），直接启动峰值 367.289 A @ 26.642 ms；τ_e = 10 ms、τ_m = 100 ms，真实时间常数 11.271 / 88.505 ms；空载 2095.606 r/min，0.8 s 加 20 N·m 后 2000.351 r/min、21.047 A、轴功率 4189.526 W；串 5 Ω 启动电阻峰值 39.798 A、1.5 s 转速 1541.150 r/min；Simulink 基础库 / Simscape Electrical DC Motor 步骤，MATLAB / Python 可运行脚本）
- 修改 · `rl-step`、`motor-vf` · 相关链接指向已完成的 `dc-motor-open-loop`

## 2026-11-13
- 新增 · `motor-vf` · 异步电机 V/f 恒压频比控制案例卡（4 极示例，Y 接，380 V / 50 Hz；Rs = 2.2 Ω、Rr = 1.8 Ω、Lls = Llr = 0.012 H、Lm = 0.24 H、J = 0.04 kg·m²；空载 1500 r/min；15 N·m 时滑差 0.035724、转速 1446.414 r/min、轴功率 2272.022 W；5 Hz、8 N·m 无补偿 95.613 r/min，低频提升 Vboost = 25 V 后 142.469 r/min，最大转矩比 4.102879 等于电压比的平方；Simscape Electrical 步骤，MATLAB / Python 可运行脚本）
- 修改 · `three-phase-inverter-spwm` · 相关链接指向已完成的 `motor-vf`

## 2026-11-11
- 新增 · `three-phase-inverter-spwm` · 三相 SPWM 逆变器案例卡（Vdc = 200 V、m = 0.8、f1 = 50 Hz、mf = 21、fc = 1050 Hz；线性区 V_LL,1(peak) = (√3/2)·m·Vdc = 138.564 V；过调制 m = 1.2 时自然采样 191.196 V，不再外推到 207.846 V；六阶梯上限 (2√3/π)·Vdc = 220.532 V；星形 RL 为 R = 10 Ω、L = 20 mH；Simulink 步骤，MATLAB / Python 可运行脚本）
- 修改 · `single-phase-rectifier` · 相关链接指向已完成的 `three-phase-inverter-spwm`

## 2026-11-06
- 新增 · `single-phase-rectifier` · 单相桥式整流 + 电容滤波案例卡（Vm = 100 V、f = 50 Hz、R = 100 Ω、C = 1000 μF；不接电容平均 2Vm/π ≈ 63.662 V；纹波估算 ΔV = 10 V、平均估算 95 V；导通角解析纹波 8.282 V、平均 96.007 V；C 加倍估算 5 V；过零合闸电流约 10π A；峰值合闸串 1 Ω 时 i(0+) = 100 A；Simscape Electrical 步骤，MATLAB / Python 可运行脚本）
- 修改 · `boost-pwm` · 相关链接指向已完成的 `single-phase-rectifier`

## 2026-11-04
- 新增 · `boost-pwm` · Boost 变换器开环 PWM 案例卡（Vin = 12 V、D = 0.5、fs = 10 kHz、L = 1 mH、C = 100 μF、R = 48 Ω；Vo = 24 V，IL = 1 A，ΔiL = 0.6 A，Δvo = 0.25 V；CCM/DCM 边界 R = 160 Ω，R = 320 Ω 断续约 30.7 V；启动过冲约 43.7 V；Simscape Electrical 步骤，MATLAB / Python 可运行脚本）
- 修改 · `buck-pwm` · 相关链接指向已完成的 `boost-pwm`

## 2026-11-02
- 新增 · `buck-pwm` · Buck 变换器开环 PWM 案例卡（Vin = 12 V、D = 0.5、fs = 10 kHz、L = 1 mH、C = 25 μF、R = 6 Ω；Vo = 6 V，ΔiL = 0.3 A，Δvo ≈ 0.15 V；CCM/DCM 边界 R = 40 Ω；Simscape Electrical 步骤，MATLAB / Python 可运行脚本）
- 修改 · `rlc-series-step` · 相关链接指向已完成的 `buck-pwm`

## 2026-10-30
- 新增 · `rlc-series-step` · RLC 串联阶跃案例卡（L = 100 mH、C = 10 μF 固定，ω0 = 1000 rad/s；R = 100/200/400 Ω 对应 ζ = 0.5/1/2；欠阻尼超调 16.3%；Simulink 步骤，MATLAB / Python 可运行脚本）
- 修改 · `rl-step`、`rc-charge` · 相关链接指向已完成的 `rlc-series-step`

## 2026-10-28
- 新增 · `rc-charge` · RC 充放电案例卡（τ = RC = 10 ms；充电电压 63.2%，放电电压 36.8%；Simulink 步骤，MATLAB / Python 可运行脚本）
- 修改 · `rl-step` · 相关链接改为指向已完成的 `rc-charge` 案例卡

## 2026-10-27
- 新增 · `rl-step` · RL 电路阶跃响应案例卡（τ = L/R = 10 ms；Simulink 步骤，MATLAB / Python 可运行脚本）

## 2026-10-26
- 初始化仓库骨架：README、规范文档、主题队列、索引、LICENSE（MIT + 教学声明）。

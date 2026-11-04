# CHANGELOG

格式：`YYYY-MM-DD` · 新增/修改 · slug · 简述

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

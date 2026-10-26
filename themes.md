# 主题队列 · 仿真案例卡

状态：`todo` 待做 / `doing` 进行中 / `done` 已完成。自动化按顺序取第一条 `todo`。

| # | slug | 主题 | 状态 | 备注 |
|---|------|------|------|------|
| 1 | `rl-step` | RL 电路阶跃响应 | todo | 时间常数 τ=L/R |
| 2 | `rc-charge` | RC 充放电 | todo | τ=RC，63.2% |
| 3 | `rlc-series-step` | RLC 串联阶跃：欠/临界/过阻尼 | todo | ζ 对比 |
| 4 | `buck-pwm` | Buck 变换器开环 PWM | todo | 纹波与占空比 |
| 5 | `boost-pwm` | Boost 变换器开环 PWM | todo | Vo=Vin/(1-D) |
| 6 | `single-phase-rectifier` | 单相桥式整流 + 电容滤波 | todo | 纹波估算 |
| 7 | `three-phase-inverter-spwm` | 三相 SPWM 逆变器 | todo | 调制比与线电压 |
| 8 | `motor-vf` | 异步电机 V/f 恒压频比控制 | todo | 低频补偿 |
| 9 | `dc-motor-open-loop` | 直流电机开环启动 | todo | 电流冲击 |
| 10 | `pi-speed-loop` | 直流电机 PI 转速环 | todo | Kp/Ki 调试 |
| 11 | `transformer-inrush` | 变压器励磁涌流 | todo | 合闸角影响 |
| 12 | `three-phase-short-circuit` | 三相短路电流暂态 | todo | 非周期分量 |
| 13 | `pll-grid-sync` | SRF-PLL 锁相 | todo | 动态跟踪 |
| 14 | `pv-mppt-po` | 光伏 MPPT 扰动观察法 | todo | 步长取舍 |

# Simulink 搭建步骤 · motor-vf

物理模型优先用 Simscape Electrical 的笼型感应电机。没有该工具箱时，用后面的基础模块搭 V/f 前端，电机方程放进 MATLAB Function。本目录只放步骤，不提交二进制 `.slx`。

参数与案例卡一致：极对数 `2`，线电压 380 V，50 Hz，`Rs = 2.2 Ω`，`Rr' = 1.8 Ω`，`Lls = Llr' = 0.012 H`，`Lm = 0.24 H`，`J = 0.04 kg·m²`，摩擦 0。主工况不开低频提升：1.5 s 内频率从 0 升到 50 Hz，2.2 s 突加 15 N·m。空载转速对 1500 r/min，带载对 1446 r/min。

5 Hz 对照单独做一次：无补偿线电压 38 V，有提升线电压 76.971 V，负载 8 N·m。转速大约 96 r/min 和 142 r/min。

## 1. Simscape Electrical

新建空白模型，保存为 `motor_vf.slx`。

三只受控电压源接成 Y，中性点接电气参考。电机定子三相接到三个相端子，中性点接到同一参考。转子轴接理想转矩源，定子机壳和转矩源的另一侧都接机械旋转参考。

```text
        va                         vb                         vc
         |                          |                          |
      [受控源]                   [受控源]                   [受控源]
         |                          |                          |
         +----------- 中性点 n -----+--------------------------+---- 电气参考
         |                          |                          |
         a                          b                          c
         |                          |                          |
         +---------------- 感应电机定子 ------------------------+
                                    |
                              转子 R    机壳 C
                                |         |
                           [转矩源]       |
                                |         |
                                +----+----+---- 机械旋转参考
                                     |
                            Solver Configuration 接到电气参考
```

电压不在这张电气图里。频率斜坡同时走出角度和相电压峰值，三路正弦再经 Simulink-PS Converter 进受控源。

### 模块与参数

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Solver Configuration | Simscape / Utilities | 默认。接到电气参考 |
| Controlled Voltage Source ×3 | Simscape / Foundation Library / Electrical / Electrical Sources | 输入是相电压瞬时值，单位 V |
| Induction Machine Squirrel Cage | Simscape / Electrical / Electromechanical / Asynchronous（有的版本在 Induction Machines） | 见下表。电气端口选展开的 a、b、c、n |
| Ideal Torque Source | Simscape / Foundation Library / Mechanical / Mechanical Sources | 转矩单位 N·m。方向见下文 |
| Mechanical Rotational Reference | Simscape / Foundation Library / Mechanical / Rotational Elements | 接机壳和转矩源的定子侧 |
| Electrical Reference | Simscape / Foundation Library / Electrical / Electrical Elements | 接中性点 |
| Ramp | Simulink / Sources | Slope = `50/1.5`，Start time = `0`，Initial output = `0` |
| Saturation | Simulink / Discontinuities | Upper limit = `50`，Lower limit = `0`。输出是频率 f，单位 Hz |
| Gain `2π` | Simulink / Math Operations | Gain = `2*pi`。输出是电角速度，单位 rad/s |
| Integrator | Simulink / Continuous | Initial condition = `0`。输入 `2πf`，输出角度 θ |
| Gain `Vpeak/f` | Simulink / Math Operations | Gain = `sqrt(2)*380/sqrt(3)/50`。无补偿时，f 乘这个数就是相电压峰值 |
| Trigonometric Function ×3 | Simulink / Math Operations | Function = `sin` |
| Constant ×2 | Simulink / Sources | `-2*pi/3` 和 `+2*pi/3` |
| Sum ×2 | Simulink / Math Operations | 两路相位。`θ − 2π/3`、`θ + 2π/3` |
| Product ×3 | Simulink / Math Operations | 相电压峰值乘正弦，得到 va、vb、vc |
| Step | Simulink / Sources | Step time = `2.2`，Initial value = `0`，Final value = `15` |
| Simulink-PS Converter ×4 | Simscape / Utilities | 三路电压单位 `V`，转矩单位 `N*m`。Filtering and derivatives 选 Provide signals |
| PS-Simulink Converter | Simscape / Utilities | 若用传感器读转速，单位 `rad/s` |
| Scope ×2 | Simulink / Sinks | 一个看转速，一个看电磁转矩。有余量再看一相电流 |

感应电机参数（不饱和，SI）：

| 对话框 | 取值 |
|--------|------|
| Number of pole pairs | `2` |
| Stator resistance Rs | `2.2` Ω |
| Stator leakage inductance Lls | `0.012` H |
| Magnetizing inductance Lm | `0.24` H |
| Rotor resistance Rr' | `1.8` Ω |
| Rotor leakage inductance Llr' | `0.012` H |
| Inertia | `0.04` kg·m² |
| Rotor damping / friction | `0` |
| 初值（电流、转速、角度） | `0` |

有的版本把漏感写成额定频率下的漏抗。50 Hz 时 `Xls = Xlr = 2*pi*50*0.012 = 3.7699 Ω`，`Xm = 2*pi*50*0.24 = 75.398 Ω`。单位看对话框，不要把欧姆填进亨利的框。

饱和、铁耗有开关时保持关闭。本卡的读数是线性电感。

### 连线顺序

1. 放下电机、三只受控电压源、转矩源、两个参考、Solver Configuration。
2. 三只受控源的一端接 a、b、c，另一端接在一起成为 n。n 接 Electrical Reference，也接电机的 n。Solver Configuration 接到这个参考上。
3. 电机机壳 C 接 Mechanical Rotational Reference。转子 R 接 Ideal Torque Source 的转子侧，转矩源另一侧接同一个机械参考。
4. Ramp 进 Saturation，得到 f。f 分成两路：一路乘 `2*pi` 再积分得到 θ；一路乘 `sqrt(2)*380/sqrt(3)/50` 得到相电压峰值。
5. θ、`θ−2π/3`、`θ+2π/3` 各自取正弦，再乘相电压峰值。三路信号经 Simulink-PS Converter 进三只受控源。不要给这三路加滤波，否则电压滞后，滑差会被滤波器吃掉一点。
6. Step 经 Simulink-PS Converter 进转矩源。先用 `+15`。加载后转速应下降大约 5.6 rad/s。若转速上升，终值改成 `-15`。不同版本里理想转矩源的箭头方向不一样，以这个现象为准。
7. 转速优先在 Simscape Results Explorer 里读电机的转子角速度。也可以在转子上串 Rotational Motion Sensor。读到的是 rad/s：空载 `2*pi*1500/60 = 157.08`，带载约 `151.47`。乘 `60/(2*pi)` 得到 r/min。

电机模块里面已经有惯量。外面不要再接 Inertia，否则转速会爬得比 1.5 s 的斜坡慢一截。

### 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `3` |
| Type | Variable-step |
| Solver | `ode23t` |
| Max step size | `1e-4` |
| Relative tolerance | `1e-4` |

定子瞬态在几毫秒。最大步长留成 auto 时，转矩会发毛，空载转速也可能在 1500 r/min 附近晃几个转。`1e-4` 和脚本的 0.2 ms 同一量级。

### 读数

- 1.6 s 到 2.2 s：转速 157.08 rad/s，也就是 1500 r/min。转矩在 0 附近。
- 2.5 s 以后：转速约 151.47 rad/s，也就是 1446 r/min。电磁转矩约 15 N·m。
- 相电流有效值空载约 2.77 A，带载约 4.91 A。示波器上的正弦峰值是有效值的 √2，空载峰值约 3.92 A，带载峰值约 6.95 A。
- 加速段转矩大约 4 N·m，用来跟随斜坡。不要把这一段的峰值当成稳态。

摩擦若不能填 0，填一个很小的数并在讲义里声明：空载转速会比 1500 略低。本卡的 1500 对应摩擦为 0。

### 5 Hz 对照

把 Ramp / Saturation 换成 Constant `5`，或把 Saturation 上限改成 `5` 并且 Ramp 的斜率足够大，让频率在几毫秒内就到 5 Hz。Stop time 改成 `1.6`。Step time 改成 `0.4`，终值改成 `8`（符号仍以加载后转速下降为准）。

相电压峰值不要再用 `f * sqrt(2)*380/sqrt(3)/50` 以外的数去「手调」。两个对照的峰值是：

| 对照 | 相电压峰值 | 表达式 |
|------|------------|--------|
| 无补偿 | 31.026870 V | `sqrt(2)*380/sqrt(3)*5/50` |
| 低频提升 | 62.846675 V | `sqrt(2)*(25+(380/sqrt(3)-25)*5/50)` |

无补偿可以直接把频率定在 5 Hz，增益保持 `sqrt(2)*380/sqrt(3)/50`，因为 `5` 乘这个增益就是 31.026870 V。提升那一次要把增益换成上面的常数，或改用 `V = 25 + (380/sqrt(3)-25)*f/50` 再乘 `sqrt(2)`。

1.3 s 以后：无补偿转速约 10.01 rad/s（95.6 r/min），有提升约 14.92 rad/s（142.5 r/min）。转矩都回到 8 N·m。

## 2. 只有基础库时

不安装 Simscape 也能把同一组微分方程积出来。V/f 仍用上一节的 Ramp、Saturation、积分器和三路正弦。电机用 6 个积分器，导数由 MATLAB Function 按案例卡第 2 节计算。

```text
Ramp -- Saturation -- f --+-- Gain(2*pi) -- Integrator -- θ
                          |
                          +-- Gain(sqrt(2)*380/sqrt(3)/50) -- Vpeak

θ、Vpeak、六个状态  -->  MATLAB Function  -->  六个导数、Te
六个导数 --> 六个 Integrator（初值 0）--> 回到 Function
Te 和转速进 Scope
Step(T_L) 也进 Function
```

MATLAB Function 的状态顺序与脚本一致：`theta, isa, isb, psira, psirb, wm`。导数就是案例卡里的那一组。下面这份和 `motor_vf.m` 使用相同的常数，无补偿。

```matlab
function [dtheta, dia, dib, dpr, dpb, dw, te] = vfMotor(theta, isa, isb, psira, psirb, wm, freq, tl)
    np = 2; Rs = 2.2; Lm = 0.24; J = 0.04;
    Ls = 0.252; Lr = 0.252; tauR = 0.14; kPsi = Lm / Lr;
    Lsig = (1 - (Lm * Lm) / (Ls * Lr)) * Ls;
    vph = 380 / sqrt(3) * freq / 50;
    vm = sqrt(2) * vph;
    wr = np * wm;
    vsa = vm * cos(theta);
    vsb = vm * sin(theta);
    dpr = (-psira / tauR) - wr * psirb + (Lm / tauR) * isa;
    dpb = (-psirb / tauR) + wr * psira + (Lm / tauR) * isb;
    dia = (vsa - Rs * isa - kPsi * dpr) / Lsig;
    dib = (vsb - Rs * isb - kPsi * dpb) / Lsig;
    te = 1.5 * np * kPsi * (psira * isb - psirb * isa);
    dw = (te - tl) / J;
    dtheta = 2 * pi * freq;
end
```

六个积分器的初值都是 0。`freq` 接 Saturation 的输出，`tl` 接 Step。Function 的采样不要改成离散，让求解器自己去积连续状态。

求解器：**Variable-step / ode45**，Stop time = `3`，Max step size = `1e-4`。ode45 的轨迹和脚本里的定步长 RK4 在稳态应重合到 0.1 r/min 以内；命令行那 6 位小数以 RK4 脚本为准。

低频提升时，把函数里的 `vph` 换成

```matlab
vph = 25 + (380 / sqrt(3) - 25) * freq / 50;
```

频率和负载改成第 1 节的 5 Hz 对照。

Sine Wave 模块的 Frequency 框单位是 rad/s。本搭建不用它来产生变频正弦。角度来自 `∫ 2π f dt`。把 50 填进 Sine Wave 的 Frequency，周期会变成大约 126 ms。

## 3. 可选：Specialized Power Systems

仅当已安装该工具箱、课堂又习惯 `powergui` 时使用。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. Asynchronous Machine SI Units：
   - Rotor type = `Squirrel-cage`
   - Reference frame = `Stationary`
   - Nominal power = `4000` VA，Nominal voltage = `380` V，Nominal frequency = `50` Hz。饱和关掉以后，这三个数不进入电磁方程，只用来满足对话框。
   - Rs = `2.2`，Lls = `0.012`，Lm = `0.24`，Rr' = `1.8`，Llr' = `0.012`
   - J = `0.04`，Friction F = `0`，Pole pairs p = `2`
   - 初值保持 0
3. 三只 Controlled Voltage Source 接成 Y，公共端接地（和电机中性点一起接地）。控制信号用第 1 节的 va、vb、vc。
4. 电机的 Tm 接 Step：Step time = `2.2`，终值先填 `15`。这个库里通常 `Te − Tm = J dω/dt`，Tm 为正是负载。加载后若转速上升，终值改成 `-15`。
5. 测量输出 m 接 Machines Measurement Demux，勾选定子电流、转子转速、电磁转矩。
6. 求解器 `ode23tb`，Stop time = `3`，Max step size = `1e-4`。

转速解复用出来的单位若是 rad/s，空载读 157.08，带载读 151.47。若读数停在 314 附近，那是电角速度，再除以极对数 2。换算成 r/min 以后应对 1500 和 1446。

5 Hz 对照同样把频率和电压换成第 1 节的两列峰值，Stop time = `1.6`，0.4 s 加 8 N·m。

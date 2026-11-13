% motor_vf.m  异步电机 V/f 恒压频比与低频电压提升（教学演示）
%
% 示例电机：4 极（极对数 2）、Y 接、380 V / 50 Hz。参数是课堂用的圆整数，
% 不是某一台电机的铭牌。
%
% 静止 αβ、幅值不变 Clarke。状态 [theta, isa, isb, psira, psirb, wm]。
%   dtheta/dt = 2*pi*f
%   vsα = sqrt(2)*Vph(f)*cos(theta)
%   vsβ = sqrt(2)*Vph(f)*sin(theta)
%   dψrα/dt = -ψrα/τr - wr*ψrβ + (Lm/τr)*isα
%   dψrβ/dt = -ψrβ/τr + wr*ψrα + (Lm/τr)*isβ
%   σ*Ls*disα/dt = vsα - Rs*isα - (Lm/Lr)*dψrα/dt
%   σ*Ls*disβ/dt = vsβ - Rs*isβ - (Lm/Lr)*dψrβ/dt
%   Te = 1.5*np*(Lm/Lr)*(ψrα*isβ - ψrβ*isα)
%   J*dwm/dt = Te - Tl
%   wr = np*wm ，τr = Lr/Rr ，σ = 1 - Lm^2/(Ls*Lr)
%
% 无补偿：Vph = Vph_n * f/fn
% 低频提升：Vph = Vboost + (Vph_n - Vboost)*f/fn ，f = fn 时回到额定电压
%
% 主工况不带提升，1.5 s 内频率从 0 升到 50 Hz，2.2 s 突加 15 N·m。
% 低速对照把频率定在 5 Hz，0.4 s 加上 8 N·m，比较有无提升。
% 稳态数字用 T 型等效电路。积分是定步长 RK4，步长 0.2 ms，和 Python 同一套
% 循环，打印的 6 位小数应逐行相同。只用基础函数，不需要工具箱。
% 文件末尾的局部函数从 R2016b 起属于基础语言。
%
% 运行:
%   run('cases/motor-vf/matlab/motor_vf.m')

clear; clc;

P.np = 2;
P.fn = 50;
P.vll = 380;
P.vph = 380 / sqrt(3);
P.rs = 2.2;
P.rr = 1.8;
P.lls = 0.012;
P.llr = 0.012;
P.lm = 0.24;
P.J = 0.04;
P.vBoost = 25;
P.fLow = 5;
P.tl = 15;
P.tlLow = 8;
P.tRamp = 1.5;
P.tLoad = 2.2;
P.tEnd = 3.0;
P.tLowOn = 0.4;
P.tLowEnd = 1.6;
P.h = 2e-4;
P.ls = P.lls + P.lm;
P.lr = P.llr + P.lm;
P.sigma = 1 - (P.lm * P.lm) / (P.ls * P.lr);
P.tauR = P.lr / P.rr;
P.kPsi = P.lm / P.lr;
P.lsig = P.sigma * P.ls;

rated = motorRk4('rated', P.tEnd, P);
low0 = motorRk4('low0', P.tLowEnd, P);
low1 = motorRk4('low1', P.tLowEnd, P);

[i0Th, te0] = motorSteady(P.fn, false, 0, P);
[iStart, teStart] = motorSteady(P.fn, false, 1, P);
[slipBd, teMax] = motorTorqueMax(P.fn, false, P);
slipLoad = motorSlipFor(P.fn, false, P.tl, slipBd, P);
[iLoad, teLoad] = motorSteady(P.fn, false, slipLoad, P);
nLoad = (1 - slipLoad) * P.fn * 60 / P.np;
wmLoad = (1 - slipLoad) * 2 * pi * P.fn / P.np;
pShaft = P.tl * wmLoad;

[slipBdLow, teMaxLow] = motorTorqueMax(P.fLow, false, P);
[~, teMaxOn] = motorTorqueMax(P.fLow, true, P);
slipOff = motorSlipFor(P.fLow, false, P.tlLow, slipBdLow, P);
slipOn = motorSlipFor(P.fLow, true, P.tlLow, slipBdLow, P);
[iOff, ~] = motorSteady(P.fLow, false, slipOff, P);
[iOn, ~] = motorSteady(P.fLow, true, slipOn, P);
nOff = (1 - slipOff) * P.fLow * 60 / P.np;
nOn = (1 - slipOn) * P.fLow * 60 / P.np;

nNoload = motorWindow(rated.t, rated.n, 2.0, 2.2);
iNoload = motorWindow(rated.t, rated.i, 2.0, 2.2);
nMeas = motorWindow(rated.t, rated.n, 2.7, 3.0);
teMeas = motorWindow(rated.t, rated.te, 2.7, 3.0);
iMeas = motorWindow(rated.t, rated.i, 2.7, 3.0);
nM0 = motorWindow(low0.t, low0.n, 1.3, 1.6);
nM1 = motorWindow(low1.t, low1.n, 1.3, 1.6);
teM0 = motorWindow(low0.t, low0.te, 1.3, 1.6);
teM1 = motorWindow(low1.t, low1.te, 1.3, 1.6);
iM0 = motorWindow(low0.t, low0.i, 1.3, 1.6);
iM1 = motorWindow(low1.t, low1.i, 1.3, 1.6);

vphOff = motorVoltage(P.fLow, false, P);
vphOn = motorVoltage(P.fLow, true, P);

keys = {
    'np_pairs', P.np
    'f_rated_Hz', P.fn
    'Vll_rated_V', P.vll
    'Vph_rated_V', P.vph
    'Vph_peak_V', P.vph * sqrt(2)
    'Rs_ohm', P.rs
    'Rr_ohm', P.rr
    'Lls_H', P.lls
    'Llr_H', P.llr
    'Lm_H', P.lm
    'Ls_H', P.ls
    'Lr_H', P.lr
    'sigma', P.sigma
    'tau_r_s', P.tauR
    'J_kgm2', P.J
    'Vboost_V', P.vBoost
    'Vll_boost0_V', P.vBoost * sqrt(3)
    'I_boost_preset_A', P.vBoost / P.rs
    'vf_line_V_per_Hz', P.vll / P.fn
    'n_sync_rpm', P.fn * 60 / P.np
    'I0_theory_A', i0Th
    'Tl_Nm', P.tl
    's_bd_rated', slipBd
    'Temax_rated_Nm', teMax
    's_load', slipLoad
    'n_load_theory_rpm', nLoad
    'I_load_theory_A', iLoad
    'P_shaft_W', pShaft
    'n_meas_noload_rpm', nNoload
    'I_meas_noload_A', iNoload
    'n_meas_load_rpm', nMeas
    'Te_meas_load_Nm', teMeas
    'I_meas_load_A', iMeas
    'f_low_Hz', P.fLow
    'n_sync_low_rpm', P.fLow * 60 / P.np
    'Tl_low_Nm', P.tlLow
    'Vph_low_off_V', vphOff
    'Vph_low_on_V', vphOn
    'Vll_low_off_V', vphOff * sqrt(3)
    'Vll_low_on_V', vphOn * sqrt(3)
    's_bd_low', slipBdLow
    'Temax_low_off_Nm', teMaxLow
    'Temax_low_on_Nm', teMaxOn
    'Temax_ratio_low', teMaxOn / teMaxLow
    'V_ratio_sq_low', (vphOn / vphOff) ^ 2
    'I_start_50_A', iStart
    'Te_start_50_Nm', teStart
    's_low_off', slipOff
    'n_low_off_theory_rpm', nOff
    'I_low_off_theory_A', iOff
    's_low_on', slipOn
    'n_low_on_theory_rpm', nOn
    'I_low_on_theory_A', iOn
    'n_meas_low_off_rpm', nM0
    'n_meas_low_on_rpm', nM1
    'Te_meas_low_off_Nm', teM0
    'Te_meas_low_on_Nm', teM1
    'I_meas_low_off_A', iM0
    'I_meas_low_on_A', iM1
    'dn_boost_rpm', nM1 - nM0
    'abs_err_noload_rpm', nNoload - P.fn * 60 / P.np
    'abs_err_n_load_rpm', nMeas - nLoad
    'abs_err_n_low_off_rpm', nM0 - nOff
    'abs_err_n_low_on_rpm', nM1 - nOn
    'rel_err_n_load', (nMeas - nLoad) / nLoad
    'rel_err_I_load', (iMeas - iLoad) / iLoad
    'rel_err_n_low_off', (nM0 - nOff) / nOff
    'rel_err_n_low_on', (nM1 - nOn) / nOn
    };

fprintf('Induction motor V/f\n');
for k = 1:size(keys, 1)
    fprintf('  %-26s = %s\n', keys{k, 1}, motorFmt6(keys{k, 2}));
end

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
motorPlot(rated, low0, low1, nOff, nOn, P);
outFile = fullfile(outDir, 'motor_vf_curves_matlab.png');
print(gcf, outFile, '-dpng', '-r140');
fprintf('  %-26s = %s\n', 'figure', outFile);

if te0 ~= 0
    error('motor_vf:NoLoad', '空载等效电路转矩应为 0。');
end
if abs(teLoad - P.tl) / P.tl > 1e-6
    error('motor_vf:TorqueFit', '等效电路没有把负载转矩配平。');
end
if abs(nNoload - 1500) > 0.05
    error('motor_vf:NoLoadSpeed', '空载转速没有落到 1500 r/min。');
end
if abs(nMeas - nLoad) > 0.3
    error('motor_vf:LoadSpeed', '带载转速偏离同步转速减滑差。');
end
if abs(teMeas - P.tl) > 0.05
    error('motor_vf:LoadTorque', '带载电磁转矩没有回到负载转矩。');
end
if abs(iMeas - iLoad) > 0.02 || abs(iNoload - i0Th) > 0.02
    error('motor_vf:Current', '电流偏离等效电路。');
end
if nMeas >= 1500
    error('motor_vf:SlipSign', '带载转速应低于同步转速。');
end
if teMaxOn <= teMaxLow * 2
    error('motor_vf:Boost', '低频提升应明显提高最大转矩。');
end
if abs(teM0 - P.tlLow) > 0.05 || abs(teM1 - P.tlLow) > 0.05
    error('motor_vf:LowTorque', '5 Hz 稳态转矩没有等于 8 N·m。');
end
if abs(nM0 - nOff) > 0.3 || abs(nM1 - nOn) > 0.3
    error('motor_vf:LowSpeed', '5 Hz 转速偏离等效电路。');
end
if nM1 < nM0 + 30
    error('motor_vf:LowGap', '低频提升应把转速拉近同步转速。');
end
if nM0 > 120
    error('motor_vf:LowSlip', '无补偿的 5 Hz 运行滑差应明显偏大。');
end
if slipBdLow <= slipLoad
    error('motor_vf:Breakdown', '低速临界滑差应大于额定工况滑差。');
end

% --- 局部函数（R2016b 及以后的基础 MATLAB，不是工具箱） ---

function text = motorFmt6(value)
    if abs(value) < 5e-7
        text = '0.000000';
    else
        text = sprintf('%.6f', value);
    end
end

function v = motorVoltage(freq, boost, P)
    frac = freq / P.fn;
    if frac > 1
        frac = 1;
    end
    if boost
        v = P.vBoost + (P.vph - P.vBoost) * frac;
    else
        v = P.vph * frac;
    end
end

function [freq, load, boost] = motorSchedule(t, mode, P)
    if strcmp(mode, 'rated')
        if t < P.tRamp
            freq = P.fn * t / P.tRamp;
        else
            freq = P.fn;
        end
        if t >= P.tLoad
            load = P.tl;
        else
            load = 0;
        end
        boost = false;
    elseif strcmp(mode, 'low0')
        freq = P.fLow;
        if t >= P.tLowOn
            load = P.tlLow;
        else
            load = 0;
        end
        boost = false;
    elseif strcmp(mode, 'low1')
        freq = P.fLow;
        if t >= P.tLowOn
            load = P.tlLow;
        else
            load = 0;
        end
        boost = true;
    else
        error('motor_vf:Mode', '未知工况。');
    end
end

function [d, te] = motorDeriv(t, x, mode, P)
    theta = x(1);
    isa = x(2);
    isb = x(3);
    psira = x(4);
    psirb = x(5);
    wm = x(6);
    [freq, load, boost] = motorSchedule(t, mode, P);
    wr = P.np * wm;
    vm = sqrt(2) * motorVoltage(freq, boost, P);
    vsa = vm * cos(theta);
    vsb = vm * sin(theta);
    dpr = (-psira / P.tauR) - wr * psirb + (P.lm / P.tauR) * isa;
    dpb = (-psirb / P.tauR) + wr * psira + (P.lm / P.tauR) * isb;
    dia = (vsa - P.rs * isa - P.kPsi * dpr) / P.lsig;
    dib = (vsb - P.rs * isb - P.kPsi * dpb) / P.lsig;
    te = 1.5 * P.np * P.kPsi * (psira * isb - psirb * isa);
    dw = (te - load) / P.J;
    d = [2 * pi * freq, dia, dib, dpr, dpb, dw];
end

function [irms, te] = motorSteady(freq, boost, slip, P)
    w = 2 * pi * freq;
    xs = w * P.lls;
    xr = w * P.llr;
    xm = w * P.lm;
    volt = motorVoltage(freq, boost, P);
    if slip <= 0
        zin = P.rs + 1j * (xs + xm);
        irms = abs(volt / zin);
        te = 0;
        return
    end
    zr = (P.rr / slip) + 1j * xr;
    zm = 1j * xm;
    zpar = 1 / (1 / zm + 1 / zr);
    zin = (P.rs + 1j * xs) + zpar;
    ist = volt / zin;
    ir = ist * zm / (zm + zr);
    pag = 3 * (abs(ir) ^ 2) * (P.rr / slip);
    te = pag / (w / P.np);
    irms = abs(ist);
end

function [slip, te] = motorTorqueMax(freq, boost, P)
    lo = 1e-6;
    hi = 1;
    for k = 1:80
        m1 = lo + (hi - lo) / 3;
        m2 = hi - (hi - lo) / 3;
        [~, t1] = motorSteady(freq, boost, m1, P);
        [~, t2] = motorSteady(freq, boost, m2, P);
        if t1 < t2
            lo = m1;
        else
            hi = m2;
        end
    end
    slip = 0.5 * (lo + hi);
    [~, te] = motorSteady(freq, boost, slip, P);
end

function slip = motorSlipFor(freq, boost, load, slipBd, P)
    lo = 0;
    hi = slipBd;
    for k = 1:70
        mid = 0.5 * (lo + hi);
        [~, te] = motorSteady(freq, boost, mid, P);
        if te < load
            lo = mid;
        else
            hi = mid;
        end
    end
    slip = 0.5 * (lo + hi);
end

function out = motorRk4(mode, tEnd, P)
    n = round(tEnd / P.h);
    x = zeros(1, 6);
    tOut = zeros(n, 1);
    nOut = zeros(n, 1);
    teOut = zeros(n, 1);
    iOut = zeros(n, 1);
    fOut = zeros(n, 1);
    for i = 0:(n - 1)
        t = i * P.h;
        [k1, ~] = motorDeriv(t, x, mode, P);
        x2 = x + 0.5 * P.h * k1;
        [k2, ~] = motorDeriv(t + 0.5 * P.h, x2, mode, P);
        x3 = x + 0.5 * P.h * k2;
        [k3, ~] = motorDeriv(t + 0.5 * P.h, x3, mode, P);
        x4 = x + P.h * k3;
        [k4, ~] = motorDeriv(t + P.h, x4, mode, P);
        coef = P.h / 6;
        x = x + coef * (k1 + 2 * k2 + 2 * k3 + k4);
        ts = t + P.h;
        te = 1.5 * P.np * P.kPsi * (x(4) * x(3) - x(5) * x(2));
        irms = hypot(x(2), x(3)) / sqrt(2);
        [freq, ~, ~] = motorSchedule(ts, mode, P);
        idx = i + 1;
        tOut(idx) = ts;
        nOut(idx) = x(6) * 60 / (2 * pi);
        teOut(idx) = te;
        iOut(idx) = irms;
        fOut(idx) = freq;
    end
    out.t = tOut;
    out.n = nOut;
    out.te = teOut;
    out.i = iOut;
    out.f = fOut;
end

function m = motorWindow(t, y, t0, t1)
    mask = (t >= t0) & (t < t1);
    m = mean(y(mask));
end

function motorPlot(rated, low0, low1, nOff, nOn, P)
    blue = [31, 78, 121] / 255;
    orange = [224, 122, 61] / 255;
    gray = [156, 163, 175] / 255;
    green = [91, 138, 114] / 255;
    figure('Color', 'w', 'Position', [80, 80, 960, 900]);

    subplot(3, 2, 1);
    plot(rated.t, rated.n, 'Color', blue, 'LineWidth', 1.2); hold on;
    plot(rated.t, rated.f * 60 / P.np, '--', 'Color', gray, 'LineWidth', 1.0);
    plot([P.tLoad, P.tLoad], [-40, 1680], ':', 'Color', orange);
    xlim([0, P.tEnd]); ylim([-40, 1680]); grid on;
    ylabel('转速 / (r/min)');
    title('转速    无提升斜坡到 50 Hz');
    legend('转速', '同步转速', '突加负载', 'Location', 'southeast');

    subplot(3, 2, 2);
    plot(rated.t, rated.te, 'Color', blue, 'LineWidth', 1.1); hold on;
    loadLine = zeros(size(rated.t));
    loadLine(rated.t >= P.tLoad) = P.tl;
    plot(rated.t, loadLine, '--', 'Color', orange, 'LineWidth', 1.0);
    xlim([0, P.tEnd]); ylim([-4, 28]); grid on;
    ylabel('转矩 / (N·m)');
    title('电磁转矩');
    legend('电磁转矩', '负载转矩', 'Location', 'northeast');

    subplot(3, 2, 3);
    plot(rated.t, rated.i, 'Color', blue, 'LineWidth', 1.1); hold on;
    ylineNoload = motorWindow(rated.t, rated.i, 2.0, 2.2);
    ylineLoad = motorWindow(rated.t, rated.i, 2.7, 3.0);
    plot([0, P.tEnd], [ylineNoload, ylineNoload], '--', 'Color', gray);
    plot([0, P.tEnd], [ylineLoad, ylineLoad], '--', 'Color', orange);
    xlim([0, P.tEnd]); ylim([0, 8]); grid on;
    ylabel('电流 / A');
    title('定子相电流有效值');
    legend('相电流有效值', '空载', '带载', 'Location', 'northeast');

    subplot(3, 2, 4);
    freq = linspace(0, P.fn, 200);
    vOff = zeros(size(freq));
    vOn = zeros(size(freq));
    for k = 1:numel(freq)
        vOff(k) = motorVoltage(freq(k), false, P) * sqrt(3);
        vOn(k) = motorVoltage(freq(k), true, P) * sqrt(3);
    end
    plot(freq, vOff, 'Color', blue, 'LineWidth', 1.3); hold on;
    plot(freq, vOn, 'Color', orange, 'LineWidth', 1.3);
    plot(P.fLow, motorVoltage(P.fLow, false, P) * sqrt(3), 'o', 'Color', blue);
    plot(P.fLow, motorVoltage(P.fLow, true, P) * sqrt(3), 'o', 'Color', orange);
    plot(P.fn, P.vll, 'o', 'Color', blue);
    xlim([0, P.fn]); ylim([0, 420]); grid on;
    xlabel('频率 f / Hz'); ylabel('线电压有效值 / V');
    title('V/f');
    legend('无补偿', '低频提升', 'Location', 'northwest');

    subplot(3, 2, 5);
    plot(low0.t, low0.n, 'Color', blue, 'LineWidth', 1.2); hold on;
    plot(low1.t, low1.n, 'Color', orange, 'LineWidth', 1.2);
    plot([0, P.tLowEnd], [150, 150], '--', 'Color', gray);
    plot([P.tLowOn, P.tLowOn], [-10, 200], ':', 'Color', green);
    xlim([0, P.tLowEnd]); ylim([-10, 200]); grid on;
    xlabel('时间 t / s'); ylabel('转速 / (r/min)');
    title('5 Hz 转速');
    legend('无补偿', '低频提升', '同步 150 r/min', '加上 8 N·m', 'Location', 'northeast');

    subplot(3, 2, 6);
    slips = linspace(0.004, 1, 500);
    teOff = zeros(size(slips));
    teOn = zeros(size(slips));
    for k = 1:numel(slips)
        [~, teOff(k)] = motorSteady(P.fLow, false, slips(k), P);
        [~, teOn(k)] = motorSteady(P.fLow, true, slips(k), P);
    end
    speed = (1 - slips) * (P.fLow * 60 / P.np);
    plot(speed, teOff, 'Color', blue, 'LineWidth', 1.2); hold on;
    plot(speed, teOn, 'Color', orange, 'LineWidth', 1.2);
    plot([0, 160], [P.tlLow, P.tlLow], '--', 'Color', green);
    plot(nOff, P.tlLow, 'o', 'Color', blue);
    plot(nOn, P.tlLow, 'o', 'Color', orange);
    xlim([0, 160]); ylim([0, 46]); grid on;
    xlabel('转速 / (r/min)'); ylabel('电磁转矩 / (N·m)');
    title('5 Hz 转矩-转速');
    legend('无补偿', '低频提升', '8 N·m', 'Location', 'northeast');
end

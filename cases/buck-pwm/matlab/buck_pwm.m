% buck_pwm.m  理想 Buck 开环 PWM（教学演示，仅用 MATLAB 基础函数）
%
% 开关模型，无损耗、电容无 ESR。状态是 iL 和 vo。
%   开关导通:       vL = Vin - vo
%   关断且 iL > 0:  vL = -vo
%   两端都断:       vL = 0，iL 保持 0（DCM）
%   L diL/dt = vL
%   C dvo/dt = iL - vo/R
%
% CCM 教科书公式（vo 在一个周期内看成常数）:
%   Vo  = D*Vin
%   diL = Vin*D*(1-D)/(L*fs)
%   dvo = diL/(8*C*fs)
% DCM（K = 2*L*fs/R < 1-D）:
%   Vo/Vin = 2 / (1 + sqrt(1 + 4*K/D^2))
%
% 默认参数与案例卡一致:
%   Vin = 12 V，D = 0.5，fs = 10 kHz，L = 1e-3 H，C = 25e-6 F，R = 6 ohm
%   Vo = 6 V，diL = 0.3 A，dvo = 0.15 V，Rcrit = 40 ohm
%   R = 80 ohm 用作断续对照
%
% 定步长 RK4，步长 T/200，开关沿对齐步长。CCM 主工况用 expm 对照 RK4。
%
% 运行: 在 MATLAB 中打开本文件并按 Run，或
%   run('cases/buck-pwm/matlab/buck_pwm.m')

clear; clc;

Vin = 12;          % V
duty = 0.5;
fs = 10000;        % Hz
L = 1e-3;          % H
C = 25e-6;         % F
Rccm = 6;          % ohm
Rdcm = 80;         % ohm
dutyLow = 0.25;
dutyHigh = 0.75;
nPer = 200;
nCycles = 50;      % 5 ms
nCyclesDcm = 400;  % 40 ms
Tsw = 1 / fs;
h = Tsw / nPer;

[tCcm, ilCcm, voCcm] = buckSimulate(duty, Rccm, nCycles, Vin, L, C, fs, nPer, h);
[ilRef, voRef] = buckExpmCcm(duty, Rccm, nCycles, Vin, L, C, fs, nPer, h);
errI = max(abs(ilCcm - ilRef));
errV = max(abs(voCcm - voRef));

[~, ilS, voS] = buckWindow(tCcm, ilCcm, voCcm, nCycles, 2, Tsw);
st = buckStats(ilS, voS);
[voTh, diTh, dvTh] = buckRipple(duty, Vin, L, C, fs);
iTh = voTh / Rccm;
Rcrit = 2 * L * fs / (1 - duty);
Icrit = voTh / Rcrit;
Kccm = 2 * L * fs / Rccm;
avg = buckAveragedPeak(L, C, Rccm, voTh);
[~, peakIdx] = max(voCcm);

[tLo, ilLo, voLo] = buckSimulate(dutyLow, Rccm, nCycles, Vin, L, C, fs, nPer, h);
[~, ilLoS, voLoS] = buckWindow(tLo, ilLo, voLo, nCycles, 2, Tsw);
stLo = buckStats(ilLoS, voLoS);
[voLoTh, diLoTh, dvLoTh] = buckRipple(dutyLow, Vin, L, C, fs);

[tHi, ilHi, voHi] = buckSimulate(dutyHigh, Rccm, nCycles, Vin, L, C, fs, nPer, h);
[~, ilHiS, voHiS] = buckWindow(tHi, ilHi, voHi, nCycles, 2, Tsw);
stHi = buckStats(ilHiS, voHiS);
[voHiTh, diHiTh, dvHiTh] = buckRipple(dutyHigh, Vin, L, C, fs);

[tDcm, ilDcm, voDcm] = buckSimulate(duty, Rdcm, nCyclesDcm, Vin, L, C, fs, nPer, h);
[~, ilDS, voDS] = buckWindow(tDcm, ilDcm, voDcm, nCyclesDcm, 2, Tsw);
stD = buckStats(ilDS, voDS);
[Kdcm, Mdcm, voDcmTh] = buckDcmGain(duty, Rdcm, Vin, L, fs);

keys = { ...
    'Vin_V', Vin; ...
    'duty', duty; ...
    'fs_Hz', fs; ...
    'L_H', L; ...
    'C_F', C; ...
    'R_ohm', Rccm; ...
    'Vo_theory_V', voTh; ...
    'iL_theory_A', iTh; ...
    'delta_iL_theory_A', diTh; ...
    'delta_vo_theory_V', dvTh; ...
    'iL_min_theory_A', iTh - 0.5 * diTh; ...
    'iL_max_theory_A', iTh + 0.5 * diTh; ...
    'Rcrit_ohm', Rcrit; ...
    'Icrit_A', Icrit; ...
    'K_ccm', Kccm; ...
    'omega0_rad_s', avg.omega0; ...
    'zeta', avg.zeta; ...
    'tp_avg_ms', avg.tp_ms; ...
    'os_avg_frac', avg.os_frac; ...
    'vo_peak_avg_model_V', avg.vpeak; ...
    'Vo_meas_V', st.Vo; ...
    'iL_meas_A', st.iL; ...
    'delta_iL_meas_A', st.diL; ...
    'delta_vo_meas_V', st.dvo; ...
    'iL_min_meas_A', st.imin; ...
    'iL_max_meas_A', st.imax; ...
    'rel_err_Vo', (st.Vo - voTh) / voTh; ...
    'rel_err_diL', (st.diL - diTh) / diTh; ...
    'rel_err_dvo', (st.dvo - dvTh) / dvTh; ...
    'vo_peak_switched_V', voCcm(peakIdx); ...
    't_peak_ms', tCcm(peakIdx) * 1e3; ...
    'duty_low', dutyLow; ...
    'Vo_theory_low_V', voLoTh; ...
    'delta_iL_theory_low_A', diLoTh; ...
    'delta_vo_theory_low_V', dvLoTh; ...
    'Vo_meas_low_V', stLo.Vo; ...
    'delta_iL_meas_low_A', stLo.diL; ...
    'delta_vo_meas_low_V', stLo.dvo; ...
    'duty_high', dutyHigh; ...
    'Vo_theory_high_V', voHiTh; ...
    'delta_iL_theory_high_A', diHiTh; ...
    'delta_vo_theory_high_V', dvHiTh; ...
    'Vo_meas_high_V', stHi.Vo; ...
    'delta_iL_meas_high_A', stHi.diL; ...
    'delta_vo_meas_high_V', stHi.dvo; ...
    'R_dcm_ohm', Rdcm; ...
    'K_dcm', Kdcm; ...
    'M_dcm', Mdcm; ...
    'Vo_dcm_theory_V', voDcmTh; ...
    'Vo_dcm_if_ccm_V', duty * Vin; ...
    'Vo_dcm_meas_V', stD.Vo; ...
    'iL_max_dcm_A', stD.imax; ...
    'iL_min_dcm_A', stD.imin; ...
    'zero_frac_dcm', stD.zeroFrac ...
    };

fprintf('Buck open-loop PWM\n');
for k = 1:size(keys, 1)
    fprintf('  %-26s = %s\n', keys{k, 1}, fmt6(keys{k, 2}));
end
fprintf('  %-26s = %.3e\n', 'max_abs_err_il_A', errI);
fprintf('  %-26s = %.3e\n', 'max_abs_err_vo_V', errV);

il0Expected = (Vin / L) * h;
if abs(ilCcm(1) - il0Expected) > 1e-6
    error('buck_pwm:InitialSlope', '第一步的 di/dt 应为 Vin/L。');
end
if abs(st.Vo - voTh) > 1e-3 || abs(st.diL - diTh) / diTh > 0.02 || abs(st.dvo - dvTh) / dvTh > 0.02
    error('buck_pwm:CcmRipple', 'CCM 平均电压或纹波偏离教科书公式。');
end
if st.imin <= 0.5 * diTh
    error('buck_pwm:Ccm', 'R = 6 ohm 应连续导通。');
end
if errI > 1e-4 || errV > 1e-4
    error('buck_pwm:Expm', 'RK4 与 expm 不一致（电流 %.3e A，电压 %.3e V）。', errI, errV);
end
if abs(stLo.Vo - voLoTh) > 1e-3 || abs(stHi.Vo - voHiTh) > 1e-3
    error('buck_pwm:Duty', '平均电压没有跟着占空比走。');
end
if abs(stLo.diL - diLoTh) / diLoTh > 0.02 || abs(stHi.diL - diHiTh) / diHiTh > 0.02
    error('buck_pwm:DutyRipple', 'D = 0.25 或 0.75 的电流纹波偏离公式。');
end
if abs(stLo.diL - stHi.diL) > 1e-3 || st.diL <= stLo.diL
    error('buck_pwm:Symmetry', '电流纹波应在 D = 0.5 最大，且 0.25 与 0.75 对称。');
end
if Kdcm >= 1 - duty || stD.imin > 1e-9 || stD.zeroFrac < 0.15 || stD.zeroFrac > 0.25
    error('buck_pwm:Dcm', 'R = 80 ohm 应进入断续，电流回到 0。');
end
if abs(stD.Vo - voDcmTh) / voDcmTh > 0.01 || stD.Vo < duty * Vin + 1
    error('buck_pwm:DcmVo', '断续电压应高于 D*Vin，并接近 DCM 公式。');
end
if voCcm(peakIdx) < 6.5 || voCcm(peakIdx) > 7.5
    error('buck_pwm:Startup', '启动峰值不在平均模型附近。');
end

voThBand = 0.5 * dvTh;
[tZoom, ilZoom, voZoom] = buckWindow(tCcm, ilCcm, voCcm, nCycles, 4, Tsw);
iMinTri = iTh - 0.5 * diTh;
ilTri = idealIl(tZoom, Tsw, duty, iMinTri, diTh);
[tDzoom, ilDzoom, voDzoom] = buckWindow(tDcm, ilDcm, voDcm, nCyclesDcm, 4, Tsw);

figure('Name', 'Buck open-loop PWM', 'Color', 'w');
navy = [0.12 0.31 0.47];
orange = [0.88 0.48 0.24];
gray = [0.61 0.64 0.67];

subplot(3, 2, 1);
plot(tCcm * 1e3, voCcm, 'Color', navy, 'LineWidth', 1.2); hold on;
plot([0, nCycles * Tsw * 1e3], [voTh, voTh], 'Color', gray, 'LineWidth', 0.9);
plot(tCcm(peakIdx) * 1e3, voCcm(peakIdx), 'o', 'MarkerSize', 5, ...
    'MarkerEdgeColor', navy, 'MarkerFaceColor', navy);
grid on;
xlim([0, nCycles * Tsw * 1e3]); ylim([-0.4, 7.8]);
ylabel('vo / V');
title('启动    CCM，D = 0.5，R = 6 \Omega');
legend('RK4 开关模型', 'D·Vin = 6 V', '启动峰值', 'Location', 'southeast');

subplot(3, 2, 2);
plot(tCcm * 1e3, ilCcm, 'Color', navy, 'LineWidth', 1.2); hold on;
plot([0, nCycles * Tsw * 1e3], [iTh, iTh], 'Color', gray, 'LineWidth', 0.9);
grid on;
xlim([0, nCycles * Tsw * 1e3]); ylim([-0.05, 1.65]);
ylabel('iL / A');
title('启动    电感电流');
legend('RK4 开关模型', '平均 1 A', 'Location', 'northeast');

subplot(3, 2, 3);
plot(tZoom * 1e3, voZoom, 'Color', navy, 'LineWidth', 1.4); hold on;
plot(tZoom([1, end]) * 1e3, [voTh, voTh], 'Color', gray, 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
plot(tZoom([1, end]) * 1e3, [voTh + voThBand, voTh + voThBand], '--', ...
    'Color', orange, 'LineWidth', 1.0);
plot(tZoom([1, end]) * 1e3, [voTh - voThBand, voTh - voThBand], '--', ...
    'Color', orange, 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid on;
ylim([voTh - 0.16, voTh + 0.16]);
ylabel('vo / V');
title('稳态纹波    最后 4 个周期');
legend('RK4', '公式峰 / 谷', 'Location', 'northeast');

subplot(3, 2, 4);
plot(tZoom * 1e3, ilZoom, 'Color', navy, 'LineWidth', 1.4); hold on;
plot(tZoom * 1e3, ilTri, '--', 'Color', orange, 'LineWidth', 1.1);
grid on;
ylim([0.78, 1.22]);
ylabel('iL / A');
title('稳态纹波    电感电流');
legend('RK4', '公式折线', 'Location', 'northeast');

subplot(3, 2, 5);
plot(tDzoom * 1e3, voDzoom, 'Color', navy, 'LineWidth', 1.4); hold on;
plot(tDzoom([1, end]) * 1e3, [voDcmTh, voDcmTh], '--', 'Color', orange, 'LineWidth', 1.0);
plot(tDzoom([1, end]) * 1e3, [duty * Vin, duty * Vin], ':', 'Color', gray, 'LineWidth', 1.0);
grid on;
ylim([5.6, 7.9]);
xlabel('t / ms'); ylabel('vo / V');
title('轻载断续    R = 80 \Omega，D = 0.5');
legend('RK4', 'DCM 公式', '误用 D·Vin', 'Location', 'east');

subplot(3, 2, 6);
plot(tDzoom * 1e3, ilDzoom, 'Color', navy, 'LineWidth', 1.4); hold on;
plot(tDzoom([1, end]) * 1e3, [0, 0], 'Color', gray, 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
grid on;
ylim([-0.02, 0.30]);
xlabel('t / ms'); ylabel('iL / A');
title('轻载断续    电流每个周期回到 0');
legend('RK4', 'Location', 'northeast');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outFile = fullfile(outDir, 'buck_pwm_curves_matlab.png');
% print 属于基础绘图，避免依赖较新的 exportgraphics。
print(gcf, outFile, '-dpng', '-r140');
fprintf('  %-26s = %s\n', 'figure', outFile);

% --- 局部函数（R2016b 及以后的基础 MATLAB，不是工具箱） ---

function text = fmt6(value)
    if abs(value) < 5e-7
        text = '0.000000';
    else
        text = sprintf('%.6f', value);
    end
end

function [vo, diL, dvo] = buckRipple(duty, Vin, L, C, fs)
    vo = duty * Vin;
    diL = Vin * duty * (1 - duty) / (L * fs);
    dvo = diL / (8 * C * fs);
end

function [K, M, vo] = buckDcmGain(duty, resistance, Vin, L, fs)
    K = 2 * L * fs / resistance;
    M = 2 / (1 + sqrt(1 + 4 * K / (duty * duty)));
    vo = M * Vin;
end

function avg = buckAveragedPeak(L, C, resistance, voSteady)
    w0 = 1 / sqrt(L * C);
    zeta = sqrt(L / C) / (2 * resistance);
    wd = w0 * sqrt(1 - zeta * zeta);
    osFrac = exp(-zeta * pi / sqrt(1 - zeta * zeta));
    avg.omega0 = w0;
    avg.zeta = zeta;
    avg.tp_ms = (pi / wd) * 1e3;
    avg.os_frac = osFrac;
    avg.vpeak = voSteady * (1 + osFrac);
end

function [di, dv] = buckDeriv(switchOn, current, voltage, idle, resistance, Vin, L, C)
    if idle
        di = 0;
        dv = -voltage / (resistance * C);
        return
    end
    if switchOn
        vL = Vin - voltage;
    else
        vL = -voltage;
    end
    di = vL / L;
    dv = (current - voltage / resistance) / C;
end

function [ilN, voN] = buckRk4(switchOn, il, vo, idle, resistance, step, Vin, L, C)
    [k1i, k1v] = buckDeriv(switchOn, il, vo, idle, resistance, Vin, L, C);
    [k2i, k2v] = buckDeriv(switchOn, il + 0.5 * step * k1i, vo + 0.5 * step * k1v, ...
        idle, resistance, Vin, L, C);
    [k3i, k3v] = buckDeriv(switchOn, il + 0.5 * step * k2i, vo + 0.5 * step * k2v, ...
        idle, resistance, Vin, L, C);
    [k4i, k4v] = buckDeriv(switchOn, il + step * k3i, vo + step * k3v, ...
        idle, resistance, Vin, L, C);
    ilN = il + (step / 6) * (k1i + 2 * k2i + 2 * k3i + k4i);
    voN = vo + (step / 6) * (k1v + 2 * k2v + 2 * k3v + k4v);
end

function [t, ilOut, voOut] = buckSimulate(duty, resistance, nCycles, Vin, L, C, fs, nPer, step)
    if abs(duty * nPer - round(duty * nPer)) > 1e-9
        error('buck_pwm:Grid', '占空比必须让开通时间等于整数步。');
    end
    nOn = round(duty * nPer);
    nTot = nCycles * nPer;
    t = zeros(nTot, 1);
    ilOut = zeros(nTot, 1);
    voOut = zeros(nTot, 1);
    il = 0;
    vo = 0;
    p = 0;
    for k = 0:(nCycles - 1)
        for i = 0:(nPer - 1)
            switchOn = i < nOn;
            idle = (~switchOn) && (il <= 0);
            [il, vo] = buckRk4(switchOn, il, vo, idle, resistance, step, Vin, L, C);
            if (~switchOn) && (il < 0)
                il = 0;
            end
            p = p + 1;
            t(p) = (k * nPer + i + 1) * step;
            ilOut(p) = il;
            voOut(p) = vo;
        end
    end
    %#ok<*NASGU>
end

function Ad = buckDiscrete(A, b, step)
    M = zeros(3, 3);
    M(1:2, 1:2) = A;
    M(1:2, 3) = b(:);
    Ad = expm(M * step);
end

function [ilOut, voOut] = buckExpmCcm(duty, resistance, nCycles, Vin, L, C, fs, nPer, step)
    nOn = round(duty * nPer);
    A = [0, -1 / L; 1 / C, -1 / (resistance * C)];
    AdOn = buckDiscrete(A, [Vin / L; 0], step);
    AdOff = buckDiscrete(A, [0; 0], step);
    il = 0;
    vo = 0;
    nTot = nCycles * nPer;
    ilOut = zeros(nTot, 1);
    voOut = zeros(nTot, 1);
    p = 0;
    for k = 0:(nCycles - 1) %#ok<NASGU>
        for i = 0:(nPer - 1)
            if i < nOn
                Ad = AdOn;
            else
                Ad = AdOff;
            end
            aug = Ad * [il; vo; 1];
            il = aug(1);
            vo = aug(2);
            p = p + 1;
            ilOut(p) = il;
            voOut(p) = vo;
        end
    end
end

function [tz, ilz, voz] = buckWindow(t, il, vo, nCyclesTotal, nKeep, Tsw)
    tEnd = nCyclesTotal * Tsw;
    mask = t >= (tEnd - nKeep * Tsw);
    tz = t(mask);
    ilz = il(mask);
    voz = vo(mask);
end

function s = buckStats(il, vo)
    s.Vo = mean(vo);
    s.dvo = max(vo) - min(vo);
    s.iL = mean(il);
    s.diL = max(il) - min(il);
    s.imin = min(il);
    s.imax = max(il);
    s.zeroFrac = mean(il <= 0);
end

function iRef = idealIl(t, Tsw, duty, iMin, diL)
    tau = mod(t, Tsw);
    tOn = duty * Tsw;
    iRef = iMin + diL - diL .* ((tau - tOn) ./ (Tsw - tOn));
    rise = tau < tOn;
    iRef(rise) = iMin + diL .* (tau(rise) ./ tOn);
end

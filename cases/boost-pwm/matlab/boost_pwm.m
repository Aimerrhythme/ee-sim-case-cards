% boost_pwm.m  理想 Boost 开环 PWM（教学演示，仅用 MATLAB 基础函数）
%
% 开关模型，无损耗、电容无 ESR。状态是 iL 和 vo。
%   开关导通:                vL = Vin，       C dvo/dt = -vo/R
%   关断且二极管导通:        vL = Vin - vo，  C dvo/dt = iL - vo/R
%   关断、iL = 0 且 vo>Vin:  vL = 0，iL 保持 0（DCM）
%
% CCM 教科书公式:
%   Vo  = Vin/(1-D)          IL  = Vo/(R*(1-D))
%   diL = Vin*D/(L*fs)       dvo = (Vo/R)*D/(C*fs)
% DCM（K = 2*L*fs/R < D*(1-D)^2）:
%   Vo/Vin = (1 + sqrt(1 + 4*D^2/K)) / 2
%
% 默认参数与案例卡一致:
%   Vin = 12 V，D = 0.5，fs = 10 kHz，L = 1e-3 H，C = 100e-6 F，R = 48 ohm
%   Vo = 24 V，IL = 1 A，diL = 0.6 A，dvo = 0.25 V，Rcrit = 160 ohm
%   R = 320 ohm 用作断续对照
%
% 定步长 RK4，步长 T/200，开关沿对齐步长。
% 积分精度用「同步整流」版本（电流可为负，永远连续）对照 expm。
%
% 运行: 在 MATLAB 中打开本文件并按 Run，或
%   run('cases/boost-pwm/matlab/boost_pwm.m')

clear; clc;

P.Vin = 12;          % V
P.L = 1e-3;          % H
P.C = 100e-6;        % F
P.fs = 10000;        % Hz
P.Nper = 200;        % 每周期步数
P.T = 1 / P.fs;
P.h = P.T / P.Nper;

duty = 0.5;
Rccm = 48;           % ohm
Rdcm = 320;          % ohm
dutyLow = 0.25;
dutyHigh = 0.75;
nCycles = 1000;      % 100 ms，开环 Boost 阻尼很小
nCyclesDcm = 1500;   % 150 ms

% ---------- 主工况 ----------
[t, iL, vo] = simulate(P, duty, Rccm, nCycles, false);
[~, iLs, vos] = simulate(P, duty, Rccm, nCycles, true);
[iLref, voref] = expmCcm(P, duty, Rccm, nCycles);
errI = max(abs(iLs - iLref));
errV = max(abs(vos - voref));

th = ccmTheory(P, duty, Rccm);
Kcrit = duty * (1 - duty)^2;
Rcrit = 2 * P.L * P.fs / Kcrit;
Kccm = 2 * P.L * P.fs / Rccm;
avg = averagedModel(P, duty, Rccm);
[voPk, iPk] = max(vo);
zeroIdx = find(iL <= 0);
if isempty(zeroIdx)
    tDcmFrom = 0; tDcmTo = 0;
else
    tDcmFrom = t(zeroIdx(1)) * 1e3; tDcmTo = t(zeroIdx(end)) * 1e3;
end
st = steadyStats(P, t, iL, vo, nCycles, 2);

% ---------- 另外两个占空比 ----------
[tl, il_l, vo_l] = simulate(P, dutyLow, Rccm, nCycles, false);
stLo = steadyStats(P, tl, il_l, vo_l, nCycles, 2);
thLo = ccmTheory(P, dutyLow, Rccm);
[th_, il_h, vo_h] = simulate(P, dutyHigh, Rccm, nCycles, false);
stHi = steadyStats(P, th_, il_h, vo_h, nCycles, 2);
thHi = ccmTheory(P, dutyHigh, Rccm);

% ---------- 断续对照 ----------
[td, il_d, vo_d] = simulate(P, duty, Rdcm, nCyclesDcm, false);
stD = steadyStats(P, td, il_d, vo_d, nCyclesDcm, 2);
Kdcm = 2 * P.L * P.fs / Rdcm;
Mdcm = 0.5 * (1 + sqrt(1 + 4 * duty^2 / Kdcm));
VoDcmTh = Mdcm * P.Vin;

rows = {
    'Vin_V', P.Vin; 'duty', duty; 'fs_Hz', P.fs; 'L_H', P.L; 'C_F', P.C; 'R_ohm', Rccm;
    'Vo_theory_V', th.Vo; 'Io_theory_A', th.Io; 'iL_theory_A', th.IL;
    'delta_iL_theory_A', th.diL; 'delta_vo_theory_V', th.dvo;
    'iL_min_theory_A', th.IL - th.diL/2; 'iL_max_theory_A', th.IL + th.diL/2;
    'K_ccm', Kccm; 'K_crit', Kcrit; 'Rcrit_ohm', Rcrit;
    'omega0_rad_s', avg.w0; 'zeta', avg.zeta; 'tp_avg_ms', avg.tp_ms;
    'os_avg_frac', avg.os; 'vo_peak_avg_model_V', avg.vpk; 'settle_2pct_avg_ms', avg.ts_ms;
    'vo_peak_switched_V', voPk; 't_peak_ms', t(iPk) * 1e3; 'iL_peak_startup_A', max(iL);
    'startup_dcm_from_ms', tDcmFrom; 'startup_dcm_to_ms', tDcmTo;
    'iL_min_sync_startup_A', min(iLs); 'vo_peak_sync_V', max(vos);
    'Vo_meas_V', st.Vo; 'iL_meas_A', st.iL; 'delta_iL_meas_A', st.diL; 'delta_vo_meas_V', st.dvo;
    'iL_min_meas_A', st.imin; 'iL_max_meas_A', st.imax;
    'rel_err_Vo', (st.Vo - th.Vo)/th.Vo; 'rel_err_diL', (st.diL - th.diL)/th.diL;
    'rel_err_dvo', (st.dvo - th.dvo)/th.dvo;
    'duty_low', dutyLow; 'Vo_theory_low_V', thLo.Vo; 'iL_theory_low_A', thLo.IL;
    'delta_iL_theory_low_A', thLo.diL; 'delta_vo_theory_low_V', thLo.dvo;
    'Vo_meas_low_V', stLo.Vo; 'iL_meas_low_A', stLo.iL;
    'delta_iL_meas_low_A', stLo.diL; 'delta_vo_meas_low_V', stLo.dvo;
    'duty_high', dutyHigh; 'Vo_theory_high_V', thHi.Vo; 'iL_theory_high_A', thHi.IL;
    'delta_iL_theory_high_A', thHi.diL; 'delta_vo_theory_high_V', thHi.dvo;
    'Vo_meas_high_V', stHi.Vo; 'iL_meas_high_A', stHi.iL;
    'delta_iL_meas_high_A', stHi.diL; 'delta_vo_meas_high_V', stHi.dvo;
    'R_dcm_ohm', Rdcm; 'K_dcm', Kdcm; 'M_dcm', Mdcm; 'Vo_dcm_theory_V', VoDcmTh;
    'Vo_dcm_if_ccm_V', th.Vo; 'Vo_dcm_meas_V', stD.Vo;
    'iL_max_dcm_A', stD.imax; 'iL_min_dcm_A', stD.imin; 'zero_frac_dcm', stD.zeroFrac
};

fprintf('Boost open-loop PWM\n');
for k = 1:size(rows, 1)
    v = rows{k, 2};
    if abs(v) < 5e-7, v = 0; end
    fprintf('  %-26s = %.6f\n', rows{k, 1}, v);
end
fprintf('  %-26s = %.3e\n', 'max_abs_err_il_A', errI);
fprintf('  %-26s = %.3e\n', 'max_abs_err_vo_V', errV);

% ---------- 校验 ----------
problems = {};
if abs(iL(1) - P.Vin / P.L * P.h) > 1e-6, problems{end+1} = '第一步的 di/dt 不是 Vin/L'; end
if abs(st.Vo - th.Vo) / th.Vo > 0.005, problems{end+1} = 'CCM 平均电压偏离 Vin/(1-D)'; end
if abs(st.diL - th.diL) / th.diL > 0.03, problems{end+1} = 'ΔiL 偏离公式超过 3%'; end
if abs(st.dvo - th.dvo) / th.dvo > 0.03, problems{end+1} = 'Δvo 偏离公式超过 3%'; end
if st.imin <= 0, problems{end+1} = 'R = 48 Ω 稳态应连续导通'; end
if errI > 1e-4 || errV > 1e-4, problems{end+1} = 'RK4 与 expm 不一致'; end
if abs(stLo.Vo - thLo.Vo) / thLo.Vo > 0.005 || abs(stHi.Vo - thHi.Vo) / thHi.Vo > 0.005
    problems{end+1} = 'D = 0.25 或 0.75 的平均电压没有跟着 Vin/(1-D) 走';
end
if Kdcm >= Kcrit, problems{end+1} = 'R = 320 Ω 的 K 应小于 D(1-D)^2'; end
if stD.imin > 1e-9, problems{end+1} = '断续工况电流应回到 0'; end
if abs(stD.Vo - VoDcmTh) / VoDcmTh > 0.01, problems{end+1} = '断续电压偏离 DCM 公式超过 1%'; end

% ---------- 作图 ----------
fig = figure('Name', 'boost-pwm', 'Position', [80 60 960 920]);
subplot(3, 2, 1);
plot(t * 1e3, vo, 'Color', [0.12 0.31 0.47]); hold on;
plot([0 nCycles * P.T * 1e3], [th.Vo th.Vo], 'Color', [0.6 0.6 0.6]);
plot(t(iPk) * 1e3, voPk, 'o', 'Color', [0.75 0.22 0.17], 'MarkerFaceColor', [0.75 0.22 0.17]);
ylim([-1 48]); grid on; ylabel('vo / V'); title('启动  CCM，D = 0.5，R = 48 Ω');
legend('开关模型', 'Vin/(1-D) = 24 V', '启动峰值', 'Location', 'northeast');

subplot(3, 2, 2);
plot(t * 1e3, iL, 'Color', [0.12 0.31 0.47]); hold on;
plot([0 nCycles * P.T * 1e3], [th.IL th.IL], 'Color', [0.6 0.6 0.6]);
grid on; ylabel('iL / A'); title('启动  电感电流');

mask = t >= (nCycles - 4) * P.T - P.h / 2;
subplot(3, 2, 3);
plot(t(mask) * 1e3, vo(mask), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.4); hold on;
vTop = max(vo(mask));
xl = [t(find(mask, 1)) t(end)] * 1e3;
plot(xl, [vTop vTop], '--', 'Color', [0.88 0.48 0.24]);
plot(xl, [vTop vTop] - th.dvo, '--', 'Color', [0.88 0.48 0.24]);
ylim([th.Vo - 0.3, th.Vo + 0.3]); grid on; ylabel('vo / V'); title('稳态纹波  最后 4 个周期');
legend('开关模型', '峰值 − 公式 Δvo', 'Location', 'northeast');

subplot(3, 2, 4);
plot(t(mask) * 1e3, iL(mask), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.4);
ylim([0.55 1.45]); grid on; ylabel('iL / A'); title('稳态纹波  电感电流');

maskD = td >= (nCyclesDcm - 4) * P.T - P.h / 2;
subplot(3, 2, 5);
plot(td(maskD) * 1e3, vo_d(maskD), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.4); hold on;
xd = [td(find(maskD, 1)) td(end)] * 1e3;
plot(xd, [VoDcmTh VoDcmTh], '--', 'Color', [0.88 0.48 0.24]);
plot(xd, [th.Vo th.Vo], ':', 'Color', [0.6 0.6 0.6]);
ylim([22 33]); grid on; xlabel('t / ms'); ylabel('vo / V'); title('轻载断续  R = 320 Ω');
legend('开关模型', 'DCM 公式', '误用 Vin/(1-D)', 'Location', 'east');

subplot(3, 2, 6);
plot(td(maskD) * 1e3, il_d(maskD), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.4);
ylim([-0.05 0.7]); grid on; xlabel('t / ms'); ylabel('iL / A'); title('轻载断续  电流每周期回到 0');

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
imgDir = fullfile(here, '..', 'img');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end
outPng = fullfile(imgDir, 'boost_pwm_curves_matlab.png');
saveas(fig, outPng);
fprintf('  %-26s = %s\n', 'figure', outPng);

if ~isempty(problems)
    error('校验未通过：%s', strjoin(problems, '；'));
end

% ======================= 局部函数 =======================
function th = ccmTheory(P, duty, R)
    th.Vo = P.Vin / (1 - duty);
    th.Io = th.Vo / R;
    th.IL = th.Io / (1 - duty);
    th.diL = P.Vin * duty / (P.L * P.fs);
    th.dvo = th.Io * duty / (P.C * P.fs);
end

function avg = averagedModel(P, duty, R)
    dp = 1 - duty;
    avg.w0 = dp / sqrt(P.L * P.C);
    avg.zeta = sqrt(P.L / P.C) / (2 * R * dp);
    wd = avg.w0 * sqrt(1 - avg.zeta^2);
    avg.os = exp(-avg.zeta * pi / sqrt(1 - avg.zeta^2));
    avg.tp_ms = pi / wd * 1e3;
    avg.vpk = P.Vin / dp * (1 + avg.os);
    avg.ts_ms = 4 / (avg.zeta * avg.w0) * 1e3;
end

function [di, dv] = deriv(P, mode, il, vo, R)
    if mode == 0
        di = P.Vin / P.L;          dv = -vo / (R * P.C);
    elseif mode == 1
        di = (P.Vin - vo) / P.L;   dv = (il - vo / R) / P.C;
    else
        di = 0;                    dv = -vo / (R * P.C);
    end
end

function [t, iLo, voo] = simulate(P, duty, R, nCycles, sync)
    nOn = round(duty * P.Nper);
    if abs(duty * P.Nper - nOn) > 1e-9
        error('占空比必须让开通时间等于整数步。');
    end
    n = nCycles * P.Nper;
    t = (1:n)' * P.h;
    iLo = zeros(n, 1); voo = zeros(n, 1);
    il = 0; vo = 0; h = P.h; p = 0;
    for k = 1:nCycles
        for i = 0:P.Nper-1
            if i < nOn
                mode = 0;
            elseif sync || il > 0 || vo < P.Vin
                mode = 1;
            else
                mode = 2;
            end
            [a1, b1] = deriv(P, mode, il, vo, R);
            [a2, b2] = deriv(P, mode, il + h/2*a1, vo + h/2*b1, R);
            [a3, b3] = deriv(P, mode, il + h/2*a2, vo + h/2*b2, R);
            [a4, b4] = deriv(P, mode, il + h*a3, vo + h*b3, R);
            il = il + h/6 * (a1 + 2*a2 + 2*a3 + a4);
            vo = vo + h/6 * (b1 + 2*b2 + 2*b3 + b4);
            if mode == 1 && il < 0 && ~sync
                il = 0;
            end
            p = p + 1;
            iLo(p) = il; voo(p) = vo;
        end
    end
end

function [iLo, voo] = expmCcm(P, duty, R, nCycles)
    nOn = round(duty * P.Nper);
    aOn = [0 0; 0 -1/(R*P.C)];
    aOff = [0 -1/P.L; 1/P.C -1/(R*P.C)];
    b = [P.Vin / P.L; 0];
    Mon = expm([aOn b; 0 0 0] * P.h);
    Moff = expm([aOff b; 0 0 0] * P.h);
    x = [0; 0; 1];
    n = nCycles * P.Nper;
    iLo = zeros(n, 1); voo = zeros(n, 1); p = 0;
    for k = 1:nCycles
        for i = 0:P.Nper-1
            if i < nOn, x = Mon * x; else, x = Moff * x; end
            p = p + 1;
            iLo(p) = x(1); voo(p) = x(2);
        end
    end
end

function s = steadyStats(P, t, iL, vo, nCycles, nKeep)
    m = t >= (nCycles - nKeep) * P.T - P.h / 2;
    s.Vo = mean(vo(m)); s.dvo = max(vo(m)) - min(vo(m));
    s.iL = mean(iL(m)); s.diL = max(iL(m)) - min(iL(m));
    s.imin = min(iL(m)); s.imax = max(iL(m));
    s.zeroFrac = mean(iL(m) <= 0);
end

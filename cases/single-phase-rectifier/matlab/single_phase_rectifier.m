% single_phase_rectifier.m  理想单相桥式整流 + 电容滤波（教学演示）
%
% 交流源 vs = Vm sin(wt)，理想二极管桥（Vf = 0），电容与负载并联。
% 主工况 Rs = 0：导通时 vC = |vs|，截止时按 RC 放电。
% 不接电容时平均值是 2*Vm/pi。
% 课堂估算（放电时间当成半个周期、电流当成 Vm/R）：
%   Io = Vm/R = 1 A，dV = Io/(2*f*C) = 10 V，Vdc = Vm - dV/2 = 95 V
% 估算偏大。脚本用导通角解析解和同一套步进对照。
% 峰值合闸（初相 90°）讨论冲击时串 Rs = 1 ohm，i(0+) = Vm/Rs = 100 A。
%
% 只用基础函数（sin/exp/atan、定步长循环、plot），不需要工具箱。
% 文件末尾的局部函数从 R2016b 起属于基础语言。
%
% 运行:
%   run('cases/single-phase-rectifier/matlab/single_phase_rectifier.m')

clear; clc;

P.Vm = 100;            % V
P.f = 50;              % Hz
P.R = 100;             % ohm
P.C = 1000e-6;         % F
P.C2 = 2000e-6;        % F
P.RsIn = 1;            % ohm，只用于峰值合闸
P.phiIn = pi/2;        % rad
P.dt = 5e-6;           % s
P.tEnd = 0.2;          % s
P.omega = 2 * pi * P.f;
P.nSteady = 2;

IoEst = P.Vm / P.R;
dVEst = IoEst / (2 * P.f * P.C);
VdcEst = P.Vm - 0.5 * dVEst;
dVEst2 = IoEst / (2 * P.f * P.C2);
VdcEst2 = P.Vm - 0.5 * dVEst2;
VavgNocThy = 2 * P.Vm / pi;

thy = conduction(P, P.C);
thy2 = conduction(P, P.C2);
[t, vc, idc] = simulateIdeal(P, P.C);
[t2, vc2, idc2] = simulateIdeal(P, P.C2);
st = steadyStats(P, t, vc, idc);
st2 = steadyStats(P, t2, vc2, idc2);

mNoc = steadyMask(P, t);
VavgNoc = mean(abs(P.Vm * sin(P.omega * t(mNoc))));
iFirst = max(idc(t <= 5e-3 + 0.5 * P.dt));
mExp = steadyMask(P, t);
errExp = expError(t(mExp), vc(mExp), idc(mExp), P.R * P.C);
mExp2 = steadyMask(P, t2);
errExp2 = expError(t2(mExp2), vc2(mExp2), idc2(mExp2), P.R * P.C2);

[tRs, vcRs, idcRs] = simulateRs(P, P.RsIn, 0, P.C);
[tPk, vcPk, idcPk] = simulateRs(P, P.RsIn, P.phiIn, P.C);
stRs = steadyStats(P, tRs, vcRs, idcRs);
stPk = steadyStats(P, tPk, vcPk, idcPk);
iStartRs = max(idcRs(tRs <= 0.01 + 0.5 * P.dt));
idx90 = find(vcPk >= 90, 1, 'first');
if isempty(idx90)
    tVc90 = inf;
else
    tVc90 = tPk(idx90) * 1e3;
end
% Rs 序列含 t = 0，MATLAB 下标比 t/dt 大 1。
vcIn2 = vcPk(round(0.002 / P.dt) + 1);
vcIn10 = vcPk(round(0.010 / P.dt) + 1);

rows = {
    'Vm_V', P.Vm; 'f_Hz', P.f; 'R_ohm', P.R; 'C_F', P.C; 'Rs_main_ohm', 0;
    'omega_rad_s', P.omega; 'Io_est_A', IoEst; 'dV_est_V', dVEst; 'Vdc_est_V', VdcEst;
    'Vavg_noc_theory_V', VavgNocThy; 'Vavg_noc_meas_V', VavgNoc;
    'theta1_deg', thy.theta1 * 180 / pi; 'theta2_deg', thy.theta2 * 180 / pi;
    't_on_ms', thy.tOn * 1e3; 't_off_ms', thy.tOff * 1e3; 'cond_frac_theory', thy.cond;
    'Vmax_theory_V', thy.vmax; 'Vmin_theory_V', thy.vmin; 'dV_theory_V', thy.dV;
    'Vavg_theory_V', thy.vavg; 'Io_theory_A', thy.vavg / P.R;
    'i_ss_peak_theory_A', thy.iSs; 'i0_theory_A', thy.i0; 'i_first_peak_theory_A', thy.iFirst;
    'Vmax_meas_V', st.Vmax; 'Vmin_meas_V', st.Vmin; 'dV_meas_V', st.dV; 'Vavg_meas_V', st.Vavg;
    'i_ss_peak_meas_A', st.iPeak; 'i_first_peak_meas_A', iFirst; 'cond_frac_meas', st.cond;
    'rel_err_Vavg_vs_est', (st.Vavg - VdcEst) / VdcEst;
    'rel_err_dV_vs_est', (st.dV - dVEst) / dVEst;
    'rel_err_Vavg_vs_thy', (st.Vavg - thy.vavg) / thy.vavg;
    'rel_err_dV_vs_thy', (st.dV - thy.dV) / thy.dV;
    'C2_F', P.C2; 'dV_est_C2_V', dVEst2; 'Vdc_est_C2_V', VdcEst2;
    'dV_theory_C2_V', thy2.dV; 'Vavg_theory_C2_V', thy2.vavg; 'Vmin_theory_C2_V', thy2.vmin;
    'dV_meas_C2_V', st2.dV; 'Vavg_meas_C2_V', st2.Vavg; 'Vmin_meas_C2_V', st2.Vmin;
    'Rs_inrush_ohm', P.RsIn; 'phi_inrush_deg', P.phiIn * 180 / pi;
    'i_inrush_theory_A', P.Vm / P.RsIn; 'i_inrush_peak_A', max(idcPk);
    't_vc90_inrush_ms', tVc90; 'vc_inrush_2ms_V', vcIn2; 'vc_inrush_10ms_V', vcIn10;
    'Vavg_Rs_meas_V', stRs.Vavg; 'dV_Rs_meas_V', stRs.dV;
    'Vmax_Rs_meas_V', stRs.Vmax; 'Vmin_Rs_meas_V', stRs.Vmin;
    'i_ss_Rs_A', stRs.iPeak; 'i_start_Rs_A', iStartRs; 'Vavg_Rs90_meas_V', stPk.Vavg
};

fprintf('Single-phase bridge rectifier + C filter\n');
for k = 1:size(rows, 1)
    v = rows{k, 2};
    if abs(v) < 5e-7, v = 0; end
    fprintf('  %-26s = %.6f\n', rows{k, 1}, v);
end
fprintf('  %-26s = %.3e\n', 'exp_err_V', errExp);
fprintf('  %-26s = %.3e\n', 'exp_err_C2_V', errExp2);

% ---------- 作图 ----------
fig = figure('Name', 'single-phase-rectifier', 'Position', [80 60 960 920]);
vrect = abs(P.Vm * sin(P.omega * t));

subplot(3, 2, 1);
m = t <= 0.04 + 0.5 * P.dt;
plot(t(m) * 1e3, vrect(m), 'Color', [0.61 0.64 0.67]); hold on;
plot(t(m) * 1e3, vc(m), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.2);
plot([0 40], [VdcEst VdcEst], '--', 'Color', [0.88 0.48 0.24]);
plot([0 40], [VavgNocThy VavgNocThy], ':', 'Color', [0.36 0.54 0.45]);
xlim([0 40]); ylim([-2 115]); grid on;
ylabel('电压 / V'); title('启动  过零合闸，Rs = 0');
legend('|v_s|', 'vC，接上 C', '估算平均 95 V', '不接 C 的平均', 'Location', 'southeast');

subplot(3, 2, 2);
plot(t(m) * 1e3, idc(m), 'Color', [0.12 0.31 0.47]); hold on;
plot([0 40], [thy.i0 thy.i0], '--', 'Color', [0.88 0.48 0.24]);
xlim([0 40]); grid on;
ylabel('电流 / A'); title('启动电流  第一下最大');
legend('桥电流 iD', 'C\omegaVm = 10\pi A', 'Location', 'northeast');

subplot(3, 2, 3);
ms = steadyMask(P, t);
m2 = steadyMask(P, t2);
plot(t(ms) * 1e3, vrect(ms), 'Color', [0.61 0.64 0.67]); hold on;
plot(t(ms) * 1e3, vc(ms), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.3);
plot(t2(m2) * 1e3, vc2(m2), 'Color', [0.88 0.48 0.24], 'LineWidth', 1.2);
xl = [t(find(ms, 1)) t(find(ms, 1, 'last'))] * 1e3;
plot(xl, [90 90], ':', 'Color', [0.75 0.22 0.17]);
ylim([86 104]); grid on;
ylabel('电压 / V'); title('稳态  最后两个工频周期');
legend('|v_s|', 'C = 1000 μF', 'C = 2000 μF', '估算谷值 90 V', 'Location', 'southwest');

subplot(3, 2, 4);
plot(t(ms) * 1e3, idc(ms), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.2);
grid on; ylabel('电流 / A'); title('稳态  电流只在顶点附近出现');

subplot(3, 2, 5);
mn = t <= 0.04 + 0.5 * P.dt;
plot(t(mn) * 1e3, vrect(mn), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.2); hold on;
plot([0 40], [VavgNocThy VavgNocThy], '--', 'Color', [0.88 0.48 0.24]);
plot([0 40], [thy.vavg thy.vavg], ':', 'Color', [0.36 0.54 0.45]);
xlim([0 40]); ylim([-2 115]); grid on;
xlabel('t / ms'); ylabel('电压 / V'); title('不接电容  馒头波');
legend('|v_s|', '平均 2Vm/\pi', '接上 C 之后的平均', 'Location', 'northwest');

subplot(3, 2, 6);
early = tPk <= 0.02 + 0.5 * P.dt;
yyaxis left;
plot(tPk(early) * 1e3, idcPk(early), 'Color', [0.75 0.22 0.17], 'LineWidth', 1.2); hold on;
plot([0 20], [100 100], '--', 'Color', [0.88 0.48 0.24]);
ylabel('电流 / A');
yyaxis right;
plot(tPk(early) * 1e3, vcPk(early), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.1);
ylim([-2 115]); ylabel('vC / V');
xlim([0 20]); grid on;
xlabel('t / ms'); title('峰值合闸  初相 90°，串 1 Ω');

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
imgDir = fullfile(here, '..', 'img');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end
outPng = fullfile(imgDir, 'single_phase_rectifier_curves_matlab.png');
saveas(fig, outPng);
fprintf('  %-26s = %s\n', 'figure', outPng);

% ---------- 校验 ----------
problems = {};
if abs(IoEst - 1) > 1e-12 || abs(dVEst - 10) > 1e-12 || abs(VdcEst - 95) > 1e-12
    problems{end+1} = '主工况手算没有落在 Io = 1 A、ΔV = 10 V、Vdc = 95 V';
end
if abs(dVEst2 - 5) > 1e-12 || abs(VdcEst2 - 97.5) > 1e-12
    problems{end+1} = '电容加倍后的手算不是 ΔV = 5 V、Vdc = 97.5 V';
end
if abs(thy.i0 - 10 * pi) > 1e-9
    problems{end+1} = '过零合闸的 C·ω·Vm 应等于 10π';
end
if abs(VavgNoc - VavgNocThy) > 1e-4
    problems{end+1} = '不接电容的平均值偏离 2Vm/π';
end
if abs(st.Vmax - P.Vm) > 1e-6
    problems{end+1} = '理想桥的稳态峰值应到 Vm';
end
if abs(st.Vmin - thy.vmin) > 2e-3 || abs(st.Vavg - thy.vavg) > 2e-3
    problems{end+1} = '稳态电压偏离导通角解析解';
end
if abs(st.dV - thy.dV) > 2e-3
    problems{end+1} = '纹波偏离导通角解析解';
end
if ~(st.dV > 0.7 * dVEst && st.dV < dVEst)
    problems{end+1} = '实测纹波应小于手算 10 V，又不能小得离谱';
end
if abs(st.Vavg - VdcEst) / VdcEst > 0.02
    problems{end+1} = '平均电压离手算 95 V 超过 2%';
end
if abs(iFirst - thy.iFirst) > 0.05
    problems{end+1} = '第一下充电电流偏离 sqrt(1+(ωRC)^2)·Vm/R';
end
if abs(st.iPeak - thy.iSs) > 0.15
    problems{end+1} = '稳态电流脉冲峰值偏离导通起点的解析值';
end
if abs(st.cond - thy.cond) > 0.01
    problems{end+1} = '导通时间占比偏离解析导通角';
end
if errExp > 1e-6 || errExp2 > 1e-6
    problems{end+1} = '截止区间不是 RC 指数放电';
end
if abs(st2.dV - thy2.dV) > 2e-3 || abs(st2.Vavg - thy2.vavg) > 2e-3
    problems{end+1} = 'C = 2000 μF 偏离导通角解析解';
end
ratio = st2.dV / st.dV;
if ratio < 0.45 || ratio > 0.65
    problems{end+1} = '电容加倍后纹波应大约减半';
end
if max(vc) > P.Vm + 1e-6 || max(vc2) > P.Vm + 1e-6
    problems{end+1} = '理想二极管桥的电容电压不应超过 Vm';
end
if min(vc) < -1e-8
    problems{end+1} = '电容电压不应为负';
end
if abs(idcPk(1) - P.Vm / P.RsIn) > 1e-9
    problems{end+1} = '峰值合闸瞬间电流应等于 Vm/Rs = 100 A';
end
if max(idcPk) > P.Vm / P.RsIn + 1e-6
    problems{end+1} = '峰值合闸的电流峰值应出现在 t = 0+';
end
if ~(vcIn2 > 70 && vcIn2 < 85) 
    problems{end+1} = '峰值合闸后 2 ms，电容应停在 70–85 V，第一下充不满';
end
if vcIn10 < 90 || tVc90 > 12
    problems{end+1} = '串 1 Ω 时要到下一个半周才充到 90 V';
end
if abs(stRs.Vavg - stPk.Vavg) > 0.01
    problems{end+1} = '过零合闸和峰值合闸的稳态平均电压应相同';
end
if ~(stRs.Vavg > 90 && stRs.Vavg < thy.vavg)
    problems{end+1} = '串 1 Ω 后平均电压应低于理想值，仍远高于不接电容';
end
if iFirst <= st.iPeak
    problems{end+1} = '过零合闸的第一下电流应大于稳态脉冲';
end

if ~isempty(problems)
    error('校验未通过：%s', strjoin(problems, '；'));
end

% ======================= 局部函数 =======================
function thy = conduction(P, cap)
    wrc = P.omega * P.R * cap;
    theta2 = pi - atan(wrc);
    s2 = sin(theta2);
    lo = 1e-12;
    hi = pi / 2;
    if gap(lo, s2, theta2, wrc) * gap(hi, s2, theta2, wrc) > 0
        error('导通角方程没有落在 (0, pi/2) 里。');
    end
    for iter = 1:80
        mid = 0.5 * (lo + hi);
        if gap(lo, s2, theta2, wrc) * gap(mid, s2, theta2, wrc) <= 0
            hi = mid;
        else
            lo = mid;
        end
    end
    theta1 = 0.5 * (lo + hi);
    vmin = P.Vm * sin(theta1);
    intSin = P.Vm * (-cos(theta2) + cos(theta1));
    decay = exp(-(pi + theta1 - theta2) / wrc);
    intExp = P.Vm * s2 * wrc * (1 - decay);
    span = theta2 - theta1;
    thy.theta1 = theta1;
    thy.theta2 = theta2;
    thy.vmin = vmin;
    thy.vmax = P.Vm;
    thy.dV = P.Vm - vmin;
    thy.vavg = (intSin + intExp) / pi;
    thy.cond = span / pi;
    thy.tOn = span / P.omega;
    thy.tOff = (pi - span) / P.omega;
    thy.iSs = cap * P.Vm * P.omega * cos(theta1) + P.Vm * sin(theta1) / P.R;
    thy.i0 = cap * P.omega * P.Vm;
    thy.iFirst = (P.Vm / P.R) * sqrt(1 + wrc * wrc);
end

function g = gap(theta1, s2, theta2, wrc)
    g = sin(theta1) - s2 * exp(-(pi + theta1 - theta2) / wrc);
end

function [t, vc, idc] = simulateIdeal(P, cap)
    n = round(P.tEnd / P.dt);
    t = (1:n)' * P.dt;
    vc = zeros(n, 1);
    idc = zeros(n, 1);
    v = 0;
    rc = P.R * cap;
    for k = 1:n
        tNext = k * P.dt;
        vOff = v * exp(-P.dt / rc);
        s = sin(P.omega * tNext);
        c = cos(P.omega * tNext);
        vr = abs(P.Vm * s);
        if vr >= vOff
            if s >= 0
                dabs = P.Vm * P.omega * c;
            else
                dabs = -P.Vm * P.omega * c;
            end
            iTry = cap * dabs + vr / P.R;
            if iTry >= 0
                v = vr;
                i = iTry;
            else
                v = vOff;
                i = 0;
            end
        else
            v = vOff;
            i = 0;
        end
        vc(k) = v;
        idc(k) = i;
    end
end

function i = diodeCurrent(P, t, vc, rs, phi)
    vr = abs(P.Vm * sin(P.omega * t + phi));
    if vr > vc
        i = (vr - vc) / rs;
    else
        i = 0;
    end
end

function dvc = derivRs(P, t, vc, rs, phi, cap)
    i = diodeCurrent(P, t, vc, rs, phi);
    dvc = (i - vc / P.R) / cap;
end

function [t, vc, idc] = simulateRs(P, rs, phi, cap)
    n = round(P.tEnd / P.dt);
    t = (0:n)' * P.dt;
    vc = zeros(n + 1, 1);
    idc = zeros(n + 1, 1);
    v = 0;
    idc(1) = diodeCurrent(P, 0, v, rs, phi);
    for k = 1:n
        tt = (k - 1) * P.dt;
        k1 = derivRs(P, tt, v, rs, phi, cap);
        k2 = derivRs(P, tt + 0.5 * P.dt, v + 0.5 * P.dt * k1, rs, phi, cap);
        k3 = derivRs(P, tt + 0.5 * P.dt, v + 0.5 * P.dt * k2, rs, phi, cap);
        k4 = derivRs(P, tt + P.dt, v + P.dt * k3, rs, phi, cap);
        v = v + (P.dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
        tt = k * P.dt;
        vc(k + 1) = v;
        idc(k + 1) = diodeCurrent(P, tt, v, rs, phi);
    end
end

function mask = steadyMask(P, t)
    t0 = P.tEnd - P.nSteady / P.f;
    mask = (t > t0 + 0.25 * P.dt) & (t <= P.tEnd + 0.25 * P.dt);
end

function s = steadyStats(P, t, vc, idc)
    m = steadyMask(P, t);
    vs = vc(m);
    is = idc(m);
    s.Vavg = mean(vs);
    s.Vmax = max(vs);
    s.Vmin = min(vs);
    s.dV = s.Vmax - s.Vmin;
    s.iPeak = max(is);
    s.cond = mean(is > 0);
end

function err = expError(t, vc, idc, rc)
    off = idc <= 0;
    best = 0;
    i0 = 1;
    i1 = 1;
    i = 1;
    n = numel(off);
    while i <= n
        if ~off(i)
            i = i + 1;
            continue
        end
        j = i;
        while j <= n && off(j)
            j = j + 1;
        end
        if (j - i) > best
            best = j - i;
            i0 = i;
            i1 = j - 1;
        end
        i = j;
    end
    pred = vc(i0) * exp(-(t(i1) - t(i0)) / rc);
    err = abs(pred - vc(i1));
end

% rlc_series_step.m  串联 RLC 直流阶跃：欠阻尼 / 临界 / 过阻尼
% （教学演示，仅用 MATLAB 基础函数）
%
% 零状态：vC(0) = 0，i(0) = 0。t >= 0 接通直流 Vs。
%   L di/dt + R i + vC = Vs
%   i = C dvC/dt
%   omega0 = 1/sqrt(L*C)
%   zeta   = (R/2)*sqrt(C/L)
%
% 默认参数与案例卡一致：
%   Vs = 10 V，L = 0.1 H，C = 1e-5 F，omega0 = 1000 rad/s
%   R = 100 / 200 / 400 ohm 对应 zeta = 0.5 / 1 / 2
% 稳态都是 vC -> Vs，i -> 0。只改 R，不改 L、C。
%
% 运行：在 MATLAB 中打开本文件并按 Run，或
%   run('cases/rlc-series-step/matlab/rlc_series_step.m')

clear; clc;

Vs = 10;          % V
L  = 0.1;         % H
C  = 1e-5;        % F（10 uF）
w0 = 1 / sqrt(L * C);
tEnd = 0.025;     % s
t = linspace(0, tEnd, 2001).';

Runder = 100;
Rcrit  = 200;
Rover  = 400;
names = {'under', 'critical', 'over'};
resistors = [Runder, Rcrit, Rover];
expectZeta = [0.5, 1, 2];

for k = 1:3
    zeta = 0.5 * resistors(k) * sqrt(C / L);
    if abs(zeta - expectZeta(k)) > 1e-9
        error('rlc_series_step:Zeta', ...
            '%s 的 zeta = %.6f，与案例卡的 %.1f 不一致。', ...
            names{k}, zeta, expectZeta(k));
    end
end

[vcU, iU, dU] = rlcAnalytic(t, Runder, Vs, L, C, w0);
[vcC, iC, dC] = rlcAnalytic(t, Rcrit, Vs, L, C, w0);
[vcO, iO, dO] = rlcAnalytic(t, Rover, Vs, L, C, w0);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
errV = zeros(1, 3);
errI = zeros(1, 3);
odeVc = cell(1, 3);
odeI = cell(1, 3);
odeT = cell(1, 3);
refs = {vcU, vcC, vcO};
refI = {iU, iC, iO};
for k = 1:3
    R = resistors(k);
    odefun = @(~, y) [y(2) / C; (Vs - R * y(2) - y(1)) / L];
    [tOde, yOde] = ode45(odefun, [0, tEnd], [0; 0], opts);
    vcOn = interp1(tOde, yOde(:, 1), t, 'pchip');
    iOn = interp1(tOde, yOde(:, 2), t, 'pchip');
    errV(k) = max(abs(vcOn - refs{k}));
    errI(k) = max(abs(iOn - refI{k}));
    odeT{k} = tOde;
    odeVc{k} = yOde(:, 1);
    odeI{k} = yOde(:, 2);
end

kvlU = max(abs(Vs - Runder * iU - L * dU - vcU));
kvlC = max(abs(Vs - Rcrit * iC - L * dC - vcC));
kvlO = max(abs(Vs - Rover * iO - L * dO - vcO));
errVmax = max(errV);
errImax = max(errI);
kvlMax = max([kvlU, kvlC, kvlO]);

if abs(dU(1) - Vs / L) > 1e-8 || abs(dC(1) - Vs / L) > 1e-8 || abs(dO(1) - Vs / L) > 1e-8
    error('rlc_series_step:InitialSlope', 't = 0 的 di/dt 应为 Vs/L。');
end
if max(vcU) <= Vs
    error('rlc_series_step:Under', '欠阻尼电容电压应越过电源电压。');
end
if max(vcC) > Vs + 1e-6 || max(vcO) > Vs + 1e-6
    error('rlc_series_step:NoOvershoot', '临界和过阻尼不应过冲。');
end

zetaU = 0.5;
alphaU = zetaU * w0;
wd = w0 * sqrt(1 - zetaU^2);
beta = sqrt(1 - zetaU^2);
tp = pi / wd;
tValley = 2 * pi / wd;
osFrac = exp(-zetaU * pi / beta);
vPeak = Vs * (1 + osFrac);
vValley = Vs * (1 - exp(-alphaU * tValley));
tEnv = -log(0.02 * beta) / alphaU;

% 谷值之后从下向上穿过 0.98*Vs。80 次二分，与 Python 脚本相同。
lo = tValley;
hi = 3 * pi / wd;
target = 0.98 * Vs;
for n = 1:80
    mid = 0.5 * (lo + hi);
    vMid = rlcVoltageAt(mid, Runder, Vs, L, C, w0);
    if vMid < target
        lo = mid;
    else
        hi = mid;
    end
end
t2pct = 0.5 * (lo + hi);

tiU = atan(wd / alphaU) / wd;
tiNeg = (pi + atan(wd / alphaU)) / wd;
iNeg = rlcCurrentAt(tiNeg, Runder, Vs, L, C, w0);
tiC = 1 / w0;
zetaO = 2;
alphaO = zetaO * w0;
hyp = w0 * sqrt(zetaO^2 - 1);
s1 = -alphaO + hyp;
s2 = -alphaO - hyp;
tiO = log(s2 / s1) / (s1 - s2);

iPeakU = rlcCurrentAt(tiU, Runder, Vs, L, C, w0);
iPeakC = rlcCurrentAt(tiC, Rcrit, Vs, L, C, w0);
iPeakO = rlcCurrentAt(tiO, Rover, Vs, L, C, w0);

vcU15 = rlcVoltageAt([0.001; 0.005], Runder, Vs, L, C, w0);
vcC1510 = rlcVoltageAt([0.001; 0.005; 0.010], Rcrit, Vs, L, C, w0);
vcOEnd = rlcVoltageAt([0.001; 0.005; 0.010; tEnd], Rover, Vs, L, C, w0);

keys = { ...
    'omega0_rad_s', w0; ...
    'di_dt0_A_per_s', Vs / L; ...
    'curvature0_V_per_s2', Vs / (L * C); ...
    'under_R_ohm', Runder; ...
    'under_zeta', 0.5; ...
    'under_wd_rad_s', wd; ...
    'under_Td_ms', (2 * pi / wd) * 1e3; ...
    'under_tp_ms', tp * 1e3; ...
    'under_v_peak_V', vPeak; ...
    'under_os_frac', osFrac; ...
    'under_v_valley_V', vValley; ...
    'under_ti_ms', tiU * 1e3; ...
    'under_i_peak_A', iPeakU; ...
    'under_ti_neg_ms', tiNeg * 1e3; ...
    'under_i_neg_A', iNeg; ...
    'under_t_2pct_ms', t2pct * 1e3; ...
    'under_t_env2pct_ms', tEnv * 1e3; ...
    'under_vc_1ms_V', vcU15(1); ...
    'under_vc_5ms_V', vcU15(2); ...
    'crit_R_ohm', Rcrit; ...
    'crit_zeta', 1; ...
    'crit_ti_ms', tiC * 1e3; ...
    'crit_i_peak_A', iPeakC; ...
    'crit_vc_1ms_V', vcC1510(1); ...
    'crit_vc_5ms_V', vcC1510(2); ...
    'crit_vc_10ms_V', vcC1510(3); ...
    'over_R_ohm', Rover; ...
    'over_zeta', 2; ...
    'over_s1_rad_s', s1; ...
    'over_s2_rad_s', s2; ...
    'over_ti_ms', tiO * 1e3; ...
    'over_i_peak_A', iPeakO; ...
    'over_vc_1ms_V', vcOEnd(1); ...
    'over_vc_5ms_V', vcOEnd(2); ...
    'over_vc_10ms_V', vcOEnd(3); ...
    'over_vc_25ms_V', vcOEnd(4) ...
    };

fprintf('RLC series step response\n');
for k = 1:size(keys, 1)
    fprintf('  %-22s = %.6f\n', keys{k, 1}, keys{k, 2});
end
fprintf('  %-22s = %.3e\n', 'max_abs_err_vc_V', errVmax);
fprintf('  %-22s = %.3e\n', 'max_abs_err_i_A', errImax);
fprintf('  %-22s = %.3e\n', 'max_abs_kvl_V', kvlMax);

if errVmax > 1e-4 || errImax > 1e-5 || kvlMax > 1e-8
    error('rlc_series_step:CheckFailed', ...
        'ode45 或 KVL 未通过（电压 %.3e V，电流 %.3e A，KVL %.3e V）。', ...
        errVmax, errImax, kvlMax);
end

colors = [0.12 0.31 0.47; 0.18 0.49 0.29; 0.77 0.36 0.15];
labels = {'欠阻尼  \zeta=0.5', '临界  \zeta=1', '过阻尼  \zeta=2'};
vcPlot = {vcU, vcC, vcO};
iPlot = {iU, iC, iO};
tPeakMs = [tiU, tiC, tiO] * 1e3;
iPeak = [iPeakU, iPeakC, iPeakO];

figure('Name', 'RLC series step', 'Color', 'w');
subplot(2, 1, 1);
hold on;
for k = 1:3
    plot(t * 1e3, vcPlot{k}, 'Color', colors(k, :), 'LineWidth', 1.6);
    plot(odeT{k} * 1e3, odeVc{k}, '--', 'Color', colors(k, :), 'LineWidth', 1.05, ...
        'HandleVisibility', 'off');
end
plot([0, tEnd * 1e3], [Vs, Vs], ':', 'Color', [0.65 0.65 0.65]);
plot(tp * 1e3, vPeak, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', colors(1, :), 'MarkerFaceColor', colors(1, :));
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.6, 13.2]);
ylabel('v_C / V');
title(sprintf('串联 RLC 阶跃    \\omega_0 = %.0f rad/s    只改 R', w0));
legend([labels, {'稳态 10 V', '欠阻尼峰值'}], 'Location', 'northeast');

subplot(2, 1, 2);
hold on;
for k = 1:3
    plot(t * 1e3, iPlot{k} * 1e3, 'Color', colors(k, :), 'LineWidth', 1.6);
    plot(odeT{k} * 1e3, odeI{k} * 1e3, '--', 'Color', colors(k, :), 'LineWidth', 1.05, ...
        'HandleVisibility', 'off');
    plot(tPeakMs(k), iPeak(k) * 1e3, 'o', 'MarkerSize', 5, ...
        'MarkerEdgeColor', colors(k, :), 'MarkerFaceColor', colors(k, :), ...
        'HandleVisibility', 'off');
end
grid on;
xlim([0, tEnd * 1e3]); ylim([-15, 70]);
xlabel('t / ms'); ylabel('i / mA');
legend(labels, 'Location', 'northeast');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outFile = fullfile(outDir, 'rlc_series_step_curves_matlab.png');
% print 属于基础绘图，避免依赖较新的 exportgraphics。
print(gcf, outFile, '-dpng', '-r140');
fprintf('  %-22s = %s\n', 'figure', outFile);

% --- 局部函数（R2016b 及以后的基础 MATLAB，不是工具箱） ---

function [vc, current, didt] = rlcAnalytic(t, R, Vs, L, C, w0)
    zeta = 0.5 * R * sqrt(C / L);
    alpha = zeta * w0;
    if zeta < 1 - 1e-9
        wd = w0 * sqrt(1 - zeta^2);
        beta = sqrt(1 - zeta^2);
        phi = acos(zeta);
        decay = exp(-alpha * t);
        vc = Vs * (1 - decay ./ beta .* sin(wd * t + phi));
        amp = Vs / (wd * L);
        current = amp * decay .* sin(wd * t);
        didt = amp * decay .* (-alpha * sin(wd * t) + wd * cos(wd * t));
    elseif zeta <= 1 + 1e-9
        decay = exp(-w0 * t);
        vc = Vs * (1 - decay .* (1 + w0 * t));
        current = (Vs / L) * t .* decay;
        didt = (Vs / L) * decay .* (1 - w0 * t);
    else
        hyp = w0 * sqrt(zeta^2 - 1);
        s1 = -alpha + hyp;
        s2 = -alpha - hyp;
        e1 = exp(s1 * t);
        e2 = exp(s2 * t);
        vc = Vs * (1 + (s2 * e1 - s1 * e2) / (s1 - s2));
        kAmp = (Vs / L) / (s1 - s2);
        current = kAmp * (e1 - e2);
        didt = kAmp * (s1 * e1 - s2 * e2);
    end
end

function vc = rlcVoltageAt(t, R, Vs, L, C, w0)
    [vc, ~, ~] = rlcAnalytic(t, R, Vs, L, C, w0);
end

function current = rlcCurrentAt(t, R, Vs, L, C, w0)
    [~, current, ~] = rlcAnalytic(t, R, Vs, L, C, w0);
end

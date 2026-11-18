%% 直流电机 PI 转速环（单环，教学演示）· pi-speed-loop
% 电机沿用 dc-motor-open-loop 卡（Ke = Kt = K）：
%   La dia/dt = Va - Ra*ia - K*w
%   J  dw/dt  = K*ia - B*w - TL
% 转速 PI 直接给出电枢电压，限幅 0..220 V；抗积分饱和用钳位：
%   e = wref - w,  Va = sat(Kp*e + z),  dz/dt = Ki*e（顶到限幅且误差同向时停积分）
%
% 工况 1：给定 0 -> 1500 r/min 斜坡 1 s，t = 1.5 s 突加 20 N·m，算到 3 s，
%         对比 P (4, 0)、PI-A (4, 45)、PI-B (4, 200)。
% 工况 2：给定直接阶跃，PI-A，有 / 无抗饱和。
%
% 只用基础 MATLAB（ode45、eig、plot），不需要工具箱；脚本末尾有局部函数，
% 需要 R2016b 以上。数值解与 Python（RK45）同一组方程、同样分段，
% 打印数字应与 Python 一致到小数点后 2~3 位（不同积分器的末位可能差 1）。
% 用法：run('cases/pi-speed-loop/matlab/pi_speed_loop.m')

clear; clc; close all;

P.Ra = 0.5; P.La = 0.005; P.K = 1.0; P.J = 0.2; P.B = 0.005;
P.Vmax = 220; P.Vmin = 0;
P.nRef = 1500; P.rpm = 60 / (2 * pi); P.wRef = P.nRef / P.rpm;
P.tRamp = 1.0; P.TL = 20; P.tLoad = 1.5; P.tEnd = 3.0;
In = 20;                         % A，示例额定电流
names = {'P', 'PI-A', 'PI-B'};
gains = [4, 0; 4, 45; 4, 200];   % [Kp (V·s/rad), Ki (V/rad)]

fprintf('DC motor PI speed loop (single loop, Va limited to 0..220 V)\n');
fprintf('  %-26s= %.3f\n', 'n_ref_rpm', P.nRef);
fprintf('  %-26s= %.6f\n', 'w_ref_rad_s', P.wRef);
fprintf('  %-26s= %.3f\n', 'open_loop_drop_rpm', P.Ra * P.TL / (P.K^2 + P.Ra * P.B) * P.rpm);

figure('Position', [80, 80, 1200, 760]);
cols = {[0.5 0.5 0.5], [0.12 0.47 0.71], [0.84 0.15 0.16]};
for g = 1:3
    kp = gains(g, 1); ki = gains(g, 2);
    [t, x, va] = piSim(P, kp, ki, true, true);
    n = x(:, 2) * P.rpm;
    A = [-P.Ra / P.La, -(P.K + kp) / P.La, 1 / P.La; P.K / P.J, -P.B / P.J, 0; 0, -ki, 0];
    p = eig(A);
    pc = p(abs(imag(p)) > 1e-9);
    if isempty(pc), zeta = 1; else, zeta = -real(pc(1)) / abs(pc(1)); end
    if ki > 0
        wss = P.wRef;
    else
        wss = (kp * P.wRef - P.Ra * P.TL / P.K) / (kp + P.K + P.Ra * P.B / P.K);
    end
    iss = (P.TL + P.B * wss) / P.K;
    k0 = find(t < P.tLoad, 1, 'last');
    after = t >= P.tLoad;
    fprintf('[%s] Kp = %g V·s/rad, Ki = %g V/rad\n', names{g}, kp, ki);
    fprintf('  %-26s= %.3f\n', 'zeta', zeta);
    fprintf('  %-26s= %.3f\n', 'n_before_load_rpm', n(k0));
    fprintf('  %-26s= %.3f\n', 'n_ss_loaded_rpm', wss * P.rpm);
    fprintf('  %-26s= %.3f\n', 'n_end_rpm', n(end));
    fprintf('  %-26s= %.3f\n', 'dip_rpm', n(k0) - min(n(after)));
    fprintf('  %-26s= %.3f\n', 'hold_max_minus_ref_rpm', max(n(t >= P.tRamp & t < P.tLoad)) - P.nRef);
    fprintf('  %-26s= %.3f\n', 'ia_max_A', max(x(:, 1)));
    fprintf('  %-26s= %.3f\n', 'ia_ss_A', iss);
    fprintf('  %-26s= %.3f\n', 'va_ss_V', P.Ra * iss + P.K * wss);
    fprintf('  %-26s= %.3f\n', 'va_max_V', max(va));
    if abs(x(end, 2) - wss) > 0.01
        error('%s：末值与稳态解析值偏差超限', names{g});
    end
    subplot(2, 2, 1); hold on; plot(t, n, 'Color', cols{g}, 'LineWidth', 1.1);
    subplot(2, 2, 3); hold on; plot(t, x(:, 1), 'Color', cols{g}, 'LineWidth', 1.1);
end
subplot(2, 2, 1);
tr = linspace(0, P.tEnd, 601);
plot(tr, min(tr / P.tRamp, 1) * P.nRef, 'k--');
grid on; ylim([0, 1700]); ylabel('n / (r/min)');
title('工况 1：斜坡给定 + 1.5 s 突加 20 N·m');
legend('P (4, 0)', 'PI-A (4, 45)', 'PI-B (4, 200)', '给定', 'Location', 'southeast');
subplot(2, 2, 3); grid on; xlabel('t / s'); ylabel('i_a / A');

aw = [true, false]; lab = {'step+AW', 'step-noAW'};
for k = 1:2
    [t, x, va] = piSim(P, 4, 45, false, aw(k));
    n = x(:, 2) * P.rpm;
    sat = t(va >= P.Vmax - 1e-9);
    if isempty(sat), tsat = 0; else, tsat = max(sat) * 1e3; end
    os = max(n(t < P.tLoad)) - P.nRef;
    if abs(os) < 5e-4, os = 0; end
    fprintf('[%s] PI-A, 给定阶跃到 %g r/min\n', lab{k}, P.nRef);
    fprintf('  %-26s= %.3f\n', 'ia_max_A', max(x(:, 1)));
    fprintf('  %-26s= %.3f\n', 'ia_max_over_In', max(x(:, 1)) / In);
    fprintf('  %-26s= %.3f\n', 't_sat_end_ms', tsat);
    fprintf('  %-26s= %.3f\n', 'n_peak_rpm', max(n(t < P.tLoad)));
    fprintf('  %-26s= %.3f\n', 'overshoot_rpm', os);
    fprintf('  %-26s= %.3f\n', 'n_end_rpm', n(end));
    subplot(2, 2, 2); hold on; plot(t, n, 'LineWidth', 1.1);
    subplot(2, 2, 4); hold on; plot(t, x(:, 1), 'LineWidth', 1.1);
end
subplot(2, 2, 2); grid on; ylabel('n / (r/min)');
title('工况 2：给定阶跃（PI-A），0–220 V 限幅');
legend('有抗饱和（钳位）', '无抗饱和', 'Location', 'southeast');
subplot(2, 2, 4); grid on; xlabel('t / s'); ylabel('i_a / A');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'pi_speed_loop_curves_matlab.png');
print(gcf, outFile, '-dpng', '-r140');
fprintf('  %-26s= %s\n', 'figure', outFile);

% ===================== 局部函数 =====================
function [t, x, va] = piSim(P, kp, ki, ramp, antiWindup)
% 在斜坡结束、加载时刻分段积分，避免步长跨过不连续点
if ramp
    edges = [0, P.tRamp, P.tLoad, P.tEnd];
else
    edges = [0, P.tLoad, P.tEnd];
end
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'MaxStep', 1e-4);
t = []; x = []; x0 = [0; 0; 0];
for k = 1:numel(edges) - 1
    a = edges(k); b = edges(k + 1);
    tl = 0;
    if a >= P.tLoad, tl = P.TL; end
    tSpan = linspace(a, b, round((b - a) / 1e-4) + 1);
    [tk, xk] = ode45(@(tt, xx) piRhs(tt, xx, P, kp, ki, ramp, antiWindup, tl), tSpan, x0, opts);
    x0 = xk(end, :).';
    if k > 1, tk = tk(2:end); xk = xk(2:end, :); end
    t = [t; tk]; x = [x; xk]; %#ok<AGROW>
end
va = zeros(size(t));
for i = 1:numel(t)
    va(i) = piCtrl(t(i), x(i, :).', P, kp, ramp);
end
end

function [va, u, e] = piCtrl(tt, xx, P, kp, ramp)
if ramp
    wr = P.wRef * min(max(tt / P.tRamp, 0), 1);
else
    wr = P.wRef;
end
e = wr - xx(2);
u = kp * e + xx(3);
va = min(max(u, P.Vmin), P.Vmax);
end

function dx = piRhs(tt, xx, P, kp, ki, ramp, antiWindup, tl)
[va, u, e] = piCtrl(tt, xx, P, kp, ramp);
dz = ki * e;
if antiWindup && ((u > P.Vmax && e > 0) || (u < P.Vmin && e < 0))
    dz = 0;
end
dx = [(va - P.Ra * xx(1) - P.K * xx(2)) / P.La;
      (P.K * xx(1) - P.B * xx(2) - tl) / P.J;
      dz];
end

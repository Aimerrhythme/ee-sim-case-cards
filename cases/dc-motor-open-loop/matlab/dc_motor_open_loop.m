% dc_motor_open_loop.m  他励（励磁恒定）/ 永磁直流电机开环直接启动（教学演示）
% 只用 MATLAB 基础函数（expm、eig、ode45、plot），不需要工具箱。
%
% 励磁不变，Ke = Kt = K（SI 单位）:
%   La dia/dt = Va - Ra*ia - K*w
%   J  dw/dt  = K*ia - B*w - TL
% 状态 x = [ia; w]，dx/dt = A x + bv*Va + bl*TL
%   A = [-Ra/La, -K/La;  K/J, -B/J]
%
% 主工况：t = 0 加 Va = 220 V，空载；t = 0.8 s 突加 TL = 20 N*m；算到 1.5 s。
% 对照工况：电枢串启动电阻 Rst = 5 ohm，空载。
% 解析解用矩阵指数，峰值电流用特征根闭式；ode45 只用来核对。
% 打印的数字与 python/dc_motor_open_loop.py 同一套公式，应逐行相同。
%
% 运行: 在 MATLAB 中打开本文件并按 Run，或
%   run('cases/dc-motor-open-loop/matlab/dc_motor_open_loop.m')

clear; clc;

Va  = 220;     % V
Ra  = 0.5;     % ohm
La  = 0.005;   % H（5 mH）
K   = 1.0;     % V*s/rad = N*m/A
J   = 0.2;     % kg*m^2
B   = 0.005;   % N*m*s/rad
TL  = 20;      % N*m
In  = 20;      % A，示例额定电流
Rst = 5;       % ohm，启动电阻
tLoad = 0.8;   % s
tEnd  = 1.5;   % s
rpm = 60 / (2 * pi);

t = linspace(0, tEnd, 3001).';

% ---- 解析解 ----
mainRef = dcAnalytic(t, Ra, true,  Va, La, K, J, B, TL, tLoad);
rstRef  = dcAnalytic(t, Ra + Rst, false, Va, La, K, J, B, TL, tLoad);

% ---- 数值核对 ----
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'MaxStep', 2e-4);
[~, xMain] = ode45(@(tt, x) dcRhs(tt, x, Ra, true,  Va, La, K, J, B, TL, tLoad), t, [0; 0], opts);
[~, xRst]  = ode45(@(tt, x) dcRhs(tt, x, Ra + Rst, false, Va, La, K, J, B, TL, tLoad), t, [0; 0], opts);
errI = max([max(abs(xMain(:, 1) - mainRef(:, 1))), max(abs(xRst(:, 1) - rstRef(:, 1)))]);
errW = max([max(abs(xMain(:, 2) - mainRef(:, 2))), max(abs(xRst(:, 2) - rstRef(:, 2)))]);

% ---- 关键数字 ----
A  = dcA(Ra, La, K, J, B);
p  = sort(real(eig(A)));
pS = sort(real(eig(dcA(Ra + Rst, La, K, J, B))));
[ia0, w0] = dcSteady(Ra, 0,  Va, K, B);
[ia1, w1] = dcSteady(Ra, TL, Va, K, B);
[tPk, iPk]     = dcPeak(Ra, Va, La, K, J, B);
[tPkS, iPkS]   = dcPeak(Ra + Rst, Va, La, K, J, B);
xAtLoad = dcAnalytic(tLoad - 1e-12, Ra, false, Va, La, K, J, B, TL, tLoad);
xEndS   = dcAnalytic(tEnd, Ra + Rst, false, Va, La, K, J, B, TL, tLoad);

names = {'tau_e_s', 'tau_m_s', 'tau_fast_s', 'tau_slow_s', 'i_stall_A', ...
    'i_stall_over_In', 't_peak_ms', 'i_peak_A', 'i_peak_over_In', 'w0_rad_s', ...
    'n0_rpm', 'ia0_A', 'n_at_load_rpm', 'w1_rad_s', 'n1_rpm', 'ia1_A', ...
    'dn_load_rpm', 'e1_V', 'p_shaft_W', 'i_stall_rst_A', 'i_peak_rst_A', ...
    't_peak_rst_ms', 'tau_slow_rst_s', 'n_end_rst_rpm'};
vals = [La / Ra, J * Ra / K^2, -1 / p(1), -1 / p(2), Va / Ra, ...
    Va / Ra / In, tPk * 1e3, iPk, iPk / In, w0, ...
    w0 * rpm, ia0, xAtLoad(2) * rpm, w1, w1 * rpm, ia1, ...
    (w0 - w1) * rpm, K * w1, TL * w1, Va / (Ra + Rst), iPkS, ...
    tPkS * 1e3, -1 / pS(2), xEndS(2) * rpm];

fprintf('DC motor open-loop start (separately excited / PM)\n');
for k = 1:numel(names)
    fprintf('  %-18s = %.6f\n', names{k}, vals(k));
end
fprintf('  %-18s = %.3e\n', 'max_abs_err_i_A', errI);
fprintf('  %-18s = %.3e\n', 'max_abs_err_w', errW);

if errI > 0.05 || errW > 0.01
    error('dc_motor_open_loop:CheckFailed', ...
        'ode45 与矩阵指数解偏差过大：ia %.3e A，w %.3e rad/s。', errI, errW);
end

% ---- 作图 ----
figure('Name', 'DC motor open-loop start', 'Color', 'w');
subplot(3, 1, 1);
plot(t * 1e3, mainRef(:, 1), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(t * 1e3, rstRef(:, 1), 'Color', [0.88 0.48 0.24], 'LineWidth', 1.4);
plot(tPk * 1e3, iPk, 'o', 'MarkerFaceColor', [0.12 0.31 0.47], 'MarkerEdgeColor', [0.12 0.31 0.47]);
plot([0, tEnd * 1e3], [Va / Ra, Va / Ra], ':', 'Color', [0.6 0.6 0.6]);
grid on; xlim([0, tEnd * 1e3]); ylim([-10, 470]);
ylabel('i_a / A');
title(sprintf('直流电机开环启动  峰值 %.1f A @ %.2f ms', iPk, tPk * 1e3));
legend('直接启动', '串 R_{st} = 5 \Omega', '峰值', 'V_a/R_a = 440 A', 'Location', 'east');

subplot(3, 1, 2);
plot(t * 1e3, mainRef(:, 1), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6);
grid on; xlim([0, 100]); ylim([-10, 470]);
ylabel('i_a / A（前 100 ms）');

subplot(3, 1, 3);
plot(t * 1e3, mainRef(:, 2) * rpm, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(t * 1e3, rstRef(:, 2) * rpm, 'Color', [0.88 0.48 0.24], 'LineWidth', 1.4);
grid on; xlim([0, tEnd * 1e3]); ylim([0, 2300]);
xlabel('t / ms'); ylabel('n / (r/min)');
legend('直接启动', '串 R_{st} = 5 \Omega', 'Location', 'southeast');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outFile = fullfile(outDir, 'dc_motor_open_loop_curves_matlab.png');
% print 属于基础绘图，避免依赖较新的 exportgraphics。
print(gcf, outFile, '-dpng', '-r140');
fprintf('  %-18s = %s\n', 'figure', outFile);

% ===================== 局部函数 =====================
function A = dcA(R, La, K, J, B)
A = [-R / La, -K / La; K / J, -B / J];
end

function [ia, w] = dcSteady(R, tl, Va, K, B)
% Va = R*ia + K*w，K*ia = tl + B*w
w  = (K * Va - R * tl) / (K^2 + R * B);
ia = (tl + B * w) / K;
end

function dx = dcRhs(tt, x, R, withLoad, Va, La, K, J, B, TL, tLoad)
tl = 0;
if withLoad && tt >= tLoad
    tl = TL;
end
dx = dcA(R, La, K, J, B) * x + [Va / La; -tl / J];
end

function X = dcAnalytic(t, R, withLoad, Va, La, K, J, B, TL, tLoad)
% 返回 N×2：第 1 列 ia，第 2 列 w
A = dcA(R, La, K, J, B);
[i0, w0] = dcSteady(R, 0, Va, K, B);
xs0 = [i0; w0];
X = zeros(numel(t), 2);
for k = 1:numel(t)
    if ~withLoad || t(k) < tLoad
        X(k, :) = (xs0 + expm(A * t(k)) * (-xs0)).';
    else
        xSw = xs0 + expm(A * tLoad) * (-xs0);
        [i1, w1] = dcSteady(R, TL, Va, K, B);
        xs1 = [i1; w1];
        X(k, :) = (xs1 + expm(A * (t(k) - tLoad)) * (xSw - xs1)).';
    end
end
end

function [tPk, iPk] = dcPeak(R, Va, La, K, J, B)
% 零初值空载阶跃，过阻尼：ia - ia_ss = c1 e^{p1 t} + c2 e^{p2 t}
p = sort(real(eig(dcA(R, La, K, J, B))));
p1 = p(1); p2 = p(2);
[iss, ~] = dcSteady(R, 0, Va, K, B);
c = [1, 1; p1, p2] \ [-iss; Va / La];
tPk = log(-c(2) * p2 / (c(1) * p1)) / (p1 - p2);
iPk = iss + c(1) * exp(p1 * tPk) + c(2) * exp(p2 * tPk);
end

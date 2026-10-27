% rl_step.m  串联 RL 直流阶跃响应（教学演示，仅用 MATLAB 基础函数）
%
% 状态方程（t >= 0，i(0) = 0）:
%   L di/dt = Vs - R i
% 解析解:
%   tau = L/R
%   i(t)   = (Vs/R) * (1 - exp(-t/tau))
%   vL(t)  = Vs * exp(-t/tau)
%   vR(t)  = R * i(t)
%
% 默认参数与案例卡一致: Vs = 10 V, R = 10 ohm, L = 0.1 H
% tau = 10 ms，稳态电流 1 A。t = 0 切线在 t = tau 处碰到稳态值。
%
% 运行: 在 MATLAB 中打开本文件并按 Run，或
%   run('cases/rl-step/matlab/rl_step.m')

clear; clc;

Vs = 10;        % V
R  = 10;        % ohm
L  = 0.1;       % H
tau = L / R;    % s
tEnd = 5 * tau; % 5*tau

t = linspace(0, tEnd, 1001).';
iAnalytic = (Vs / R) * (1 - exp(-t / tau));
vL = Vs * exp(-t / tau);
vR = R * iAnalytic;

odefun = @(~, i) (Vs - R * i) / L;
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[tOde, iOde] = ode45(odefun, [0, tEnd], 0, opts);
iOdeOnGrid = interp1(tOde, iOde, t, 'pchip');
err = max(abs(iOdeOnGrid - iAnalytic));

iTau = (Vs / R) * (1 - exp(-1));
i3 = (Vs / R) * (1 - exp(-3));
i5 = (Vs / R) * (1 - exp(-5));

fprintf('RL series step response\n');
fprintf('  tau_s            = %.6f\n', tau);
fprintf('  i_inf_A          = %.6f\n', Vs / R);
fprintf('  i_tau_A          = %.6f\n', iTau);
fprintf('  i_3tau_A         = %.6f\n', i3);
fprintf('  i_5tau_A         = %.6f\n', i5);
fprintf('  v_l_0_V          = %.6f\n', Vs);
fprintf('  v_l_tau_V        = %.6f\n', Vs * exp(-1));
fprintf('  slope0_A_per_s   = %.6f\n', Vs / L);
fprintf('  max_abs_err_i_A  = %.3e\n', err);

if err > 1e-4
    error('rl_step:CheckFailed', ...
        'ode45 与解析解偏差 %.3e A，超过 1e-4 A。', err);
end

figure('Name', 'RL step response', 'Color', 'w');
subplot(2, 1, 1);
plot(t * 1e3, iAnalytic, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(tOde * 1e3, iOde, '--', 'Color', [0.88 0.48 0.24], 'LineWidth', 1.1);
plot([0, tau * 1e3], [0, (Vs / L) * tau], ':', 'Color', [0.42 0.45 0.50]);
plot(tau * 1e3, iTau, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', [0.12 0.31 0.47], 'MarkerFaceColor', [0.12 0.31 0.47]);
plot([0, tEnd * 1e3], [Vs / R, Vs / R], ':', 'Color', [0.65 0.65 0.65], ...
    'HandleVisibility', 'off');
grid on;
xlim([0, tEnd * 1e3]); ylim([0, 1.15]);
xlabel('t / ms'); ylabel('i / A');
title(sprintf('串联 RL 阶跃    \\tau = L/R = %.0f ms', tau * 1e3));
legend('解析解', 'ode45', 't=0 切线', 't=\tau，63.2%', 'Location', 'southeast');

subplot(2, 1, 2);
plot(t * 1e3, vL, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(t * 1e3, vR, 'Color', [0.18 0.49 0.29], 'LineWidth', 1.6);
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.4, 11]);
xlabel('t / ms'); ylabel('v / V');
legend('v_L', 'v_R', 'Location', 'east');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outFile = fullfile(outDir, 'rl_step_curves_matlab.png');
% print 属于基础绘图，避免依赖较新的 exportgraphics。
print(gcf, outFile, '-dpng', '-r140');
fprintf('  figure           = %s\n', outFile);

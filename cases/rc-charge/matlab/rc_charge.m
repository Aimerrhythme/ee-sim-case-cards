% rc_charge.m  串联 RC 充电与放电（教学演示，仅用 MATLAB 基础函数）
%
% 充电（零状态，vC(0) = 0）:
%   C dvC/dt = (Vs - vC) / R
%   tau = R*C
%   vC(t) = Vs * (1 - exp(-t/tau))
%   i(t)  = (Vs/R) * exp(-t/tau)
%
% 放电（零输入，vC(0) = Vs，电源已断开）:
%   C dvC/dt = -vC / R
%   vC(t)    = Vs * exp(-t/tau)
%   i_dis(t) = (Vs/R) * exp(-t/tau)
% i_dis 从电容正极板流出、经电阻回到负极，画成正值。
%
% 默认参数与案例卡一致: Vs = 10 V, R = 10e3 ohm, C = 1e-6 F
% tau = 10 ms。充电电压在 t = tau 到稳态的 63.2%；
% 放电电压在 t = tau 剩初值的 36.8%。
%
% 运行: 在 MATLAB 中打开本文件并按 Run，或
%   run('cases/rc-charge/matlab/rc_charge.m')

clear; clc;

Vs = 10;          % V
R  = 10e3;        % ohm（10 kΩ）
C  = 1e-6;        % F（1 μF）
tau = R * C;      % s
tEnd = 5 * tau;   % 5*tau

t = linspace(0, tEnd, 1001).';

vCharge = Vs * (1 - exp(-t / tau));
iCharge = (Vs / R) * exp(-t / tau);
vDis = Vs * exp(-t / tau);
iDis = (Vs / R) * exp(-t / tau);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
odeCharge = @(~, vc) (Vs - vc) / (R * C);
odeDis = @(~, vc) -vc / (R * C);
[tOdeC, vOdeC] = ode45(odeCharge, [0, tEnd], 0, opts);
[tOdeD, vOdeD] = ode45(odeDis, [0, tEnd], Vs, opts);
vOdeCOnGrid = interp1(tOdeC, vOdeC, t, 'pchip');
vOdeDOnGrid = interp1(tOdeD, vOdeD, t, 'pchip');
errC = max(abs(vOdeCOnGrid - vCharge));
errD = max(abs(vOdeDOnGrid - vDis));

vTau = Vs * (1 - exp(-1));
v3 = Vs * (1 - exp(-3));
v5 = Vs * (1 - exp(-5));
vDisTau = Vs * exp(-1);
i0_mA = (Vs / R) * 1e3;
iTau_mA = i0_mA * exp(-1);

fprintf('RC charge and discharge\n');
fprintf('  tau_s                  = %.6f\n', tau);
fprintf('  v_inf_V                = %.6f\n', Vs);
fprintf('  v_c_tau_charge_V       = %.6f\n', vTau);
fprintf('  pct_charge_at_tau      = %.6f\n', vTau / Vs);
fprintf('  v_c_3tau_charge_V      = %.6f\n', v3);
fprintf('  v_c_5tau_charge_V      = %.6f\n', v5);
fprintf('  v_c_tau_discharge_V    = %.6f\n', vDisTau);
fprintf('  pct_discharge_at_tau   = %.6f\n', vDisTau / Vs);
fprintf('  i0_mA                  = %.6f\n', i0_mA);
fprintf('  i_tau_mA               = %.6f\n', iTau_mA);
fprintf('  slope0_V_per_s         = %.6f\n', Vs / tau);
fprintf('  max_abs_err_vc_charge  = %.3e\n', errC);
fprintf('  max_abs_err_vc_dis     = %.3e\n', errD);

if errC > 1e-4 || errD > 1e-4
    error('rc_charge:CheckFailed', ...
        'ode45 与解析解偏差超过 1e-4 V（充电 %.3e，放电 %.3e）。', errC, errD);
end

figure('Name', 'RC charge and discharge', 'Color', 'w');

subplot(2, 2, 1);
plot(t * 1e3, vCharge, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(tOdeC * 1e3, vOdeC, '--', 'Color', [0.88 0.48 0.24], 'LineWidth', 1.1);
plot([0, tau * 1e3], [0, Vs], ':', 'Color', [0.42 0.45 0.50]);
plot(tau * 1e3, vTau, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', [0.12 0.31 0.47], 'MarkerFaceColor', [0.12 0.31 0.47]);
plot([tau * 1e3, tau * 1e3], [-0.5, 11.6], ':', 'Color', [0.75 0.75 0.75], ...
    'HandleVisibility', 'off');
plot([0, tEnd * 1e3], [Vs, Vs], ':', 'Color', [0.65 0.65 0.65], ...
    'HandleVisibility', 'off');
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.5, 11.6]);
ylabel('v_C / V');
title(sprintf('充电  电容电压    \\tau = RC = %.0f ms', tau * 1e3));
legend('解析解', 'ode45', 't=0 切线', 't=\tau，63.2%', 'Location', 'southeast');

subplot(2, 2, 2);
plot(t * 1e3, vDis, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(tOdeD * 1e3, vOdeD, '--', 'Color', [0.88 0.48 0.24], 'LineWidth', 1.1);
plot([0, tau * 1e3], [Vs, 0], ':', 'Color', [0.42 0.45 0.50]);
plot(tau * 1e3, vDisTau, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', [0.12 0.31 0.47], 'MarkerFaceColor', [0.12 0.31 0.47]);
plot([tau * 1e3, tau * 1e3], [-0.5, 11.6], ':', 'Color', [0.75 0.75 0.75], ...
    'HandleVisibility', 'off');
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.5, 11.6]);
title('放电  电容电压');
legend('解析解', 'ode45', 't=0 切线', 't=\tau，36.8%', 'Location', 'northeast');

subplot(2, 2, 3);
plot(t * 1e3, iCharge * 1e3, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(tau * 1e3, iTau_mA, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', [0.12 0.31 0.47], 'MarkerFaceColor', [0.12 0.31 0.47]);
plot([tau * 1e3, tau * 1e3], [-0.05, 1.15], ':', 'Color', [0.75 0.75 0.75], ...
    'HandleVisibility', 'off');
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.05, 1.15]);
xlabel('t / ms'); ylabel('i / mA');
title('充电  电流');
legend('解析解', 't=\tau，36.8%', 'Location', 'northeast');

subplot(2, 2, 4);
plot(t * 1e3, iDis * 1e3, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.6); hold on;
plot(tau * 1e3, iTau_mA, 'o', 'MarkerSize', 6, ...
    'MarkerEdgeColor', [0.12 0.31 0.47], 'MarkerFaceColor', [0.12 0.31 0.47]);
plot([tau * 1e3, tau * 1e3], [-0.05, 1.15], ':', 'Color', [0.75 0.75 0.75], ...
    'HandleVisibility', 'off');
grid on;
xlim([0, tEnd * 1e3]); ylim([-0.05, 1.15]);
xlabel('t / ms');
title('放电  电流');
legend('解析解', 't=\tau，36.8%', 'Location', 'northeast');

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
outDir = fullfile(thisDir, '..', 'img');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
outFile = fullfile(outDir, 'rc_charge_curves_matlab.png');
% print 属于基础绘图，避免依赖较新的 exportgraphics。
print(gcf, outFile, '-dpng', '-r140');
fprintf('  figure                 = %s\n', outFile);

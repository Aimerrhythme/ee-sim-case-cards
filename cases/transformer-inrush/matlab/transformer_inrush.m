%% 单相变压器空载合闸励磁涌流（教学演示）· transformer-inrush
% 一次侧空载合闸，二次侧开路；只留 R、漏感 Ls 和非线性励磁支路（不计铁耗、磁滞）：
%   v = Vm*sin(w*t + alpha) = R*i + Ls*di/dt + dlam/dt,   i = f(lam)
% f 为两段线性 λ-i 曲线（拐点 1.2 pu，饱和后只剩空心电感 Lair）。
% 令 psi = lam + Ls*i（仍是两段线性），单状态：dpsi/dt = v - R*i(psi)。
% 剩磁：t = 0 时铁心磁链设为 lam_r（不计磁滞，i(0+) = lam_r/Lm）。
%
% 工况 A~F 与 Python 脚本相同：
%   A 0°/+0.8，B 0°/0，C 0°/-0.8，D 90°/0，E 90°/+0.8，F 选相 acos(-0.8)/+0.8
%
% 只用基础 MATLAB（定步长 RK4、fft、plot），不需要工具箱；末尾有局部函数，
% 需要 R2016b 以上。积分器、步长与 Python 完全相同（RK4，dt = 10 us），
% 打印数字应与 Python 一致到小数点后 3 位。约需 1~2 min（MATLAB 循环）。
% 用法：run('cases/transformer-inrush/matlab/transformer_inrush.m')

clear; clc; close all;

SN = 10e3; UN = 220; f = 50; w = 2*pi*f;
Zb = UN^2 / SN; IN = SN / UN; Vm = sqrt(2) * UN; lamN = Vm / w;
P.R = 0.01 * Zb; P.Ls = 0.02 * Zb / w; P.Lm = 50 * Zb / w; P.Lair = 0.13 * Zb / w;
P.lamk = 1.2 * lamN; P.ik = P.lamk / P.Lm; P.psik = P.lamk + P.Ls * P.ik;
P.Vm = Vm; P.w = w;
lr = 0.8;
dt = 1e-5; Tend = 2.0; T = 1 / f; npc = round(T / dt);

alphaSel = acosd(-lr);
names = {'A', 'B', 'C', 'D', 'E', 'F'};
alphas = [0, 0, 0, 90, 90, alphaSel];
lamr = [lr, 0, -lr, 0, lr, lr] * lamN;

i0pk = Vm / hypot(P.R, w * (P.Lm + P.Ls));
fprintf('Single-phase transformer no-load energization (inrush)\n');
fprintf('  %-26s= %.3f\n', 'I_N_A', IN);
fprintf('  %-26s= %.4f\n', 'L_m_H', P.Lm);
fprintf('  %-26s= %.4f\n', 'L_air_mH', P.Lair * 1e3);
fprintf('  %-26s= %.4f\n', 'I0_peak_steady_A', i0pk);
fprintf('  %-26s= %.3f\n', 'tau_sat_ms', (P.Lair + P.Ls) / P.R * 1e3);
fprintf('  %-26s= %.3f\n', 'alpha_select_deg', alphaSel);

[t, I] = rk4All(P, deg2rad(alphas), lamr, dt, Tend);

figure('Position', [80, 80, 1200, 760]);
cols = lines(6);
for j = 1:6
    i = I(j, :);
    nc = floor((numel(i) - 1) / npc);
    seg = reshape(i(1:nc * npc), npc, nc).';   % 每行一个周期
    S = fft(seg, [], 2) / npc;
    i1 = 2 * abs(S(:, 2)); i2 = 2 * abs(S(:, 3)); dc = real(S(:, 1));
    pk = max(abs(seg), [], 2);
    hr = i2 ./ max(i1, 1e-12);
    [~, k] = max(abs(i));
    fprintf('[%s] alpha = %.2f deg, lambda_r = %.1f pu\n', names{j}, alphas(j), lamr(j) / lamN);
    fprintf('  %-26s= %.3f\n', 'i_peak_A', i(k));
    fprintf('  %-26s= %.3f\n', 't_peak_ms', t(k) * 1e3);
    fprintf('  %-26s= %.3f\n', 'peak_over_IN_rms', abs(i(k)) / IN);
    fprintf('  %-26s= %.3f\n', 'peak_over_IN_peak', abs(i(k)) / (sqrt(2) * IN));
    if pk(1) > 2 * i0pk
        fprintf('  %-26s= %.3f\n', 'cyc1_I2_over_I1_pct', hr(1) * 100);
        fprintf('  %-26s= %.3f\n', 'cyc1_Idc_over_I1_pct', dc(1) / i1(1) * 100);
        fprintf('  %-26s= %.3f\n', 'cyc_peak_10', pk(10));
        fprintf('  %-26s= %.3f\n', 'cyc_peak_50', pk(50));
        idx = find(pk >= 0.3 * pk(1));
        idx = idx(idx == (1:numel(idx)).');
        c = polyfit((idx - 1) * T, log(pk(idx)), 1);
        fprintf('  %-26s= %.3f\n', 'tau_fit_ms', -1 / c(1) * 1e3);
        fprintf('  %-26s= %d\n', 'cycles_to_half_peak', find(pk < 0.5 * pk(1), 1) - 1);
        fprintf('  %-26s= %d\n', 'cycles_to_below_IN_peak', find(pk < sqrt(2) * IN, 1) - 1);
        subplot(2, 2, 3); hold on;
        plot(1:20, hr(1:20) * 100, 'o-', 'Color', cols(j, :), 'DisplayName', names{j});
    else
        fprintf('  %-26s= %.4f\n', 'cyc1_peak_A', pk(1));
    end
    subplot(2, 2, 1); hold on;
    m = t <= 0.2;
    plot(t(m) * 1e3, i(m), 'Color', cols(j, :), 'DisplayName', names{j});
    subplot(2, 2, 2);
    semilogy(1:nc, pk, 'Color', cols(j, :), 'DisplayName', names{j}); hold on;
end
subplot(2, 2, 1); grid on; xlabel('t / ms'); ylabel('i / A'); title('前 10 个周期'); legend show;
subplot(2, 2, 2); grid on; xlabel('周期序号'); ylabel('|i|max / A'); title('每周期峰值'); legend show;
subplot(2, 2, 3); grid on; yline(15, '--'); yline(20, '--');
xlabel('周期序号'); ylabel('I2/I1 / %'); title('二次谐波含量'); legend show;
subplot(2, 2, 4);
ii = linspace(-800, 800, 2001);
plot(ii, lamOfI(P, ii) / lamN, 'k'); grid on; xlabel('i / A'); ylabel('\lambda / \lambda_N');
title('\lambda-i 饱和曲线');
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
saveas(gcf, fullfile(here, '..', 'img', 'transformer_inrush_curves_matlab.png'));

%% ---- 局部函数 ----
function [t, I] = rk4All(P, a, lamr, dt, Tend)
    n = round(Tend / dt);
    t = (0:n) * dt;
    a = a(:); psi = lamr(:) + P.Ls * lamr(:) / P.Lm;
    I = zeros(numel(a), n + 1);
    I(:, 1) = iOfPsi(P, psi);
    rhs = @(tt, p) P.Vm * sin(P.w * tt + a) - P.R * iOfPsi(P, p);
    for k = 1:n
        tk = t(k);
        k1 = rhs(tk, psi);
        k2 = rhs(tk + dt / 2, psi + dt / 2 * k1);
        k3 = rhs(tk + dt / 2, psi + dt / 2 * k2);
        k4 = rhs(tk + dt, psi + dt * k3);
        psi = psi + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        I(:, k + 1) = iOfPsi(P, psi);
    end
end

function i = iOfPsi(P, psi)
    a = abs(psi);
    i = psi / (P.Lm + P.Ls);
    s = a > P.psik;
    i(s) = sign(psi(s)) .* (P.ik + (a(s) - P.psik) / (P.Lair + P.Ls));
end

function lam = lamOfI(P, i)
    a = abs(i);
    lam = i * P.Lm;
    s = a > P.ik;
    lam(s) = sign(i(s)) .* (P.lamk + (a(s) - P.ik) * P.Lair);
end

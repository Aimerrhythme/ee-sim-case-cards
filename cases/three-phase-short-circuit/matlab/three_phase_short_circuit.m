%% 无限大电源三相短路电流暂态（教学演示）· three-phase-short-circuit
% 每相：e --[R]--(L)--●--[Rl]--(Ll)-- N，t = t_f 时 ● 点三相同时接地短路（负荷被短接）。
%   短路前 (L+Ll) di/dt = e - (R+Rl) i；短路后 L di/dt = e - R i（线路电流连续）
% e_a = Em*sin(w(t-t_f) + alpha)，b、c 相依次滞后 120°。
% 解析解（短路后 t'）：
%   i_a = Ipm sin(w t' + alpha - phik) + [I0m sin(alpha - phi0) - Ipm sin(alpha - phik)] e^(-t'/Ta)
%
% 工况与 Python 脚本相同：
%   A 带负荷、a 相非周期分量最大；B 带负荷、a 相非周期分量为零；C 空载、alpha = phik + 90°
%
% 只用基础 MATLAB（定步长 RK4、polyfit、plot），不需要工具箱；末尾有局部函数，
% 需要 R2016b 以上。积分器、步长与 Python 完全相同（RK4，dt = 10 us），
% 打印数字应与 Python 一致到小数点后 3~4 位。
% 用法：run('cases/three-phase-short-circuit/matlab/three_phase_short_circuit.m')

clear; clc; close all;

UN = 10.5e3; f = 50; w = 2*pi*f;
Eph = UN / sqrt(3); P.Em = sqrt(2) * Eph; P.w = w;
X = 0.5; Ta = 0.05; P.L = X / w; P.R = P.L / Ta;
Rl = 13.6; Xl = 6.6; P.Ll = Xl / w; P.Rl = Rl;
dt = 1e-5; T = 1 / f; npc = round(T / dt);
P.tf = 2 * T; Tend = P.tf + 15 * T;

Zk = complex(P.R, X); Z0 = complex(P.R + Rl, X + Xl);
Ipm = P.Em / abs(Zk); I0m = P.Em / abs(Z0);
phik = angle(Zk); phi0 = angle(Z0);
dc0fun = @(a, i0m) i0m * sin(a - phi0) - Ipm * sin(a - phik);

fprintf('Three-phase symmetrical short circuit fed by an infinite bus\n');
fprintf('  %-28s= %.6f\n', 'R_ohm', P.R);
fprintf('  %-28s= %.4f\n', 'L_mH', P.L * 1e3);
fprintf('  %-28s= %.3f\n', 'X_over_R', X / P.R);
fprintf('  %-28s= %.3f\n', 'phi_k_deg', rad2deg(phik));
fprintf('  %-28s= %.4f\n', 'Ipm_kA', Ipm / 1e3);
fprintf('  %-28s= %.4f\n', 'Ip_rms_kA', Ipm / sqrt(2) / 1e3);
fprintf('  %-28s= %.3f\n', 'pre_fault_I_rms_A', I0m / sqrt(2));
fprintf('  %-28s= %.4f\n', 'Kim_textbook', 1 + exp(-0.01 / Ta));

% 工况角度
ag = deg2rad(0:0.01:359.99);
[~, k] = max(abs(dc0fun(ag, I0m))); alphaA = rad2deg(ag(k));
if alphaA > 180, alphaA = alphaA - 180; end
ab = 0:0.001:179.999;
[~, k] = min(abs(dc0fun(deg2rad(ab), I0m))); alphaB = ab(k);
alphaC = rad2deg(phik) + 90;
cases = {'A', alphaA, true; 'B', alphaB, true; 'C', alphaC, false};
shift = deg2rad([0; -120; 120]);

figure('Position', [80, 80, 1200, 760]);
for c = 1:3
    name = cases{c, 1}; ad = cases{c, 2}; wl = cases{c, 3};
    [t, I] = rk4Three(P, deg2rad(ad) + shift, wl, dt, Tend);
    kf = round(P.tf / dt) + 1;
    tp = t(kf:end) - P.tf; post = I(:, kf:end);
    a = deg2rad(ad) + shift;
    i0m = I0m * wl;
    dc0 = dc0fun(a, i0m);
    ref = Ipm * sin(w * tp + a - phik) + dc0 .* exp(-tp / Ta);
    err = max(abs(post(:) - ref(:))) / Ipm;
    [pkAbs, kk] = max(abs(post), [], 2);
    [~, j] = max(pkAbs);
    fprintf('[%s] alpha = %.3f deg, with_load = %d\n', name, ad, wl);
    for p = 1:3
        fprintf('  %-28s= %.4f\n', sprintf('dc0_%c_kA', 'a' + p - 1), dc0(p) / 1e3);
        fprintf('  %-28s= %.4f\n', sprintf('peak_%c_kA', 'a' + p - 1), post(p, kk(p)) / 1e3);
    end
    fprintf('  %-28s= %c\n', 'worst_phase', 'a' + j - 1);
    fprintf('  %-28s= %.4f\n', 'i_im_kA', pkAbs(j) / 1e3);
    fprintf('  %-28s= %.4f\n', 'Kim_measured', pkAbs(j) / Ipm);
    nc = floor((numel(tp) - 1) / npc);
    cm = mean(reshape(post(j, 1:nc * npc), npc, nc), 1);
    pf = polyfit(((0:9) + 0.5) * T, log(abs(cm(1:10))), 1);
    fprintf('  %-28s= %.3f\n', 'Ta_fit_ms', -1e3 / pf(1));
    fprintf('  %-28s= %.1e\n', 'rk4_vs_analytic_max_rel', err);
    if c == 1
        subplot(2, 2, 1);
        m = t <= P.tf + 8 * T;
        plot((t(m) - P.tf) * 1e3, I(:, m) / 1e3); grid on;
        xlabel('t - t_f / ms'); ylabel('i / kA'); title('工况 A 三相电流'); legend('i_a', 'i_b', 'i_c');
        subplot(2, 2, 2);
        mp = tp <= 8 * T;
        plot(tp(mp) * 1e3, post(j, mp) / 1e3, tp(mp) * 1e3, dc0(j) * exp(-tp(mp) / Ta) / 1e3, 'b'); grid on;
        xlabel('t - t_f / ms'); ylabel('i / kA'); title('最严重相：全电流与非周期分量');
    end
end

% alpha 扫描（解析解，首周期三相最大瞬时值）
tp = 0:dt:T; al = 0:0.1:180; pk = zeros(size(al));
for n = 1:numel(al)
    a = deg2rad(al(n)) + shift;
    ii = Ipm * sin(w * tp + a - phik) + dc0fun(a, I0m) .* exp(-tp / Ta);
    pk(n) = max(abs(ii(:)));
end
[mx, k1] = max(pk); [mn, k2] = min(pk);
fprintf('  %-28s= %.1f\n', 'alpha_scan_worst_deg', al(k1));
fprintf('  %-28s= %.4f\n', 'alpha_scan_max_peak_kA', mx / 1e3);
fprintf('  %-28s= %.4f\n', 'alpha_scan_min_peak_kA', mn / 1e3);
subplot(2, 2, 3); plot(al, pk / 1e3, 'k'); grid on; ylim([0, 36]);
xlabel('\alpha / °'); ylabel('|i|max / kA'); title('短路初相角扫描');

% X/R 扫描：教材近似
xr = logspace(0, log10(60), 60);
subplot(2, 2, 4); semilogx(xr, 1 + exp(-0.01 * w ./ xr), 'b'); grid on; ylim([1, 2]);
xlabel('X/R'); ylabel('K_{im}'); title('冲击系数 K_{im} = 1 + e^{-0.01/T_a}');

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
saveas(gcf, fullfile(here, '..', 'img', 'three_phase_short_circuit_curves_matlab.png'));

%% ---- 局部函数 ----
function [t, I] = rk4Three(P, a, withLoad, dt, Tend)
    n = round(Tend / dt);
    t = (0:n) * dt;
    kf = round(P.tf / dt);
    if withLoad
        z0 = complex(P.R + P.Rl, P.w * (P.L + P.Ll));
        i = P.Em / abs(z0) * sin(a - P.w * P.tf - angle(z0));
    else
        i = zeros(3, 1);
    end
    I = zeros(3, n + 1);
    I(:, 1) = i;
    for k = 1:n
        tk = t(k);
        if k <= kf
            if ~withLoad, I(:, k + 1) = 0; continue; end
            r = P.R + P.Rl; l = P.L + P.Ll;
        else
            r = P.R; l = P.L;
        end
        rhs = @(tt, x) (P.Em * sin(P.w * (tt - P.tf) + a) - r * x) / l;
        k1 = rhs(tk, i);
        k2 = rhs(tk + dt / 2, i + dt / 2 * k1);
        k3 = rhs(tk + dt / 2, i + dt / 2 * k2);
        k4 = rhs(tk + dt, i + dt * k3);
        i = i + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        I(:, k + 1) = i;
    end
end

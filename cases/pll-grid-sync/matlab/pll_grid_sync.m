%% 三相电网同步 SRF-PLL 锁相与动态跟踪（教学演示）· pll-grid-sync
% 信号链：va,vb,vc --Clarke(等幅)--> valpha,vbeta --Park(theta_hat)--> vd, vq
%   vq = -valpha*sin(th) + vbeta*cos(th) = Vm*sin(thg - th) ≈ Vm*(thg - th)
%   w_hat = w0 + Kp*vq + Ki*∫vq，th = ∫w_hat
% 小信号：wn^2 = Ki*Vm，2*zeta*wn = Kp*Vm  =>  Kp = 2*zeta*wn/Vm，Ki = wn^2/Vm
%
% 工况与 Python 脚本相同（扰动时刻 0.1 s）：
%   A 相位跳变 +30°；B 频率阶跃 50 -> 49.5 Hz；C 频率斜坡 -5 Hz/s 持续 100 ms；
%   D 电压暂降 0.5 pu + 跳变 +30°（无 / 有幅值归一化）；E 10% 负序 + 5% 5 次谐波
%
% 只用基础 MATLAB（for 循环 RK4、plot），不需要工具箱；末尾有局部函数，需要 R2016b 以上。
% 积分器、步长与 Python 相同（RK4，dt = 10 us），打印数字应与 Python 一致到小数点后 2~3 位。
% 用法：run('cases/pll-grid-sync/matlab/pll_grid_sync.m')

clear; clc; close all;

P.Vm = sqrt(2) * 380 / sqrt(3);      % 相电压峰值 310.27 V
P.f0 = 50; P.w0 = 2 * pi * P.f0;
P.tev = 0.10;                         % 扰动时刻
dt = 1e-5; Tend = 0.40;
fn = 20; zeta = 1 / sqrt(2);
[Kp, Ki, wn] = pllGains(fn, zeta, P.Vm);

fprintf('SRF-PLL grid synchronization (teaching example)\n');
fprintf('  %-30s= %.3f\n', 'Vm_phase_peak_V', P.Vm);
fprintf('  %-30s= %.3f\n', 'wn_rad_s', wn);
fprintf('  %-30s= %.5f\n', 'Kp_rad_s_per_V', Kp);
fprintf('  %-30s= %.4f\n', 'Ki_rad_s2_per_V', Ki);

base = struct('jump', 0, 'df', 0, 'ramp', 0, 'tramp', 0, 'sag', 1, 'k2', 0, 'k5', 0);

% ---------- A ----------
cA = base; cA.jump = 30;
rA = pllSim(P, cA, Kp, Ki, false, dt, Tend);
mA = phaseMetrics(rA, P.tev, 30);
fprintf('[A] 相位跳变 +30°\n');
fprintf('  %-30s= %.2f\n', 'lock_time_phase_2deg_ms', mA.ltPh * 1e3);
fprintf('  %-30s= %.2f\n', 'lock_time_freq_0.1Hz_ms', mA.ltF * 1e3);
fprintf('  %-30s= %.2f\n', 'overshoot_pct', mA.os);
fprintf('  %-30s= %.3f\n', 'peak_freq_dev_Hz', mA.fdev);

% ---------- B ----------
cB = base; cB.df = -0.5;
rB = pllSim(P, cB, Kp, Ki, false, dt, Tend);
mB = phaseMetrics(rB, P.tev, 0);
fprintf('[B] 频率阶跃 50 -> 49.5 Hz\n');
fprintf('  %-30s= %.4f\n', 'phase_err_peak_deg', min(rad2deg(rB.e(rB.t > P.tev))));
fprintf('  %-30s= %.2f\n', 'lock_time_freq_0.1Hz_ms', mB.ltF * 1e3);
fprintf('  %-30s= %.5f\n', 'f_pll_final_Hz', rB.fpll(end));

% ---------- C ----------
cC = base; cC.ramp = -5; cC.tramp = 0.1;
rC = pllSim(P, cC, Kp, Ki, false, dt, Tend);
m = rC.t > 0.17 & rC.t <= 0.20;
fprintf('[C] 频率斜坡 -5 Hz/s，持续 100 ms\n');
fprintf('  %-30s= %.4f\n', 'ramp_ss_err_sim_deg', mean(rad2deg(rC.e(m))));
fprintf('  %-30s= %.4f\n', 'ramp_ss_err_theory_deg', rad2deg(2 * pi * -5 / (Ki * P.Vm)));

% ---------- D ----------
cD = base; cD.jump = 30; cD.sag = 0.5;
rD = pllSim(P, cD, Kp, Ki, false, dt, Tend);
rDn = pllSim(P, cD, Kp, Ki, true, dt, Tend);
mD = phaseMetrics(rD, P.tev, 30); mDn = phaseMetrics(rDn, P.tev, 30);
fprintf('[D] 电压暂降 0.5 pu + 跳变 +30°\n');
fprintf('  %-30s= %.2f / %.2f\n', 'lock_phase_ms no_norm/norm', mD.ltPh * 1e3, mDn.ltPh * 1e3);
fprintf('  %-30s= %.2f / %.2f\n', 'lock_freq_ms no_norm/norm', mD.ltF * 1e3, mDn.ltF * 1e3);
fprintf('  %-30s= %.2f / %.2f\n', 'overshoot_pct no_norm/norm', mD.os, mDn.os);

% ---------- E ----------
cE = base; cE.k2 = 0.10; cE.k5 = 0.05;
rE = pllSim(P, cE, Kp, Ki, false, dt, Tend);
m = rE.t >= 0.30;
fprintf('[E] 10%% 负序 + 5%% 5 次谐波（t >= 0.3 s）\n');
fprintf('  %-30s= %.2f\n', 'vq_pp_V', max(rE.vq(m)) - min(rE.vq(m)));
fprintf('  %-30s= %.3f\n', 'phase_err_pp_deg', rad2deg(max(rE.e(m)) - min(rE.e(m))));
fprintf('  %-30s= %.3f\n', 'f_pll_pp_Hz', max(rE.fpll(m)) - min(rE.fpll(m)));

% ---------- 带宽扫描（A 的锁定时间）----------
fns = [5 10 20 40 80]; lt = zeros(size(fns));
for k = 1:numel(fns)
    [kp_, ki_] = pllGains(fns(k), zeta, P.Vm);
    te = 0.40; if fns(k) < 10, te = 0.60; end
    mm = phaseMetrics(pllSim(P, cA, kp_, ki_, false, dt, te), P.tev, 30);
    lt(k) = mm.lt;
    fprintf('  fn = %4.1f Hz  lock(±2° & ±0.1 Hz) = %7.2f ms  fdev = %7.3f Hz\n', fns(k), mm.lt * 1e3, mm.fdev);
end

% ---------- 作图 ----------
figure('Position', [60, 60, 1300, 760]);
subplot(2, 3, 1);
w = rA.t >= 0.08 & rA.t <= 0.16;
plot((rA.t(w) - P.tev) * 1e3, rA.vd(w), 'k', (rA.t(w) - P.tev) * 1e3, rA.vq(w), 'm'); grid on;
xlabel('t - t_{ev} / ms'); ylabel('V'); title('工况 A：vd、vq'); legend('v_d', 'v_q');
subplot(2, 3, 2);
w = rA.t >= 0.09 & rA.t <= 0.20;
plot((rA.t(w) - P.tev) * 1e3, rad2deg(rA.e(w)), 'k'); grid on; hold on;
plot([-10 100], [2 2], 'g:', [-10 100], [-2 -2], 'g:');
xlabel('t - t_{ev} / ms'); ylabel('\theta_g - \theta / °'); title('工况 A：相位误差');
subplot(2, 3, 3);
w = rB.t >= 0.08 & rB.t <= 0.30;
plot(rB.t(w) * 1e3, rB.fgrid(w), 'k:', rB.t(w) * 1e3, rB.fpll(w), 'b', ...
     rC.t(w) * 1e3, rC.fgrid(w), 'k--', rC.t(w) * 1e3, rC.fpll(w), 'r'); grid on; ylim([49.35 50.08]);
xlabel('t / ms'); ylabel('f / Hz'); title('B 频率阶跃 / C 频率斜坡'); legend('B f_g', 'B f_{pll}', 'C f_g', 'C f_{pll}');
subplot(2, 3, 4);
w = rD.t >= 0.09 & rD.t <= 0.25;
plot((rD.t(w) - P.tev) * 1e3, rad2deg(rD.e(w)), 'r', (rDn.t(w) - P.tev) * 1e3, rad2deg(rDn.e(w)), 'b'); grid on;
xlabel('t - t_{ev} / ms'); ylabel('\theta_g - \theta / °'); title('工况 D：暂降 + 跳变'); legend('无归一化', '幅值归一化');
subplot(2, 3, 5);
w = rE.t >= 0.30 & rE.t <= 0.34;
plot((rE.t(w) - 0.3) * 1e3, rE.vq(w), 'm'); grid on;
xlabel('t - 0.3 s / ms'); ylabel('v_q / V'); title('工况 E：vq 的 100 Hz + 300 Hz 脉动');
subplot(2, 3, 6);
loglog(fns, lt * 1e3, 'ko-'); grid on;
xlabel('f_n / Hz'); ylabel('锁定时间 / ms'); title('带宽扫描（工况 A）');

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
saveas(gcf, fullfile(here, '..', 'img', 'pll_grid_sync_curves_matlab.png'));

%% ---- 局部函数 ----
function [Kp, Ki, wn] = pllGains(fn, zeta, Vm)
    wn = 2 * pi * fn;
    Kp = 2 * zeta * wn / Vm;
    Ki = wn^2 / Vm;
end

function [th, f] = gridTheta(P, t, c)
    th = P.w0 * t; f = P.f0;
    if t >= P.tev
        tau = t - P.tev;
        th = th + deg2rad(c.jump) + 2 * pi * c.df * tau;
        f = f + c.df;
        if c.ramp ~= 0
            tt = min(tau, c.tramp);
            th = th + 2 * pi * (0.5 * c.ramp * tt^2 + c.ramp * c.tramp * (tau - tt));
            f = f + c.ramp * tt;
        end
    end
end

function [vd, vq] = parkOut(P, t, c, th, norm)
    thg = gridTheta(P, t, c);
    v = P.Vm; if t >= P.tev, v = P.Vm * c.sag; end
    s = [0, -2 * pi / 3, 2 * pi / 3];
    vabc = v * (cos(thg + s) + c.k2 * cos(thg - s) + c.k5 * cos(5 * (thg + s)));
    al = (2 / 3) * (vabc(1) - 0.5 * vabc(2) - 0.5 * vabc(3));
    be = (vabc(2) - vabc(3)) / sqrt(3);
    vd = al * cos(th) + be * sin(th);
    vq = -al * sin(th) + be * cos(th);
    if norm
        vq = vq * P.Vm / max(hypot(al, be), 1e-6);
    end
end

function r = pllSim(P, c, Kp, Ki, norm, dt, Tend)
    n = round(Tend / dt);
    t = (0:n) * dt;
    xi = 0; th = 0;                          % 从锁定状态起算
    XI = zeros(1, n + 1); TH = zeros(1, n + 1);
    for k = 1:n
        tk = t(k);
        [a1, b1] = pllRhs(P, c, Kp, Ki, norm, tk, xi, th);
        [a2, b2] = pllRhs(P, c, Kp, Ki, norm, tk + dt / 2, xi + dt / 2 * a1, th + dt / 2 * b1);
        [a3, b3] = pllRhs(P, c, Kp, Ki, norm, tk + dt / 2, xi + dt / 2 * a2, th + dt / 2 * b2);
        [a4, b4] = pllRhs(P, c, Kp, Ki, norm, tk + dt, xi + dt * a3, th + dt * b3);
        xi = xi + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
        th = th + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4);
        XI(k + 1) = xi; TH(k + 1) = th;
    end
    thg = zeros(1, n + 1); fg = thg; vd = thg; vq = thg;
    for k = 1:n + 1
        [thg(k), fg(k)] = gridTheta(P, t(k), c);
        [vd(k), vq(k)] = parkOut(P, t(k), c, TH(k), norm);
    end
    r.t = t;
    r.e = angle(exp(1i * (thg - TH)));
    r.fpll = (P.w0 + Kp * vq + Ki * XI) / (2 * pi);
    r.fgrid = fg; r.vd = vd; r.vq = vq;
end

function [dxi, dth] = pllRhs(P, c, Kp, Ki, norm, t, xi, th)
    [~, vq] = parkOut(P, t, c, th, norm);
    dxi = vq;
    dth = P.w0 + Kp * vq + Ki * xi;
end

function lt = lockTime(t, err, band, t0)
    m = t >= t0; tt = t(m); ee = abs(err(m));
    k = find(ee > band, 1, 'last');
    if isempty(k), lt = 0;
    elseif k == numel(tt), lt = inf;
    else, lt = tt(k + 1) - t0;
    end
end

function mm = phaseMetrics(r, tev, jump)
    e = rad2deg(r.e); ferr = r.fpll - r.fgrid; m = r.t > tev;
    mm.ltPh = lockTime(r.t, e, 2, tev);
    mm.ltF = lockTime(r.t, ferr, 0.1, tev);
    mm.lt = max(mm.ltPh, mm.ltF);
    if jump > 0, mm.os = max(0, -min(e(m))) / jump * 100; else, mm.os = 0; end
    mm.fdev = max(abs(ferr(m)));
end

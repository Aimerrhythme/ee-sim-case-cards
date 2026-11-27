%% 光伏 MPPT 扰动观察法（P&O）：单二极管 PV 阵列 + Boost 平均模型 + 步长对比
% 与 python/pv_mppt_po.py 同方程、同积分器（RK4，dt = 50 us）、同参数。
% 打印字段是 Python 的子集：MPP、工况 A 三种步长、工况 C 斜坡、[D] 启动卡死。
% 只用基础函数，不需要工具箱；末尾有局部函数，需要 R2016b 以上。
% 教学示例参数，不是某一型号组件的实测数据。

clear; clc;

% ---- PV 阵列（3 块组件串联，每块 60 片）----
P.kB = 1.380649e-23; P.q = 1.602176634e-19; P.T = 298.15;
P.Vt = P.kB * P.T / P.q;                 % 25.693 mV
P.Ns = 60; P.n = 1.3; P.Isc = 8.5; P.VocMod = 37.5;
P.RsMod = 0.3; P.RshMod = 250; P.Nser = 3;
P.a   = P.Nser * P.Ns * P.n * P.Vt;      % 6.0121 V
P.Rs  = P.Nser * P.RsMod;                % 0.9 Ohm
P.Rsh = P.Nser * P.RshMod;               % 750 Ohm
P.I0  = P.Isc / (exp(P.Nser * P.VocMod / P.a) - 1);

% ---- Boost 平均模型 ----
P.Vbus = 200; P.L = 3e-3; P.Cin = 47e-6; P.RL = 0.2;
P.Dmin = 0.05; P.Dmax = 0.90; P.D0 = 0.45;

% ---- MPPT / 仿真 ----
P.Tmppt = 20e-3; P.dt = 50e-6; P.Tend = 3.0; P.Tstep = 1.5;
P.G1 = 1000; P.G2 = 600;
steps = [0.002 0.005 0.02];
win1 = [1.2 1.5]; win2 = [2.7 3.0];
P.ramp = []; P.rampT = [1.5 2.5]; P.rampG = [600 1000];

fprintf('PV MPPT perturb & observe (teaching example, MATLAB)\n');
fprintf('  %-30s= %.4f\n', 'a_array_V', P.a);
fprintf('  %-30s= %.4e\n', 'I0_A', P.I0);
Gtab = linspace(500, 1000, 51); MPPtab = zeros(51, 3);
for k = 1:51, [MPPtab(k, 1), MPPtab(k, 2), MPPtab(k, 3)] = pvMpp(P, Gtab(k)); end
for g = [P.G1 P.G2]
    [vmp, imp, pmp] = pvMpp(P, g); voc = pvVoc(P, g); isc = pvCurrent(P, 0, g, []);
    fprintf('[MPP] G = %d W/m2\n', g);
    fprintf('  %-30s= %.3f\n', 'Isc_A', isc);
    fprintf('  %-30s= %.3f\n', 'Voc_V', voc);
    fprintf('  %-30s= %.3f\n', 'Vmp_V', vmp);
    fprintf('  %-30s= %.3f\n', 'Imp_A', imp);
    fprintf('  %-30s= %.3f\n', 'Pmp_W', pmp);
    fprintf('  %-30s= %.4f\n', 'D_mpp', 1 - (vmp + P.RL * imp) / P.Vbus);
end

% ---------- 工况 A ----------
fprintf('[A] D0 = %.2f, %.1f s irradiance %d -> %d W/m2\n', P.D0, P.Tstep, P.G1, P.G2);
R = cell(1, 3);
for s = 1:3
    c = struct('tstep', P.Tstep, 'ramp', false, 'd0', P.D0);
    r = poSim(P, steps(s), c, P.Tend);
    R{s} = r;
    pm = interp1(Gtab, MPPtab(:, 3), r.g);
    pre = r.t < P.Tstep;
    k = find(pre & r.p >= 0.99 * pm, 1);
    if isempty(k), ttrk = inf; else, ttrk = r.t(k); end
    m1 = r.t >= win1(1) & r.t < win1(2);
    m2 = r.t >= win2(1) & r.t < win2(2);
    ma = r.t >= P.Tstep & r.t < P.Tstep + 0.3;
    ratio = r.p(ma) ./ pm(ma); ta = r.t(ma);
    kb = find(ratio < 0.99, 1); kr = find(ratio(kb:end) >= 0.99, 1);
    fprintf('  dD = %.3f (dV ~ %.1f V/step)\n', steps(s), steps(s) * P.Vbus);
    fprintf('    %-28s= %.3f\n', 't_track_99%_s', ttrk);
    fprintf('    %-28s= %.2f\n', 'P_pp_1000_W', max(r.p(m1)) - min(r.p(m1)));
    fprintf('    %-28s= %.3f\n', 'eta_ss_1000_pct', 100 * mean(r.p(m1)) / mean(pm(m1)));
    fprintf('    %-28s= %.2f\n', 'P_min/Pmpp_after_step_pct', 100 * min(ratio));
    fprintf('    %-28s= %.2f\n', 't_back_in_99%_ms', 1e3 * (ta(kb + kr - 1) - P.Tstep));
    fprintf('    %-28s= %.2f\n', 'P_pp_600_W', max(r.p(m2)) - min(r.p(m2)));
    fprintf('    %-28s= %.3f\n', 'eta_ss_600_pct', 100 * mean(r.p(m2)) / mean(pm(m2)));
    fprintf('    %-28s= %.3f\n', 'eta_total_0-3s_pct', 100 * trapz(r.t, r.p) / trapz(r.t, pm));
end

% ---------- 工况 C：辐照斜坡 600 -> 1000 W/m2 ----------
fprintf('[C] ramp %d -> %d W/m2 (%.1f ~ %.1f s), D0 = 0.50\n', P.rampG, P.rampT);
for s = 1:3
    c = struct('tstep', inf, 'ramp', true, 'd0', 0.50);
    r = poSim(P, steps(s), c, 3.5);
    pm = interp1(Gtab, MPPtab(:, 3), r.g); vm = interp1(Gtab, MPPtab(:, 1), r.g);
    m = r.t >= P.rampT(1) & r.t <= P.rampT(2);
    fprintf('  dD=%.3f  eta_ramp=%.3f%%  max|Vpv-Vmp|=%.2f V\n', steps(s), ...
        100 * mean(r.p(m)) / mean(pm(m)), max(abs(r.v(m) - vm(m))));
end

% ---------- [D] 启动卡死 ----------
fprintf('[D] 600 W/m2 constant, D0 = %.2f\n', P.D0);
for s = 1:3
    c = struct('tstep', -1, 'ramp', false, 'd0', P.D0);
    r = poSim(P, steps(s), c, 1.5);
    k = r.t >= 1.4;
    fprintf('  dD=%.3f  mean P(1.4~1.5 s) = %.2f W, D = %.3f\n', steps(s), mean(r.p(k)), r.d(end));
end

% ---------- 作图（工况 A）----------
figure('Name', 'pv-mppt-po');
subplot(2, 1, 1); hold on;
for s = 1:3, plot(R{s}.t, R{s}.p); end
plot(R{1}.t, interp1(Gtab, MPPtab(:, 3), R{1}.g), 'k--');
legend('dD=0.002', 'dD=0.005', 'dD=0.02', 'Pmpp(G)', 'Location', 'southeast');
xlabel('t / s'); ylabel('Ppv / W'); grid on; title('P&O: startup + irradiance step 1000 -> 600 W/m^2');
subplot(2, 1, 2); hold on;
for s = 1:3, plot(R{s}.t, R{s}.v); end
plot(R{1}.t, interp1(Gtab, MPPtab(:, 1), R{1}.g), 'k--');
xlabel('t / s'); ylabel('Vpv / V'); grid on;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
saveas(gcf, fullfile(here, '..', 'img', 'pv_mppt_po_curves_matlab.png'));

% ====================== 局部函数 ======================
function i = pvCurrent(P, v, g, i)
    iph = P.Isc * g / 1000;
    if isempty(i), i = iph; end
    for it = 1:60
        e = exp(min((v + i * P.Rs) / P.a, 700));
        f = iph - P.I0 * (e - 1) - (v + i * P.Rs) / P.Rsh - i;
        df = -P.I0 * P.Rs / P.a * e - P.Rs / P.Rsh - 1;
        di = f / df; i = i - di;
        if abs(di) < 1e-12, break; end
    end
end

function voc = pvVoc(P, g)
    lo = 0; hi = 2 * P.Nser * P.VocMod;
    for it = 1:100
        mid = (lo + hi) / 2;
        if pvCurrent(P, mid, g, []) > 0, lo = mid; else, hi = mid; end
    end
    voc = (lo + hi) / 2;
end

function [v, i, p] = pvMpp(P, g)
    lo = 0; hi = pvVoc(P, g); r = (sqrt(5) - 1) / 2;
    x1 = hi - r * (hi - lo); x2 = lo + r * (hi - lo);
    f1 = x1 * pvCurrent(P, x1, g, []); f2 = x2 * pvCurrent(P, x2, g, []);
    for it = 1:120
        if f1 > f2
            hi = x2; x2 = x1; f2 = f1; x1 = hi - r * (hi - lo); f1 = x1 * pvCurrent(P, x1, g, []);
        else
            lo = x1; x1 = x2; f1 = f2; x2 = lo + r * (hi - lo); f2 = x2 * pvCurrent(P, x2, g, []);
        end
    end
    v = (lo + hi) / 2; i = pvCurrent(P, v, g, []); p = v * i;
end

function g = irr(P, t, c)
    if c.ramp
        if t <= P.rampT(1), g = P.rampG(1);
        elseif t >= P.rampT(2), g = P.rampG(2);
        else, g = P.rampG(1) + diff(P.rampG) * (t - P.rampT(1)) / diff(P.rampT);
        end
    elseif t < c.tstep, g = P.G1;
    else, g = P.G2;
    end
end

function [dv, di] = boostRhs(P, t, v, il, d, ig, c)
    ipv = pvCurrent(P, v, irr(P, t, c), ig);
    dv = (ipv - il) / P.Cin;
    di = (v - P.RL * il - (1 - d) * P.Vbus) / P.L;
    if il <= 0 && di < 0, di = 0; end   % 二极管阻断
end

function r = poSim(P, dstep, c, Tend)
    dt = P.dt; n = round(Tend / dt); nm = round(P.Tmppt / dt);
    t = (0:n) * dt;
    V = zeros(1, n + 1); I = V; D = V; G = V;
    v = pvVoc(P, irr(P, 0, c)); il = 0; d = c.d0;
    ig = pvCurrent(P, v, irr(P, 0, c), []);
    first = true; vp = 0; pp = 0;
    for k = 1:n + 1
        tk = t(k);
        if mod(k - 1, nm) == 0 && k > 1          % ---- P&O ----
            ps = v * ig;
            if first
                d = d + dstep; first = false;
            else
                dP = ps - pp; dV = v - vp;
                if dP ~= 0
                    if dP * dV > 0, d = d - dstep; else, d = d + dstep; end
                end
            end
            d = min(max(d, P.Dmin), P.Dmax);
            vp = v; pp = ps;
        end
        ig = pvCurrent(P, v, irr(P, tk, c), ig);
        V(k) = v; I(k) = ig; D(k) = d; G(k) = irr(P, tk, c);
        if k == n + 1, break; end
        [a1, b1] = boostRhs(P, tk, v, il, d, ig, c);
        [a2, b2] = boostRhs(P, tk + dt / 2, v + dt / 2 * a1, il + dt / 2 * b1, d, ig, c);
        [a3, b3] = boostRhs(P, tk + dt / 2, v + dt / 2 * a2, il + dt / 2 * b2, d, ig, c);
        [a4, b4] = boostRhs(P, tk + dt, v + dt * a3, il + dt * b3, d, ig, c);
        v = v + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
        il = max(il + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4), 0);
    end
    r.t = t; r.v = V; r.i = I; r.d = D; r.g = G; r.p = V .* I;
end

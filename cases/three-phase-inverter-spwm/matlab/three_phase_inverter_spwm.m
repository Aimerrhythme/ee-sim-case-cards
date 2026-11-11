% three_phase_inverter_spwm.m  三相两电平 SPWM 逆变器（教学演示）
%
% 载波峰值是 1，调制波是 m*sin。上管在调制波不低于载波时导通。
% 线性区（m <= 1）自然采样的基波：
%   Vao1(peak) = m * Vdc / 2
%   Vll1(peak) = (sqrt(3)/2) * m * Vdc
%   Vll1(rms)  = (sqrt(6)/4) * m * Vdc
% m > 1 不再沿这条直线走。上限是六阶梯 (2*sqrt(3)/pi)*Vdc。
%
% 自然采样：每段载波斜坡二分求交点，再对分段常数波形积傅里叶系数。
% 另外在均匀网格上比较正弦和三角波，做 FFT，并给星形 RL 积分电流。
%
% 默认：Vdc = 200 V，m = 0.8，f1 = 50 Hz，mf = 21，R = 10 ohm，L = 20 mH。
% 只用基础函数（sin、cos、asin、定步长循环、plot），不需要工具箱。
% 频谱图用基础库的 fft。文件末尾的局部函数从 R2016b 起属于基础语言。
%
% 运行:
%   run('cases/three-phase-inverter-spwm/matlab/three_phase_inverter_spwm.m')

clear; clc;

P.Vdc = 200;          % V
P.m = 0.8;
P.mLow = 0.5;
P.mUnity = 1.0;
P.mOver = 1.2;
P.mDeep = 1.5;
P.f1 = 50;            % Hz
P.mf = 21;
P.R = 10;             % ohm
P.L = 20e-3;          % H
P.spc = 1000;         % 每个载波的采样点，只用于波形、FFT 和电流
P.nSim = 3;           % 积分 3 个基波周期，读最后一个
P.hMax = 84;          % 含 4 倍载波
P.bisect = 60;
P.fc = P.mf * P.f1;
P.omega = 2 * pi * P.f1;
P.tFund = 1 / P.f1;
P.half = P.Vdc / 2;

mainCase = analyzeCase(P, P.m);
lowCase = analyzeCase(P, P.mLow);
unityCase = analyzeCase(P, P.mUnity);
overCase = analyzeCase(P, P.mOver);
deepCase = analyzeCase(P, P.mDeep);
sixCase = analyzeCase(P, NaN);

vaoTh = P.m * P.half;
vllTh = vllTheory(P, P.m);
vllRms = vllTh / sqrt(2);
kRms = sqrt(6) / 4;

[t, tri, ma, mb, mc, vao, vab, van] = sampleWaveforms(P);
nPer = P.mf * P.spc;
dt = P.tFund / nPer;
ia = integrateRl(P, van, dt);
idx = (2 * nPer + 1):(3 * nPer);
vAbC = vab(idx);
vAnC = van(idx);
iC = ia(idx);
vllFft = dftPeak(vAbC, 1);
[cV, sV] = dftCs(vAnC, 1);
[cI, sI] = dftCs(iC, 1);
i1Fft = hypot(cI, sI);
phiV = atan2(cV, sV);
phiI = atan2(cI, sI);
lag = (phiV - phiI) * 180 / pi;
if lag < 0
    lag = lag + 360;
end
zMag = hypot(P.R, P.omega * P.L);
phiDeg = atan2(P.omega * P.L, P.R) * 180 / pi;
i1Th = vaoTh / zMag;
thdISq = 0;
for h = 2:P.hMax
    [ch, sh] = dftCs(iC, h);
    ih = hypot(ch, sh);
    thdISq = thdISq + ih * ih;
end
thdI = sqrt(thdISq) / i1Fft;
fundErr = mainCase.Vll1 - vllTh;

rows = {
    'Vdc_V', P.Vdc; 'm', P.m; 'f1_Hz', P.f1; 'mf', P.mf; 'fc_Hz', P.fc;
    'R_ohm', P.R; 'L_H', P.L;
    'Vao1_theory_V', vaoTh; 'Vll1_theory_V', vllTh; 'Vll1_rms_theory_V', vllRms; 'k_rms', kRms;
    'Vao1_natural_V', mainCase.Vao1; 'Vll1_natural_V', mainCase.Vll1;
    'Vll1_bc_natural_V', mainCase.Vll1bc; 'Vll1_ca_natural_V', mainCase.Vll1ca;
    'rel_err_Vll1', fundErr / vllTh;
    'h3_V', mainCase.h3; 'h5_V', mainCase.h5; 'h7_V', mainCase.h7;
    'h19_V', mainCase.h19; 'h23_V', mainCase.h23;
    'h_side_n', mainCase.sideN; 'h_side_V', mainCase.sideV; 'h43_V', mainCase.h43;
    'thd_v_to_h84', mainCase.thd;
    'Vll1_fft_V', vllFft; 'rel_err_fft', (vllFft - vllTh) / vllTh;
    'm_low', P.mLow; 'Vll1_theory_low_V', vllTheory(P, P.mLow); 'Vll1_natural_low_V', lowCase.Vll1;
    'm_unity', P.mUnity; 'Vll1_theory_unity_V', vllTheory(P, P.mUnity);
    'Vll1_natural_unity_V', unityCase.Vll1;
    'm_over', P.mOver; 'Vll1_linear_over_V', vllTheory(P, P.mOver);
    'Vll1_clip_over_V', vllClip(P, P.mOver); 'Vll1_natural_over_V', overCase.Vll1;
    'h5_over_V', overCase.h5; 'h7_over_V', overCase.h7;
    'm_deep', P.mDeep; 'Vll1_linear_deep_V', vllTheory(P, P.mDeep);
    'Vll1_clip_deep_V', vllClip(P, P.mDeep); 'Vll1_natural_deep_V', deepCase.Vll1;
    'Vll1_six_theory_V', vllSix(P); 'Vll1_six_V', sixCase.Vll1; 'Vao1_six_V', sixCase.Vao1;
    'h5_six_V', sixCase.h5; 'h7_six_V', sixCase.h7;
    'Z_ohm', zMag; 'phi_deg', phiDeg; 'I1_theory_A', i1Th; 'I1_fft_A', i1Fft;
    'rel_err_I1', (i1Fft - i1Th) / i1Th; 'lag_meas_deg', lag; 'thd_i_to_h84', thdI
};

fprintf('Three-phase SPWM inverter\n');
for k = 1:size(rows, 1)
    v = rows{k, 2};
    if abs(v) < 5e-7, v = 0; end
    fprintf('  %-26s = %.6f\n', rows{k, 1}, v);
end
fprintf('  %-26s = %.3e\n', 'fund_err_V', fundErr);

% ---------- 作图 ----------
fig = figure('Name', 'three-phase-inverter-spwm', 'Position', [80 60 960 920]);
tMs = t * 1e3;
tLast = tMs(idx) - 40;
theta0 = P.omega * t(idx);
theta0 = theta0 - theta0(1);

subplot(3, 2, 1);
zoom = tMs <= 3;
plot(tMs(zoom), tri(zoom), 'Color', [0.61 0.64 0.67]); hold on;
plot(tMs(zoom), ma(zoom), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.2);
plot(tMs(zoom), mb(zoom), 'Color', [0.88 0.48 0.24]);
plot(tMs(zoom), mc(zoom), 'Color', [0.36 0.54 0.45]);
xlim([0 3]); ylim([-1.35 1.35]); grid on;
ylabel('调制波 / 载波'); title('比较  前 3 ms');
legend('载波', 'm_a', 'm_b', 'm_c', 'Location', 'northeast');

subplot(3, 2, 2);
plot(tLast, vao(idx), 'Color', [0.12 0.31 0.47]); hold on;
plot(tLast, vaoTh * sin(theta0), 'Color', [0.88 0.48 0.24], 'LineWidth', 1.2);
xlim([0 20]); ylim([-130 150]); grid on;
ylabel('电压 / V'); title('相电压（对直流中点）  两电平');
legend('v_{AO}', '基波 80 V', 'Location', 'northeast');

subplot(3, 2, 3);
plot(tLast, vAbC, 'Color', [0.12 0.31 0.47]); hold on;
plot(tLast, vllTh * cos(theta0 - pi / 3), 'Color', [0.88 0.48 0.24], 'LineWidth', 1.2);
xlim([0 20]); ylim([-260 280]); grid on;
ylabel('电压 / V'); title('线电压  三电平');
legend('v_{AB}', '基波 (\surd3/2) m V_{dc}', 'Location', 'northeast');

subplot(3, 2, 4);
spec = fft(vAbC);
nSpec = numel(vAbC);
mag = abs(spec) * (2 / nSpec);
mag(1) = abs(spec(1)) / nSpec;
nShow = floor(2500 / P.f1) + 1;
stem((0:nShow-1) * P.f1, mag(1:nShow), 'Color', [0.12 0.31 0.47], 'Marker', 'none'); hold on;
plot(P.f1, vllTh, 'o', 'Color', [0.75 0.22 0.17], 'MarkerFaceColor', [0.75 0.22 0.17]);
plot([P.fc P.fc], [0 160], '--', 'Color', [0.88 0.48 0.24]);
xlim([0 2500]); grid on;
ylabel('幅值 / V'); title('线电压 FFT');
legend('FFT', '公式 138.56 V', '载波 1050 Hz', 'Location', 'northeast');

subplot(3, 2, 5);
plot(tLast, iC, 'Color', [0.12 0.31 0.47], 'LineWidth', 1.1); hold on;
plot(tLast, i1Th * sin(theta0 - phiDeg * pi / 180), '--', 'Color', [0.88 0.48 0.24]);
xlim([0 20]); grid on;
xlabel('t / ms'); ylabel('电流 / A'); title('星形 RL  电流');
legend('i_A', '基波相量', 'Location', 'northeast');

subplot(3, 2, 6);
mLine = linspace(0, 1.6, 200);
vLin = (sqrt(3) / 2) * mLine * P.Vdc;
plot(mLine(mLine <= 1), vLin(mLine <= 1), 'Color', [0.12 0.31 0.47], 'LineWidth', 1.3); hold on;
plot(mLine(mLine >= 1), vLin(mLine >= 1), ':', 'Color', [0.12 0.31 0.47]);
mClip = mLine(mLine >= 1);
vClip = zeros(size(mClip));
for ii = 1:numel(mClip)
    vClip(ii) = vllClip(P, mClip(ii));
end
plot(mClip, vClip, '--', 'Color', [0.36 0.54 0.45]);
plot([0 1.65], [vllSix(P) vllSix(P)], 'Color', [0.6 0.6 0.6]);
mx = [P.mLow P.m P.mUnity P.mOver P.mDeep];
my = [lowCase.Vll1 mainCase.Vll1 unityCase.Vll1 overCase.Vll1 deepCase.Vll1];
plot(mx, my, 'o', 'Color', [0.75 0.22 0.17], 'MarkerFaceColor', [0.75 0.22 0.17]);
xlim([0 1.65]); ylim([0 280]); grid on;
xlabel('m'); ylabel('线电压基波峰值 / V'); title('调制比  过调制以后不再是直线');
legend('线性区公式', '把直线用到 m>1', '削顶平均模型', '六阶梯上限', '自然采样', ...
    'Location', 'northwest');

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
imgDir = fullfile(here, '..', 'img');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end
outPng = fullfile(imgDir, 'three_phase_inverter_spwm_curves_matlab.png');
saveas(fig, outPng);
fprintf('  %-26s = %s\n', 'figure', outPng);

% ---------- 校验 ----------
problems = {};
if abs(vaoTh - 80) > 1e-12 || abs(vllTh - 80 * sqrt(3)) > 1e-12
    problems{end+1} = '主工况手算应是相电压 80 V、线电压 80√3 V'; %#ok<SAGROW>
end
if abs(vllTheory(P, 1) - sqrt(3) / 2 * P.Vdc) > 1e-12
    problems{end+1} = 'm = 1 的线电压基波峰值应是 (√3/2)·Vdc';
end
if abs(mainCase.Vll1 - vllTh) > 1e-8
    problems{end+1} = '线性区自然采样的线电压基波偏离 (√3/2)·m·Vdc';
end
if abs(mainCase.Vao1 - vaoTh) > 1e-8
    problems{end+1} = '线性区自然采样的相电压基波偏离 m·Vdc/2';
end
if abs(mainCase.Vll1 - sqrt(3) * mainCase.Vao1) > 1e-8
    problems{end+1} = '线电压基波应是相电压基波的 √3 倍';
end
if abs(lowCase.Vll1 - vllTheory(P, P.mLow)) > 1e-8 || abs(unityCase.Vll1 - vllTheory(P, 1)) > 1e-8
    problems{end+1} = 'm = 0.5 或 1 的线电压基波没有落在直线上';
end
if mainCase.h3 > 1e-6 || mainCase.h5 > 1e-6 || mainCase.h7 > 1e-6
    problems{end+1} = '线性区线电压的低次谐波应只剩数值噪声';
end
if abs(mainCase.h19 - mainCase.h23) > 1e-6
    problems{end+1} = '19 次和 23 次边带应成对、幅值相同';
end
if abs(mainCase.h43 - mainCase.sideV) > 1e-6
    problems{end+1} = '41 次和 43 次边带应成对、幅值相同';
end
if abs(mainCase.Vll1bc - mainCase.Vll1) > 1e-8 || abs(mainCase.Vll1ca - mainCase.Vll1) > 1e-8
    problems{end+1} = '三条线电压的基波幅值应相同';
end
if abs(vllFft - vllTh) / vllTh > 1e-3
    problems{end+1} = '定步长 FFT 的基波离公式超过 0.1%';
end
if ~(vllTheory(P, 1) < overCase.Vll1 && overCase.Vll1 < vllTheory(P, P.mOver))
    problems{end+1} = 'm = 1.2 的基波应落在线性区上限和直线外推之间';
end
if abs(overCase.Vll1 - vllClip(P, P.mOver)) / vllClip(P, P.mOver) > 0.01
    problems{end+1} = 'm = 1.2 的自然采样离削顶模型超过 1%';
end
if overCase.h5 < 1
    problems{end+1} = '过调制后线电压应出现明显的 5 次谐波';
end
if ~(overCase.Vll1 < deepCase.Vll1 && deepCase.Vll1 < vllSix(P))
    problems{end+1} = 'm = 1.5 应比 m = 1.2 更靠近六阶梯，但仍低于上限';
end
if abs(sixCase.Vll1 - vllSix(P)) > 1e-8
    problems{end+1} = '六阶梯线电压基波应是 (2√3/π)·Vdc';
end
if abs(sixCase.Vao1 - 2 * P.Vdc / pi) > 1e-8
    problems{end+1} = '六阶梯相电压基波应是 2·Vdc/π';
end
if abs(sixCase.h5 - sixCase.Vll1 / 5) > 1e-6
    problems{end+1} = '六阶梯 5 次谐波应是基波的 1/5';
end
if abs(sixCase.h7 - sixCase.Vll1 / 7) > 1e-6
    problems{end+1} = '六阶梯 7 次谐波应是基波的 1/7';
end
if abs(i1Fft - i1Th) / i1Th > 2e-3
    problems{end+1} = '电流基波偏离相电压基波除以 |Z|';
end
if abs(lag - phiDeg) > 0.05
    problems{end+1} = '电流滞后角偏离 atan(ωL/R)';
end
if thdI > 0.1
    problems{end+1} = '20 mH 把电流谐波压到 10% 以内，否则电感参数不对';
end
vabLevels = unique(round(vab(1:nPer), 6));
if numel(vabLevels) ~= 3 || abs(vabLevels(1) + P.Vdc) > 1e-6 || abs(vabLevels(end) - P.Vdc) > 1e-6
    problems{end+1} = '线电压应只取 −Vdc、0、+Vdc';
end
vanLevels = unique(round(van(1:nPer), 6));
expectVan = ([-2; -1; 0; 1; 2]) * P.Vdc / 3;
if numel(vanLevels) ~= 5 || max(abs(vanLevels - expectVan)) > 1e-6
    problems{end+1} = '悬浮星形的相电压应取 ±Vdc/3、±2Vdc/3 和 0';
end
if mod(mainCase.sideN, 3) == 0
    problems{end+1} = '线电压里最大的边带不应落在 3 的倍数次上';
end

if ~isempty(problems)
    error('校验未通过：%s', strjoin(problems, '；'));
end

% ======================= 局部函数 =======================
function y = modMinusTri(t, modulation, phase, t0, rising, P)
    if rising
        tri = -1 + 4 * P.fc * (t - t0);
    else
        tri = 3 - 4 * P.fc * (t - t0);
    end
    y = modulation * sin(P.omega * t + phase) - tri;
end

function segs = poleSegments(P, modulation, phase, tEnd)
    nCar = round(tEnd * P.fc);
    if abs(nCar / P.fc - tEnd) > 1e-9
        error('分析窗口必须是载波周期的整数倍。');
    end
    segs = zeros(nCar * 4, 3);
    nSeg = 0;
    half = P.half;
    for p = 0:nCar-1
        t0 = p / P.fc;
        tAList = [t0, t0 + 0.5 / P.fc];
        tBList = [t0 + 0.5 / P.fc, t0 + 1 / P.fc];
        for q = 1:2
            rising = (q == 1);
            tA = tAList(q);
            tB = tBList(q);
            fLo = modMinusTri(tA, modulation, phase, t0, rising, P);
            fHi = modMinusTri(tB, modulation, phase, t0, rising, P);
            if rising
                tri0 = -1;
            else
                tri0 = 1;
            end
            leftOn = modulation * sin(P.omega * tA + phase) >= tri0;
            hasCross = false;
            cross = 0;
            if fLo == 0
                hasCross = true;
                cross = tA;
            elseif fLo * fHi < 0
                lo = tA;
                hi = tB;
                fLeft = fLo;
                for iter = 1:P.bisect %#ok<NASGU>
                    mid = 0.5 * (lo + hi);
                    fMid = modMinusTri(mid, modulation, phase, t0, rising, P);
                    if fLeft * fMid <= 0
                        hi = mid;
                    else
                        lo = mid;
                        fLeft = fMid;
                    end
                end
                hasCross = true;
                cross = 0.5 * (lo + hi);
            end
            if ~hasCross
                nSeg = nSeg + 1;
                if leftOn
                    segs(nSeg, :) = [tA, tB, half];
                else
                    segs(nSeg, :) = [tA, tB, -half];
                end
            else
                if leftOn
                    level0 = half;
                else
                    level0 = -half;
                end
                nSeg = nSeg + 1;
                segs(nSeg, :) = [tA, cross, level0];
                nSeg = nSeg + 1;
                segs(nSeg, :) = [cross, tB, -level0];
            end
        end
    end
    segs = segs(1:nSeg, :);
end

function segs = sixStepSegments(P, phase, tEnd)
    nMin = floor(phase / pi) - 1;
    nMax = ceil((P.omega * tEnd + phase) / pi) + 1;
    times = [0; tEnd];
    for n = nMin:nMax
        tZero = (n * pi - phase) / P.omega;
        if tZero > 0 && tZero < tEnd
            times(end + 1, 1) = tZero; %#ok<AGROW>
        end
    end
    times = sort(times);
    nInt = numel(times) - 1;
    segs = zeros(nInt, 3);
    half = P.half;
    for i = 1:nInt
        tA = times(i);
        tB = times(i + 1);
        mid = 0.5 * (tA + tB);
        if sin(P.omega * mid + phase) >= 0
            level = half;
        else
            level = -half;
        end
        segs(i, :) = [tA, tB, level];
    end
end

function cs = fourierAll(P, segs, hMax, tEnd)
    accC = zeros(hMax, 1);
    accS = zeros(hMax, 1);
    for i = 1:size(segs, 1)
        t0 = segs(i, 1);
        t1 = segs(i, 2);
        level = segs(i, 3);
        if t1 <= t0
            continue
        end
        for h = 1:hMax
            w = h * P.omega;
            accC(h) = accC(h) + level * (sin(w * t1) - sin(w * t0)) / w;
            accS(h) = accS(h) + level * (cos(w * t0) - cos(w * t1)) / w;
        end
    end
    scale = 2 / tEnd;
    cs = [accC, accS] * scale;
end

function v = linePeak(cs, h, iLeg, jLeg)
    c = cs{iLeg}(h, 1) - cs{jLeg}(h, 1);
    s = cs{iLeg}(h, 2) - cs{jLeg}(h, 2);
    v = hypot(c, s);
end

function r = analyzeCase(P, modulation)
    phases = [0, -2 * pi / 3, 2 * pi / 3];
    cs = cell(1, 3);
    for iLeg = 1:3
        if isnan(modulation)
            segs = sixStepSegments(P, phases(iLeg), P.tFund);
        else
            segs = poleSegments(P, modulation, phases(iLeg), P.tFund);
        end
        cs{iLeg} = fourierAll(P, segs, P.hMax, P.tFund);
    end
    r.Vao1 = hypot(cs{1}(1, 1), cs{1}(1, 2));
    r.Vll1 = linePeak(cs, 1, 1, 2);
    r.Vll1bc = linePeak(cs, 1, 2, 3);
    r.Vll1ca = linePeak(cs, 1, 3, 1);
    r.h3 = linePeak(cs, 3, 1, 2);
    r.h5 = linePeak(cs, 5, 1, 2);
    r.h7 = linePeak(cs, 7, 1, 2);
    r.h19 = linePeak(cs, 19, 1, 2);
    r.h23 = linePeak(cs, 23, 1, 2);
    r.h43 = linePeak(cs, 43, 1, 2);
    sideN = 2;
    sideV = 0;
    thdSq = 0;
    for h = 2:P.hMax
        vh = linePeak(cs, h, 1, 2);
        thdSq = thdSq + vh * vh;
        if vh > sideV
            sideV = vh;
            sideN = h;
        end
    end
    r.sideN = sideN;
    r.sideV = sideV;
    r.thd = sqrt(thdSq) / r.Vll1;
end

function v = vllTheory(P, modulation)
    v = (sqrt(3) / 2) * modulation * P.Vdc;
end

function v = vllClip(P, modulation)
    if modulation <= 1
        v = vllTheory(P, modulation);
        return
    end
    alpha = asin(1 / modulation);
    bracket = modulation * alpha + sqrt(1 - 1 / (modulation * modulation));
    v = sqrt(3) * (P.Vdc / pi) * bracket;
end

function v = vllSix(P)
    v = (2 * sqrt(3) / pi) * P.Vdc;
end

function [t, tri, ma, mb, mc, vao, vab, van] = sampleWaveforms(P)
    nPer = P.mf * P.spc;
    n = P.nSim * nPer;
    k = (0:n-1)';
    theta = 2 * pi * k / nPer;
    pos = mod(k, P.spc) / P.spc;
    tri = 1 - 4 * abs(pos - 0.5);
    ma = P.m * sin(theta);
    mb = P.m * sin(theta - 2 * pi / 3);
    mc = P.m * sin(theta + 2 * pi / 3);
    vao = p2level(ma, tri, P.half);
    vbo = p2level(mb, tri, P.half);
    vco = p2level(mc, tri, P.half);
    vab = vao - vbo;
    van = vao - (vao + vbo + vco) / 3;
    t = k / (P.f1 * nPer);
end

function v = p2level(modWave, tri, half)
    v = half * ones(size(modWave));
    v(modWave < tri) = -half;
end

function [c, s] = dftCs(x, harmonic)
    n = numel(x);
    k = (0:n-1)';
    th = 2 * pi * harmonic * k / n;
    c = (2 / n) * sum(x .* cos(th));
    s = (2 / n) * sum(x .* sin(th));
end

function pk = dftPeak(x, harmonic)
    [c, s] = dftCs(x, harmonic);
    pk = hypot(c, s);
end

function iOut = integrateRl(P, voltage, dt)
    a = exp(-dt / (P.L / P.R));
    b = (1 - a) / P.R;
    iOut = zeros(numel(voltage), 1);
    acc = 0;
    for k = 1:numel(voltage)
        acc = a * acc + b * voltage(k);
        iOut(k) = acc;
    end
end

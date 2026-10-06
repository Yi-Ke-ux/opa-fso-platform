/* ============================================================
   OPA-FSO 仿真平台 · 公共物理引擎与 UI 工具库
   纯原生 JS，无依赖（ECharts 由各页面 CDN 引入）
   ============================================================ */
(function (global) {
  'use strict';

  const D2R = Math.PI / 180;
  const R2D = 180 / Math.PI;
  const C0 = 299792458;

  /* ---------------- 基础数学 ---------------- */
  function sinc(x) { return Math.abs(x) < 1e-12 ? 1 : Math.sin(x) / x; }

  // 互补误差函数（Abramowitz-Stegun 7.1.26）
  function erfc(x) {
    const z = Math.abs(x);
    const t = 1 / (1 + 0.3275911 * z);
    // 下列逼近的是 erf(|x|)
    const erfAbs = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-z * z);
    const erfx = x >= 0 ? erfAbs : -erfAbs;
    return 1 - erfx;   // erfc = 1 - erf
  }
  // Q 函数（高斯右尾）
  function qfunc(x) { return 0.5 * erfc(x / Math.SQRT2); }
  function dbA(x) { return 20 * Math.log10(x); }   // 幅度 -> dB
  function dbP(x) { return 10 * Math.log10(x); }   // 功率 -> dB

  /* ============================================================
     一、线性光学相控阵（一维，横向 / 相位控制）
     AF(θ)=Σ a_n·exp[j(n·k·d·sinθ + φ_n)]
     ============================================================ */

  // 生成阵元相位：线性相位（指向 steerDeg）+ 可选相位误差 + 自定义相位
  function linearPhases(N, steerDeg, d, lambda, phaseErr) {
    const k = 2 * Math.PI / lambda;
    const dphi = k * d * Math.sin(steerDeg * D2R);   // 相邻阵元所需相位差（延迟量）
    const ph = new Array(N);
    for (let n = 0; n < N; n++) {
      // AF 项为 kd sinθ·n + phase，主瓣在 +steer 需 phase = −n·dphi
      ph[n] = -n * dphi + (phaseErr && phaseErr[n] ? phaseErr[n] : 0);
    }
    return { phases: ph, dphi };
  }

  // 通用一维阵列因子（复数模）。weights: 幅度加权数组(可选)，phases: 相位数组
  // 返回 {ang:[度], psi:[|AF|], db:[归一化dB]}
  function arrayFactor1D(opts) {
    const { N, d, lambda, phases, weights, angMin, angMax, angStep } = opts;
    const k = 2 * Math.PI / lambda;
    const ang = [];
    for (let a = angMin; a <= angMax + 1e-9; a += angStep) ang.push(a);
    const psi = new Array(ang.length).fill(0);
    for (let i = 0; i < ang.length; i++) {
      let re = 0, im = 0;
      const base = k * d * Math.sin(ang[i] * D2R);
      for (let n = 0; n < N; n++) {
        const ph = base * n + (phases ? phases[n] : 0);
        const w = weights ? weights[n] : 1;
        re += w * Math.cos(ph);
        im += w * Math.sin(ph);
      }
      psi[i] = Math.hypot(re, im) / N;   // 归一化（除以 N，主瓣≈1）
    }
    let mx = Math.max(...psi);
    const db = psi.map(v => v <= 1e-9 ? -90 : Math.max(-90, dbA(v / mx)));
    return { ang, psi, db };
  }

  // 便捷：直接给转向角，返回理想（无误差）方向图
  function steeredPattern(N, d, lambda, steerDeg, range, step, weights, phaseErr) {
    const { phases } = linearPhases(N, steerDeg, d, lambda, phaseErr);
    return arrayFactor1D({ N, d, lambda, phases, weights, angMin: steerDeg - range, angMax: steerDeg + range, angStep: step });
  }

  // 转向角 <-> 相邻相位差
  function steerToDphi(steerDeg, d, lambda) { return (2 * Math.PI / lambda) * d * Math.sin(steerDeg * D2R); }
  function dphiToSteer(dphi, d, lambda) { return Math.asin(Math.max(-1, Math.min(1, dphi * lambda / (2 * Math.PI * d)))) * R2D; }

  /* ---------------- 波束宽度 / 视场（端射线阵理论，论文4） ---------------- */
  // 无栅瓣单侧最大角 sin^-1(λ/2d)
  function fovEdge(lambda, d) { return Math.asin(Math.min(1, lambda / (2 * d))) * R2D; }
  // 均匀孔径半功率波束宽度（度）：0.886 λ/(N d)
  function hpbwUniform(lambda, N, d) { return 0.886 * lambda / (N * d) * R2D; }
  // 角分辨率（瑞利，主瓣零点/光栅）—论文5用0.3°，提供通用主瓣零点宽 λ/(Nd)
  function nullToNull(lambda, N, d) { return 2 * lambda / (N * d) * R2D; }

  /* ============================================================
     二、光栅天线（纵向 / 波长控制）
     sinθ = n_eff − λ/Λ
     ============================================================ */
  function gratingAngle(lambda, neff, Lambda) {
    const s = neff - lambda / Lambda;
    if (s < -1 || s > 1) return NaN;
    return Math.asin(s) * R2D;
  }
  // 论文1形式：sinθ=(λ·n_eff − λ0·n_cl)/Λ
  function gratingAngleV2(lambda, neff, lambda0, ncl, Lambda) {
    const s = (lambda * neff - lambda0 * ncl) / Lambda;
    if (s < -1 || s > 1) return NaN;
    return Math.asin(s) * R2D;
  }
  // 由目标角反解光栅周期 Λ = λ/(n_eff − sinθ)
  function gratingPeriod(lambda, neff, angleDeg) { return lambda / (neff - Math.sin(angleDeg * D2R)); }

  /* ============================================================
     三、高斯光束
     ============================================================ */
  function gaussDivergence(lambda, w0) { return lambda / (Math.PI * w0) * R2D; } // 远场半角(度)
  function gaussIntensity(r, w) { return Math.exp(-2 * r * r / (w * w)); }

  /* ============================================================
     四、方向图特征：峰值 / FWHM / PSLR(SMSR)
     ============================================================ */
  function analyzePattern(ang, psi) {
    // 找全局最大
    let ip = 0;
    for (let i = 1; i < psi.length; i++) if (psi[i] > psi[ip]) ip = i;
    const peak = psi[ip], peakAng = ang[ip];
    const half = peak / Math.SQRT2; // 半功率（幅度 1/√2 → 功率一半）
    // FWHM（线性插值）
    function cross(i0, dir) {
      for (let i = i0; (dir > 0 ? i < psi.length - 1 : i > 0); i += dir) {
        const a = psi[i], b = psi[i + dir];
        if ((a - half) * (b - half) <= 0) {
          const t = (half - a) / (b - a);
          return ang[i] + t * (ang[i + dir] - ang[i]);
        }
      }
      return ang[i0];
    }
    const lo = cross(ip, -1), hi = cross(ip, 1);
    const fwhm = hi - lo;
    // 主瓣零点（找峰值两侧第一个极小值）
    function nullIdx(i0, dir) {
      for (let i = i0; (dir > 0 ? i < psi.length - 2 : i > 1); i += dir) {
        if (psi[i] <= psi[i - dir] && psi[i] <= psi[i + dir] && psi[i] < peak * 0.6) return i;
      }
      return -1;
    }
    const nl = nullIdx(ip, -1), nr = nullIdx(ip, 1);
    // 最高旁瓣（主瓣零点之外）
    let side = 0;
    const lEnd = nl >= 0 ? nl : 0, rStart = nr >= 0 ? nr : psi.length - 1;
    for (let i = 0; i < lEnd; i++) side = Math.max(side, psi[i]);
    for (let i = rStart; i < psi.length; i++) side = Math.max(side, psi[i]);
    const pslr = side > 0 ? dbA(peak / side) : 90;
    return { peak, peakAng, fwhm, lo, hi, pslr, nl, nr };
  }

  /* ============================================================
     五、通信链路：BER（OOK/IM-DD 等效高斯模型，论文2）
     ============================================================ */
  // 由接收功率(dBm) -> BER。SNR_dB = a + b*P，BER=Q(sqrt(SNR/2))
  function berFromPower(pDbm, rateGbps) {
    const slope = 2.2;                 // SNR 随功率斜率(dB/dBm)
    const snrDb = 95.7 + slope * pDbm; // 默认贴合论文2: -36dBm≈1e-6,-35≈1e-9
    let snr = Math.pow(10, snrDb / 10);
    // 速率越高，等效噪声带宽越大，灵敏度略降（简化）
    if (rateGbps) snr /= Math.max(1, rateGbps / 2.8);
    return qfunc(Math.sqrt(snr / 2));
  }
  function powerSweep(pMin, pMax, step, rateGbps) {
    const P = [], B = [];
    for (let p = pMin; p <= pMax; p += step) { P.push(+p.toFixed(2)); B.push(berFromPower(p, rateGbps)); }
    return { P, B };
  }

  // 眼图采样点（供 Canvas 绘制）：生成带噪声的0/1电平轨迹
  function eyeTrace(bitRateGbps, snrDb, nTraces, ptsPerBit) {
    const traces = [];
    const sigma = Math.pow(10, -snrDb / 20) * 0.5;
    for (let t = 0; t < nTraces; t++) {
      const bits = [Math.random() > .5 ? 1 : 0, Math.random() > .5 ? 1 : 0, Math.random() > .5 ? 1 : 0];
      const line = [];
      for (let b = 0; b < 2; b++) {
        for (let q = 0; q < ptsPerBit; q++) {
          const x = b + q / ptsPerBit;
          // 过渡在比特边界
          let v;
          const prev = bits[b], cur = bits[b + 1];
          const u = q / ptsPerBit;
          v = prev + (cur - prev) * smooth(Math.max(0, (u - 0.35) / 0.3));
          v += gaussRand() * sigma;
          line.push([x, v]);
        }
      }
      traces.push(line);
    }
    return traces;
  }
  function smooth(t) { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); }
  function gaussRand() { let u = 0, v = 0; while (u === 0) u = Math.random(); while (v === 0) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

  /* ============================================================
     六、二维阵列因子（2D 光斑 / 扫描点阵）
     ============================================================ */
  // Nx×Ny 网格，相位指向 (sx,sy)。返回角度轴 + 强度矩阵
  function arrayFactor2D(opts) {
    const { Nx, Ny, dx, dy, lambda, steerX, steerY, span, step, phaseErrX, phaseErrY } = opts;
    const k = 2 * Math.PI / lambda;
    const ax = [];
    for (let a = steerX - span; a <= steerX + span + 1e-9; a += step) ax.push(a);
    const ay = [];
    for (let a = steerY - span; a <= steerY + span + 1e-9; a += step) ay.push(a);
    const dpx = k * dx * Math.sin(steerX * D2R), dpy = k * dy * Math.sin(steerY * D2R);
    const M = Array.from({ length: ay.length }, () => new Array(ax.length).fill(0));
    let mx = 0;
    for (let j = 0; j < ay.length; j++) {
      for (let i = 0; i < ax.length; i++) {
        let re = 0, im = 0;
        const bx = k * dx * Math.sin(ax[i] * D2R);
        const by = k * dy * Math.sin(ay[j] * D2R);
        for (let ny = 0; ny < Ny; ny++) for (let nx = 0; nx < Nx; nx++) {
          const ph = bx * nx + by * ny - dpx * nx - dpy * ny
            + (phaseErrX ? phaseErrX[nx] || 0 : 0) + (phaseErrY ? phaseErrY[ny] || 0 : 0);
          re += Math.cos(ph); im += Math.sin(ph);
        }
        const v = Math.hypot(re, im) / (Nx * Ny);
        M[j][i] = v; if (v > mx) mx = v;
      }
    }
    for (let j = 0; j < ay.length; j++) for (let i = 0; i < ax.length; i++) M[j][i] /= mx;
    return { ax, ay, M };
  }

  /* ============================================================
     七、可视化：ECharts 深色封装
     ============================================================ */
  const ECHART_TEXT = '#505f7b', ECHART_AXIS = '#d3dbe6';
  function makeChart(idOrEl) {
    const el = typeof idOrEl === 'string' ? document.getElementById(idOrEl) : idOrEl;
    const ch = echarts.init(el, null, { renderer: 'canvas' });
    const base = {
      backgroundColor: 'transparent',
      textStyle: { fontFamily: "'Noto Sans SC',sans-serif", color: ECHART_TEXT },
      grid: { left: 62, right: 24, top: 48, bottom: 52 },
      tooltip: { backgroundColor: 'rgba(12,18,32,.94)', borderColor: '#2a3a5e', textStyle: { color: '#e7eefb', fontSize: 12 } },
      xAxis: { axisLine: { lineStyle: { color: ECHART_AXIS } }, axisLabel: { color: ECHART_TEXT }, splitLine: { lineStyle: { color: 'rgba(31,45,80,.08)' } }, nameTextStyle: { color: ECHART_TEXT } },
      yAxis: { axisLine: { lineStyle: { color: ECHART_AXIS } }, axisLabel: { color: ECHART_TEXT }, splitLine: { lineStyle: { color: 'rgba(31,45,80,.08)' } }, nameTextStyle: { color: ECHART_TEXT } }
    };
    ch._base = base;
    window.addEventListener('resize', () => ch.resize());
    return ch;
  }
  // 通用方向图折线
  function patternSeries(ang, db, name, color) {
    return {
      name: name || '方向图', type: 'line', data: ang.map((a, i) => [a, db[i]]),
      showSymbol: false, smooth: false, lineStyle: { width: 2.4, color: color || '#0c8fa0' },
      areaStyle: { color: { type: 'linear', x: 0, y: 0, x2: 0, y2: 1, colorStops: [{ offset: 0, color: 'rgba(12,143,160,.22)' }, { offset: 1, color: 'rgba(12,143,160,0)' }] } },
      animationDuration: 400
    };
  }

  /* ============================================================
     八、可视化：Canvas 2D 热图 / 光斑
     ============================================================ */
  // 热力配色（深蓝→青→金）
  function heatColor(t) {
    t = Math.max(0, Math.min(1, t));
    const stops = [[6, 10, 26], [20, 60, 110], [30, 150, 180], [120, 220, 200], [246, 200, 90], [255, 255, 230]];
    const f = t * (stops.length - 1), i = Math.floor(f), u = f - i;
    const a = stops[i], b = stops[Math.min(stops.length - 1, i + 1)];
    return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
  }
  // 将 2D 强度矩阵渲染到 canvas
  function renderHeatmap(canvas, M, ax, ay, opts) {
    opts = opts || {};
    const W = canvas.width, H = canvas.height;
    const ctx = canvas.getContext('2d');
    const img = ctx.createImageData(W, H);
    const ny = M.length, nx = M[0].length;
    const gamma = opts.gamma || 0.6;
    for (let y = 0; y < H; y++) {
      const jj = Math.min(ny - 1, Math.floor((1 - y / H) * ny));
      for (let x = 0; x < W; x++) {
        const ii = Math.floor((x / W) * nx);
        let v = M[jj][ii];
        v = Math.pow(v, gamma);
        const c = heatColor(v);
        const o = (y * W + x) * 4;
        img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2]; img.data[o + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    // 十字准星
    ctx.strokeStyle = 'rgba(255,255,255,.25)'; ctx.setLineDash([4, 5]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(W / 2, 0); ctx.lineTo(W / 2, H); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
    ctx.setLineDash([]);
  }
  // 清空为深色底
  function clearCanvas(canvas) {
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#05080f'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  }
  // 在 canvas 上画高斯光斑（cx,cy 为 0..1 归一化，r 为半径像素，颜色）
  function drawGaussianSpot(canvas, cx, cy, r, rgb, alpha) {
    const ctx = canvas.getContext('2d');
    const X = cx * canvas.width, Y = cy * canvas.height;
    const g = ctx.createRadialGradient(X, Y, 0, X, Y, r);
    g.addColorStop(0, `rgba(${rgb},${alpha})`);
    g.addColorStop(0.4, `rgba(${rgb},${alpha * 0.5})`);
    g.addColorStop(1, `rgba(${rgb},0)`);
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(X, Y, r, 0, 7); ctx.fill();
  }
  function fitCanvas(canvas, cssH) {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(2, Math.floor(rect.width * dpr));
    canvas.height = Math.floor((cssH || rect.height) * dpr);
    canvas.style.height = (cssH || rect.height) + 'px';
    return { w: canvas.width, h: canvas.height, dpr };
  }

  /* ============================================================
     九、UI 工具
     ============================================================ */
  // Tab 切换（按钮加 data-target=pane id）
  function setupTabs(scope) {
    const root = scope || document;
    root.querySelectorAll('.tab-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const group = btn.closest('.tabs');
        group.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
        const tgt = document.getElementById(btn.dataset.target);
        if (tgt) {
          tgt.classList.add('active');
          // 触发图表 resize
          setTimeout(() => window.dispatchEvent(new Event('resize')), 30);
        }
      });
    });
  }
  // 滑块绑定：input 元素，回调(value, 显示元素)
  function bindSlider(input, onChange, fmt) {
    const out = input.parentElement.querySelector('.val') || input.closest('.ctrl').querySelector('.val');
    function upd() {
      const v = parseFloat(input.value);
      if (out) out.textContent = fmt ? fmt(v) : v;
      if (onChange) onChange(v);
    }
    input.addEventListener('input', upd);
    upd();
  }
  function setSliderText(input, text) {
    const out = input.parentElement.querySelector('.val') || input.closest('.ctrl').querySelector('.val');
    if (out) out.textContent = text;
  }
  // 日志
  function logTo(el, msg, cls) {
    const time = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    el.textContent += `[${time}] ${msg}\n`;
    el.scrollTop = el.scrollHeight;
  }
  function clearLog(el) { el.textContent = ''; }
  function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

  // 高亮公式（KaTeX auto-render）
  function renderMath(scope) {
    if (global.renderMathInElement) {
      global.renderMathInElement(scope || document.body, {
        delimiters: [
          { left: '$$', right: '$$', display: true },
          { left: '$', right: '$', display: false },
          { left: '\\[', right: '\\]', display: true },
          { left: '\\(', right: '\\)', display: false }
        ],
        throwOnError: false
      });
    }
  }

  /* ---------------- 导出 ---------------- */
  global.OPA = {
    D2R, R2D, C0,
    erfc, qfunc, dbA, dbP, sinc,
    linearPhases, arrayFactor1D, steeredPattern, steerToDphi, dphiToSteer,
    fovEdge, hpbwUniform, nullToNull,
    gratingAngle, gratingAngleV2, gratingPeriod,
    gaussDivergence, gaussIntensity,
    analyzePattern,
    berFromPower, powerSweep, eyeTrace, gaussRand,
    arrayFactor2D,
    makeChart, patternSeries,
    heatColor, renderHeatmap, clearCanvas, drawGaussianSpot, fitCanvas,
    setupTabs, bindSlider, setSliderText, logTo, clearLog, sleep, renderMath
  };
})(window);

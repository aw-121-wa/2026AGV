/* 航向环自动调参：调试台 agv_console.html 和 node 测试 pid_tune_test.js 共用。
   1. 解析固件的调参日志（格式见 firmware/User/Comm/tune_log.h）。
   2. 从日志拟合车的模型：下发角速度 → 延迟 delay → 一阶惯性 tau → × K（打滑） → 航向，
      平移时再加 cx·vx + cy·vy 的航向扰动。
   3. 按固件的航向环原样仿真：Chassis_HeadingControl 的 PI、积分区、限幅、到位死区，
      角加速度斜坡，0.1 RPM 轮速量化，路径的“进容差就算到”。
   4. 在拟合出来的模型上搜 kp、ki：转 90°/20°/5° 和平移的到位时间、超调、平移时航向误差，
      模型参数在拟合误差范围里随机抽一批（蒙特卡洛），挑这批车上都表现好的。 */
(function (root) {
  'use strict';

  const DT = 0.005;                // 固件控制周期
  const IMU_STEP = 180 / 32768;    // HWT101 航向分辨率
  const MAX_WHEEL_X10 = 4000;      // CHASSIS_MAX_WHEEL_RPM × 10
  const SIGN_VY = [-1, 1, 1, -1];  // 逆运动学符号，和 chassis.c 的 k_sign 一致（LF RF LR RR）
  const SIGN_WZ = [-1, 1, -1, 1];
  const HOLD_S = 0.6;              // 进容差后至少稳这么久才算到位
  /* 算“到位”用的带子比航向环的死区宽 0.05°：航向环进了死区就不再修，车可能正好停在死区边上，
     噪声把它来回推过边，用死区本身当标准，到位时间就成了看噪声运气。 */
  const BAND_EXTRA = 0.05;
  const WINDOW_S = 6;              // 仿真最长时间

  const wrap = a => { a %= 360; return a > 180 ? a - 360 : a < -180 ? a + 360 : a; }; // Math_WrapDeg
  const clamp = (v, lim) => (v > lim ? lim : v < -lim ? -lim : v);
  const roundAway = x => (x < 0 ? -Math.round(-x) : Math.round(x)); // lroundf

  /* ---------- 1. 解析 ---------- */
  function parseFields(text) {
    const out = {};
    for (const item of text.trim().split(/\s+/)) {
      const eq = item.indexOf('=');
      if (eq > 0) out[item.slice(0, eq)] = item.slice(eq + 1);
    }
    return out;
  }

  /* 一行一行喂；一段完整的日志返回 {kind:'record', record}，作废的返回 {kind:'abort'}，
     没有日志 {kind:'none'}，缺行 {kind:'broken'}；其余返回 null。 */
  function createParser() {
    let cur = null;
    return {
      feed(line) {
        if (line.startsWith('L ')) {
          if (!cur) return null;
          const v = line.slice(2).split(' ').map(Number);
          if (v.length !== 9 || v.some(x => !Number.isFinite(x)) || v[0] !== cur.rows.length) cur.broken = true;
          else cur.rows.push(v);
          return null;
        }
        if (line.startsWith('LOG begin ')) {
          const f = parseFields(line.slice(10));
          cur = { n: Number(f.n), mode: f.mode, end: f.end, trunc: f.trunc === '1', imu: f.imu !== '0', params: {}, rows: [], broken: false };
          return null;
        }
        if (line.startsWith('LOG param ')) {
          if (cur) for (const [k, v] of Object.entries(parseFields(line.slice(10)))) cur.params[k] = Number(v);
          return null;
        }
        if (line.startsWith('LOG end')) {
          const r = cur;
          cur = null;
          if (!r) return null;
          if (r.broken || r.rows.length !== r.n || Number(parseFields(line.slice(7)).n) !== r.n || r.n < 10) return { kind: 'broken' };
          return { kind: 'record', record: finishRecord(r) };
        }
        if (line === 'LOG abort') { cur = null; return { kind: 'abort' }; }
        if (line === 'LOG none') return { kind: 'none' };
        return null;
      },
      reset() { cur = null; },
      active() { return cur !== null; }, // 一段日志正传到一半
    };
  }

  /* 航向展开成连续的（跨 ±180° 不跳），目标航向跟着展开，误差不变。 */
  function finishRecord(r) {
    const n = r.rows.length;
    const rec = {
      n, mode: r.mode, end: r.end, trunc: r.trunc, imu: r.imu, params: r.params,
      motion: new Uint8Array(n), moving: new Uint8Array(n), target: new Float64Array(n), yaw: new Float64Array(n),
      pid: new Float64Array(n), cmd: new Float64Array(n), vx: new Float64Array(n), vy: new Float64Array(n),
    };
    let yaw = 0, raw = 0;
    r.rows.forEach((v, k) => {
      const y = v[4] / 1000;
      yaw = k ? yaw + wrap(y - raw) : y;
      raw = y;
      rec.motion[k] = v[1];
      rec.moving[k] = v[2];
      rec.yaw[k] = yaw;
      rec.target[k] = yaw + wrap(v[3] / 1000 - y);
      rec.pid[k] = v[5] / 100;
      rec.cmd[k] = v[6] / 100;
      rec.vx[k] = v[7];
      rec.vy[k] = v[8];
    });
    return rec;
  }

  /* 固件参数名 → 仿真用的航向环参数。 */
  function gainsFromParams(p) {
    return {
      kp: p.heading_kp, ki: p.heading_ki, iMax: p.heading_i_max, iZone: p.heading_i_zone,
      tmax: p.turn_max_speed, acc: p.acc_angular, tol: p.heading_tol, accLin: p.acc_linear,
      chassisK: p.chassis_k, wheelDia: p.wheel_dia,
    };
  }

  /* ---------- 2. 实测指标 ---------- */
  /* 一段日志里可能有几个动作（间隔不到 0.4 s 的记在一起）：按运动模式切开。 */
  function segments(rec) {
    const out = [];
    for (let k = 0; k < rec.n;) {
      if (!rec.motion[k]) { ++k; continue; }
      const start = k;
      while (k < rec.n && rec.motion[k]) ++k;
      let next = k;
      while (next < rec.n && !rec.motion[next]) ++next;
      let moving = false;
      for (let j = start; j < k; ++j) if (rec.moving[j]) moving = true;
      out.push({ start, end: k, until: next, kind: moving ? 'move' : 'turn', step: rec.target[start] - rec.yaw[start] });
    }
    return out;
  }

  /* 到位时间：从动作开始到航向进容差、之后一直没出去（看到下一个动作或日志结束）。
     尾巴只有 0.4 s，最后还在容差外的记 Infinity。 */
  function measure(rec, seg, tol) {
    let lastOut = -1, overshoot = 0, peakMove = 0, maxV = 0;
    const dir = Math.sign(seg.step) || 1;
    for (let k = seg.start; k < seg.until; ++k) {
      const err = rec.target[k] - rec.yaw[k];
      if (Math.abs(err) > tol + BAND_EXTRA) lastOut = k;
      overshoot = Math.max(overshoot, -err * dir);
      if (rec.moving[k]) peakMove = Math.max(peakMove, Math.abs(err));
      maxV = Math.max(maxV, Math.abs(rec.vx[k]), Math.abs(rec.vy[k]));
    }
    const settled = lastOut < seg.until - 1;
    return {
      kind: seg.kind, step: seg.step, done: (seg.end - seg.start) * DT,
      settle: settled ? (lastOut + 1 - seg.start) * DT : Infinity,
      overshoot: seg.kind === 'turn' ? overshoot : 0, peakMove, speed: maxV,
    };
  }

  function recordMetrics(rec) {
    const tol = rec.params.heading_tol ?? 0.1;
    return segments(rec).map(seg => measure(rec, seg, tol));
  }

  /* ---------- 3. 仿真（固件航向环 + 车的模型） ---------- */
  /* 车体速度 → 四轮 0.1 RPM（整体等比限幅、取整）→ 再换回车体速度，和 Chassis_Solve 一样。 */
  function makeSolver(g) {
    const perMm = 600 / (Math.PI * g.wheelDia);
    const raw = new Float64Array(4);
    const out = { vx: 0, vy: 0, wz: 0 };
    return (vx, vy, wz) => {
      const wmm = g.chassisK * wz * Math.PI / 180;
      let peak = 0;
      for (let i = 0; i < 4; ++i) {
        raw[i] = (vx + SIGN_VY[i] * vy + SIGN_WZ[i] * wmm) * perMm;
        peak = Math.max(peak, Math.abs(raw[i]));
      }
      const scale = peak > MAX_WHEEL_X10 ? MAX_WHEEL_X10 / peak : 1;
      let sx = 0, sy = 0, sw = 0;
      for (let i = 0; i < 4; ++i) {
        const q = roundAway(raw[i] * scale) / perMm;
        sx += q; sy += SIGN_VY[i] * q; sw += SIGN_WZ[i] * q;
      }
      out.vx = sx / 4; out.vy = sy / 4; out.wz = sw / 4 / g.chassisK * 180 / Math.PI;
      return out;
    };
  }

  function rng32(seed) {
    let s = seed >>> 0;
    return () => {
      s = (s + 0x6D2B79F5) >>> 0;
      let t = s;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gaussFrom(rand) {
    return () => {
      let s = 0;
      for (let i = 0; i < 12; ++i) s += rand();
      return s - 6;
    };
  }

  /* 一拍一拍按固件的顺序算：读航向 → 航向环 → 斜坡 → 轮速量化 → 车动 5 ms。
     sc: {n, target(数或数组), moving, vx, vy（数组，可省）, phase（数组，可省）, yaw0}
       phase 给了就照它（回放实测日志）：0 动作中，1 结束那一拍，2 结束后保持航向，3 停了不管；
       每段动作第一拍清积分（start 数组标出来）。
       不给 phase：动作中不平移且航向进容差就算到（和路径一样），之后保持航向。
     返回指标：settle 到位时间（进容差后再没出去，至少稳 HOLD_S）、overshoot、peakMove。 */
  function simulate(g, plant, sc, opt = {}) {
    const n = sc.n;
    const solve = makeSolver(g);
    const alpha = 1 - Math.exp(-DT / Math.max(plant.tau, 1e-4));
    const dTicks = Math.max(0, plant.delay) / DT, m = Math.floor(dTicks), frac = dTicks - m;
    const size = m + 2;
    const qw = new Float64Array(size), qx = new Float64Array(size), qy = new Float64Array(size);
    const gauss = plant.noise > 0 ? gaussFrom(rng32(opt.seed || 1)) : null;
    const trace = opt.trace ? { yaw: new Float64Array(n), pid: new Float64Array(n), cmd: new Float64Array(n) } : null;
    const constTarget = typeof sc.target === 'number';
    const step = (constTarget ? sc.target : sc.target[0]) - (sc.yaw0 || 0);
    const dir = Math.sign(step) || 1;
    let yaw = sc.yaw0 || 0, w = 0, px = 0, py = 0, cur = 0, integ = 0, phase = 0;
    let lastOut = -1, overshoot = 0, peakMove = 0, moveEnd = 0, k = 0;
    if (sc.moving) for (let j = 0; j < n; ++j) if (sc.moving[j]) moveEnd = j + 1;
    for (; k < n; ++k) {
      const meas = gauss ? Math.round((yaw + plant.noise * gauss()) / IMU_STEP) * IMU_STEP : Math.round(yaw / IMU_STEP) * IMU_STEP;
      const target = constTarget ? sc.target : sc.target[k];
      const err = wrap(target - meas);
      const moving = sc.moving ? sc.moving[k] : 0;
      const ph = sc.phase ? sc.phase[k] : phase;
      if (sc.start && sc.start[k]) integ = 0;
      let out = 0;
      if (ph === 1 || ph === 3) {
        cur = 0;
        integ = 0;
      } else {
        if (!moving && Math.abs(err) <= g.tol) integ = 0;
        else {
          integ = Math.abs(err) < g.iZone && g.ki > 0 ? clamp(integ + g.ki * err * DT, g.iMax) : 0;
          out = clamp(g.kp * err + integ, g.tmax);
        }
        if (!sc.phase && phase === 0 && !moving && k >= moveEnd && Math.abs(err) <= g.tol) {
          phase = 2; // 路径进容差就算到：Chassis_Finish 清掉斜坡和积分
          out = 0;
          cur = 0;
          integ = 0;
        }
        const s = g.acc * DT;
        cur = out - cur > s ? cur + s : out - cur < -s ? cur - s : out;
      }
      const vx = sc.vx ? sc.vx[k] : 0, vy = sc.vy ? sc.vy[k] : 0;
      const q = solve(vx, vy, cur);
      const head = k % size;
      qw[head] = q.wz; qx[head] = q.vx; qy[head] = q.vy;
      const a0 = (k - m + size * 4) % size, a1 = (k - m - 1 + size * 4) % size;
      const old = k - m - 1 >= 0;
      const uw = k - m >= 0 ? (1 - frac) * qw[a0] + (old ? frac * qw[a1] : 0) : 0;
      const ux = k - m >= 0 ? (1 - frac) * qx[a0] + (old ? frac * qx[a1] : 0) : 0;
      const uy = k - m >= 0 ? (1 - frac) * qy[a0] + (old ? frac * qy[a1] : 0) : 0;
      w += alpha * (uw - w);
      px += alpha * (ux - px);
      py += alpha * (uy - py);
      if (trace) { trace.yaw[k] = meas; trace.pid[k] = out; trace.cmd[k] = cur; }
      if (Math.abs(err) > g.tol + BAND_EXTRA) lastOut = k;
      overshoot = Math.max(overshoot, -err * dir);
      if (moving) peakMove = Math.max(peakMove, Math.abs(err));
      yaw += DT * (plant.K * w + plant.cx * px + plant.cy * py);
      if (opt.stopWhenSettled && phase === 2 && (k - lastOut) * DT >= HOLD_S) { ++k; break; }
    }
    const settled = (k - 1 - lastOut) * DT >= (opt.stopWhenSettled ? HOLD_S : 0) && lastOut < k - 1;
    return { settle: settled ? (lastOut + 1) * DT : Infinity, overshoot, peakMove, trace };
  }

  /* 照一段实测日志原样回放：同样的目标、平移、动作起止，用模型算航向，和实测比。 */
  function replay(rec, plant) {
    const g = gainsFromParams(rec.params);
    const phase = new Uint8Array(rec.n), start = new Uint8Array(rec.n);
    const segs = segments(rec);
    for (let k = 0; k < rec.n; ++k) {
      if (rec.motion[k]) {
        phase[k] = 0;
        start[k] = k === 0 || !rec.motion[k - 1] ? 1 : 0;
      } else if (k > 0 && rec.motion[k - 1]) phase[k] = 1;
      else {
        const later = segs.some(s => s.start > k);
        phase[k] = later || rec.end === 'done' ? 2 : 3;
      }
    }
    const sim = simulate(g, plant, {
      n: rec.n, target: rec.target, moving: rec.moving, vx: rec.vx, vy: rec.vy, phase, start, yaw0: rec.yaw[0],
    }, { trace: true });
    let sum = 0;
    for (let k = 0; k < rec.n; ++k) sum += (sim.trace.yaw[k] - rec.yaw[k]) ** 2;
    return { trace: sim.trace, rmsYaw: Math.sqrt(sum / rec.n) };
  }

  /* ---------- 4. 辨识 ---------- */
  /* 每段日志先把下发量化成轮子真实收到的（和仿真同一个 Chassis_Solve），和延迟、惯性无关，只算一次。 */
  function prepare(rec) {
    const solve = makeSolver(gainsFromParams(rec.params));
    const n = rec.n, w = new Float64Array(n), x = new Float64Array(n), y = new Float64Array(n), z = new Float64Array(n);
    const y0 = (rec.yaw[0] + rec.yaw[1] + rec.yaw[2]) / 3; // 起步前几拍车还没动，取平均压一点噪声
    for (let k = 0; k < n; ++k) {
      const q = solve(rec.vx[k], rec.vy[k], rec.cmd[k]);
      w[k] = q.wz; x[k] = q.vx; y[k] = q.vy; z[k] = rec.yaw[k] - y0;
    }
    let turn = 0;
    for (let k = 0; k < n; ++k) turn = Math.max(turn, Math.abs(w[k]));
    return { n, w, x, y, z, turn };
  }

  /* 给定延迟和惯性，K、cx、cy 是线性的：最小二乘一步解出来。useX/useY 为 false 时那一列不拟合。 */
  function regress(data, delay, tau, use) {
    const alpha = 1 - Math.exp(-DT / Math.max(tau, 1e-4));
    const dTicks = delay / DT, m = Math.floor(dTicks), f = dTicks - m;
    const cols = [true, use.x, use.y];
    const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
    let zz = 0, count = 0;
    for (const d of data) {
      let w = 0, px = 0, py = 0, s0 = 0, s1 = 0, s2 = 0;
      for (let k = 0; k < d.n - 1; ++k) {
        const i0 = k - m, i1 = k - m - 1;
        const uw = (i0 >= 0 ? (1 - f) * d.w[i0] : 0) + (i1 >= 0 ? f * d.w[i1] : 0);
        const ux = (i0 >= 0 ? (1 - f) * d.x[i0] : 0) + (i1 >= 0 ? f * d.x[i1] : 0);
        const uy = (i0 >= 0 ? (1 - f) * d.y[i0] : 0) + (i1 >= 0 ? f * d.y[i1] : 0);
        w += alpha * (uw - w); px += alpha * (ux - px); py += alpha * (uy - py);
        s0 += DT * w; s1 += DT * px; s2 += DT * py;
        const r = [s0, s1, s2], z = d.z[k + 1];
        for (let i = 0; i < 3; ++i) {
          b[i] += r[i] * z;
          for (let j = 0; j < 3; ++j) A[i][j] += r[i] * r[j];
        }
        zz += z * z;
        ++count;
      }
    }
    const idx = [0, 1, 2].filter(i => cols[i]);
    const M = idx.map(i => idx.map(j => A[i][j] + (i === j ? 1e-9 * (A[i][i] + 1) : 0)));
    const v = idx.map(i => b[i]);
    const sol = solveLinear(M, v);
    if (!sol) return null;
    const theta = [0, 0, 0];
    idx.forEach((i, j) => { theta[i] = sol[j]; });
    let sse = zz;
    for (let i = 0; i < 3; ++i) {
      sse -= 2 * theta[i] * b[i];
      for (let j = 0; j < 3; ++j) sse += theta[i] * theta[j] * A[i][j];
    }
    return { K: theta[0], cx: theta[1], cy: theta[2], sse: Math.max(sse, 0), count };
  }

  function solveLinear(M, v) {
    const n = v.length, a = M.map((row, i) => [...row, v[i]]);
    for (let c = 0; c < n; ++c) {
      let p = c;
      for (let r = c + 1; r < n; ++r) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
      if (Math.abs(a[p][c]) < 1e-12) return null;
      [a[c], a[p]] = [a[p], a[c]];
      for (let r = 0; r < n; ++r) {
        if (r === c) continue;
        const k = a[r][c] / a[c][c];
        for (let j = c; j <= n; ++j) a[r][j] -= k * a[c][j];
      }
    }
    return a.map((row, i) => row[n] / row[i]);
  }

  /* 一维找最小：先按格子扫一遍框住，再黄金分割细找。f 返回 {sse, ...} 或 null。 */
  function minimize1d(f, grid) {
    let best = null, at = 0;
    grid.forEach((x, i) => {
      const r = f(x);
      if (r && (!best || r.sse < best.sse)) { best = r; at = i; }
    });
    if (!best) return null;
    let lo = grid[Math.max(0, at - 1)], hi = grid[Math.min(grid.length - 1, at + 1)];
    const phi = (Math.sqrt(5) - 1) / 2;
    for (let i = 0; i < 24; ++i) {
      const a = hi - phi * (hi - lo), b = lo + phi * (hi - lo);
      const ra = f(a), rb = f(b);
      if (ra && (!rb || ra.sse <= rb.sse)) { hi = b; if (ra.sse < best.sse) best = ra; }
      else { lo = a; if (rb && rb.sse < best.sse) best = rb; }
    }
    return best;
  }

  const DELAY_GRID = Array.from({ length: 17 }, (_, i) => i * 0.005);                 // 0..80 ms
  const LOG_TAU_GRID = Array.from({ length: 16 }, (_, i) => Math.log(0.002) + i * Math.log(200) / 15); // 2..400 ms

  /* 延迟加惯性的总量决定转完以后差多少，误差曲面是沿“延迟 + 惯性 = 常数”的一道窄沟：
     每个延迟下把惯性一维找准，再把延迟一维找准。 */
  function fitGrid(data, use) {
    const atDelay = delay => minimize1d(lt => {
      const r = regress(data, delay, Math.exp(lt), use);
      return r && { ...r, delay, tau: Math.exp(lt) };
    }, LOG_TAU_GRID);
    return minimize1d(atDelay, DELAY_GRID);
  }

  /* 拟合车的模型。返回 {plant: {K, tau, delay, cx, cy, noise}, spread, rms, warnings, perRecord}。 */
  function identify(records) {
    const warnings = [];
    const data = records.map(prepare);
    const turning = data.filter(d => d.turn > 3);
    if (!turning.length) return { ok: false, warnings: ['日志里没有转向，测不出模型。先跑一次自动测试。'] };
    // 前后和左右分开：只有前后平移的数据时，左右那列拟合不出来
    const has = axis => data.some(d => d[axis].some(v => Math.abs(v) > 50));
    const use = { x: has('x'), y: has('y') };
    const best = fitGrid(data, use);
    if (!best) return { ok: false, warnings: ['拟合失败：数据不够或者全是零。'] };
    const rms = Math.sqrt(best.sse / best.count);
    const plant = { K: best.K, tau: best.tau, delay: best.delay, cx: best.cx, cy: best.cy, noise: Math.min(Math.max(rms, 0.002), 0.05) };
    if (records.some(r => !r.imu)) warnings.push('固件没接陀螺仪（CHASSIS_USE_IMU=0），航向是轮子算的，看不到打滑，K 会接近 1。');
    if (plant.K < 0) return { ok: false, plant, warnings: ['车转的方向和下发的相反：先查 CHASSIS_YAW_SIGN 和轮子方向，这不是增益问题。'] };
    if (plant.K < 0.3) return { ok: false, plant, warnings: ['下发了转速，车几乎没转：轮子离地了，或者陀螺仪没装在车上。车放地上再测。'] };
    // 只转不平移、转得够多的日志各自拟合一次，看模型在不同日志之间差多少，蒙特卡洛按这个范围抽。
    // 平移日志里只有航向环的小修正，单独拟合出来的东西没有意义。
    const perRecord = data.filter(d => d.turn > 5 && !d.x.some(v => Math.abs(v) > 50) && !d.y.some(v => Math.abs(v) > 50))
      .map(d => fitGrid([d], { x: false, y: false }))
      .filter(r => r && r.K > 0.3)
      .map(r => ({ K: r.K, tau: r.tau, delay: r.delay }));
    const spread = spreadOf(perRecord, plant);
    let scale = 0;
    for (const d of data) for (let k = 0; k < d.n; ++k) scale = Math.max(scale, Math.abs(d.z[k]));
    if (rms > Math.max(0.3, 0.03 * scale)) warnings.push(`模型和实测对不上（拟合误差 ${rms.toFixed(2)}°）：可能打滑严重、碰到东西或者数据断了，推荐值只能参考。`);
    if (perRecord.length < 3) warnings.push('转向的日志少于 3 段，模型范围是估的。');
    return { ok: true, plant, spread, rms, perRecord, warnings, records: records.length };
  }

  /* 各段之间的离散程度（中位数绝对偏差折成标准差，个别怪段不会把范围撑大），有上下限。 */
  function spreadOf(list, plant) {
    const sd = (f, center) => {
      if (list.length < 2) return 0;
      const dev = list.map(r => Math.abs(f(r) - center)).sort((a, b) => a - b);
      return 1.4826 * dev[Math.floor(dev.length / 2)];
    };
    const bound = (v, lo, hi) => Math.min(Math.max(v, lo), hi);
    return {
      K: bound(sd(r => r.K, plant.K), 0.03 * plant.K, 0.15 * plant.K),
      tauRel: bound(sd(r => Math.log(r.tau), Math.log(plant.tau)), 0.25, 0.6),
      delay: bound(sd(r => r.delay, plant.delay), 0.005, 0.015),
    };
  }

  /* ---------- 5. 搜参数 ---------- */
  /* 平移：按加速度起步、匀速、减速，和固件下发的斜坡后车速形状差不多。 */
  function moveProfile(axis, speed, accel, distance) {
    const ta = speed / accel, da = speed * ta;
    const cruise = Math.max(0, distance - da) / speed;
    const n = Math.ceil((2 * ta + cruise) / DT) + Math.ceil(WINDOW_S / 2 / DT);
    const v = new Float64Array(n), moving = new Uint8Array(n);
    const total = 2 * ta + cruise;
    for (let k = 0; k < n; ++k) {
      const t = k * DT;
      if (t >= total) break;
      v[k] = speed * Math.min(1, t / ta, (total - t) / ta);
      moving[k] = 1;
    }
    return { n, target: 0, moving, vx: axis === 'x' ? v : null, vy: axis === 'y' ? v : null };
  }

  function scenarios(opts) {
    const list = [90, 20, 5].map(a => ({ label: `转 ${a}°`, kind: 'turn', step: a, sc: { n: Math.round(WINDOW_S / DT), target: a } }));
    for (const axis of ['x', 'y']) {
      if (!opts.moves || !opts.moves[axis]) continue;
      list.push({ label: axis === 'x' ? '前后平移' : '左右平移', kind: 'move', sc: moveProfile(axis, opts.speed, opts.accLin, 300) });
    }
    return list;
  }

  /* 一组参数在一辆车上的分数（秒）：到位时间 + 超调和平移误差折成时间；不到位重罚。 */
  function costOn(g, plant, list, seed) {
    let cost = 0;
    const rows = [];
    for (const s of list) {
      const r = simulate(g, plant, s.sc, { seed, stopWhenSettled: true });
      const settle = Number.isFinite(r.settle) ? r.settle : WINDOW_S + 10;
      const overshoot = s.kind === 'turn' ? r.overshoot : 0;
      cost += settle + Math.max(0, overshoot - 0.3) + (s.kind === 'move' ? 2 * r.peakMove : 0);
      rows.push({ label: s.label, kind: s.kind, settle: r.settle, overshoot, peakMove: r.peakMove });
    }
    return { cost, rows };
  }

  function plantSamples(model, count, seed) {
    const rand = rng32(seed), gauss = gaussFrom(rand);
    const p = model.plant, s = model.spread;
    const list = [p];
    for (let i = 0; i < count; ++i)
      list.push({
        K: Math.max(0.2, p.K + s.K * gauss()), tau: p.tau * Math.exp(s.tauRel * gauss()),
        delay: Math.max(0, p.delay + s.delay * (2 * rand() - 1) * 1.7), cx: p.cx * (1 + 0.3 * gauss()),
        cy: p.cy * (1 + 0.3 * gauss()), noise: p.noise,
      });
    return list;
  }

  function evaluate(g, plants, list) {
    let sum = 0, worst = 0, nominal = null, worstRows = null;
    plants.forEach((p, i) => {
      const r = costOn(g, p, list, 1000 + i);
      sum += r.cost;
      if (i === 0) nominal = r;
      if (r.cost >= worst) { worst = r.cost; worstRows = r.rows; }
    });
    const mean = sum / plants.length;
    return { score: mean + 0.5 * (worst - mean), mean, worst, nominal: nominal.rows, worstRows };
  }

  const KPS = [0.5, 0.7, 1, 1.4, 2, 2.8, 4, 5.6, 8, 11, 16, 22, 30];
  const KIS = [0, 0.25, 0.5, 1, 2, 4, 8, 16];
  const round2 = v => Math.round(v * 100) / 100;

  /* 打分用的动作和一批车：模型认出了平移扰动才考平移；opts.samples 是随机抽几辆（0 = 只用拟合值）。 */
  function setup(model, base, opts) {
    const plant = model.plant;
    const moves = { x: Math.abs(plant.cx) > 1e-4, y: Math.abs(plant.cy) > 1e-4 };
    return {
      list: scenarios({ moves, speed: opts.speed || 400, accLin: base.accLin || 2000 }),
      plants: plantSamples(model, opts.samples ?? 11, 7),
    };
  }

  /* 一组参数在模型上的分数和各个动作的预测（手动改参数时也能看）。 */
  function scoreGains(model, gains, opts = {}) {
    const { list, plants } = setup(model, gains, opts);
    return { gains, ...evaluate(gains, plants, list), margins: margins(gains, model.plant) };
  }

  /* 搜 kp、ki；i_max 不够顶住平移扰动时一起加大。base 是现在的参数（gainsFromParams 的格式）。
     opts: {speed, samples, onProgress(0..1)}；返回 {current, best, plants, scenarios}。 */
  async function optimize(model, base, opts = {}) {
    const plant = model.plant;
    const speed = opts.speed || 400;
    const { list, plants } = setup(model, base, opts);
    const need = Math.max(Math.abs(plant.cx), Math.abs(plant.cy)) * speed / Math.max(plant.K, 0.3);
    const iMax = Math.max(base.iMax, Math.ceil(need * 1.5 * 2) / 2);
    const tryGains = (kp, ki) => ({ ...base, kp: round2(kp), ki: round2(ki), iMax });
    const cache = new Map();
    const score = g => {
      const key = `${g.kp}/${g.ki}`;
      if (!cache.has(key)) cache.set(key, evaluate(g, plants, list));
      return cache.get(key);
    };
    let best = null, done = 0;
    const total = KPS.length * KIS.length;
    for (const kp of KPS) {
      for (const ki of KIS) {
        const g = tryGains(kp, ki), r = score(g);
        if (!best || r.score < best.r.score) best = { g, r };
        ++done;
      }
      if (opts.onProgress) opts.onProgress(0.8 * done / total);
      await new Promise(resolve => setTimeout(resolve, 0)); // 让页面别卡住
    }
    // 在最好的那格附近细搜：每次试上下左右，变好就挪过去，步长逐步减半
    for (let stepK = 1.3, stepI = 1.3; stepK > 1.03; stepK = Math.sqrt(stepK), stepI = Math.sqrt(stepI)) {
      let moved = true;
      while (moved) {
        moved = false;
        const g0 = best.g;
        const ki0 = g0.ki || 0.25;
        for (const [kp, ki] of [[g0.kp * stepK, g0.ki], [g0.kp / stepK, g0.ki], [g0.kp, ki0 * stepI], [g0.kp, g0.ki / stepI]]) {
          const g = tryGains(kp, ki < 0.05 ? 0 : ki), r = score(g);
          if (r.score < best.r.score - 1e-9) { best = { g, r }; moved = true; }
        }
      }
      if (opts.onProgress) opts.onProgress(0.8 + 0.2 * (1.3 - stepK) / 0.27);
      await new Promise(resolve => setTimeout(resolve, 0));
    }
    return {
      plants: plants.length, scenarios: list.map(s => s.label),
      current: { gains: base, ...evaluate(base, plants, list), margins: margins(base, plant) },
      best: { gains: best.g, ...best.r, margins: margins(best.g, plant) },
    };
  }

  /* 线性化以后的相位裕度、增益裕度（只看小误差，限幅和积分区不算在内）。 */
  function margins(g, plant) {
    const td = plant.delay + 1.5 * DT; // 量化保持半拍 + 读航向晚一拍
    const loop = w => ({
      mag: Math.hypot(g.kp, g.ki / w) * plant.K / (w * Math.hypot(1, w * plant.tau)),
      phase: -Math.atan2(g.ki / w, g.kp) - Math.PI / 2 - Math.atan(w * plant.tau) - w * td,
    });
    let lo = 1e-3, hi = 1e4;
    for (let i = 0; i < 80; ++i) {
      const mid = Math.sqrt(lo * hi);
      if (loop(mid).mag > 1) lo = mid; else hi = mid;
    }
    const wc = Math.sqrt(lo * hi), pm = 180 + loop(wc).phase * 180 / Math.PI;
    let gm = Infinity;
    for (let w = wc, prev = loop(wc).phase; w < 1e4; w *= 1.01) {
      const ph = loop(w).phase;
      if (ph <= -Math.PI && prev > -Math.PI) { gm = -20 * Math.log10(loop(w).mag); break; }
      prev = ph;
    }
    return { pm, gm };
  }

  /* ---------- 6. 验证：实测和模型预测比 ---------- */
  const median = list => [...list].sort((a, b) => a - b)[Math.floor(list.length / 2)];

  /* 每个动作用日志里的参数和命令（目标、平移）在模型上重跑，到位时间和超调跟实测对照。
     噪声换几串种子取中位数，一次实测本来也带着噪声。 */
  function verify(records, plant) {
    const rows = [];
    for (const rec of records) {
      const g = gainsFromParams(rec.params);
      for (const seg of segments(rec)) {
        const real = measure(rec, seg, g.tol);
        const n = Math.max(seg.until - seg.start, Math.round(WINDOW_S / DT));
        const sc = { n, target: rec.target[seg.start] - rec.yaw[seg.start], yaw0: 0 };
        if (seg.kind === 'move') {
          sc.moving = new Uint8Array(n); sc.vx = new Float64Array(n); sc.vy = new Float64Array(n);
          for (let k = seg.start; k < seg.until; ++k) {
            sc.moving[k - seg.start] = rec.moving[k]; sc.vx[k - seg.start] = rec.vx[k]; sc.vy[k - seg.start] = rec.vy[k];
          }
        }
        const sims = [1, 2, 3, 4, 5].map(seed => simulate(g, plant, sc, { seed, stopWhenSettled: true }));
        rows.push({
          kind: seg.kind, step: seg.step, gains: g, measured: real,
          predicted: {
            settle: median(sims.map(s => s.settle)),
            overshoot: seg.kind === 'turn' ? median(sims.map(s => s.overshoot)) : 0,
            peakMove: median(sims.map(s => s.peakMove)),
          },
        });
      }
    }
    return rows;
  }

  const api = {
    DT, createParser, gainsFromParams, segments, recordMetrics, simulate, replay, identify, optimize, scoreGains,
    margins, verify,
  };
  root.PidTune = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);

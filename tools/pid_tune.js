/* 航向环自动调参：调试台 agv_console.html 和 node 测试 pid_tune_test.js 共用。
   1. 解析固件的调参日志（格式见 firmware/User/Comm/tune_log.h）。
   2. 从日志拟合车的模型：下发角速度 → 延迟 delay → 一阶惯性 tau → × K（打滑，转得快时再多滑一点 kq） → 航向；
      平移时车身被带着转：一直转的部分 cx·vx + cy·vy，加减速时扭一下、停了就回来的部分 ax·v̇x + ay·v̇y。
      带偏量每段日志轮流去掉再拟合一遍（刀切法），看它稳不稳，稳才拿去做前馈。
   3. 按固件的航向环原样仿真：Chassis_HeadingControl 的 PI、积分区、限幅、到位死区、刹车曲线、
      平移停下清积分，平移前馈，角加速度斜坡，0.1 RPM 轮速量化，路径的“进容差就算到”。
   4. 搜 kp、ki、刹车曲线，前馈直接取 −带偏量 / K。打分用三类动作：
      - 转 90°/20°/5°（模型）；
      - 日志里的平移原样回放：实测航向里不是转向命令带来的那部分当成扰动加回去，模型解释不了的
        （地面、打滑、噪声）也在里面，不靠“扰动和车速成正比”这个假设；
      - 按指令速度走一段带匀速的长距离（模型）：测试只走几百毫米，积分攒起来、停下以后的尾巴看不到。
      模型参数在拟合误差范围里随机抽一批（蒙特卡洛），挑这批车上都表现好的。 */
(function (root) {
  'use strict';

  const DT = 0.005;                // 固件控制周期
  const IMU_STEP = 180 / 32768;    // HWT101 航向分辨率
  const SIGN_VY = [-1, 1, 1, -1];  // 逆运动学符号，和 chassis.c 的 k_sign 一致（LF RF LR RR）
  const SIGN_WZ = [-1, 1, -1, 1];
  const HOLD_S = 0.6;              // 进容差后至少稳这么久才算到位
  /* 算“到位”用的带子比航向环的死区宽 0.05°：航向环进了死区就不再修，车可能正好停在死区边上，
     噪声把它来回推过边，用死区本身当标准，到位时间就成了看噪声运气。 */
  const BAND_EXTRA = 0.05;
  const WINDOW_S = 6;              // 仿真最长时间
  const MOVE_MIN = 50;             // mm/s，车速超过这么多才算平移过
  const MOVE_FREE = 0.1;           // 平移时航向最大差在这以内就不扣分（固件的验收线是 0.2°，留一半余量）
  const MOVE_WEIGHT = 3;           // 超出的部分每度算这么多秒；均方根每度另算 1 s，分不出高下时看它
  const PM_MIN = 45, GM_MIN = 8;   // 推荐值至少留这么多相位裕度（°）、增益裕度（dB）
  const KP_MAX = 20, KI_MAX = 20;  // 固件 /set 接受的上限（console.c）
  const NCOL = 6;                  // 辨识的线性参数个数，见 normal()
  const REPLAYS_PER_AXIS = 6;      // 每个方向最多回放最近这么多段平移

  const wrap = a => { a %= 360; return a > 180 ? a - 360 : a < -180 ? a + 360 : a; }; // Math_WrapDeg
  const clamp = (v, lim) => (v > lim ? lim : v < -lim ? -lim : v);
  const bound = (v, lo, hi) => Math.min(Math.max(v, lo), hi);
  const roundAway = x => (x < 0 ? -Math.round(-x) : Math.round(x)); // lroundf
  const round2 = v => Math.round(v * 100) / 100;

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

  /* 固件参数名 → 仿真用的航向环参数。ext：固件有刹车曲线、平移前馈、平移停下清积分（日志的参数行里有
     heading_brake）；旧固件的日志没有，按没有这些功能回放。ffx、ffy 是每 1 m/s 平移补多少 deg/s。 */
  function gainsFromParams(p) {
    return {
      kp: p.heading_kp, ki: p.heading_ki, iMax: p.heading_i_max, iZone: p.heading_i_zone,
      tmax: p.turn_max_speed, acc: p.acc_angular, tol: p.heading_tol, accLin: p.acc_linear,
      chassisK: p.chassis_k, wheelDia: p.wheel_dia,
      ext: p.heading_brake !== undefined, brake: p.heading_brake || 0, ffx: p.heading_ff_x || 0, ffy: p.heading_ff_y || 0,
      wheelMax: p.max_wheel_rpm || 400,
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
     尾巴只有 0.4 s，最后还在容差外的记 Infinity。平移看平移那几拍的航向误差：最大、均方根。
     overrun：平移冲过头又退回来多少 mm。日志里没有位置，按下发的车速沿走的方向积起来算：最远到过哪里
     减最后停在哪里。固件发现过了头才会往回发车速，所以这个数和真冲过去的差不多。 */
  function measure(rec, seg, tol) {
    let lastOut = -1, overshoot = 0, peakMove = 0, sumMove = 0, nMove = 0, maxV = 0, sx = 0, sy = 0;
    const dir = Math.sign(seg.step) || 1;
    for (let k = seg.start; k < seg.until; ++k) {
      const err = rec.target[k] - rec.yaw[k];
      if (Math.abs(err) > tol + BAND_EXTRA) lastOut = k;
      overshoot = Math.max(overshoot, -err * dir);
      if (rec.moving[k]) { peakMove = Math.max(peakMove, Math.abs(err)); sumMove += err * err; ++nMove; }
      maxV = Math.max(maxV, Math.abs(rec.vx[k]), Math.abs(rec.vy[k]));
      sx += rec.vx[k];
      sy += rec.vy[k];
    }
    const length = Math.hypot(sx, sy) || 1;
    let at = 0, far = 0;
    for (let k = seg.start; k < seg.until; ++k) {
      at += (rec.vx[k] * sx + rec.vy[k] * sy) / length * DT;
      far = Math.max(far, at);
    }
    const settled = lastOut < seg.until - 1;
    return {
      kind: seg.kind, step: seg.step, done: (seg.end - seg.start) * DT,
      settle: settled ? (lastOut + 1 - seg.start) * DT : Infinity,
      overshoot: seg.kind === 'turn' ? overshoot : 0, peakMove, rmsMove: nMove ? Math.sqrt(sumMove / nMove) : 0, speed: maxV,
      overrun: seg.kind === 'move' ? far - at : 0,
    };
  }

  function recordMetrics(rec) {
    const tol = rec.params.heading_tol ?? 0.1;
    return segments(rec).map(seg => measure(rec, seg, tol));
  }

  /* ---------- 3. 仿真（固件航向环 + 车的模型） ---------- */
  /* 车体速度 → 四轮 0.1 RPM（整体等比限幅、取整）→ 再换回车体速度，和 Chassis_Solve 一样。 */
  function makeSolver(g) {
    const perMm = 600 / (Math.PI * g.wheelDia), limit = (g.wheelMax || 400) * 10;
    const raw = new Float64Array(4);
    const out = { vx: 0, vy: 0, wz: 0 };
    return (vx, vy, wz) => {
      const wmm = g.chassisK * wz * Math.PI / 180;
      let peak = 0;
      for (let i = 0; i < 4; ++i) {
        raw[i] = (vx + SIGN_VY[i] * vy + SIGN_WZ[i] * wmm) * perMm;
        peak = Math.max(peak, Math.abs(raw[i]));
      }
      const scale = peak > limit ? limit / peak : 1;
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

  /* 一拍一拍按固件的顺序算：读航向 → 航向环 → 斜坡 → 前馈 → 轮速量化 → 车动 5 ms。
     sc: {n, target(数或数组), moving, vx, vy（数组，可省）, phase（数组，可省）, yaw0, dist（数组，可省）}
       phase 给了就照它（回放实测日志）：0 动作中，1 结束那一拍，2 结束后保持航向，3 停了不管；
       每段动作第一拍清积分（start 数组标出来）。
       不给 phase：动作中不平移且航向进容差就算到（和路径一样），之后保持航向。
       dist 给了就是回放实测扰动：航向 = 模型算的转向响应 + dist[k]，不另加噪声，模型里的平移带偏不用
       （都在 dist 里）。
     返回指标：settle 到位时间（进容差后再没出去，至少稳 HOLD_S）、overshoot、平移那几拍的航向误差
     peakMove（最大）、rmsMove（均方根）。 */
  function simulate(g, plant, sc, opt = {}) {
    const n = sc.n;
    const solve = makeSolver(g);
    const alpha = 1 - Math.exp(-DT / Math.max(plant.tau, 1e-4));
    const dTicks = Math.max(0, plant.delay) / DT, m = Math.floor(dTicks), frac = dTicks - m;
    const size = m + 2;
    const qw = new Float64Array(size), qx = new Float64Array(size), qy = new Float64Array(size);
    const dist = sc.dist || null;
    const gauss = !dist && plant.noise > 0 ? gaussFrom(rng32(opt.seed || 1)) : null;
    const cx = dist ? 0 : plant.cx, cy = dist ? 0 : plant.cy, ax = dist ? 0 : plant.ax || 0, ay = dist ? 0 : plant.ay || 0;
    const kq = (plant.kq || 0) / 100;
    const trace = opt.trace ? { yaw: new Float64Array(n), pid: new Float64Array(n), cmd: new Float64Array(n) } : null;
    const constTarget = typeof sc.target === 'number';
    const step = (constTarget ? sc.target : sc.target[0]) - (sc.yaw0 || 0);
    const dir = Math.sign(step) || 1;
    let yaw = sc.yaw0 || 0, w = 0, px = 0, py = 0, cur = 0, integ = 0, phase = 0, still = true;
    let lastOut = -1, overshoot = 0, peakMove = 0, sumMove = 0, nMove = 0, moveEnd = 0, k = 0;
    if (sc.moving) for (let j = 0; j < n; ++j) if (sc.moving[j]) moveEnd = j + 1;
    for (; k < n; ++k) {
      const meas = dist ? yaw + dist[k] : Math.round((gauss ? yaw + plant.noise * gauss() : yaw) / IMU_STEP) * IMU_STEP;
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
        if (g.ext && !moving && !still) integ = 0; // 平移刚停下：清积分
        still = !moving;
        if (!moving && Math.abs(err) <= g.tol) integ = 0;
        else {
          integ = Math.abs(err) < g.iZone && g.ki > 0 ? clamp(integ + g.ki * err * DT, g.iMax) : 0;
          let p = g.kp * err;
          if (g.brake > 0) p = clamp(p, Math.sqrt(2 * g.brake * Math.abs(err))); // 刹车曲线只管比例项
          out = clamp(p + integ, g.tmax);
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
      const q = solve(vx, vy, cur + (g.ffx * vx + g.ffy * vy) / 1000);
      const head = k % size;
      qw[head] = q.wz; qx[head] = q.vx; qy[head] = q.vy;
      const a0 = (k - m + size * 4) % size, a1 = (k - m - 1 + size * 4) % size;
      const old = k - m - 1 >= 0;
      const uw = k - m >= 0 ? (1 - frac) * qw[a0] + (old ? frac * qw[a1] : 0) : 0;
      const ux = k - m >= 0 ? (1 - frac) * qx[a0] + (old ? frac * qx[a1] : 0) : 0;
      const uy = k - m >= 0 ? (1 - frac) * qy[a0] + (old ? frac * qy[a1] : 0) : 0;
      const px0 = px, py0 = py;
      w += alpha * (uw - w);
      px += alpha * (ux - px);
      py += alpha * (uy - py);
      if (trace) { trace.yaw[k] = meas; trace.pid[k] = out; trace.cmd[k] = cur; }
      if (Math.abs(err) > g.tol + BAND_EXTRA) lastOut = k;
      overshoot = Math.max(overshoot, -err * dir);
      if (moving) { peakMove = Math.max(peakMove, Math.abs(err)); sumMove += err * err; ++nMove; }
      yaw += DT * (plant.K * w + kq * w * Math.abs(w) + cx * px + cy * py) + ax * (px - px0) + ay * (py - py0);
      if (opt.stopWhenSettled && phase === 2 && (k - lastOut) * DT >= HOLD_S) { ++k; break; }
    }
    const settled = (k - 1 - lastOut) * DT >= (opt.stopWhenSettled ? HOLD_S : 0) && lastOut < k - 1;
    return { settle: settled ? (lastOut + 1) * DT : Infinity, overshoot, peakMove, rmsMove: nMove ? Math.sqrt(sumMove / nMove) : 0, trace };
  }

  /* 一段实测日志 → 仿真要的命令：同样的目标、平移、动作起止（航向从 0 算起）。 */
  function recordScenario(rec) {
    const n = rec.n, phase = new Uint8Array(n), start = new Uint8Array(n), target = new Float64Array(n);
    let lastMotion = -1;
    for (let k = 0; k < n; ++k) if (rec.motion[k]) lastMotion = k;
    for (let k = 0; k < n; ++k) {
      target[k] = rec.target[k] - rec.yaw[0];
      if (rec.motion[k]) start[k] = k === 0 || !rec.motion[k - 1] ? 1 : 0;
      else if (k > 0 && rec.motion[k - 1]) phase[k] = 1;
      else phase[k] = k < lastMotion || rec.end === 'done' ? 2 : 3;
    }
    return { n, target, moving: rec.moving, vx: rec.vx, vy: rec.vy, phase, start, yaw0: 0 };
  }

  /* 照一段实测日志原样回放：同样的目标、平移、动作起止，用模型算航向，和实测比。 */
  function replay(rec, plant) {
    const sim = simulate(gainsFromParams(rec.params), plant, recordScenario(rec), { trace: true });
    let sum = 0;
    for (let k = 0; k < rec.n; ++k) {
      sim.trace.yaw[k] += rec.yaw[0];
      sum += (sim.trace.yaw[k] - rec.yaw[k]) ** 2;
    }
    return { trace: sim.trace, rmsYaw: Math.sqrt(sum / rec.n) };
  }

  /* 一段日志里不是转向命令带来的那部分航向：实测 − 模型按日志里真正下发的转速算出来的转向响应。
     平移带偏、地面、打滑、噪声都在里面。换一组参数回放时把它原样加回去（simulate 的 sc.dist）：
     用日志自己的参数回放正好得到实测，换参数就是“同样的扰动下这组参数会怎样”。 */
  function disturbance(rec, plant) {
    const g = gainsFromParams(rec.params), solve = makeSolver(g), n = rec.n;
    const alpha = 1 - Math.exp(-DT / Math.max(plant.tau, 1e-4));
    const dTicks = Math.max(0, plant.delay) / DT, m = Math.floor(dTicks), frac = dTicks - m;
    const sent = new Float64Array(n), d = new Float64Array(n), kq = (plant.kq || 0) / 100;
    let yaw = 0, w = 0;
    for (let k = 0; k < n; ++k) {
      d[k] = rec.yaw[k] - rec.yaw[0] - yaw;
      sent[k] = solve(rec.vx[k], rec.vy[k], rec.cmd[k] + (g.ffx * rec.vx[k] + g.ffy * rec.vy[k]) / 1000).wz;
      const uw = (k - m >= 0 ? (1 - frac) * sent[k - m] : 0) + (k - m - 1 >= 0 ? frac * sent[k - m - 1] : 0);
      w += alpha * (uw - w);
      yaw += DT * (plant.K * w + kq * w * Math.abs(w));
    }
    return d;
  }

  /* ---------- 4. 辨识 ---------- */
  /* 每段日志先把下发量化成轮子真实收到的（和仿真同一个 Chassis_Solve，前馈也算进去），和延迟、惯性
     无关，只算一次。 */
  function prepare(rec) {
    const g = gainsFromParams(rec.params), solve = makeSolver(g);
    const n = rec.n, w = new Float64Array(n), x = new Float64Array(n), y = new Float64Array(n), z = new Float64Array(n);
    const y0 = (rec.yaw[0] + rec.yaw[1] + rec.yaw[2]) / 3; // 起步前几拍车还没动，取平均压一点噪声
    let turn = 0, mx = 0, my = 0;
    for (let k = 0; k < n; ++k) {
      const q = solve(rec.vx[k], rec.vy[k], rec.cmd[k] + (g.ffx * rec.vx[k] + g.ffy * rec.vy[k]) / 1000);
      w[k] = q.wz; x[k] = q.vx; y[k] = q.vy; z[k] = rec.yaw[k] - y0;
      turn = Math.max(turn, Math.abs(rec.cmd[k]));
      mx = Math.max(mx, Math.abs(q.vx));
      my = Math.max(my, Math.abs(q.vy));
    }
    return { n, w, x, y, z, turn, mx, my, rec };
  }

  /* 一段日志在给定延迟、惯性下的正规方程。列（都过了延迟和惯性）：∫转速、∫vx、∫vy、vx、vy、∫转速·|转速|/100，
     对应 K、cx、cy（平移时一直转）、ax、ay（加减速时扭一下，停了就回来）、kq（转得快时多打滑：
     每快 100 deg/s，K 变这么多，一般是负的）：
     航向 = K·∫w + cx·∫vx + cy·∫vy + ax·vx + ay·vy + kq·∫w|w|/100。只存上三角。 */
  function normal(d, delay, tau) {
    const alpha = 1 - Math.exp(-DT / Math.max(tau, 1e-4));
    const dTicks = delay / DT, m = Math.floor(dTicks), f = dTicks - m;
    const A = new Float64Array(NCOL * NCOL), b = new Float64Array(NCOL), r = new Float64Array(NCOL);
    let zz = 0, w = 0, px = 0, py = 0;
    for (let k = 0; k < d.n - 1; ++k) {
      const i0 = k - m, i1 = k - m - 1;
      const uw = (i0 >= 0 ? (1 - f) * d.w[i0] : 0) + (i1 >= 0 ? f * d.w[i1] : 0);
      const ux = (i0 >= 0 ? (1 - f) * d.x[i0] : 0) + (i1 >= 0 ? f * d.x[i1] : 0);
      const uy = (i0 >= 0 ? (1 - f) * d.y[i0] : 0) + (i1 >= 0 ? f * d.y[i1] : 0);
      w += alpha * (uw - w); px += alpha * (ux - px); py += alpha * (uy - py);
      r[0] += DT * w; r[1] += DT * px; r[2] += DT * py; r[3] = px; r[4] = py; r[5] += DT * w * Math.abs(w) / 100;
      const z = d.z[k + 1];
      for (let i = 0; i < NCOL; ++i) {
        b[i] += r[i] * z;
        for (let j = i; j < NCOL; ++j) A[i * NCOL + j] += r[i] * r[j];
      }
      zz += z * z;
    }
    return { A, b, zz, count: d.n - 1 };
  }

  function addNormal(sum, part, sign = 1) {
    for (let i = 0; i < NCOL * NCOL; ++i) sum.A[i] += sign * part.A[i];
    for (let i = 0; i < NCOL; ++i) sum.b[i] += sign * part.b[i];
    sum.zz += sign * part.zz;
    sum.count += sign * part.count;
    return sum;
  }
  const emptyNormal = () => ({ A: new Float64Array(NCOL * NCOL), b: new Float64Array(NCOL), zz: 0, count: 0 });
  const at = (A, i, j) => (i <= j ? A[i * NCOL + j] : A[j * NCOL + i]);

  /* 残差平方和：zz − 2θ·b + θᵀAθ。 */
  function sseOf(part, theta) {
    let sse = part.zz;
    for (let i = 0; i < NCOL; ++i) {
      sse -= 2 * theta[i] * part.b[i];
      for (let j = 0; j < NCOL; ++j) sse += theta[i] * theta[j] * at(part.A, i, j);
    }
    return Math.max(sse, 0);
  }

  /* 最小二乘解 θ = [K, cx, cy, ax, ay, kq]。cols[i] 是 true 的列拟合，是数的列固定成那个数，false 的列是 0
     （那个方向没平移过，或者转速没有快有慢、分不出 kq）。 */
  function solveNormal(part, cols) {
    const idx = [], theta = new Array(NCOL).fill(0);
    cols.forEach((c, i) => { if (c === true) idx.push(i); else if (c) theta[i] = c; });
    const M = idx.map(i => idx.map(j => at(part.A, i, j) + (i === j ? 1e-9 * (at(part.A, i, i) + 1) : 0)));
    const rhs = idx.map(i => {
      let v = part.b[i];
      for (let j = 0; j < NCOL; ++j) if (cols[j] !== true) v -= at(part.A, i, j) * theta[j];
      return v;
    });
    const sol = solveLinear(M, rhs);
    if (!sol) return null;
    idx.forEach((i, j) => { theta[i] = sol[j]; });
    return theta;
  }

  const columns = use => [true, use.x, use.y, use.x, use.y, use.q];

  /* 给定延迟和惯性，其余参数是线性的：最小二乘一步解出来。 */
  function regress(data, delay, tau, cols) {
    const sum = emptyNormal();
    for (const d of data) addNormal(sum, normal(d, delay, tau));
    const theta = solveNormal(sum, cols);
    return theta && { theta, sse: sseOf(sum, theta), count: sum.count };
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
    for (let i = 0; i < 16; ++i) { // 区间缩到原来的万分之五，再细没有意义
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
  function fitGrid(data, cols) {
    const atDelay = delay => minimize1d(lt => {
      const r = regress(data, delay, Math.exp(lt), cols);
      return r && { ...r, delay, tau: Math.exp(lt) };
    }, LOG_TAU_GRID);
    return minimize1d(atDelay, DELAY_GRID);
  }

  /* 陀螺仪读数的噪声：航向的二阶差分里小的那七成（转向加减速那几拍是大的，不算），折成标准差。 */
  function noiseOf(records) {
    let count = 0;
    for (const r of records) count += Math.max(0, r.n - 2);
    if (count < 50) return 0.004;
    const d2 = new Float64Array(count);
    let i = 0;
    for (const r of records) for (let k = 2; k < r.n; ++k) d2[i++] = Math.abs(r.yaw[k] - 2 * r.yaw[k - 1] + r.yaw[k - 2]);
    d2.sort();
    const m = Math.floor(count * 0.7);
    let sum = 0;
    for (let k = 0; k < m; ++k) sum += d2[k];
    return bound(sum / m / 0.4737 / Math.sqrt(6), 0.002, 0.02); // 0.4737：正态分布绝对值里小的七成的平均
  }

  /* 拟合车的模型。返回 {ok, plant: {K, tau, delay, cx, cy, ax, ay, noise}, spread, drift, rms, rmsTurn,
     rmsMove, warnings, perRecord, records, moves}。
     drift.x / drift.y：那个方向的平移带偏。seen 有没有数据；c 每 mm/s 带出来多少 deg/s、se 它的标准误差
     （刀切法）；explained 平移时的航向偏差有多少是带偏模型解释得了的（0~1）；ff 抵消它要的前馈
     （每 1 m/s 补多少 deg/s）；use 稳不稳、值不值得用。 */
  function identify(records) {
    const warnings = [];
    const data = records.map(prepare);
    if (!data.some(d => d.turn > 3)) return { ok: false, warnings: ['日志里没有转向，测不出模型。先跑一次自动测试。'] };
    // 前后和左右分开：只有前后平移的数据时，左右那几列拟合不出来。
    // “转得快多打滑”要有快有慢的转向才分得出来：最快的超过 60 deg/s，还有一段不到它一半的。
    const fastest = Math.max(...data.map(d => d.turn));
    const use = {
      x: data.some(d => d.mx > MOVE_MIN), y: data.some(d => d.my > MOVE_MIN),
      q: fastest > 60 && data.some(d => d.turn > 3 && d.turn < 0.5 * fastest),
    };
    const best = fitGrid(data, columns(use));
    if (!best) return { ok: false, warnings: ['拟合失败：数据不够或者全是零。'] };
    const [K, cx, cy, ax, ay, kq] = best.theta;
    const rms = Math.sqrt(best.sse / best.count);
    const plant = { K, kq, tau: best.tau, delay: best.delay, cx, cy, ax, ay, noise: noiseOf(records) };
    if (records.some(r => !r.imu)) warnings.push('固件没接陀螺仪（CHASSIS_USE_IMU=0），航向是轮子算的，看不到打滑，K 会接近 1。');
    if (K < 0) return { ok: false, plant, warnings: ['车转的方向和下发的相反：先查 CHASSIS_YAW_SIGN 和轮子方向，这不是增益问题。'] };
    if (K < 0.3) return { ok: false, plant, warnings: ['下发了转速，车几乎没转：轮子离地了，或者陀螺仪没装在车上。车放地上再测。'] };

    const parts = data.map(d => normal(d, best.delay, best.tau));
    const moved = d => d.mx > MOVE_MIN || d.my > MOVE_MIN;
    const rmsOver = keep => {
      let sse = 0, count = 0;
      data.forEach((d, i) => { if (keep(d)) { sse += sseOf(parts[i], best.theta); count += parts[i].count; } });
      return count ? Math.sqrt(sse / count) : 0;
    };
    // 刀切法：每段日志轮流去掉再解一次，看带偏量跟着变多少
    const total = emptyNormal();
    for (const p of parts) addNormal(total, p);
    const jack = parts.map(p => solveNormal(addNormal(addNormal(emptyNormal(), total), p, -1), columns(use)));
    const se = i => {
      const list = jack.filter(Boolean).map(t => t[i]);
      if (list.length < 3 || list.length < jack.length) return Infinity;
      const mean = list.reduce((a, b) => a + b, 0) / list.length;
      return Math.sqrt((list.length - 1) / list.length * list.reduce((a, b) => a + (b - mean) ** 2, 0));
    };
    const axis = (name, ci, ai, max) => {
      const c = best.theta[ci], dev = se(ci);
      if (!use[name]) return { seen: false, c: 0, a: 0, se: Infinity, explained: 0, ff: 0, use: false };
      const without = best.theta.slice();
      without[ci] = without[ai] = 0;
      let full = 0, bare = 0;
      data.forEach((d, i) => { if (d[max] > MOVE_MIN) { full += sseOf(parts[i], best.theta); bare += sseOf(parts[i], without); } });
      // 至少每 1 m/s 0.2 deg/s 才值得补；比两倍标准误差小就分不清是不是 0
      const steady = Number.isFinite(dev) && Math.abs(c) > 2 * dev && Math.abs(c) * 1000 >= 0.2;
      return { seen: true, c, a: best.theta[ai], se: dev, explained: bare > 0 ? bound(1 - full / bare, 0, 1) : 0, ff: bound(round2(-c / K * 1000), -30, 30), use: steady };
    };
    const drift = { x: axis('x', 1, 3, 'mx'), y: axis('y', 2, 4, 'my') };

    // 只转不平移、转得够多的日志各自拟合一次（kq 用总的），看模型在不同日志之间差多少，蒙特卡洛按这个
    // 范围抽。平移日志里只有航向环的小修正，单独拟合出来的东西没有意义。
    const perRecord = data.filter(d => d.turn > 5 && !moved(d))
      .map(d => fitGrid([d], [true, false, false, false, false, kq]))
      .filter(r => r && r.theta[0] > 0.3)
      .map(r => ({ K: r.theta[0], tau: r.tau, delay: r.delay }));
    const spread = spreadOf(perRecord, plant, drift);
    let scale = 0;
    for (const d of data) for (let k = 0; k < d.n; ++k) scale = Math.max(scale, Math.abs(d.z[k]));
    if (rms > Math.max(0.3, 0.03 * scale)) warnings.push(`模型和实测对不上（拟合误差 ${rms.toFixed(2)}°）：可能打滑严重、碰到东西或者数据断了，推荐值只能参考。`);
    if (perRecord.length < 3) warnings.push('转向的日志少于 3 段，模型范围是估的。');
    for (const [name, label] of [['x', '前后'], ['y', '左右']]) {
      const a = drift[name];
      if (a.seen && !a.use && Math.abs(a.c) * 1000 >= 0.2)
        warnings.push(`${label}平移的带偏量不稳（每 1 m/s ${(a.c * 1000).toFixed(1)} ± ${Number.isFinite(a.se) ? (a.se * 1000).toFixed(1) : '?'} °/s），没给前馈：多测几遍，或者把平移距离加长。`);
    }
    return {
      ok: true, plant, spread, drift, rms, rmsTurn: rmsOver(d => !moved(d)), rmsMove: rmsOver(moved), perRecord, warnings,
      records: records.length, moves: data.filter(moved).map(d => d.rec),
    };
  }

  /* 各段之间的离散程度（中位数绝对偏差折成标准差，个别怪段不会把范围撑大），有上下限：下限是换了地面、
     电池总会有的变化；上限防止模型结构对不上时（各段拟合出来的差别其实是系统性的）把范围撑得离谱，
     搜出来的参数保守得没法用。
     延迟和惯性不分开看：数据定得准的是两个加起来（总滞后），怎么分是沿着那道窄沟滑的，各段拟合出来
     一个多另一个就少。按“总滞后 lagRel（相对）”和“惯性占几成 split”抽，才不会抽出总滞后离谱的车。
     带偏量换了速度、地面会变：至少按两成算。 */
  const lagOf = r => r.tau + r.delay;
  function spreadOf(list, plant, drift) {
    const sd = (f, center) => {
      if (list.length < 2) return 0;
      const dev = list.map(r => Math.abs(f(r) - center)).sort((a, b) => a - b);
      return 1.4826 * dev[Math.floor(dev.length / 2)];
    };
    const c = (value, a) => Math.max(0.2 * Math.abs(value), a && Number.isFinite(a.se) ? a.se : 0);
    return {
      K: bound(sd(r => r.K, plant.K), 0.03 * plant.K, 0.08 * plant.K),
      lagRel: bound(sd(r => Math.log(lagOf(r)), Math.log(lagOf(plant))), 0.1, 0.3),
      split: bound(sd(r => r.tau / lagOf(r), plant.tau / lagOf(plant)), 0.1, 0.25),
      cx: c(plant.cx, drift && drift.x), cy: c(plant.cy, drift && drift.y),
    };
  }

  /* ---------- 5. 搜参数 ---------- */
  /* 路径的车速，和 Chassis_PathControl 一样：按加速度起步，匀速，按 sqrt(2·a·剩余距离) 刹车，最后 20 mm
     按 8 × 剩余距离蹭过去，进 3 mm 算到；后面再留一段看航向停没停稳。stop：平移到第几拍为止。 */
  function moveProfile(axis, speed, accel, distance) {
    const v = [], step = accel * DT;
    let pos = 0, cur = 0, stop = 0;
    for (let k = 0; k < 4000; ++k) {
      const left = distance - pos;
      let target = left > 3 ? Math.min(speed, Math.sqrt(2 * accel * left)) : 0;
      if (left < 20) target = Math.min(target, 8 * left);
      cur = target - cur > step ? cur + step : target - cur < -step ? cur - step : target;
      if (target === 0 && cur === 0) break;
      v.push(cur);
      pos += cur * DT;
      if (target > 0) stop = v.length;
    }
    const n = v.length + Math.ceil(WINDOW_S / 2 / DT);
    const speeds = new Float64Array(n), moving = new Uint8Array(n);
    speeds.set(v);
    moving.fill(1, 0, stop);
    return { n, target: 0, moving, vx: axis === 'x' ? speeds : null, vy: axis === 'y' ? speeds : null, stop };
  }

  function plantSamples(model, count, seed) {
    const rand = rng32(seed), gauss = gaussFrom(rand);
    const p = model.plant, s = model.spread;
    const list = [p];
    for (let i = 0; i < count; ++i) {
      const lag = lagOf(p) * Math.exp((s.lagRel || 0) * gauss());
      const part = bound(p.tau / lagOf(p) + (s.split || 0) * gauss(), 0.1, 0.95);
      list.push({
        ...p, K: Math.max(0.2, p.K + (s.K || 0) * gauss()), tau: lag * part, delay: lag * (1 - part),
        cx: p.cx + (s.cx || 0) * gauss(), cy: p.cy + (s.cy || 0) * gauss(),
      });
    }
    return list;
  }

  const AXES = [['x', '前后'], ['y', '左右']];

  /* 打分用的动作和一批车。opts.speed 是平移的指令速度，opts.samples 是随机抽几辆（0 = 只用拟合值）。 */
  function setup(model, base, opts = {}) {
    const plant = model.plant, speed = opts.speed || 400, accLin = base.accLin || 2000;
    const plants = plantSamples(model, opts.samples ?? 11, 7);
    const turns = [90, 20, 5].map(a => ({ label: `转 ${a}°`, sc: { n: Math.round(WINDOW_S / DT), target: a } }));
    const longs = [], replays = [];
    for (const [axis, label] of AXES) {
      const seen = model.drift ? model.drift[axis].seen : Math.abs(axis === 'x' ? plant.cx : plant.cy) > 1e-5;
      if (!seen) continue;
      const distance = Math.round((speed * speed / accLin + speed) / 100) * 100; // 匀速 1 s 左右
      longs.push({ label: `${label}走 ${(distance / 1000).toFixed(1)} m`, sc: moveProfile(axis, speed, accLin, distance) });
      // 只回放目标航向不变的平移：带转向的那种，转向的模型误差也混在扰动里，换参数就不对了
      const mine = (model.moves || []).filter(rec => {
        let sx = 0, sy = 0, turn = 0;
        for (let k = 0; k < rec.n; ++k) {
          sx += Math.abs(rec.vx[k]); sy += Math.abs(rec.vy[k]);
          turn = Math.max(turn, Math.abs(rec.target[k] - rec.target[0]));
        }
        return turn < 1 && (axis === 'x' ? sx >= sy : sy > sx);
      }).slice(-REPLAYS_PER_AXIS);
      if (mine.length)
        replays.push({
          label: `回放${label} ${mine.length} 段`,
          list: mine.map(rec => ({ sc: recordScenario(rec), dists: plants.map(p => disturbance(rec, p)) })),
        });
    }
    return { turns, longs, replays, plants };
  }

  /* 一组参数在第 i 辆车上的分数（秒）：转向的到位时间 + 超出容差的超调（每度 1 s）；平移时航向最大差超出
     MOVE_FREE 的部分（每度 MOVE_WEIGHT 秒）+ 均方根（每度 1 s），长距离的再加停下以后多久稳住。
     不到位重罚。 */
  const moveCost = (peak, rms) => MOVE_WEIGHT * Math.max(0, peak - MOVE_FREE) + rms;
  function costOn(g, ctx, i) {
    const plant = ctx.plants[i], opt = { seed: 1000 + i, stopWhenSettled: true };
    const late = WINDOW_S + 10, rows = [];
    let cost = 0;
    for (const s of ctx.turns) {
      const r = simulate(g, plant, s.sc, opt);
      cost += (Number.isFinite(r.settle) ? r.settle : late) + Math.max(0, r.overshoot - g.tol);
      rows.push({ label: s.label, kind: 'turn', settle: r.settle, overshoot: r.overshoot });
    }
    for (const s of ctx.longs) {
      const r = simulate(g, plant, s.sc, opt);
      const tail = Number.isFinite(r.settle) ? Math.max(0, r.settle - s.sc.stop * DT) : Infinity;
      cost += moveCost(r.peakMove, r.rmsMove) + (Number.isFinite(tail) ? tail : late);
      rows.push({ label: s.label, kind: 'long', peakMove: r.peakMove, rmsMove: r.rmsMove, tail });
    }
    for (const group of ctx.replays) {
      let peak = 0, rms = 0;
      for (const item of group.list) {
        item.sc.dist = item.dists[i];
        const r = simulate(g, plant, item.sc);
        peak += r.peakMove;
        rms += r.rmsMove;
      }
      peak /= group.list.length;
      rms /= group.list.length;
      cost += moveCost(peak, rms);
      rows.push({ label: group.label, kind: 'replay', peakMove: peak, rmsMove: rms });
    }
    return { cost, rows };
  }

  /* 在这批车上的分数：平均和最差的中间。nominal 是拟合出来的那辆车上各个动作的预测。 */
  function evaluate(g, ctx) {
    let sum = 0, worst = 0, nominal = null, worstRows = null;
    for (let i = 0; i < ctx.plants.length; ++i) {
      const r = costOn(g, ctx, i);
      sum += r.cost;
      if (i === 0) nominal = r;
      if (r.cost >= worst) { worst = r.cost; worstRows = r.rows; }
    }
    const mean = sum / ctx.plants.length;
    return { score: mean + 0.5 * (worst - mean), mean, worst, nominal: nominal.rows, worstRows };
  }

  /* 一组参数在模型上的分数和各个动作的预测（手动改参数时也能看）。 */
  function scoreGains(model, gains, opts = {}) {
    return { gains, ...evaluate(gains, setup(model, gains, opts)), margins: margins(gains, model.plant) };
  }

  const KPS = [0.5, 0.7, 1, 1.4, 2, 2.8, 4, 5.6, 8, 11, 16, 20];
  const KIS = [0, 0.25, 0.5, 1, 2, 4, 8, 16];
  const BRAKES = [0, 0.3, 0.45, 0.6, 0.8, 1]; // 刹车曲线的减速度，角加速度斜坡的几成；0 = 不用

  /* 搜 kp、ki 和刹车曲线；前馈取模型量出来的带偏量；积分上限不够顶住剩下的平移扰动时一起加大。
     base 是现在的参数（gainsFromParams 的格式）；base.ext 为假（旧固件）时不动刹车曲线和前馈。
     opts: {speed, samples, onProgress(0..1)}；返回 {current, best, plants, scenarios, dropped, unchanged}：
     dropped 是前馈没用上的方向（回放实测反而更差），unchanged 表示现在的参数不用改。 */
  async function optimize(model, base, opts = {}) {
    const plant = model.plant, speed = opts.speed || 400, drift = model.drift || {};
    const ctx = setup(model, base, opts);
    const progress = f => opts.onProgress && opts.onProgress(f);
    const pause = () => new Promise(resolve => setTimeout(resolve, 0)); // 让页面别卡住
    const ff = { x: base.ffx, y: base.ffy };
    if (base.ext) for (const [axis] of AXES) if (drift[axis] && drift[axis].use) ff[axis] = drift[axis].ff;
    // 积分要顶得住前馈以后剩下的带偏：没抵消掉的留五成余量，抵消掉了也按带偏量两倍的不确定算
    const left = (c, f, s) => Math.max(1.5 * Math.abs(c + plant.K * f / 1000), 2 * (s || 0));
    const need = g => Math.max(left(plant.cx, g.ffx, model.spread.cx), left(plant.cy, g.ffy, model.spread.cy)) * speed / Math.max(plant.K, 0.3);
    const make = (kp, ki, brake, f = ff) => {
      const g = { ...base, kp: round2(Math.min(kp, KP_MAX)), ki: round2(Math.min(ki, KI_MAX)), brake: Math.round(brake), ffx: f.x, ffy: f.y };
      g.iMax = Math.max(base.iMax, Math.ceil(need(g) * 2) / 2);
      return g;
    };
    const key = g => `${g.kp}/${g.ki}/${g.brake}/${g.ffx}/${g.ffy}`;
    const full = new Map();
    const score = g => {
      if (!full.has(key(g))) full.set(key(g), { g, r: evaluate(g, ctx), m: margins(g, plant) });
      return full.get(key(g));
    };
    const stable = c => c.m.pm >= PM_MIN && c.m.gm >= GM_MIN;
    const better = (a, b) => !b || (stable(a) !== stable(b) ? stable(a) : a.r.score < b.r.score - 1e-9);

    // 1. 粗筛：整张格子只在拟合出来的那辆车上算，留下最好的一批（裕度够的排前面）
    const brakes = base.ext ? BRAKES.map(f => f * base.acc) : [0];
    const rough = [];
    for (const kp of KPS) {
      for (const ki of KIS) for (const brake of brakes) {
        const g = make(kp, ki, brake);
        rough.push({ g, cost: costOn(g, ctx, 0).cost, ok: stable({ m: margins(g, plant) }) });
      }
      progress(0.35 * rough.length / (KPS.length * KIS.length * brakes.length));
      await pause();
    }
    rough.sort((a, b) => b.ok - a.ok || a.cost - b.cost);
    // 2. 这一批在所有车上算
    let best = null;
    const short = rough.slice(0, 24);
    for (let i = 0; i < short.length; ++i) {
      const c = score(short[i].g);
      if (better(c, best)) best = c;
      if (i % 4 === 3) { progress(0.35 + 0.25 * (i + 1) / short.length); await pause(); }
    }
    // 3. 在最好的那格附近细搜：每个参数试大一点、小一点，变好就挪过去，步长逐步减半
    for (let step = 1.3; step > 1.03; step = Math.sqrt(step)) {
      let moved = true;
      while (moved) {
        moved = false;
        const g0 = best.g, ki0 = g0.ki || 0.25;
        const tries = [[g0.kp * step, g0.ki, g0.brake], [g0.kp / step, g0.ki, g0.brake], [g0.kp, ki0 * step, g0.brake], [g0.kp, g0.ki / step, g0.brake]];
        if (g0.brake > 0) tries.push([g0.kp, g0.ki, Math.min(g0.brake * step, base.acc)], [g0.kp, g0.ki, g0.brake / step]);
        for (const [kp, ki, brake] of tries) {
          const c = score(make(kp, ki < 0.05 ? 0 : ki, brake));
          if (better(c, best)) { best = c; moved = true; }
        }
        await pause();
      }
      progress(0.6 + 0.35 * (1.3 - step) / 0.27);
    }
    // 4. 分数差不到 2% 的里面挑离现在的参数最近的：曲面平的地方每次训练都落在同一头，参数不来回换。
    //    “kp、ki 不动，只加前馈和刹车曲线”也算进来比。
    for (const brake of new Set([best.g.brake, ...brakes])) score(make(base.kp, base.ki, brake));
    const far = c => Math.abs(Math.log((c.g.kp + 0.1) / (base.kp + 0.1))) + 0.5 * Math.abs(Math.log((c.g.ki + 0.5) / (base.ki + 0.5)));
    const top = best;
    for (const c of full.values())
      if (stable(c) === stable(top) && c.r.score <= top.r.score * 1.02 && far(c) < far(best) - 1e-9) best = c;
    // 5. 前馈在回放的实测上反而更差的方向不用（模型认的带偏和实测对不上）
    const dropped = [];
    for (const [axis] of AXES) {
      if (ff[axis] === base[axis === 'x' ? 'ffx' : 'ffy']) continue;
      const back = score(make(best.g.kp, best.g.ki, best.g.brake, { ...{ x: best.g.ffx, y: best.g.ffy }, [axis]: base[axis === 'x' ? 'ffx' : 'ffy'] }));
      if (back.r.score < best.r.score) { best = back; dropped.push(axis); }
    }
    // 6. 比现在的参数好不到 2% 就不动：差这么点在实车上分不出来，参数别每训练一次就换一组
    const now = { g: base, r: evaluate(base, ctx), m: margins(base, plant) };
    const unchanged = (stable(now) || !stable(best)) && now.r.score <= best.r.score * 1.02;
    if (unchanged) best = now;
    progress(1);
    return {
      plants: ctx.plants.length, scenarios: [...ctx.turns, ...ctx.longs, ...ctx.replays].map(s => s.label), dropped, unchanged,
      current: { gains: base, ...now.r, margins: now.m },
      best: { gains: best.g, ...best.r, margins: best.m },
    };
  }

  /* 线性化以后的相位裕度、增益裕度（只看小误差，限幅、积分区和刹车曲线不算在内）。 */
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

  /* 每个动作用日志里的参数和命令（目标、平移）在模型上重跑，到位时间、超调、平移时的航向误差跟实测对照。
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
            peakMove: median(sims.map(s => s.peakMove)), rmsMove: median(sims.map(s => s.rmsMove)),
          },
        });
      }
    }
    return rows;
  }

  const api = {
    DT, createParser, gainsFromParams, segments, recordMetrics, simulate, replay, disturbance, recordScenario, identify,
    optimize, scoreGains, margins, verify, moveProfile,
  };
  root.PidTune = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);

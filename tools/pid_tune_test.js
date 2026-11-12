/* tools/pid_tune.js 的测试。firmware/tests/run_tests.py 先跑 tune_log_test（真实固件代码 + 假车），
   把串口输出存成文件，这里拿它检查：
   1. 解析：每段日志都完整，夹在中间的其他输出不影响；缺一行的整段不要；旧固件的日志（参数行里没有
      刹车曲线、前馈）按没有这些功能处理。
   2. 回放：用假车的真实参数原样回放，仿真和固件记下的航向、航向环输出对得上，
      说明这里的航向环、斜坡、轮速量化和固件是一样的。
   3. 辨识：从日志拟合出来的模型接近假车的真实参数；带偏量稳，给得出前馈。
   4. 扰动回放：用日志自己的参数回放正好得到实测；加上前馈，同样的扰动下航向误差小得多。
      平移冲过头多少毫米，从日志里下发的车速算得出来。
   5. 搜参数：推荐值在模型上比现在的好，再训练一次不换参数；旧固件只动 kp、ki、积分上限。
   6. 放回真车：给了 tune_log_test 的可执行文件，就把现在的参数和推荐值都放到假车上跑（--bench，量的是
      真实航向），包括训练时没走过的距离。两辆车：一辆就是模型假设的那种；一辆加了模型里没有的东西
      （二阶惯性、转得快多打滑、加减速扭车身、地面随机扰动、陀螺仪 100 Hz）。
   用法：node tools/pid_tune_test.js <tune_fixture.log> [tune_log_test 可执行文件] */
'use strict';
const fs = require('fs');
const assert = require('assert');
const { execFileSync } = require('child_process');
const T = require('./pid_tune.js');

const near = (value, expect, tol, what) =>
  assert(Math.abs(value - expect) <= tol, `${what}: ${value} not within ${tol} of ${expect}`);

function parseCar(line) {
  const f = {};
  for (const item of line.slice(6).trim().split(/\s+/)) {
    const [k, v] = item.split('=');
    f[k] = Number(v);
  }
  return { K: f.slip, tau: f.tau, delay: f.delay, cx: f.cx, cy: f.cy, noise: f.noise, exact: f.exact === 1 };
}

/* 文件里一辆车一块（“# car”开头）。skip 用来装成旧固件：把含这段字的行丢掉。 */
function readBlocks(path, skip) {
  const blocks = [];
  for (const line of fs.readFileSync(path, 'utf8').split(/\r?\n/)) {
    if (line.startsWith('# car ')) {
      blocks.push({ car: parseCar(line), records: [], parser: T.createParser(), broken: 0 });
      continue;
    }
    if (skip && line.includes(skip)) continue;
    const b = blocks[blocks.length - 1];
    const r = b && b.parser.feed(line);
    if (r && r.kind === 'record') b.records.push(r.record);
    if (r && r.kind === 'broken') ++b.broken;
  }
  return blocks;
}

function testParser() {
  const p = T.createParser();
  const lines = ['LOG begin n=12 mode=path end=done trunc=0 imu=1', 'LOG param heading_kp=2.000 heading_ki=1.000',
    'EVT  READY -> READY', 'LOG param heading_brake=300 heading_ff_x=-2.20 heading_ff_y=0.00 max_wheel_rpm=750'];
  for (let i = 0; i < 12; ++i) lines.push(`L ${i} 3 0 90000 ${179000 + i * 400} 900 850 0 0`); // 跨过 180°
  lines.push('LOG end n=12');
  let out = null;
  for (const line of lines) out = p.feed(line) || out;
  assert(out && out.kind === 'record');
  const rec = out.record;
  assert(rec.n === 12 && rec.params.heading_kp === 2 && rec.mode === 'path');
  for (let k = 1; k < rec.n; ++k) near(rec.yaw[k] - rec.yaw[k - 1], 0.4, 1e-9, 'unwrapped yaw step');
  near(rec.target[0] - rec.yaw[0], wrap(90 - 179), 1e-9, 'target follows unwrapped yaw');
  const g = T.gainsFromParams(rec.params);
  assert(g.ext && g.brake === 300 && g.ffx === -2.2 && g.ffy === 0 && g.wheelMax === 750);

  const old = lines.filter((_, i) => i !== 3); // 旧固件：没有第三行参数
  for (const line of old) out = p.feed(line) || out;
  const og = T.gainsFromParams(out.record.params);
  assert(!og.ext && og.brake === 0 && og.ffx === 0 && og.wheelMax === 400);

  const broken = lines.filter((_, i) => i !== 8); // 少一行
  let r = null;
  for (const line of broken) r = p.feed(line) || r;
  assert(r.kind === 'broken');
  assert(p.feed('LOG none').kind === 'none');
}

function wrap(a) {
  a %= 360;
  return a > 180 ? a - 360 : a < -180 ? a + 360 : a;
}

/* 仿真的航向环和固件是不是一回事：用真实参数回放，航向误差只能是量化、浮点和噪声那么点；
   航向环输出逐拍对上，只有误差正好压在到位死区边上的那几拍会一边是 0、一边不是。 */
function testReplay(blocks) {
  for (const b of blocks.filter(x => x.car.exact)) {
    for (const rec of b.records) {
      const r = T.replay(rec, b.car);
      assert(r.rmsYaw < (b.car.noise ? 0.015 : 0.01), `replay yaw rms ${r.rmsYaw} (noise ${b.car.noise})`);
      let same = 0;
      for (let k = 0; k < rec.n; ++k) if (Math.abs(r.trace.pid[k] - rec.pid[k]) <= 0.1) ++same;
      // 有噪声时停在死区边上会被噪声来回推过边，两边的噪声又不是同一串，翻的拍多一些
      assert(same >= (b.car.noise ? 0.93 : 0.98) * rec.n, `replay pid matches ${same}/${rec.n} ticks`);
    }
  }
}

function testIdentify(blocks) {
  const clean = T.identify(blocks[2].records);
  assert(clean.ok, clean.warnings.join(' '));
  const c = clean.plant, cc = blocks[2].car;
  near(c.K, cc.K, 0.01 * cc.K, 'clean K');
  near(c.tau, cc.tau, 0.2 * cc.tau, 'clean tau');
  near(c.delay, cc.delay, 0.003, 'clean delay');
  near(c.cy, cc.cy, 0.0005, 'clean cy');
  near(c.cx, cc.cx, 0.0005, 'clean cx');
  // 假车没有的东西不能凭空拟合出来：加减速扭车身、转得快多打滑
  near(c.ax, 0, 0.0001, 'clean ax');
  near(c.ay, 0, 0.0001, 'clean ay');
  near(c.kq, 0, 0.005, 'clean kq');
  assert(clean.drift.y.use && !clean.drift.x.use, 'the clean car only drifts sideways');

  const noisy = T.identify([...blocks[0].records, ...blocks[1].records]);
  assert(noisy.ok, noisy.warnings.join(' '));
  const n = noisy.plant, nc = blocks[0].car;
  near(n.K, nc.K, 0.03 * nc.K, 'noisy K');
  near(n.tau, nc.tau, 0.35 * nc.tau, 'noisy tau');
  near(n.delay, nc.delay, 0.0075, 'noisy delay');
  near(n.cx, nc.cx, 0.001, 'noisy cx');
  near(n.cy, nc.cy, 0.0015, 'noisy cy');
  near(n.kq, 0, 0.01, 'noisy kq');
  assert(noisy.warnings.length === 0, noisy.warnings.join(' '));
  for (const [axis, real] of [['x', nc.cx], ['y', nc.cy]]) {
    const d = noisy.drift[axis];
    assert(d.seen && d.use && d.explained > 0.9, `drift ${axis} ${JSON.stringify(d)}`);
    near(d.ff, -real / nc.K * 1000, 0.3, `feed-forward ${axis}`);
  }

  // 模型外的车：总滞后、小转速下的 K、转得快多打滑、带偏量都认得出来（假车：20 + 15 ms 两个惯性、
  // 15 ms 延迟、陀螺仪每 10 ms 一个数；K 0.93，到 180 deg/s 掉 0.06）
  const harsh = T.identify(blocks[3].records);
  assert(harsh.ok, harsh.warnings.join(' '));
  const h = harsh.plant;
  near(h.K, 0.93, 0.03, 'harsh K');
  near(h.kq, -0.06 / 1.8, 0.015, 'harsh kq');
  near(h.tau + h.delay, 0.055, 0.012, 'harsh total lag');
  near(h.cy, blocks[3].car.cy, 0.0007, 'harsh cy');
  assert(harsh.drift.y.use && harsh.drift.y.explained > 0.8, JSON.stringify(harsh.drift.y));
  return { noisy, harsh };
}

/* 实测扰动回放：日志自己的参数回放 = 实测；同样的扰动加上前馈，平移时的航向误差小得多。 */
function testDisturbance(block, model) {
  let checked = 0;
  for (const rec of block.records) {
    const m = T.recordMetrics(rec);
    if (m.length !== 1 || m[0].kind !== 'move') continue;
    const sc = T.recordScenario(rec);
    sc.dist = T.disturbance(rec, model.plant);
    const g = T.gainsFromParams(rec.params);
    const same = T.simulate(g, model.plant, sc, { trace: true });
    let worst = 0;
    for (let k = 0; k < rec.n; ++k) worst = Math.max(worst, Math.abs(same.trace.yaw[k] - (rec.yaw[k] - rec.yaw[0])));
    assert(worst < 0.03, `replay with the logged gains differs from the log by ${worst} deg`);
    near(same.peakMove, m[0].peakMove, 0.03, 'replayed peak');
    const fixed = T.simulate({ ...g, ffx: model.drift.x.ff, ffy: model.drift.y.ff }, model.plant, sc);
    assert(fixed.peakMove < 0.25 * m[0].peakMove, `feed-forward on the same disturbance: ${fixed.peakMove} vs ${m[0].peakMove}`);
    ++checked;
  }
  assert(checked === 4);
}

/* 位置冲过头：日志里只有下发的车速，按它沿走的方向积分，最远到过哪里减最后停在哪里。
   合成一段：400 mm/s 走 0.5 s，再 -100 mm/s 退 0.2 s，退回来 20 mm。
   假车的轮子比下发的晚（延迟 + 惯性），固件的刹车曲线按 path_lag 提前减速（chassis_path.inc），
   1000 mm/s 停下只多滑几毫米；不提前要冲过去“车速 × 滞后”（60 mm）再倒回来。转向没有这一项。 */
function testOverrun(block) {
  const n = 160, rec = {
    n, params: { heading_tol: 0.1 }, motion: new Uint8Array(n).fill(3), moving: new Uint8Array(n).fill(1),
    target: new Float64Array(n), yaw: new Float64Array(n), vx: new Float64Array(n), vy: new Float64Array(n),
  };
  for (let k = 0; k < 140; ++k) rec.vx[k] = k < 100 ? 400 : -100;
  near(T.recordMetrics(rec)[0].overrun, 20, 0.01, 'synthetic overrun, mm');

  const all = block.records.flatMap(r => T.recordMetrics(r));
  const moves = all.filter(m => m.kind === 'move');
  assert(moves.length === 4 && all.filter(m => m.kind === 'turn').every(m => m.overrun === 0));
  for (const m of moves) assert(m.overrun < 8, `position overrun ${m.overrun} mm at ${m.speed} mm/s`);
}

/* 车上现在的参数：日志里的其他参数 + chassis.h 现在的默认增益。 */
const currentGains = recs => T.gainsFromParams({ ...recs[0].params, heading_kp: 5.6, heading_ki: 2.14, heading_i_max: 7 });

async function testOptimize(model, recs) {
  const base = currentGains(recs);
  const result = await T.optimize(model, base, { speed: 1000 });
  const best = result.best.gains;
  assert(best.kp > 0 && best.kp <= 20 && best.ki >= 0 && best.brake > 0 && best.brake <= base.acc, JSON.stringify(best));
  assert(result.best.score < result.current.score * 0.6, `score ${result.best.score} vs ${result.current.score}`);
  assert(result.best.margins.pm >= 45 && result.best.margins.gm >= 8, JSON.stringify(result.best.margins));
  assert(!result.unchanged && result.dropped.length === 0);
  near(best.ffx, model.drift.x.ff, 1e-9, 'recommended ff x');
  near(best.ffy, model.drift.y.ff, 1e-9, 'recommended ff y');
  // 用推荐值再训练一次：不换参数
  const again = await T.optimize(model, best, { speed: 1000 });
  assert(again.unchanged && again.best.gains === best, `retrain moved to ${JSON.stringify(again.best.gains)}`);
  return result;
}

/* 旧固件（没有刹车曲线、前馈）：只动 kp、ki、积分上限，推荐值不会比现在的差。 */
async function testOldFirmware(path) {
  const blocks = readBlocks(path, 'heading_brake=');
  const recs = [...blocks[0].records, ...blocks[1].records];
  assert(recs.length === 20 && recs.every(r => r.params.heading_brake === undefined));
  const model = T.identify(recs);
  assert(model.ok);
  const base = currentGains(recs);
  assert(!base.ext);
  const result = await T.optimize(model, base, { speed: 1000 });
  const g = result.best.gains;
  assert(g.brake === 0 && g.ffx === 0 && g.ffy === 0, JSON.stringify(g));
  assert(result.best.score <= result.current.score + 1e-9);
}

function testVerify(block, model) {
  const rows = T.verify(block.records, model.plant);
  assert(rows.length === block.records.length);
  let moves = 0;
  for (const row of rows) {
    if (row.kind === 'turn' && Number.isFinite(row.measured.settle))
      near(row.predicted.settle, row.measured.settle, Math.max(0.15, 0.25 * row.measured.settle),
        `settle prediction for ${row.step.toFixed(1)} deg`);
    if (row.kind === 'move') {
      near(row.predicted.peakMove, row.measured.peakMove, Math.max(0.1, 0.3 * row.measured.peakMove), 'peak heading error while moving');
      ++moves;
    }
  }
  assert(moves === 4);
}

/* ---------- 放回真车 ---------- */
const TURNS = ['T5', 'T-5', 'T20', 'T-20', 'T90', 'T-90'];
const MOVES = ['F800', 'B800', 'L800', 'R800', 'F1500', 'B1500', 'L1500', 'R1500', 'F300', 'L300']; // 后六个训练时没走过

function bench(exe, car, g) {
  const sets = [`/set heading_kp ${g.kp}`, `/set heading_ki ${g.ki}`, `/set heading_i_max ${g.iMax}`, `/set heading_brake ${g.brake}`,
    `/set heading_ff_x ${g.ffx}`, `/set heading_ff_y ${g.ffy}`];
  const out = execFileSync(exe, ['--bench', String(car), ...sets, ...TURNS, ...MOVES], { encoding: 'utf8' });
  const rows = out.trim().split(/\r?\n/).map(line => {
    const parts = line.split(' '), row = { name: parts[1] };
    for (const item of parts.slice(2)) { const [k, v] = item.split('='); row[k] = Number(v); }
    return row;
  });
  assert(rows.length === TURNS.length + MOVES.length);
  const turns = rows.filter(r => r.name.startsWith('T')), moves = rows.filter(r => !r.name.startsWith('T'));
  const avg = (list, f) => list.reduce((a, r) => a + f(r), 0) / list.length;
  assert(rows.every(r => r.settle >= 0), 'every motion settles');
  return {
    settle: avg(turns, r => r.settle), over: Math.max(...turns.map(r => r.over)), final: avg(rows, r => r.final),
    peak: avg(moves, r => r.peak), peakMax: Math.max(...moves.map(r => r.peak)), limit: moves.reduce((a, r) => a + r.limit, 0),
    done: avg(moves, r => r.done),
  };
}

function testOnCar(exe, car, result, limits) {
  const before = bench(exe, car, result.current.gains), after = bench(exe, car, result.best.gains);
  const text = `car ${car}: ${JSON.stringify(before)} -> ${JSON.stringify(after)}`;
  assert(after.peak < limits.peak * before.peak, `mean peak heading error while moving, ${text}`);
  assert(after.peakMax < limits.peakMax, `worst peak heading error while moving, ${text}`);
  assert(after.limit <= limits.limit * before.limit, `ticks over 0.2 deg, ${text}`);
  assert(after.settle < limits.settle * before.settle, `turns got slower, ${text}`);
  assert(after.over < 0.2, `turn overshoot, ${text}`);
  assert(after.final <= before.final + 0.01, `heading left over after each motion, ${text}`);
  assert(after.done <= before.done + 0.05, `moves take longer, ${text}`);
  return `move peak ${before.peak.toFixed(2)} -> ${after.peak.toFixed(3)} deg (worst ${before.peakMax.toFixed(2)} -> ${after.peakMax.toFixed(2)}), ` +
    `turn settle ${before.settle.toFixed(2)} -> ${after.settle.toFixed(2)} s`;
}

(async () => {
  const [fixture, exe] = process.argv.slice(2);
  testParser();
  const blocks = readBlocks(fixture);
  assert(blocks.length === 4);
  for (const b of blocks) assert(b.records.length === 10 && b.broken === 0, `records ${b.records.length}, broken ${b.broken}`);
  assert(blocks[0].car.exact && blocks[1].car.exact && blocks[2].car.exact && !blocks[3].car.exact);
  testReplay(blocks);
  const { noisy, harsh } = testIdentify(blocks);
  testDisturbance(blocks[0], noisy);
  testOverrun(blocks[0]);
  testOverrun(blocks[2]);
  const t0 = Date.now();
  const usual = await testOptimize(noisy, blocks[0].records);
  const rough = await testOptimize(harsh, blocks[3].records);
  const spent = Date.now() - t0;
  await testOldFirmware(fixture);
  testVerify(blocks[1], noisy);
  const text = r => `kp=${r.best.gains.kp} ki=${r.best.gains.ki} brake=${r.best.gains.brake} ff=${r.best.gains.ffx}/${r.best.gains.ffy}`;
  console.log(`pid_tune_test passed: K=${noisy.plant.K.toFixed(3)} tau=${(noisy.plant.tau * 1000).toFixed(1)}ms ` +
    `delay=${(noisy.plant.delay * 1000).toFixed(1)}ms -> ${text(usual)}; out-of-model car -> ${text(rough)} (4 searches ${spent} ms)`);
  if (!exe) { console.log('SKIP the true-car check: pass the tune_log_test executable as the second argument'); return; }
  // 模型里的那种车：前馈几乎全抵消；模型外的车：剩下随机的那部分，也要压到验收线 0.2° 附近
  console.log('  true car 0: ' + testOnCar(exe, 0, usual, { peak: 0.05, peakMax: 0.05, limit: 0, settle: 1.3 }));
  console.log('  true car 2: ' + testOnCar(exe, 2, rough, { peak: 0.45, peakMax: 0.3, limit: 0.05, settle: 1.15 }));
})().catch(e => {
  console.error(e);
  process.exit(1);
});

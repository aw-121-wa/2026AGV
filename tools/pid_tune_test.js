/* tools/pid_tune.js 的测试。firmware/tests/run_tests.py 先跑 tune_log_test（真实固件代码 + 假车），
   把串口输出存成文件，这里拿它检查：
   1. 解析：每段日志都完整，夹在中间的其他输出不影响；缺一行的整段不要。
   2. 回放：用假车的真实参数原样回放，仿真和固件记下的航向、航向环输出对得上，
      说明这里的航向环、斜坡、轮速量化和固件是一样的。
   3. 辨识：从日志拟合出来的模型接近假车的真实参数。
   4. 搜参数：推荐值放到真实的假车上比现在的参数好；验证表里预测和实测对得上。
   用法：node tools/pid_tune_test.js <tune_fixture.log> */
'use strict';
const fs = require('fs');
const assert = require('assert');
const T = require('./pid_tune.js');

const near = (value, expect, tol, what) =>
  assert(Math.abs(value - expect) <= tol, `${what}: ${value} not within ${tol} of ${expect}`);

function parseCar(line) {
  const f = {};
  for (const item of line.slice(6).trim().split(/\s+/)) {
    const [k, v] = item.split('=');
    f[k] = Number(v);
  }
  return { K: f.slip, tau: f.tau, delay: f.delay, cx: f.cx, cy: f.cy, noise: f.noise };
}

function readBlocks(path) {
  const blocks = [];
  for (const line of fs.readFileSync(path, 'utf8').split(/\r?\n/)) {
    if (line.startsWith('# car ')) {
      blocks.push({ car: parseCar(line), records: [], parser: T.createParser(), broken: 0 });
      continue;
    }
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
    'EVT  READY -> READY'];
  for (let i = 0; i < 12; ++i) lines.push(`L ${i} 3 0 90000 ${179000 + i * 400} 900 850 0 0`); // 跨过 180°
  lines.push('LOG end n=12');
  let out = null;
  for (const line of lines) out = p.feed(line) || out;
  assert(out && out.kind === 'record');
  const rec = out.record;
  assert(rec.n === 12 && rec.params.heading_kp === 2 && rec.mode === 'path');
  for (let k = 1; k < rec.n; ++k) near(rec.yaw[k] - rec.yaw[k - 1], 0.4, 1e-9, 'unwrapped yaw step');
  near(rec.target[0] - rec.yaw[0], wrap(90 - 179), 1e-9, 'target follows unwrapped yaw');

  const broken = lines.filter((_, i) => i !== 7); // 少一行
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
  for (const b of blocks) {
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

  const noisy = T.identify([...blocks[0].records, ...blocks[1].records]);
  assert(noisy.ok, noisy.warnings.join(' '));
  const n = noisy.plant, nc = blocks[0].car;
  near(n.K, nc.K, 0.03 * nc.K, 'noisy K');
  near(n.tau, nc.tau, 0.35 * nc.tau, 'noisy tau');
  near(n.delay, nc.delay, 0.0075, 'noisy delay');
  near(n.cx, nc.cx, 0.001, 'noisy cx');
  near(n.cy, nc.cy, 0.0015, 'noisy cy');
  assert(noisy.warnings.length === 0, noisy.warnings.join(' '));
  return noisy;
}

/* 推荐值放回真实的假车上：分数要比现在的参数好，而且预测得准。 */
async function testOptimize(blocks, model) {
  const base = T.gainsFromParams(blocks[0].records[0].params);
  const result = await T.optimize(model, base, { speed: 400 });
  const best = result.best.gains;
  assert(best.kp > 0 && best.ki >= 0);
  assert(result.best.score < result.current.score * 0.85, `score ${result.best.score} vs ${result.current.score}`);
  assert(result.best.margins.pm > 30, `phase margin ${result.best.margins.pm}`);
  // 在真实的假车上比一比（不是拟合出来的模型）
  const truth = { plant: blocks[0].car, spread: { K: 0, tauRel: 0, delay: 0 } };
  const now = T.scoreGains(truth, base, { speed: 400, samples: 0 }).score;
  const next = T.scoreGains(truth, best, { speed: 400, samples: 0 }).score;
  assert(next < now * 0.85, `true-car score ${next} vs ${now}`);
  return { best, now, next };
}

function testVerify(blocks, model) {
  const rows = T.verify(blocks[1].records, model.plant);
  assert(rows.length === blocks[1].records.length);
  for (const row of rows) {
    if (row.kind !== 'turn' || !Number.isFinite(row.measured.settle)) continue;
    near(row.predicted.settle, row.measured.settle, Math.max(0.15, 0.25 * row.measured.settle),
      `settle prediction for ${row.step.toFixed(1)} deg`);
  }
}

(async () => {
  testParser();
  const blocks = readBlocks(process.argv[2]);
  assert(blocks.length === 3);
  for (const b of blocks) assert(b.records.length === 10 && b.broken === 0, `records ${b.records.length}, broken ${b.broken}`);
  testReplay(blocks);
  const model = testIdentify(blocks);
  const t0 = Date.now();
  const opt = await testOptimize(blocks, model);
  testVerify(blocks, model);
  console.log(`pid_tune_test passed: K=${model.plant.K.toFixed(3)} tau=${(model.plant.tau * 1000).toFixed(1)}ms ` +
    `delay=${(model.plant.delay * 1000).toFixed(1)}ms -> kp=${opt.best.kp} ki=${opt.best.ki} i_max=${opt.best.iMax}, ` +
    `true-car score ${opt.now.toFixed(2)} -> ${opt.next.toFixed(2)} s (optimize ${Date.now() - t0} ms)`);
})().catch(e => {
  console.error(e);
  process.exit(1);
});

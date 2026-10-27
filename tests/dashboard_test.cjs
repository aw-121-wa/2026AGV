// Exercise the actual inline planner without browser drawing/network APIs.
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const html = fs.readFileSync('tools/dashboard/index.html', 'utf8');
for (const script of html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)) new Function(script[1]);
const constants = html.slice(html.indexOf('const FIELD ='), html.indexOf('const STATE_NAME'));
const planner = html.slice(html.indexOf('function isBlockedMm'), html.indexOf('function pathLength'));
const ctx = vm.createContext({console});
vm.runInContext(`const S={tele:{x:2250,y:2250},obstacles:[],path:[]};
const $=()=>({textContent:''}); const addLog=()=>{}; const pathLength=()=>0;
${constants}\n${planner}`, ctx);
function run(code) { return vm.runInContext(code, ctx); }
assert.strictEqual(run('planLocal(2390,2250,true)'), false, 'body must not leave the field');
assert.strictEqual(run('planLocal(520,755,true)'), false, 'goal body overlaps island');
assert.strictEqual(run('planLocal(355,355,true)'), true, 'valid route from start zone');
assert.strictEqual(run('S.path.every((p,i)=>!i||p[0]===S.path[i-1][0]||p[1]===S.path[i-1][1])'), true,
  'every segment including grid connectors must use four directions');
assert.strictEqual(run('S.path[0][0]===2250 && S.path[0][1]===2250'), true);
assert.strictEqual(run('planLocal(2250,2250,true)'), true, 'same-point route');
assert.strictEqual(run('planLocal(NaN,1200,true)'), false);
assert.strictEqual(run('planLocal(-10,1200,true)'), false);
assert.strictEqual(run('S.tele.x=50;planLocal(355,355,true)'), false, 'invalid start rejected');
console.log('PASS: dashboard body clearance, start/goal rejection, four-direction connectors');
vm.runInContext(`const sim={yaw:30,hdgT:0,mode:0,x:355,y:355};
${html.slice(html.indexOf('function simCommand'),html.indexOf('function simTick'))}`,ctx);
run('simCommand("T10")');
assert.strictEqual(run('sim.hdgT'),40,'T is relative to current yaw');
run('simCommand("A-0.125")');
assert.strictEqual(run('sim.hdgT'),-0.125,'A is absolute');
run('simCommand("S")');
assert.strictEqual(run('sim.mode===0 && sim.hdgT===sim.yaw && sim.wz===0'),true,'S cancels rotation');
console.log('PASS: simulator relative/absolute turns and stop');
const startup = vm.createContext({console,performance:{now:()=>0},
  document:{querySelectorAll:()=>[]},setInterval:()=>1,clearInterval:()=>{}});
vm.runInContext(`${constants}
${html.slice(html.indexOf('const S ='),html.indexOf('/* =====================================================================\n *  DOM'))}
const $=()=>({style:{},textContent:''});const addLog=()=>{};
const setConn=s=>S.conn=s; const planLocal=()=>{S.path=[[400,400],[1600,1200]]};
const simTick=()=>{};let simTimer=null;let rxBuffer='old data';
${html.slice(html.indexOf('const sim ='),html.indexOf('function simCommand'))}
startSim();`,startup);
assert.strictEqual(vm.runInContext('sim.x===2250 && sim.y===2250 && !sim.auto && sim.target===null && S.path.length===0',startup),true,
  'simulation starts parked in the upper departure zone without a demo route');
assert.strictEqual(vm.runInContext('S.tele.x===2250 && S.tele.y===2250',startup),true);
vm.runInContext(`const pushTrail=()=>{};
${html.slice(html.indexOf('function handleLine'),html.indexOf('function pushTrail'))}
handleLine('$T,100,2200,150,0,0,0,1,0,0,0,0,0,0,0,0,0');`,startup);
assert.strictEqual(vm.runInContext('S.tele.x===2200 && S.tele.y===150 && S.tele.valid',startup),true,
  'real telemetry overrides the placeholder, including lower departure zone');
vm.runInContext('S.path=[[1,2]];S.trail=[[3,4]];resetDepartureDisplay();',startup);
assert.strictEqual(vm.runInContext('!S.tele.valid && S.path.length===0 && S.trail.length===0 && S.tele.x===2250',startup),true);
const boxes=[],labels=[];
const canvasCtx=new Proxy({}, {get:(o,k)=>k==='fillRect'?(...a)=>boxes.push(a):
  k==='fillText'?(text)=>labels.push(text):()=>{},set:(o,k,v)=>{o[k]=v;return true;}});
const drawing=vm.createContext({document:{createElement:()=>({getContext:()=>canvasCtx})}});
vm.runInContext(`${constants}
let bgCanvas;const mapCanvas={width:2400,height:2400};const view={w:2400,h:2400,scale:1};
const w2cx=x=>x,w2cy=y=>2400-y;
${html.slice(html.indexOf('function buildBackground'),html.indexOf('function render()'))}
buildBackground(1);`,drawing);
assert.deepStrictEqual(boxes.filter(b=>b[2]===300 && b[3]===300),[[2100,0,300,300],[2100,2100,300,300]]);
assert(labels.includes('出发区 1') && labels.includes('出发区 2'));
console.log('PASS: parked startup, telemetry override, reset, both departure-zone boxes and labels');

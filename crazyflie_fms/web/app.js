/* Crazyflie FMS — 브라우저 UI.
 *
 * ROS 와의 접점은 rosbridge 웹소켓 하나뿐이고, 그 위로 오가는 것도 JSON 토픽 두 개다.
 *   구독: /fms/fleet_state  — 편대 스냅샷 (약 10 Hz)
 *   발행: /fms/command      — 버튼/클릭
 *
 * 맵 좌표계는 RViz 의 위에서 내려다본 기본 시점과 맞췄다: world +x 는 화면 위쪽,
 * world +y 는 화면 왼쪽. 그래야 RViz 와 나란히 놓고 봐도 헷갈리지 않는다.
 */
'use strict';

const BRIDGE_URL = `ws://${location.hostname || 'localhost'}:9090`;

const PHASE_COLOR = {
  idle: '#8b96a5',
  takeoff: '#ffc857',
  enroute: '#4da3ff',
  hold: '#3ddc84',
  landing: '#ffc857',
};

const state = {
  fleet: null,
  selected: null,
  takeoffHeight: 1.0,
  view: null,       // {minX, maxX, minY, maxY} — 월드 좌표 기준 표시 범위
};

/* ------------------------------------------------------------------ ROS */

const ros = new ROSLIB.Ros({ url: BRIDGE_URL });
const connEl = document.getElementById('conn');

ros.on('connection', () => {
  connEl.textContent = 'rosbridge 연결됨';
  connEl.className = 'pill on';
});
ros.on('close', () => {
  connEl.textContent = 'rosbridge 끊김 — 재연결 중';
  connEl.className = 'pill off';
  // rosbridge 나 launch 를 재시작해도 브라우저를 새로고침할 필요가 없게 한다.
  setTimeout(() => ros.connect(BRIDGE_URL), 1500);
});
ros.on('error', () => {
  connEl.textContent = 'rosbridge 연결 안 됨';
  connEl.className = 'pill off';
});

const fleetTopic = new ROSLIB.Topic({
  ros, name: '/fms/fleet_state', messageType: 'std_msgs/String',
});

const cmdTopic = new ROSLIB.Topic({
  ros, name: '/fms/command', messageType: 'std_msgs/String',
});

function send(cmd) {
  cmdTopic.publish(new ROSLIB.Message({ data: JSON.stringify(cmd) }));
}

fleetTopic.subscribe((msg) => {
  state.fleet = JSON.parse(msg.data);
  state.takeoffHeight = state.fleet.takeoff_height ?? state.takeoffHeight;
  if (state.selected === null && state.fleet.robots.length) {
    state.selected = state.fleet.robots[0].name;
  }
  renderSide();
});

/* ----------------------------------------------------------------- 맵 */

const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const coordsEl = document.getElementById('coords');

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(r.width * dpr));
  canvas.height = Math.max(1, Math.round(r.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);

/** 표시할 모든 점을 감싸는 범위. 매 프레임 다시 맞추면 화면이 떨리므로,
 *  현재 범위를 벗어나거나 지나치게 넓어졌을 때만 다시 맞춘다. */
function updateView(robots) {
  const pts = [[0, 0]];
  for (const r of robots) {
    pts.push([r.pos[0], r.pos[1]]);
    pts.push([r.initial_position[0], r.initial_position[1]]);
    if (r.goal) pts.push([r.goal[0], r.goal[1]]);
  }

  const pad = 1.0;
  let minX = Math.min(...pts.map((p) => p[0])) - pad;
  let maxX = Math.max(...pts.map((p) => p[0])) + pad;
  let minY = Math.min(...pts.map((p) => p[1])) - pad;
  let maxY = Math.max(...pts.map((p) => p[1])) + pad;

  const MIN_SPAN = 3.0;
  if (maxX - minX < MIN_SPAN) {
    const c = (maxX + minX) / 2;
    minX = c - MIN_SPAN / 2; maxX = c + MIN_SPAN / 2;
  }
  if (maxY - minY < MIN_SPAN) {
    const c = (maxY + minY) / 2;
    minY = c - MIN_SPAN / 2; maxY = c + MIN_SPAN / 2;
  }

  const v = state.view;
  const outside = !v || minX < v.minX || maxX > v.maxX || minY < v.minY || maxY > v.maxY;
  const tooBig = v && (v.maxX - v.minX) > 2 * (maxX - minX);
  if (outside || tooBig) state.view = { minX, maxX, minY, maxY };
}

/** 월드 → 화면. +x 위, +y 왼쪽. 축척은 xy 동일(원이 타원으로 찌그러지지 않게). */
function projector() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const v = state.view || { minX: -1.5, maxX: 1.5, minY: -1.5, maxY: 1.5 };
  const scale = Math.min(w / (v.maxY - v.minY), h / (v.maxX - v.minX));
  const cx = w / 2;
  const cy = h / 2;
  const midX = (v.minX + v.maxX) / 2;
  const midY = (v.minY + v.maxY) / 2;

  const toScreen = (x, y) => [cx - (y - midY) * scale, cy - (x - midX) * scale];
  const toWorld = (sx, sy) => [midX + (cy - sy) / scale, midY + (cx - sx) / scale];
  return { toScreen, toWorld, scale, w, h };
}

function drawGrid(p) {
  const v = state.view;
  if (!v) return;
  ctx.lineWidth = 1;
  ctx.font = '10px system-ui, sans-serif';

  for (let x = Math.ceil(v.minX); x <= v.maxX; x += 1) {
    const [, sy] = p.toScreen(x, 0);
    ctx.strokeStyle = x === 0 ? '#3b4654' : '#212831';
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(p.w, sy); ctx.stroke();
    ctx.fillStyle = '#5b6674';
    ctx.fillText(`x=${x}`, 6, sy - 4);
  }
  for (let y = Math.ceil(v.minY); y <= v.maxY; y += 1) {
    const [sx] = p.toScreen(0, y);
    ctx.strokeStyle = y === 0 ? '#3b4654' : '#212831';
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, p.h); ctx.stroke();
    ctx.fillStyle = '#5b6674';
    ctx.fillText(`y=${y}`, sx + 4, p.h - 6);
  }
}

function drawRobot(p, r) {
  const [sx, sy] = p.toScreen(r.pos[0], r.pos[1]);
  const selected = r.name === state.selected;
  const color = r.connected ? (PHASE_COLOR[r.phase] || '#8b96a5') : '#5b6674';

  // 이륙 지점 — 어디서 떠올랐는지 보이면 편대 배치를 가늠하기 쉽다.
  const [ix, iy] = p.toScreen(r.initial_position[0], r.initial_position[1]);
  ctx.strokeStyle = '#39424f';
  ctx.lineWidth = 1;
  ctx.strokeRect(ix - 4, iy - 4, 8, 8);

  if (r.goal) {
    const [gx, gy] = p.toScreen(r.goal[0], r.goal[1]);

    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.45;
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(gx, gy); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(gx, gy, 7, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(gx - 10, gy); ctx.lineTo(gx + 10, gy);
    ctx.moveTo(gx, gy - 10); ctx.lineTo(gx, gy + 10);
    ctx.stroke();
  }

  if (selected) {
    ctx.strokeStyle = '#4da3ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(sx, sy, 15, 0, Math.PI * 2); ctx.stroke();
  }

  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(sx, sy, 7, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#e6eaf0';
  ctx.font = '12px system-ui, sans-serif';
  ctx.fillText(r.name, sx + 12, sy - 8);
  ctx.fillStyle = '#8b96a5';
  ctx.font = '10px system-ui, sans-serif';
  ctx.fillText(`z=${r.pos[2].toFixed(2)}`, sx + 12, sy + 5);
}

function draw() {
  const robots = state.fleet ? state.fleet.robots : [];
  updateView(robots);
  const p = projector();

  ctx.fillStyle = '#0b0e12';
  ctx.fillRect(0, 0, p.w, p.h);
  drawGrid(p);
  for (const r of robots) drawRobot(p, r);

  requestAnimationFrame(draw);
}

canvas.addEventListener('mousemove', (e) => {
  const rect = canvas.getBoundingClientRect();
  const [x, y] = projector().toWorld(e.clientX - rect.left, e.clientY - rect.top);
  coordsEl.textContent = `x ${x.toFixed(2)}  y ${y.toFixed(2)}`;
});

canvas.addEventListener('click', (e) => {
  if (!state.selected) return;
  const rect = canvas.getBoundingClientRect();
  const [x, y] = projector().toWorld(e.clientX - rect.left, e.clientY - rect.top);
  send({ op: 'set_goal', robot: state.selected, goal: [x, y, state.takeoffHeight] });
});

/* ------------------------------------------------------------- 사이드바 */

const robotsEl = document.getElementById('robots');
const missionStateEl = document.getElementById('mission-state');
const missionBarEl = document.getElementById('mission-bar');

function renderSide() {
  const f = state.fleet;
  if (!f) return;

  missionStateEl.textContent = f.mission;
  missionBarEl.style.width = `${(f.mission_progress * 100).toFixed(1)}%`;

  const idle = f.mission === 'idle';
  const anyGoal = f.robots.some((r) => r.goal);
  document.getElementById('btn-start').disabled = !idle || !anyGoal;
  document.getElementById('btn-land').disabled = idle;
  document.getElementById('btn-clear').disabled = !idle;

  robotsEl.innerHTML = '';
  for (const r of f.robots) {
    const el = document.createElement('div');
    el.className = 'robot'
      + (r.name === state.selected ? ' selected' : '')
      + (r.connected ? '' : ' offline');
    el.onclick = () => { state.selected = r.name; renderSide(); };

    const color = r.connected ? (PHASE_COLOR[r.phase] || '#8b96a5') : '#5b6674';
    const meta = [];
    meta.push(`x ${r.pos[0].toFixed(2)}  y ${r.pos[1].toFixed(2)}  z ${r.pos[2].toFixed(2)}`);
    // 배터리/RSSI 는 실기(cflib/cpp)에서만 온다. sim 에서는 비워 둔다.
    if (r.battery !== null && r.battery !== undefined) meta.push(`${r.battery.toFixed(2)} V`);
    if (r.rssi !== null && r.rssi !== undefined) meta.push(`${r.rssi} dBm`);
    if (r.dist_to_goal !== null && r.dist_to_goal !== undefined) {
      meta.push(`목표까지 ${r.dist_to_goal.toFixed(2)} m`);
    }

    el.innerHTML = `
      <div class="robot-top">
        <span class="robot-name">
          <span class="dot" style="background:${color}"></span>${r.name}
        </span>
        <span class="robot-phase">${r.connected ? r.phase : '연결 끊김'}</span>
      </div>
      <div class="robot-meta">${meta.map((m) => `<span>${m}</span>`).join('')}</div>
      <div class="bar"><div class="fill" style="width:${(r.progress * 100).toFixed(1)}%;background:${color}"></div></div>
    `;
    robotsEl.appendChild(el);
  }
}

document.getElementById('btn-start').onclick = () => send({ op: 'start' });
document.getElementById('btn-land').onclick = () => send({ op: 'land' });
document.getElementById('btn-clear').onclick = () => send({ op: 'clear_goals' });
document.getElementById('btn-estop').onclick = () => {
  if (confirm('비상 정지 — 모터를 즉시 끕니다. 기체가 떨어집니다. 계속할까요?')) {
    send({ op: 'estop' });
  }
};

resize();
draw();

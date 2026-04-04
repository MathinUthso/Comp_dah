const DEFAULT_BACKEND = 'http://127.0.0.1:8000';
const API_BASE = location.protocol.startsWith('http') ? '' : DEFAULT_BACKEND;
const WS_BASE = location.protocol.startsWith('http')
  ? `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}`
  : 'ws://127.0.0.1:8000';

let isSwapped = false;
let timerInterval = null;
let secondsElapsed = 0;
let wsConnections = [];
let cameraState = {
  CAM_01: null,
  CAM_02: null,
  CAM_03: null,
  CAM_04: null,
  THERMAL: null,
};
let rosStatusInterval = null;
let cameraStatusInterval = null;

function setRosStatus(state, detail = '') {
  const badge = document.getElementById('ros-link-status');
  if (!badge) return;
  badge.classList.remove('ros-online', 'ros-offline', 'ros-degraded');
  if (state === 'online') {
    badge.classList.add('ros-online');
    badge.textContent = 'ROS: ONLINE';
    return;
  }
  if (state === 'degraded') {
    badge.classList.add('ros-degraded');
    badge.textContent = detail ? `ROS: DEGRADED (${detail})` : 'ROS: DEGRADED';
    return;
  }
  badge.classList.add('ros-offline');
  badge.textContent = detail ? `ROS: OFFLINE (${detail})` : 'ROS: OFFLINE';
}

function setCameraStatus(state, detail = '') {
  const badge = document.getElementById('camera-link-status');
  if (!badge) return;
  badge.classList.remove('cam-online', 'cam-offline', 'cam-degraded');
  if (state === 'online') {
    badge.classList.add('cam-online');
    badge.textContent = 'CAM: ONLINE';
    return;
  }
  if (state === 'degraded') {
    badge.classList.add('cam-degraded');
    badge.textContent = detail ? `CAM: DEGRADED (${detail})` : 'CAM: DEGRADED';
    return;
  }
  badge.classList.add('cam-offline');
  badge.textContent = detail ? `CAM: OFFLINE (${detail})` : 'CAM: OFFLINE';
}

function evaluateCameraHealth(metadata) {
  const slots = metadata?.slots || {};
  const thermal = metadata?.thermal || {};
  const slotHealth = Object.values(slots).map(item => !!item?.health?.connected);
  const thermalConnected = !!thermal?.health?.connected;
  const total = slotHealth.length + 1;
  const connected = slotHealth.filter(Boolean).length + (thermalConnected ? 1 : 0);
  if (connected === 0) return { state: 'offline', detail: `0/${total}` };
  if (connected < total) return { state: 'degraded', detail: `${connected}/${total}` };
  return { state: 'online', detail: `${connected}/${total}` };
}

function apiUrl(path) {
  return `${API_BASE}${path}`;
}

async function apiRequest(path, options = {}) {
  const response = await fetch(apiUrl(path), {
    headers: { 'Content-Type': 'application/json', ...(options.headers || {}) },
    ...options,
  });
  if (!response.ok) {
    const text = await response.text();
    throw new Error(`${response.status} ${response.statusText}: ${text}`);
  }
  const contentType = response.headers.get('content-type') || '';
  if (contentType.includes('application/json')) return response.json();
  return response.text();
}

function connectWebSocket(channel, onMessage) {
  const socket = new WebSocket(`${WS_BASE}/ws/${channel}`);
  socket.onopen = () => addLog(`${channel.toUpperCase()} stream connected`, 'info');
  socket.onmessage = event => {
    try {
      onMessage(JSON.parse(event.data));
    } catch (error) {
      addLog(`WS parse error on ${channel}: ${error.message}`, 'err');
    }
  };
  socket.onerror = () => addLog(`${channel.toUpperCase()} stream error`, 'warn');
  socket.onclose = () => {
    addLog(`${channel.toUpperCase()} stream disconnected`, 'warn');
    setTimeout(() => connectWebSocket(channel, onMessage), 2000);
  };
  wsConnections.push(socket);
  return socket;
}

function setupWebSockets() {
  connectWebSocket('logs', payload => {
    if (payload.event) {
      addLog(`[${payload.module}] ${payload.event}`, payload.severity?.toLowerCase() || 'info');
    } else if (payload.message) {
      addLog(String(payload.message), 'info');
    }
  });

  connectWebSocket('state', payload => {
    const state = payload.payload || payload;
    updateStatusPills(state);
  });

  connectWebSocket('camera_metadata', payload => {
    renderCameraMetadata(payload);
  });

  connectWebSocket('ros_graph', payload => {
    renderRosGraph(payload);
    const ok = payload?.ok ?? payload?.status === 'requested';
    setRosStatus(ok ? 'online' : 'degraded');
  });
}

async function pollRosStatus() {
  try {
    const payload = await apiRequest('/api/v1/labview/ros-graph');
    const ok = payload?.ok ?? payload?.status === 'requested';
    if (ok) {
      setRosStatus('online');
    } else {
      setRosStatus('degraded', 'graph');
    }
  } catch (error) {
    setRosStatus('offline', 'api');
  }
}

async function pollCameraStatus() {
  try {
    const payload = await apiRequest('/api/v1/cameras/metadata');
    const health = evaluateCameraHealth(payload);
    setCameraStatus(health.state, health.detail);
  } catch (error) {
    setCameraStatus('offline', 'api');
  }
}

function updateStatusPills(state) {
  const pills = document.querySelectorAll('.cp-status-column .status-pill');
  if (pills[0]) {
    pills[0].textContent = state.ok ? 'OK' : 'FAULT';
    pills[0].className = `status-pill ${state.ok ? 'status-ok' : 'status-fault'}`;
  }
  if (pills[1]) {
    pills[1].textContent = state.autonomous ? 'Autonomous' : 'Manual';
    pills[1].className = `status-pill ${state.autonomous ? 'status-autonomous' : 'status-manual'}`;
  }
  if (pills[2]) {
    pills[2].textContent = `${Number(state.progress || 0)}%`;
    pills[2].className = 'status-pill status-progress';
  }

  const victim = state.victim_detection;
  const crateRed = document.getElementById('crate-red');
  const crateGreen = document.getElementById('crate-green');
  if (crateRed && crateGreen) {
    crateRed.classList.toggle('on', !victim);
    crateGreen.classList.toggle('on', !!victim);
  }

  const railState = state.rail_status || 'idle';
  const railRed = document.getElementById('linear-red');
  const railGreen = document.getElementById('linear-green');
  if (railRed && railGreen) {
    railRed.classList.toggle('on', railState !== 'ready' && railState !== 'ok');
    railGreen.classList.toggle('on', railState === 'ready' || railState === 'ok');
  }

  const omniState = state.omni_status || 'idle';
  const omniRed = document.getElementById('omni-red');
  const omniGreen = document.getElementById('omni-green');
  if (omniRed && omniGreen) {
    omniRed.classList.toggle('on', omniState !== 'ready' && omniState !== 'ok');
    omniGreen.classList.toggle('on', omniState === 'ready' || omniState === 'ok');
  }
}

function renderCameraMetadata(payload) {
  const panel = ensureLabViewPanel();
  const meta = payload.payload || payload;
  const health = evaluateCameraHealth(meta);
  setCameraStatus(health.state, health.detail);
  if (!panel) return;
  const target = panel.querySelector('#labview-camera-metadata');
  if (target) {
    target.textContent = JSON.stringify(meta, null, 2);
  }
}

function renderRosGraph(payload) {
  const panel = ensureLabViewPanel();
  if (!panel) return;
  const target = panel.querySelector('#labview-ros-graph');
  if (target) {
    target.textContent = JSON.stringify(payload, null, 2);
  }
}

function addLog(msg, type = 'info') {
  const container = document.getElementById('persistent-logs');
  if (!container) return;
  const time = new Date().toLocaleTimeString('en-GB', { hour12: false });
  const entry = document.createElement('div');
  entry.className = `log-entry ${type === 'warn' ? 'log-warn' : type === 'err' ? 'log-err' : ''}`;
  entry.textContent = `[${time}] ${msg}`;
  container.appendChild(entry);
  container.scrollTop = container.scrollHeight;
}

function toggleDashboard() {
  const topArea = document.getElementById('top-area');
  const sec2 = document.getElementById('section-2');
  const sec3 = document.getElementById('section-3');
  const mainDisp = document.getElementById('main-display');
  const logContent = document.getElementById('log-dump-content');

  if (!isSwapped) {
    topArea.innerHTML = '<div class="window swap-primary-window"><div class="title-bar"></div><div class="content-area swap-primary-content">3D_VISUALIZATION (PRIMARY)</div></div>';
    topArea.style.display = 'block';
    sec2.classList.remove('span-2');
    sec3.classList.remove('hidden');
    mainDisp.style.background = '#000';
    mainDisp.style.color = '#0f0';
    mainDisp.innerText = 'SELECT FEED';

    logContent.innerHTML = `
      <div class="cam-grid">
        <div class="cam-item" onclick="focusCam(1)">CAM_01</div>
        <div class="cam-item" onclick="focusCam(2)">CAM_02</div>
        <div class="cam-item" onclick="focusCam(3)">CAM_03</div>
        <div class="cam-item" onclick="focusCam(4)">CAM_04</div>
      </div>
    `;
    isSwapped = true;
  } else {
    location.reload();
  }
}

function focusCam(id) {
  const slot = `CAM_0${id}`;
  const main = document.getElementById('main-display');
  if (main) {
    main.innerHTML = `<img src="${apiUrl(`/api/v1/cameras/${slot}/stream.mjpeg`)}" alt="${slot}" style="width:100%;height:100%;object-fit:contain;background:#000;">`;
  }
}

function switchTab(idx) {
  const tabs = document.querySelectorAll('.right-hero .window:last-child .tab');
  const bodies = document.querySelectorAll('.right-hero .window:last-child .tab-body');
  tabs.forEach((t, i) => t.classList.toggle('active', i === idx));
  bodies.forEach((b, i) => b.classList.toggle('active', i === idx));
}

function switchSysTab(evt, tabId) {
  const panes = document.querySelectorAll('.sys-pane');
  panes.forEach(p => p.classList.remove('active'));
  const tabs = document.querySelectorAll('.sys-tab');
  tabs.forEach(t => t.classList.remove('active'));
  const pane = document.getElementById(tabId);
  if (pane) pane.classList.add('active');
  evt.currentTarget.classList.add('active');
}

function toggleInspectMode(mode) {
  document.getElementById('btn-linear').classList.toggle('sub-btn-active', mode === 'linear');
  document.getElementById('btn-omni').classList.toggle('sub-btn-active', mode === 'omni');
  const fields = document.getElementById('inspect-fields');
  fields.style.opacity = '0.5';
  setTimeout(() => {
    fields.style.opacity = '1';
  }, 200);
}

function simulateDrill() {
  const btn = document.getElementById('drill-btn');
  const status = document.getElementById('drill-status');
  btn.innerText = 'DRILLING...';
  btn.style.background = '#f39200';
  status.innerText = 'RPM: 4500 | VIBRATION: HIGH';
  setTimeout(() => {
    btn.innerText = 'ROLL THE ARM (DRILL)';
    btn.style.background = '#ddd';
    status.innerText = 'TASK COMPLETE';
  }, 2000);
}

function startMission() {
  if (timerInterval) return;
  addLog('MISSION SEQUENCE STARTED', 'warn');
  document.getElementById('start-btn').disabled = true;
  document.getElementById('start-btn').style.opacity = '0.5';
  apiRequest('/api/v1/mission/start', { method: 'POST' }).catch(err => addLog(err.message, 'err'));
  timerInterval = setInterval(() => {
    secondsElapsed += 1;
    const mins = Math.floor(secondsElapsed / 60).toString().padStart(2, '0');
    const secs = (secondsElapsed % 60).toString().padStart(2, '0');
    document.getElementById('mission-timer').innerText = `${mins}:${secs}`;
    const progress = document.querySelector('.status-progress');
    if (progress) progress.textContent = `${Math.min(100, secondsElapsed)}%`;
  }, 1000);
}

function resetMission() {
  clearInterval(timerInterval);
  timerInterval = null;
  secondsElapsed = 0;
  document.getElementById('mission-timer').innerText = '00:00';
  document.getElementById('mission-timer').style.color = 'var(--apple-green)';
  document.getElementById('start-btn').disabled = false;
  document.getElementById('start-btn').style.opacity = '1';
  document.getElementById('persistent-logs').innerHTML = '<div>[SYSTEM] LOGS PURGED. READY FOR NEW SESSION.</div>';
  document.getElementById('crate-red').classList.add('on');
  document.getElementById('crate-green').classList.remove('on');
  document.getElementById('linear-red').classList.add('on');
  document.getElementById('linear-green').classList.remove('on');
  document.getElementById('omni-red').classList.add('on');
  document.getElementById('omni-green').classList.remove('on');
  addLog('MASTER RESET PERFORMED', 'err');
  apiRequest('/api/v1/mission/reset', { method: 'POST' }).catch(err => addLog(err.message, 'err'));
}

function estopMission() {
  addLog('E-STOP TRIGGERED', 'err');
  apiRequest('/api/v1/mission/estop', { method: 'POST' }).catch(err => addLog(err.message, 'err'));
}

function wireDpad() {
  const mapping = { 'dp-u': 'up', 'dp-l': 'left', 'dp-r': 'right', 'dp-d': 'down' };
  Object.entries(mapping).forEach(([cls, dir]) => {
    const node = document.querySelector(`.${cls}`);
    if (!node) return;
    node.style.cursor = 'pointer';
    node.addEventListener('click', () => apiRequest(`/api/v1/mission/dpad/${dir}`, { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  });
}

function wireBehaviorButtons() {
  const buttons = Array.from(document.querySelectorAll('.behavior-panel .tab-body .retro-btn'));
  buttons.forEach(btn => {
    const label = (btn.textContent || '').trim();
    btn.style.cursor = 'pointer';
    btn.addEventListener('click', async () => {
      try {
        if (label === 'SLAM') {
          await apiRequest('/api/v1/mission/behavior/start?behavior=SLAM', { method: 'POST' });
        } else if (label === 'Move to waypoint') {
          await apiRequest('/api/v1/mission/behavior/start?behavior=move_to_waypoint', { method: 'POST' });
        } else if (label === 'Explore') {
          await apiRequest('/api/v1/mission/behavior/start?behavior=explore', { method: 'POST' });
        } else if (label === 'Arm') {
          await apiRequest('/api/v1/mission/behavior/start?behavior=arm', { method: 'POST' });
        } else if (label === 'Recalibrate') {
          await apiRequest('/api/v1/mission/behavior/reset?behavior=recalibrate', { method: 'POST' });
        } else if (label === 'Admin System') {
          showLabView();
        }
        addLog(`Behavior command: ${label}`, 'info');
      } catch (error) {
        addLog(error.message, 'err');
      }
    });
  });
}

function renderCameraTiles() {
  const tiles = Array.from(document.querySelectorAll('#top-area .cam-window .cam-content'));
  const slots = ['CAM_01', 'CAM_02', 'CAM_03', 'CAM_04'];
  tiles.forEach((tile, index) => {
    const slot = slots[index];
    tile.innerHTML = `<img src="${apiUrl(`/api/v1/cameras/${slot}/stream.mjpeg`)}" alt="${slot}" style="width:100%;height:100%;object-fit:cover;background:#000;">`;
    tile.dataset.slot = slot;
  });

  const thermal = document.querySelector('.thermal-feed');
  if (thermal) {
    thermal.innerHTML = `<img src="${apiUrl('/api/v1/cameras/THERMAL/stream.mjpeg')}" alt="THERMAL" style="width:100%;height:100%;object-fit:cover;background:#000;">`;
  }

  const main = document.getElementById('main-display');
  if (main) {
    main.innerHTML = '<img src="' + apiUrl('/api/v1/cameras/CAM_01/stream.mjpeg') + '" alt="CAM_01" style="width:100%;height:100%;object-fit:contain;background:#000;">';
  }
}

function ensureLabViewPanel() {
  let panel = document.getElementById('labview-panel');
  if (panel) return panel;
  panel = document.createElement('div');
  panel.id = 'labview-panel';
  panel.style.cssText = 'display:none;position:fixed;inset:40px;background:#111;color:#eee;z-index:2000;padding:16px;overflow:auto;border:2px solid #4caf50;box-shadow:0 0 30px rgba(0,0,0,.6);';
  panel.innerHTML = `
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;">
      <h2 style="margin:0;">Lab_View Engineering Panel</h2>
      <button id="labview-close" class="retro-btn" style="width:auto;">Close</button>
    </div>
    <div style="display:grid;grid-template-columns:1.1fr 1fr;gap:12px;">
      <section style="background:#1b1b1b;padding:12px;border:1px solid #333;">
        <h3>Camera Control</h3>
        <button id="scan-devices" class="retro-btn" style="width:auto;">Detect devices</button>
        <pre id="labview-camera-metadata" style="white-space:pre-wrap;background:#000;padding:8px;min-height:220px;overflow:auto;">Waiting for metadata...</pre>
      </section>
      <section style="background:#1b1b1b;padding:12px;border:1px solid #333;">
        <h3>ROS2 Graph</h3>
        <button id="refresh-ros" class="retro-btn" style="width:auto;">Refresh graph</button>
        <pre id="labview-ros-graph" style="white-space:pre-wrap;background:#000;padding:8px;min-height:220px;overflow:auto;">Waiting for graph...</pre>
      </section>
      <section style="background:#1b1b1b;padding:12px;border:1px solid #333;grid-column:1 / -1;">
        <h3>Engineering Commands</h3>
        <div style="display:flex;gap:8px;flex-wrap:wrap;">
          <button id="restart-cam01" class="retro-btn" style="width:auto;">Restart CAM_01</button>
          <button id="restart-thermal" class="retro-btn" style="width:auto;">Restart THERMAL</button>
          <button id="node-restart" class="retro-btn" style="width:auto;">Restart node</button>
          <button id="node-disable" class="retro-btn" style="width:auto;">Disable node</button>
          <button id="node-enable" class="retro-btn" style="width:auto;">Enable node</button>
        </div>
      </section>
    </div>
  `;
  document.body.appendChild(panel);

  panel.querySelector('#labview-close').addEventListener('click', () => (panel.style.display = 'none'));
  panel.querySelector('#scan-devices').addEventListener('click', async () => {
    const data = await apiRequest('/api/v1/cameras/discover');
    panel.querySelector('#labview-camera-metadata').textContent = JSON.stringify(data, null, 2);
  });
  panel.querySelector('#refresh-ros').addEventListener('click', async () => {
    const data = await apiRequest('/api/v1/labview/ros-graph');
    panel.querySelector('#labview-ros-graph').textContent = JSON.stringify(data, null, 2);
  });
  panel.querySelector('#restart-cam01').addEventListener('click', () => apiRequest('/api/v1/cameras/CAM_01/restart', { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  panel.querySelector('#restart-thermal').addEventListener('click', () => apiRequest('/api/v1/cameras/THERMAL/restart', { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  panel.querySelector('#node-restart').addEventListener('click', () => apiRequest('/api/v1/labview/node/restart?node=dashboard_bridge', { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  panel.querySelector('#node-disable').addEventListener('click', () => apiRequest('/api/v1/labview/node/disable?node=dashboard_bridge', { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  panel.querySelector('#node-enable').addEventListener('click', () => apiRequest('/api/v1/labview/node/enable?node=dashboard_bridge', { method: 'POST' }).catch(err => addLog(err.message, 'err')));
  return panel;
}

function showLabView() {
  const panel = ensureLabViewPanel();
  panel.style.display = 'block';
}

function wireNavigation() {
  const labviewNav = document.getElementById('labview-nav');
  if (labviewNav) labviewNav.addEventListener('click', showLabView);
  const hardwareNav = document.getElementById('hardware-nav');
  if (hardwareNav) hardwareNav.addEventListener('click', () => addLog('Hardware panel opened', 'info'));
}

function wireMissionButtons() {
  document.getElementById('start-btn')?.addEventListener('click', startMission);
  document.querySelector('.mission-reset-btn')?.addEventListener('click', resetMission);
  document.getElementById('estop-btn')?.addEventListener('click', estopMission);
}

function initializeDashboard() {
  setupWebSockets();
  renderCameraTiles();
  wireNavigation();
  wireMissionButtons();
  wireDpad();
  wireBehaviorButtons();
  ensureLabViewPanel();

  const timestamp = document.getElementById('log-timestamp');
  setInterval(() => {
    if (timestamp) timestamp.innerText = new Date().toLocaleTimeString('en-GB', { hour12: false });
  }, 1000);

  setInterval(() => addLog('HEARTBEAT: ALL SYSTEMS NOMINAL'), 5000);

  apiRequest('/api/v1/diagnostics')
    .then(data => addLog(`Diagnostics online: ${Object.keys(data).join(', ')}`, 'info'))
    .catch(err => addLog(err.message, 'err'));

  pollRosStatus();
  if (rosStatusInterval) clearInterval(rosStatusInterval);
  rosStatusInterval = setInterval(pollRosStatus, 5000);

  pollCameraStatus();
  if (cameraStatusInterval) clearInterval(cameraStatusInterval);
  cameraStatusInterval = setInterval(pollCameraStatus, 5000);
}

document.addEventListener('DOMContentLoaded', initializeDashboard);

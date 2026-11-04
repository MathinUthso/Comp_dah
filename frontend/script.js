const DEFAULT_BACKEND = 'http://127.0.0.1:8000';
const API_BASE = location.protocol.startsWith('http') ? '' : DEFAULT_BACKEND;
const CAMERA_DEVICES = (window.RosDashboardConfig && window.RosDashboardConfig.cameraDevices) || {
  video2: '/dev/video2',
  webcam: '0',
  webcamFallback: '/dev/video0',
};
const TELEOP = (window.RosDashboardConfig && window.RosDashboardConfig.teleop) || {
  linearMps: 0.5,
  angularRadps: 1.0,
  boostMultiplier: 1.5,
  streamIntervalMs: 100,
  flipperStepRad: 0.15,
};

let isSwapped = false;
let timerInterval = null;
let secondsElapsed = 0;
let clockInterval = null;
let heartbeatInterval = null;
let cameraStatusInterval = null;
let cameraMetadataInterval = null;
let visionDetectionInterval = null;
let activeSysTabId = 'sys-home';
let lidarLastArrivalMs = 0;
let lidarSmoothedRate = 0;
let lidarLastRenderedTimestamp = 0;
let lidarLastStale = true;
let rosStore = null;
let rosUnsubscribe = null;
let latestRosState = null;
let latestCameraStatus = null;
let latestCameraMetadata = null;
let latestVisionDetection = null;
let latestVisionTile = 'CAM_01';
let cameraPollErrorReported = false;
let cameraMetadataPollErrorReported = false;
let visionDetectionPollErrorReported = false;

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
  if (contentType.includes('application/json')) {
    return response.json();
  }
  return response.text();
}

function setRosStatus(state, detail = '') {
  const badge = document.getElementById('ros-link-status');
  if (!badge) {
    return;
  }
  badge.classList.remove('ros-online', 'ros-offline', 'ros-degraded');
  if (state === 'online') {
    badge.classList.add('ros-online');
    badge.textContent = 'ROS: ONLINE';
    return;
  }
  if (state === 'degraded') {
    badge.classList.add('ros-degraded');
    badge.textContent = detail ? `ROS: RECONNECT (${detail})` : 'ROS: RECONNECT';
    return;
  }
  badge.classList.add('ros-offline');
  badge.textContent = detail ? `ROS: OFFLINE (${detail})` : 'ROS: OFFLINE';
}

function setCameraStatus(state, detail = '') {
  const badge = document.getElementById('camera-link-status');
  if (!badge) {
    return;
  }
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

function setRobotStatus(state, detail = '') {
  const badge = document.getElementById('robot-link-status');
  if (!badge) {
    return;
  }
  badge.classList.remove('robot-online', 'robot-offline', 'robot-degraded', 'robot-estop', 'robot-unknown');
  if (state === 'online') {
    badge.classList.add('robot-online');
    badge.textContent = 'ROBOT: ONLINE';
    return;
  }
  if (state === 'estop') {
    badge.classList.add('robot-estop');
    badge.textContent = 'ROBOT: E-STOP';
    return;
  }
  if (state === 'degraded') {
    badge.classList.add('robot-degraded');
    badge.textContent = detail ? `ROBOT: ${detail}` : 'ROBOT: DEGRADED';
    return;
  }
  if (state === 'unknown') {
    badge.classList.add('robot-unknown');
    badge.textContent = 'ROBOT: N/A';
    return;
  }
  badge.classList.add('robot-offline');
  badge.textContent = detail ? `ROBOT: OFFLINE (${detail})` : 'ROBOT: OFFLINE';
}

function setCameraTileHealth(el, healthy) {
  if (!el) {
    return;
  }
  el.classList.remove('health-good', 'health-bad');
  el.classList.add(healthy ? 'health-good' : 'health-bad');
  el.textContent = healthy ? 'HEALTHY' : 'OFFLINE';
}

function updateCameraHealthUi(health) {
  const video2Healthy = !!health?.video2_healthy;
  const webcamHealthy = !!health?.webcam_healthy;
  const primary = ['video2', 'webcam', 'none'].includes(health?.primary_source) ? health.primary_source : 'none';

  setCameraTileHealth(document.getElementById('video2-health'), video2Healthy);
  setCameraTileHealth(document.getElementById('webcam-health'), webcamHealthy);

  const primaryBadge = document.getElementById('camera-primary-source');
  if (primaryBadge) {
    primaryBadge.textContent = `PRIMARY: ${String(primary).toUpperCase()}`;
  }

  if (primary === 'video2' && video2Healthy) {
    setCameraStatus('online');
  } else if (primary === 'webcam' && webcamHealthy) {
    setCameraStatus('degraded', 'fallback');
  } else if (video2Healthy || webcamHealthy) {
    setCameraStatus('degraded', 'source switching');
  } else {
    setCameraStatus('offline', 'no live camera');
  }
}

function updateCameraRuntimeUi(status, metadata) {
  const bridgeConnected = document.getElementById('camera-bridge-connected');
  if (bridgeConnected) {
    bridgeConnected.textContent = status?.connected ? 'CONNECTED' : 'DISCONNECTED';
    bridgeConnected.classList.toggle('text-green', !!status?.connected);
    bridgeConnected.classList.toggle('text-red', !status?.connected);
  }

  const age = document.getElementById('camera-status-age');
  if (age) {
    if (status?.stale) {
      age.textContent = status?.age_seconds == null ? 'STALE' : `STALE (${formatNumber(status.age_seconds, 1)}s)`;
      age.classList.add('text-red');
      age.classList.remove('text-green');
    } else {
      age.textContent = status?.age_seconds == null ? 'LIVE' : `LIVE (${formatNumber(status.age_seconds, 1)}s)`;
      age.classList.add('text-green');
      age.classList.remove('text-red');
    }
  }

  const thermalDevice = document.getElementById('thermal-device');
  if (thermalDevice) {
    thermalDevice.textContent = metadata?.thermal?.device || '--';
  }

  const video2Device = document.getElementById('video2-device');
  if (video2Device) {
    video2Device.textContent = metadata?.slots?.CAM_01?.device || metadata?.slots?.CAM_02?.device || '--';
  }

  const webcamDevice = document.getElementById('webcam-device');
  if (webcamDevice) {
    webcamDevice.textContent = metadata?.slots?.CAM_03?.device || '--';
  }

  const video2Reconnects = document.getElementById('video2-reconnects');
  if (video2Reconnects) {
    const reconnects = metadata?.slots?.CAM_01?.health?.reconnects ?? metadata?.slots?.CAM_02?.health?.reconnects;
    video2Reconnects.textContent = reconnects == null ? '--' : String(reconnects);
  }

  const webcamReconnects = document.getElementById('webcam-reconnects');
  if (webcamReconnects) {
    const reconnects = metadata?.slots?.CAM_03?.health?.reconnects;
    webcamReconnects.textContent = reconnects == null ? '--' : String(reconnects);
  }
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (!el) {
    return;
  }
  el.textContent = value;
}

function setRosCameraTile(containerId, imageId, dataUrl, stale, label) {
  const container = document.getElementById(containerId);
  if (!container) {
    return;
  }

  let image = document.getElementById(imageId);
  if (dataUrl && (!image || image.tagName !== 'IMG')) {
    container.innerHTML = `<img id="${imageId}" alt="${label}" style="width:100%;height:100%;object-fit:contain;background:#000;display:block;">`;
    image = document.getElementById(imageId);
  }
  if (!dataUrl || !image) {
    return;
  }

  if (image.dataset.rosSrc !== dataUrl) {
    image.dataset.rosSrc = dataUrl;
    image.src = dataUrl;
  }
  image.classList.toggle('stale-data', !!stale);
}

function installMjpegFallback(image, label, timeoutMs = 3000) {
  if (!image) {
    return;
  }
  image.addEventListener('error', () => {
    if (image.dataset.rosSrc) {
      return;
    }
    const container = image.parentElement;
    if (container) {
      container.innerHTML = `<div class="camera-no-signal">${label}</div>`;
    }
  });

  const startedAt = Date.now();
  const timer = setInterval(() => {
    if (image.dataset.rosSrc) {
      clearInterval(timer);
      return;
    }
    if (image.naturalWidth > 0) {
      clearInterval(timer);
      return;
    }
    if (Date.now() - startedAt >= timeoutMs) {
      clearInterval(timer);
      const container = image.parentElement;
      if (container && !image.dataset.rosSrc && image.naturalWidth === 0) {
        container.innerHTML = `<div class="camera-no-signal">${label}</div>`;
      }
    }
  }, 250);
}

function updateRosCameraTiles(state) {
  setRosCameraTile(
    'cam-01-feed',
    'live-video2-feed',
    state.telemetry.cameraVideo2DataUrl,
    state.ui.stale.cameraVideo2,
    'FRONT CAM / NO SIGNAL'
  );
  setRosCameraTile(
    'usb-cam-feed',
    'live-webcam-feed',
    state.telemetry.cameraWebcamDataUrl,
    state.ui.stale.cameraWebcam,
    'REAR CAM / NO SIGNAL'
  );
}

function getCameraPreviewDataUrl(state) {
  const telemetry = state?.telemetry || {};
  // The main preview prefers the third-person simulator camera when available.
  if (telemetry.cameraDataUrl) {
    return telemetry.cameraDataUrl;
  }
  const primary = telemetry?.cameraHealth?.primarySource;
  if (primary === 'video2') {
    return telemetry.cameraVideo2DataUrl || telemetry.cameraWebcamDataUrl || '';
  }
  if (primary === 'webcam') {
    return telemetry.cameraWebcamDataUrl || telemetry.cameraVideo2DataUrl || '';
  }
  return telemetry.cameraVideo2DataUrl || telemetry.cameraWebcamDataUrl || '';
}

function nearestForwardLidarRange(scan) {
  if (!scan || !Array.isArray(scan.ranges) || !Number.isFinite(scan.angleIncrement)) {
    return NaN;
  }
  const halfAngle = Math.PI / 6;
  let nearest = Infinity;
  scan.ranges.forEach((value, index) => {
    const distance = Number(value);
    if (!Number.isFinite(distance) || distance < scan.rangeMin || distance > scan.rangeMax) {
      return;
    }
    const rawAngle = scan.angleMin + index * scan.angleIncrement;
    const angle = Math.atan2(Math.sin(rawAngle), Math.cos(rawAngle));
    if (Math.abs(angle) <= halfAngle) {
      nearest = Math.min(nearest, distance);
    }
  });
  return Number.isFinite(nearest) ? nearest : NaN;
}

function drawLidarCanvas(canvas, scan, stale) {
  if (!canvas) {
    return;
  }

  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) {
    return;
  }

  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = '#050b08';
  ctx.fillRect(0, 0, rect.width, rect.height);

  const centerX = rect.width / 2;
  const centerY = rect.height / 2;
  const maxRange = Math.max(1, Math.min(20, Number(scan?.rangeMax) || 12));
  const scale = (Math.min(rect.width, rect.height) * 0.45) / maxRange;

  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(70, 255, 150, 0.22)';
  [0.25, 0.5, 0.75, 1].forEach((fraction) => {
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRange * scale * fraction, 0, Math.PI * 2);
    ctx.stroke();
  });

  ctx.strokeStyle = 'rgba(70, 255, 150, 0.16)';
  ctx.beginPath();
  ctx.moveTo(centerX - maxRange * scale, centerY);
  ctx.lineTo(centerX + maxRange * scale, centerY);
  ctx.moveTo(centerX, centerY - maxRange * scale);
  ctx.lineTo(centerX, centerY + maxRange * scale);
  ctx.stroke();

  ctx.fillStyle = 'rgba(70, 255, 150, 0.55)';
  ctx.font = '8px "Courier New", monospace';
  ctx.textAlign = 'center';
  ctx.fillText('FWD', centerX, Math.max(10, centerY - maxRange * scale - 5));

  let validCount = 0;
  if (scan && Array.isArray(scan.ranges) && !stale) {
    scan.ranges.forEach((value, index) => {
      const distance = Number(value);
      if (!Number.isFinite(distance) || distance < scan.rangeMin || distance > scan.rangeMax) {
        return;
      }
      validCount += 1;
      const angle = scan.angleMin + index * scan.angleIncrement;
      // ROS: +X forward, +Y left. Canvas: forward is up, left is west.
      const x = centerX - Math.sin(angle) * distance * scale;
      const y = centerY - Math.cos(angle) * distance * scale;
      const normalized = Math.min(1, distance / maxRange);
      ctx.fillStyle = `hsl(${Math.round(130 - normalized * 130)}, 95%, 58%)`;
      ctx.fillRect(x - 1, y - 1, 2, 2);
    });
  }

  ctx.fillStyle = '#ffb000';
  ctx.beginPath();
  ctx.moveTo(centerX, centerY - 5);
  ctx.lineTo(centerX - 4, centerY + 4);
  ctx.lineTo(centerX + 4, centerY + 4);
  ctx.closePath();
  ctx.fill();
}

function renderLidar(scan, stale) {
  const canvases = Array.from(document.querySelectorAll('.lidar-canvas'));
  if (canvases.length === 0) {
    const fallback = document.getElementById('lidar-canvas');
    if (fallback) {
      canvases.push(fallback);
    }
  }
  canvases.forEach((canvas) => drawLidarCanvas(canvas, scan, stale));

  const nearest = nearestForwardLidarRange(scan);
  setText('lidar-point-count', scan ? `${canvases.length ? (scan.ranges.filter((value) => Number.isFinite(Number(value)) && Number(value) >= scan.rangeMin && Number(value) <= scan.rangeMax).length) : 0}/${scan.ranges.length}` : '--');
  setText('lidar-nearest', Number.isFinite(nearest) ? `${formatNumber(nearest, 2)} m` : '--');
  setText('lidar-rate', lidarSmoothedRate > 0 ? `${formatNumber(lidarSmoothedRate, 1)} Hz` : '--');
  setText('lidar-frame', scan?.frameId || '--');
  setText('footer-lidar-nearest', Number.isFinite(nearest) ? `${formatNumber(nearest, 2)} m` : '--');

  const status = document.getElementById('lidar-status');
  if (status) {
    status.textContent = scan && !stale ? 'LIVE /scan' : 'STALE / NO SCAN';
    status.classList.toggle('stale', !scan || stale);
  }
}

function updateLidarState(state) {
  const timestamp = state.timestamps.lidar;
  const stale = state.ui.stale.lidar;
  const hasNewScan = timestamp !== lidarLastRenderedTimestamp;
  if (hasNewScan) {
    const now = performance.now();
    if (lidarLastArrivalMs > 0) {
      const dt = (now - lidarLastArrivalMs) / 1000;
      if (dt > 0 && dt < 2) {
        const instant = 1 / dt;
        lidarSmoothedRate = lidarSmoothedRate > 0 ? lidarSmoothedRate * 0.85 + instant * 0.15 : instant;
      }
    }
    lidarLastArrivalMs = now;
    lidarLastRenderedTimestamp = timestamp;
  }

  if (hasNewScan || stale !== lidarLastStale) {
    lidarLastStale = stale;
    renderLidar(state.telemetry.lidar, stale);
  }
}

function updateRightTabRuntimeFields(state) {
  const temperature = Number(state?.telemetry?.thermal?.temperature);
  const proximity = Number(state?.telemetry?.proximity?.range);
  const cameraStale = !!state?.ui?.stale?.camera;
  const cameraHealthy = !cameraStale && !!getCameraPreviewDataUrl(state);

  let hazmat = 'CLEAR';
  if (Number.isFinite(temperature) && temperature >= 55) {
    hazmat = 'DETECTED (HEAT)';
  } else if (Number.isFinite(temperature) && temperature >= 40) {
    hazmat = 'ELEVATED';
  }

  const pressure = Number.isFinite(proximity) ? `${formatNumber(101.3 + (0.5 - Math.min(Math.max(proximity, 0), 1.0)), 2)} kPa` : '--';
  const acuity = cameraHealthy ? 'VISUAL_OK' : 'DEGRADED';
  const qr = latestVisionDetection?.target_id || '--';
  const motion = latestVisionDetection?.detected ? 'DETECTED' : 'NO MOTION';
  const box = latestVisionDetection?.box
    ? `${latestVisionDetection.box.x.toFixed(2)},${latestVisionDetection.box.y.toFixed(2)} ${latestVisionDetection.box.w.toFixed(2)}x${latestVisionDetection.box.h.toFixed(2)}`
    : '--';

  setText('inspect-hazmat', hazmat);
  setText('inspect-pressure', pressure);
  setText('inspect-acuity', acuity);
  setText('inspect-qr', qr);

  setText('insert-target-lock', latestVisionDetection?.detected ? 'LOCKED' : 'SEARCHING');
  setText('insert-box', box);

  setText('crate-motion', motion);
  setText('crate-hazmat', hazmat);
  setText('crate-acuity', acuity);
  setText('crate-qr', qr);
  
  // Highlight motion row when detection is active
  const motionRow = document.querySelector('.motion-row');
  if (motionRow) {
    if (latestVisionDetection?.detected) {
      motionRow.classList.add('detection-active');
    } else {
      motionRow.classList.remove('detection-active');
    }
  }
}

function updateVisionOverlay() {
  const previewShell = document.getElementById('camera-preview-shell');
  const statusChip = document.getElementById('camera-tab-detection-status');
  const overlays = [
    document.getElementById('vision-overlay-box'),
    document.getElementById('vision-overlay-inspect'),
    document.getElementById('vision-overlay-insert'),
    document.getElementById('camera-tab-vision-overlay'),
  ];

  const detected = !!latestVisionDetection?.detected && !!latestVisionDetection?.box;
  const summary = detected
    ? `${latestVisionDetection.box.gesture || latestVisionDetection.target_id || 'DETECTED'} ${latestVisionDetection.box.confidence ? `${(latestVisionDetection.box.confidence * 100).toFixed(0)}%` : ''}`.trim()
    : 'SEARCHING';

  if (previewShell) {
    previewShell.classList.toggle('detection-active', detected);
  }
  if (statusChip) {
    statusChip.textContent = detected ? `MEDIAPIPE: ${latestVisionTile} ${summary}` : `MEDIAPIPE: ${latestVisionTile} SEARCHING`;
  }

  setCameraTabPreviewTile(latestVisionTile);
  
  overlays.forEach(overlay => {
    if (!overlay) {
      return;
    }
    if (!detected) {
      overlay.classList.add('hidden');
      return;
    }

    const box = latestVisionDetection.box;
    overlay.style.left = `${Math.max(0, Math.min(1, box.x)) * 100}%`;
    overlay.style.top = `${Math.max(0, Math.min(1, box.y)) * 100}%`;
    overlay.style.width = `${Math.max(0, Math.min(1, box.w)) * 100}%`;
    overlay.style.height = `${Math.max(0, Math.min(1, box.h)) * 100}%`;
    overlay.classList.remove('hidden');

    // Update gesture label
    const label = overlay.querySelector('.vision-overlay-label');
    if (label) {
      const gesture = box.gesture || latestVisionDetection.target_id || 'DETECTED';
      const confidence = box.confidence ? `${(box.confidence * 100).toFixed(0)}%` : '';
      label.textContent = confidence ? `${gesture} ${confidence}` : gesture;
    }
  });
}

function addLog(msg, type = 'info') {
  const container = document.getElementById('persistent-logs');
  if (!container) {
    return;
  }
  const time = new Date().toLocaleTimeString('en-GB', { hour12: false });
  const entry = document.createElement('div');
  entry.className = `log-entry ${type === 'warn' ? 'log-warn' : type === 'err' ? 'log-err' : ''}`;
  entry.textContent = `[${time}] ${msg}`;
  container.appendChild(entry);
  container.scrollTop = container.scrollHeight;
}

function formatNumber(value, digits = 2) {
  const num = Number(value);
  if (!Number.isFinite(num)) {
    return '--';
  }
  return num.toFixed(digits);
}

function updateDataText(id, text, stale) {
  const el = document.getElementById(id);
  if (!el) {
    return;
  }
  el.textContent = text;
  el.classList.toggle('stale-data', !!stale);
}

function applyRosState(state) {
  if (!state) {
    return;
  }
  latestRosState = state;

  const connectionStatus = state.connection.status;
  if (connectionStatus === 'connected') {
    setRosStatus('online');
  } else if (connectionStatus === 'connecting' || connectionStatus === 'reconnecting') {
    setRosStatus('degraded', state.connection.detail || `#${state.connection.reconnectAttempt}`);
  } else {
    setRosStatus('offline', state.connection.detail || 'down');
  }

  const proximity = state.telemetry.proximity;
  const proximityStale = state.ui.stale.proximity;
  const distance = proximity ? proximity.range : NaN;
  const warning = Number.isFinite(distance) && distance < 0.30 && !proximityStale;
  updateDataText('obstacle-distance', `${formatNumber(distance, 3)} m`, proximityStale);
  const obstacleEl = document.getElementById('obstacle-distance');
  if (obstacleEl) {
    obstacleEl.classList.toggle('text-red', warning);
  }
  const marker = document.querySelector('.proximity-marker');
  if (marker && Number.isFinite(distance)) {
    const normalized = Math.max(0, Math.min(1, distance / 2.0));
    marker.style.left = `${Math.round(normalized * 100)}%`;
  }

  const thermal = state.telemetry.thermal;
  const thermalTemp = thermal ? thermal.temperature : NaN;
  updateDataText('temperature-value', `${formatNumber(thermalTemp, 1)} C`, state.ui.stale.thermal);
  const gauge = document.getElementById('temperature-gauge-fill');
  if (gauge) {
    const clamped = Math.max(0, Math.min(100, Number(thermalTemp)));
    gauge.style.width = `${clamped}%`;
  }

  const odom = state.telemetry.odometry;
  updateDataText(
    'position-value',
    odom ? `x: ${formatNumber(odom.x)} y: ${formatNumber(odom.y)}` : 'x: -- y: --',
    state.ui.stale.odometry
  );
  updateDataText('heading-value', odom ? `${formatNumber(odom.headingDeg, 1)} deg` : '-- deg', state.ui.stale.odometry);
  updateDataText(
    'velocity-value',
    odom ? `lin: ${formatNumber(odom.linearX)} ang: ${formatNumber(odom.angularZ)}` : 'lin: -- ang: --',
    state.ui.stale.odometry
  );

  const imu = state.telemetry.imu;
  void imu;

  const diagnostics = state.telemetry.diagnostics;
  const diagStale = state.ui.stale.diagnostics;
  updateDataText(
    'diag-battery',
    diagnostics ? `${diagnostics.battery.message} | ${diagnostics.battery.value}` : '--',
    diagStale
  );
  updateDataText('diag-cpu', diagnostics ? `${diagnostics.cpu.message} | ${diagnostics.cpu.value}` : '--', diagStale);
  updateDataText(
    'diag-memory',
    diagnostics ? `${diagnostics.memory.message} | ${diagnostics.memory.value}` : '--',
    diagStale
  );
  updateDataText('diag-robot-link', diagnostics ? diagnostics.robotLink.message : '--', diagStale);
  updateDataText('diag-drive', diagnostics ? diagnostics.drive.message : '--', diagStale);

  if (state.telemetry.emergency || (diagnostics && diagnostics.drive.estop)) {
    setRobotStatus('estop');
  } else if (diagStale || !diagnostics) {
    setRobotStatus('offline', 'no diagnostics');
  } else if (!diagnostics.robotLink.present) {
    setRobotStatus('unknown');
  } else if (diagnostics.robotLink.level === 0 && diagnostics.robotLink.connected) {
    setRobotStatus('online');
  } else if (diagnostics.robotLink.connected) {
    setRobotStatus('degraded', 'NO TELEMETRY');
  } else {
    setRobotStatus('offline', 'serial down');
  }

  const emergencyActive = !!state.telemetry.emergency;
  const banner = document.getElementById('emergency-banner');
  if (banner) {
    banner.classList.toggle('hidden', !emergencyActive);
  }
  const estopButton = document.getElementById('estop-btn');
  if (estopButton) {
    estopButton.disabled = !!state.actions.estopPending;
    estopButton.classList.toggle('disabled-btn', !!state.actions.estopPending);
  }

  const behavior = state.telemetry.flexbeStatus || state.actions.behaviorLabel || 'idle';
  updateDataText('active-behavior', String(behavior).toUpperCase(), state.ui.stale.flexbeStatus);

  const preview = document.getElementById('camera-preview');
  const previewDataUrl = getCameraPreviewDataUrl(state);
  if (preview && previewDataUrl) {
    const nextSrc = previewDataUrl;
    if (preview.dataset.rosSrc !== nextSrc) {
      preview.dataset.rosSrc = nextSrc;
      preview.src = nextSrc;
    }
    preview.classList.toggle('stale-data', state.ui.stale.camera);
  }

  if (!isSwapped) {
    const mainDisplay = document.getElementById('main-display');
    const mainDataUrl = state.telemetry.cameraDataUrl;
    if (mainDisplay && mainDataUrl) {
      if (mainDisplay.dataset.rosSrc !== mainDataUrl) {
        mainDisplay.dataset.rosSrc = mainDataUrl;
        mainDisplay.innerHTML = `<img src="${mainDataUrl}" alt="THIRD_PERSON" style="width:100%;height:100%;object-fit:contain;background:#000;">`;
        mainDisplay.classList.remove('main-display-default');
      }
    }
  }

  updateRosCameraTiles(state);
  updateLidarState(state);

  if (!state.ui.stale.cameraHealth) {
    updateCameraHealthUi({
      video2_healthy: state.telemetry.cameraHealth.video2Healthy,
      webcam_healthy: state.telemetry.cameraHealth.webcamHealthy,
      primary_source: state.telemetry.cameraHealth.primarySource,
    });
  } else if (latestCameraStatus) {
    updateCameraHealthUi(latestCameraStatus);
  } else {
    setCameraStatus('degraded', 'waiting for camera bridge');
  }

  updateCameraRuntimeUi(latestCameraStatus, latestCameraMetadata);
  updateRightTabRuntimeFields(state);
  updateVisionOverlay();

  if (state.ui.lastError) {
    const previous = window.__lastRosError;
    if (previous !== state.ui.lastError) {
      addLog(`ROS error: ${state.ui.lastError}`, 'err');
      window.__lastRosError = state.ui.lastError;
    }
  }
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
  panes.forEach((p) => p.classList.remove('active'));
  const tabs = document.querySelectorAll('.sys-tab');
  tabs.forEach((t) => t.classList.remove('active'));
  const pane = document.getElementById(tabId);
  if (pane) {
    pane.classList.add('active');
  }
  evt.currentTarget.classList.add('active');
  activeSysTabId = tabId;
  if (tabId === 'sys-lidar' && latestRosState) {
    renderLidar(latestRosState.telemetry.lidar, latestRosState.ui.stale.lidar);
  }
}

function toggleInspectMode(mode) {
  document.getElementById('btn-linear').classList.toggle('sub-btn-active', mode === 'linear');
  document.getElementById('btn-omni').classList.toggle('sub-btn-active', mode === 'omni');
  const fields = document.getElementById('inspect-fields');
  if (!fields) {
    return;
  }
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
  if (timerInterval) {
    return;
  }
  addLog('MISSION SEQUENCE STARTED', 'warn');
  document.getElementById('start-btn').disabled = true;
  document.getElementById('start-btn').style.opacity = '0.5';
  apiRequest('/api/v1/mission/start', { method: 'POST' }).catch((err) => addLog(err.message, 'err'));
  timerInterval = setInterval(() => {
    secondsElapsed += 1;
    const mins = Math.floor(secondsElapsed / 60)
      .toString()
      .padStart(2, '0');
    const secs = (secondsElapsed % 60).toString().padStart(2, '0');
    document.getElementById('mission-timer').innerText = `${mins}:${secs}`;
    const progress = document.querySelector('.status-progress');
    if (progress) {
      progress.textContent = `${Math.min(100, secondsElapsed)}%`;
    }
  }, 1000);
}

function resetMission() {
  clearInterval(timerInterval);
  timerInterval = null;
  secondsElapsed = 0;
  document.getElementById('mission-timer').innerText = '00:00';
  document.getElementById('start-btn').disabled = false;
  document.getElementById('start-btn').style.opacity = '1';
  document.getElementById('persistent-logs').innerHTML = '<div>[SYSTEM] LOGS PURGED. READY FOR NEW SESSION.</div>';
  addLog('MASTER RESET PERFORMED', 'err');
  apiRequest('/api/v1/mission/reset', { method: 'POST' }).catch((err) => addLog(err.message, 'err'));
}

async function estopMission() {
  if (!rosStore) {
    addLog('E-STOP unavailable: ROS store not initialized', 'err');
    return;
  }
  addLog('E-STOP REQUESTED', 'warn');
  stopKeyboardMovement();
  try {
    const response = await rosStore.triggerEstop();
    addLog(`E-STOP response: ${JSON.stringify(response)}`, 'info');
  } catch (error) {
    addLog(`E-STOP failed: ${error.message}`, 'err');
  }
}

const keyboardMovementKeys = new Set();
const MOVEMENT_KEYS = new Set(["arrowup", "arrowdown", "arrowleft", "arrowright", "w", "a", "s", "d"]);
let keyboardBoostHeld = false;

function isMovementKey(key) {
  return MOVEMENT_KEYS.has(String(key || "").toLowerCase());
}

function keyboardCommandFromPressed() {
  const boost = keyboardBoostHeld ? Number(TELEOP.boostMultiplier) || 1 : 1;
  const linearStep = (Number(TELEOP.linearMps) || 0.5) * boost;
  const angularStep = (Number(TELEOP.angularRadps) || 1.0) * boost;
  const pressed = (...keys) => keys.some((key) => keyboardMovementKeys.has(key));

  let linear = 0;
  let angular = 0;
  if (pressed("arrowup", "w")) {
    linear += linearStep;
  }
  if (pressed("arrowdown", "s")) {
    linear -= linearStep;
  }
  if (pressed("arrowleft", "a")) {
    angular += angularStep;
  }
  if (pressed("arrowright", "d")) {
    angular -= angularStep;
  }
  return { linear, angular };
}

// Real robots need a command stream, not a single message per key press: the
// drive bridge zeroes /cmd_vel when it stops arriving. Re-publish while held.
const MOVE_STREAM_INTERVAL_MS = Math.max(20, Number(TELEOP.streamIntervalMs) || 100);
let movementStreamTimer = null;
let movementStreamPublisher = null;

function publishSafely(action, label) {
  if (!rosStore) {
    return;
  }
  try {
    action();
  } catch (error) {
    addLog(`${label}: ${error.message}`, "err");
  }
}

function startMovementStream(publisher) {
  stopMovementStream(false);
  movementStreamPublisher = publisher;
  publishSafely(publisher, "cmd_vel publish failed");
  movementStreamTimer = setInterval(() => publishSafely(movementStreamPublisher, "cmd_vel publish failed"), MOVE_STREAM_INTERVAL_MS);
}

function stopMovementStream(sendStop = true) {
  if (movementStreamTimer !== null) {
    clearInterval(movementStreamTimer);
    movementStreamTimer = null;
  }
  movementStreamPublisher = null;
  if (sendStop) {
    publishSafely(() => rosStore.stopCmdVel(), "cmd_vel stop failed");
  }
}

function publishKeyboardCommand() {
  const command = keyboardCommandFromPressed();
  publishSafely(() => rosStore.publishCmdVel(command.linear, command.angular), "keyboard cmd_vel publish failed");
}

function ensureKeyboardStream() {
  if (movementStreamTimer === null) {
    startMovementStream(publishKeyboardCommand);
  }
}

function isTypingTarget(target) {
  if (!target) {
    return false;
  }
  const tag = String(target.tagName || "").toLowerCase();
  return tag === "input" || tag === "textarea" || tag === "select" || target.isContentEditable;
}

function handleKeyboardMovement(event) {
  if (!rosStore || isTypingTarget(event.target)) {
    return;
  }

  if (event.key === "Escape") {
    if (event.type === "keydown") {
      event.preventDefault();
      stopKeyboardMovement();
      addLog("Keyboard stop requested", "warn");
    }
    return;
  }

  if (event.key === "Shift") {
    keyboardBoostHeld = event.type === "keydown";
    if (keyboardMovementKeys.size > 0) {
      publishKeyboardCommand();
    }
    return;
  }

  const key = String(event.key || "").toLowerCase();
  if (!isMovementKey(key)) {
    return;
  }
  event.preventDefault();

  if (event.type === "keydown") {
    if (event.repeat) {
      return;
    }
    keyboardMovementKeys.add(key);
  } else {
    keyboardMovementKeys.delete(key);
  }
  if (keyboardMovementKeys.size === 0) {
    stopKeyboardMovement();
    return;
  }
  ensureKeyboardStream();
}

function stopKeyboardMovement() {
  keyboardMovementKeys.clear();
  keyboardBoostHeld = false;
  stopMovementStream();
}

const FLIPPER_MIN_RAD = -1.5708; // matches the URDF flipper joint limits: 90 deg down
const FLIPPER_MAX_RAD = 1.5708; // 90 deg up
const FLIPPER_STEP_RAD = Number(TELEOP.flipperStepRad) || 0.15; // one key press ~= 8.6 degrees
const FLIPPER_JOINTS = [
  "front_left_flipper_joint",
  "front_right_flipper_joint",
  "rear_left_flipper_joint",
  "rear_right_flipper_joint",
];
const flipperTargets = {};

function clampFlipperAngle(value) {
  return Math.min(FLIPPER_MAX_RAD, Math.max(FLIPPER_MIN_RAD, value));
}

// Step from the last commanded angle so rapid presses accumulate instead of being
// swallowed; fall back to the live joint angle so the first press starts where the
// flipper actually is.
function nextFlipperTarget(joint, direction) {
  const live = rosStore && rosStore.state ? Number(rosStore.state.flippers[joint]) : NaN;
  const commanded = flipperTargets[joint];
  const base = Number.isFinite(commanded) ? commanded : live;
  const next = clampFlipperAngle((Number.isFinite(base) ? base : 0) + direction * FLIPPER_STEP_RAD);
  flipperTargets[joint] = next;
  return next;
}

function handleFlipperKeys(event) {
  if (!rosStore || isTypingTarget(event.target)) {
    return;
  }
  const key = String(event.key || "").toLowerCase();
  if (!["q", "e", "z", "c", "t"].includes(key)) {
    return;
  }
  event.preventDefault();

  if (event.type !== "keydown" || event.repeat) {
    return;
  }

  if (key === "t") {
    // T = dog stance (all four straight down), Shift+T = back to normal.
    const position = event.shiftKey ? 0 : FLIPPER_MIN_RAD;
    try {
      const targets = FLIPPER_JOINTS.map((joint) => {
        flipperTargets[joint] = position;
        return position;
      });
      rosStore.publishFlipperCommand(FLIPPER_JOINTS, targets);
      addLog(event.shiftKey ? "Normal stance: flippers back to neutral" : "Dog stance: all flippers 90 deg down", "info");
    } catch (error) {
      addLog(`flipper command failed: ${error.message}`, "err");
    }
    return;
  }

  const isFront = key === "q" || key === "e";
  const isLeftKey = key === "q" || key === "z";
  const leftJoint = isFront ? "front_left_flipper_joint" : "rear_left_flipper_joint";
  const rightJoint = isFront ? "front_right_flipper_joint" : "rear_right_flipper_joint";

  let down = event.ctrlKey;
  if (event.shiftKey && !isLeftKey) {
    down = true;
  }
  if (event.shiftKey && isLeftKey) {
    down = false;
  }
  const direction = down ? -1 : 1;

  const joints = event.shiftKey ? [leftJoint, rightJoint] : isLeftKey ? [leftJoint] : [rightJoint];

  try {
    const targets = joints.map((joint) => nextFlipperTarget(joint, direction));
    rosStore.publishFlipperCommand(joints, targets);
    const detail = joints.map((joint, index) => `${joint.replace("_flipper_joint", "")}=${targets[index].toFixed(2)}`).join(", ");
    addLog(`Flipper ${detail}`, "info");
  } catch (error) {
    addLog(`flipper command failed: ${error.message}`, "err");
  }
}

function wireKeyboardControls() {
  if (!rosStore) {
    return;
  }
  window.addEventListener("keydown", handleKeyboardMovement);
  window.addEventListener("keyup", handleKeyboardMovement);
  window.addEventListener("keydown", handleFlipperKeys);
  window.addEventListener("blur", stopKeyboardMovement);
}

function wireDpad() {
  if (!rosStore) {
    return;
  }
  const linear = Number(TELEOP.linearMps) || 0.5;
  const angular = Number(TELEOP.angularRadps) || 1.0;
  const mapping = {
    'dp-u': { linear, angular: 0 },
    'dp-d': { linear: -linear, angular: 0 },
    'dp-l': { linear: 0, angular },
    'dp-r': { linear: 0, angular: -angular },
  };

  Object.entries(mapping).forEach(([cls, cmd]) => {
    const node = document.querySelector(`.${cls}`);
    if (!node) {
      return;
    }
    node.style.cursor = 'pointer';

    const start = (event) => {
      event.preventDefault();
      startMovementStream(() => rosStore.publishCmdVel(cmd.linear, cmd.angular));
    };

    const stop = (event) => {
      event.preventDefault();
      stopMovementStream();
    };

    node.addEventListener('pointerdown', start);
    node.addEventListener('pointerup', stop);
    node.addEventListener('pointerleave', stop);
    node.addEventListener('touchstart', start, { passive: false });
    node.addEventListener('touchend', stop, { passive: false });
  });
}

function wireBehaviorButtons() {
  if (!rosStore) {
    return;
  }
  const buttons = Array.from(document.querySelectorAll('.behavior-btn'));
  buttons.forEach((btn) => {
    btn.style.cursor = 'pointer';
    btn.addEventListener('click', async () => {
      const behaviorId = Number(btn.getAttribute('data-behavior-id'));
      try {
        const response = await rosStore.executeBehavior(behaviorId);
        addLog(`Behavior ${behaviorId} sent: ${JSON.stringify(response)}`, 'info');
      } catch (error) {
        addLog(`Behavior failed: ${error.message}`, 'err');
      }
    });
  });
}

function loadMjpegTiles() {
  // Load deferred MJPEG streams for <img> elements that were left without src.
  document.querySelectorAll('img[data-mjpeg-src]').forEach((image) => {
    if (image.getAttribute('src')) {
      return;
    }
    const src = image.getAttribute('data-mjpeg-src');
    const fallback = image.getAttribute('data-mjpeg-fallback');
    const label = image.getAttribute('data-mjpeg-label') || 'NO SIGNAL';
    if (!src) {
      return;
    }
    let triedFallback = false;
    image.addEventListener('error', () => {
      if (image.dataset.rosSrc) {
        return;
      }
      if (fallback && !triedFallback) {
        triedFallback = true;
        image.src = fallback;
        return;
      }
      const container = image.parentElement;
      if (container) {
        container.innerHTML = `<div class="camera-no-signal">${label}</div>`;
      }
    });
    installMjpegFallback(image, label);
    image.src = src;
  });

  const containers = ['cam-01-feed', 'usb-cam-feed', 'cam-03-feed', 'cam-04-feed'];
  containers.forEach((containerId) => {
    const container = document.getElementById(containerId);
    if (!container) {
      return;
    }
    const placeholder = container.querySelector('[data-mjpeg-src]');
    if (!placeholder) {
      return;
    }

    const tileId = placeholder.getAttribute('data-tile') || 'mjpeg-feed';
    const src = placeholder.getAttribute('data-mjpeg-src');
    const fallback = placeholder.getAttribute('data-mjpeg-fallback');
    const label = placeholder.getAttribute('data-label') || 'NO SIGNAL';

    // Prefer ROS compressed images when they are already available.
    const hasRosData =
      (containerId === 'cam-01-feed' && !!latestRosState?.telemetry?.cameraVideo2DataUrl) ||
      (containerId === 'usb-cam-feed' && !!latestRosState?.telemetry?.cameraWebcamDataUrl);
    if (hasRosData) {
      return;
    }

    container.innerHTML = `<img id="${tileId}" data-mjpeg-src="${src}" src="${src}" alt="${label}" loading="lazy" style="width:100%;height:100%;object-fit:contain;background:#000;display:block;">`;
    const image = document.getElementById(tileId);
    let triedFallback = false;
    if (fallback) {
      image?.addEventListener('error', () => {
        if (image.dataset.rosSrc) {
          return;
        }
        if (!triedFallback) {
          triedFallback = true;
          image.src = fallback;
          return;
        }
        const parent = image.parentElement;
        if (parent) {
          parent.innerHTML = `<div class="camera-no-signal">${label}</div>`;
        }
      });
    }
    installMjpegFallback(image, label);
  });
}

function renderCameraTiles() {
  const cam01 = document.getElementById('cam-01-feed');
  const usb = document.getElementById('usb-cam-feed');
  const cam03 = document.getElementById('cam-03-feed');
  const cam04 = document.getElementById('cam-04-feed');
  const armFeedInspect = document.getElementById('arm-feed-inspect');
  const armFeedInsert = document.getElementById('arm-feed-insert');
  const armFeedCrate = document.getElementById('arm-feed-crate');
  const armFeedTouch = document.getElementById('arm-feed-touch');

  if (cam01) {
    const video2Primary = apiUrl(`/api/v1/cameras/usb/stream.mjpeg?device=${encodeURIComponent(CAMERA_DEVICES.video2)}`);
    cam01.innerHTML = `<div class="camera-no-signal" data-mjpeg-src="${video2Primary}" data-tile="live-video2-feed" data-label="FRONT CAM / NO SIGNAL">LOADING FRONT CAM...</div>`;
  }
  if (usb) {
    const webcamPrimary = apiUrl(`/api/v1/cameras/usb/stream.mjpeg?device=${encodeURIComponent(CAMERA_DEVICES.webcam)}`);
    const webcamFallback = apiUrl(`/api/v1/cameras/usb/stream.mjpeg?device=${encodeURIComponent(CAMERA_DEVICES.webcamFallback)}`);
    usb.innerHTML = `<div class="camera-no-signal" data-mjpeg-src="${webcamPrimary}" data-mjpeg-fallback="${webcamFallback}" data-tile="live-webcam-feed" data-label="REAR CAM / NO SIGNAL">LOADING REAR CAM...</div>`;
  }
  if (cam03) {
    cam03.innerHTML = '<canvas id="lidar-canvas-cam03" class="lidar-canvas" aria-label="Lidar map" style="width:100%;height:100%;display:block;background:#050b08;"></canvas>';
  }
  if (cam04) {
    const cam04Src = apiUrl('/api/v1/cameras/CAM_04/stream.mjpeg');
    cam04.innerHTML = `<div class="camera-no-signal" data-mjpeg-src="${cam04Src}" data-tile="cam04-img" data-label="CAM_04 / NO SIGNAL">LOADING CAM_04...</div>`;
  }

  const armFeedPrimary = apiUrl(`/api/v1/cameras/usb/stream.mjpeg?device=${encodeURIComponent(CAMERA_DEVICES.video2)}`);
  const armFeedFallback = apiUrl('/api/v1/cameras/CAM_01/stream.mjpeg');
  [armFeedInspect, armFeedInsert, armFeedCrate, armFeedTouch].forEach((img) => {
    if (!img) {
      return;
    }
    img.removeAttribute('src');
    img.setAttribute('data-mjpeg-src', armFeedPrimary);
    img.setAttribute('data-mjpeg-fallback', armFeedFallback);
    img.setAttribute('data-mjpeg-label', 'ARM FEED / NO SIGNAL');
  });

  const main = document.getElementById('main-display');
  if (main) {
    const mainSrc = apiUrl('/api/v1/cameras/CAM_01/stream.mjpeg');
    main.innerHTML = `<div class="camera-no-signal" data-mjpeg-src="${mainSrc}" data-tile="main-display-img" data-label="MAIN VIEW / NO SIGNAL">LOADING MAIN VIEW...</div>`;
  }
}

async function pollCameraStatus() {
  try {
    const payload = await apiRequest('/api/v1/cameras/status');
    latestCameraStatus = payload;
    cameraPollErrorReported = false;
    updateCameraHealthUi(payload);
  } catch (error) {
    if (!cameraPollErrorReported) {
      addLog(`camera status poll failed: ${error.message}`, 'err');
      cameraPollErrorReported = true;
    }
  }
}

async function pollCameraMetadata() {
  try {
    const payload = await apiRequest('/api/v1/cameras/metadata');
    latestCameraMetadata = payload;
    cameraMetadataPollErrorReported = false;
    updateCameraRuntimeUi(latestCameraStatus, latestCameraMetadata);
  } catch (error) {
    if (!cameraMetadataPollErrorReported) {
      addLog(`camera metadata poll failed: ${error.message}`, 'err');
      cameraMetadataPollErrorReported = true;
    }
  }
}

function _visionTileFromPrimary(primary) {
  if (primary === 'webcam') {
    return 'CAM_02';
  }
  return 'CAM_01';
}

function _visionTilesToTry(primary) {
  const preferred = _visionTileFromPrimary(primary);
  return [preferred, 'CAM_01', 'CAM_02', 'CAM_03', 'CAM_04'].filter((tile, index, tiles) => tiles.indexOf(tile) === index);
}

function setCameraTabPreviewTile(tile) {
  const preview = document.getElementById('camera-tab-preview');
  if (!preview || !tile) {
    return;
  }

  const telemetry = latestRosState?.telemetry;
  const rosDataUrl =
    tile === 'CAM_02'
      ? telemetry?.cameraWebcamDataUrl
      : tile === 'CAM_01'
        ? telemetry?.cameraVideo2DataUrl || telemetry?.cameraDataUrl
        : '';
  if (rosDataUrl) {
    if (preview.dataset.rosSrc !== rosDataUrl) {
      preview.dataset.rosSrc = rosDataUrl;
      preview.src = rosDataUrl;
    }
    return;
  }

  const nextSrc = apiUrl(`/api/v1/cameras/${tile}/stream.mjpeg`);
  if (preview.getAttribute('src') !== nextSrc) {
    preview.dataset.rosSrc = '';
    preview.src = nextSrc;
  }
}

async function pollVisionDetections() {
  try {
    const tiles = _visionTilesToTry(latestCameraStatus?.primary_source);
    let payload = null;
    let selectedTile = latestVisionTile || tiles[0];

    for (const tile of tiles) {
      const response = await apiRequest(`/api/v1/cameras/${tile}/detections`);
      if (!payload) {
        payload = response;
        selectedTile = tile;
      }
      if (response?.detected) {
        payload = response;
        selectedTile = tile;
        break;
      }
    }

    latestVisionDetection = payload;
    latestVisionTile = selectedTile;
    visionDetectionPollErrorReported = false;
    setCameraTabPreviewTile(selectedTile);
    updateVisionOverlay();
    if (latestRosState) {
      updateRightTabRuntimeFields(latestRosState);
    }
  } catch (error) {
    if (!visionDetectionPollErrorReported) {
      addLog(`vision detection poll failed: ${error.message}`, 'err');
      visionDetectionPollErrorReported = true;
    }
  }
}

function wireMissionButtons() {
  document.getElementById('start-btn')?.addEventListener('click', startMission);
  document.querySelector('.mission-reset-btn')?.addEventListener('click', resetMission);
  document.getElementById('estop-btn')?.addEventListener('click', estopMission);
}

function initRosLayer() {
  const config = window.RosDashboardConfig;
  if (!config || !window.RosBridgeConnection || !window.createDashboardStore) {
    addLog('ROS modules missing in frontend bundle', 'err');
    setRosStatus('offline', 'modules');
    return;
  }

  const connection = new window.RosBridgeConnection({
    url: config.rosbridgeUrl,
    urls: config.rosbridgeUrls,
    reconnectBaseMs: config.reconnectBaseMs,
    reconnectMaxMs: config.reconnectMaxMs,
  });

  rosStore = window.createDashboardStore(connection, config);
  rosUnsubscribe = rosStore.subscribe(applyRosState);
  rosStore.init();

  wireDpad();
  wireBehaviorButtons();
  wireKeyboardControls();
}

function cleanupResources() {
  stopKeyboardMovement();
  if (clockInterval) {
    clearInterval(clockInterval);
    clockInterval = null;
  }
  if (heartbeatInterval) {
    clearInterval(heartbeatInterval);
    heartbeatInterval = null;
  }
  if (cameraStatusInterval) {
    clearInterval(cameraStatusInterval);
    cameraStatusInterval = null;
  }
  if (cameraMetadataInterval) {
    clearInterval(cameraMetadataInterval);
    cameraMetadataInterval = null;
  }
  if (visionDetectionInterval) {
    clearInterval(visionDetectionInterval);
    visionDetectionInterval = null;
  }
  if (timerInterval) {
    clearInterval(timerInterval);
    timerInterval = null;
  }
  window.removeEventListener("keydown", handleKeyboardMovement);
  window.removeEventListener("keyup", handleKeyboardMovement);
  window.removeEventListener("keydown", handleFlipperKeys);
  window.removeEventListener("blur", stopKeyboardMovement);
  if (rosUnsubscribe) {
    rosUnsubscribe();
    rosUnsubscribe = null;
  }
  if (rosStore) {
    rosStore.cleanup();
    rosStore = null;
  }
}

function initializeDashboard() {
  renderCameraTiles();
  setTimeout(loadMjpegTiles, 1500);
  wireMissionButtons();
  initRosLayer();
  pollCameraStatus();
  pollCameraMetadata();
  pollVisionDetections();
  cameraStatusInterval = setInterval(pollCameraStatus, 1000);
  cameraMetadataInterval = setInterval(pollCameraMetadata, 1000);
  visionDetectionInterval = setInterval(pollVisionDetections, 400);

  const timestamp = document.getElementById('log-timestamp');
  clockInterval = setInterval(() => {
    if (timestamp) {
      timestamp.innerText = new Date().toLocaleTimeString('en-GB', { hour12: false });
    }
  }, 1000);

  heartbeatInterval = setInterval(() => addLog('HEARTBEAT: UI LOOP ACTIVE'), 5000);
  addLog('Dashboard initialized', 'info');

  window.addEventListener('resize', () => {
    if (latestRosState) {
      renderLidar(latestRosState.telemetry.lidar, latestRosState.ui.stale.lidar);
    }
  });
  window.addEventListener('beforeunload', cleanupResources, { once: true });
}

document.addEventListener('DOMContentLoaded', initializeDashboard);

window.toggleDashboard = toggleDashboard;
window.focusCam = focusCam;
window.switchTab = switchTab;
window.switchSysTab = switchSysTab;
window.toggleInspectMode = toggleInspectMode;
window.simulateDrill = simulateDrill;
window.startMission = startMission;
window.resetMission = resetMission;

'use strict';

const element = (id) => document.getElementById(id);
const client = globalThis.crypto?.randomUUID?.() || String(Math.random());
let sequence = 0;
let online = false;
let heldDrive = null;
let commandPending = false;
let statusData = null;
let mapSnapshot = null;
let errorTimer;

// Commands have sequence numbers so a delayed drive cannot override a later stop.
async function sendCommand(command, parameters = {}) {
  const response = await fetch('/api/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      command,
      client,
      sequence: ++sequence,
      ...parameters,
    }),
    signal: AbortSignal.timeout(1500),
  });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.error || 'Command failed');
  }
  return data;
}

function showError(message) {
  element('error').textContent = message;
  element('error').hidden = false;
  clearTimeout(errorTimer);
  errorTimer = setTimeout(() => {
    element('error').hidden = true;
  }, 5000);
}

function stop() {
  heldDrive = null;
  document.querySelectorAll('.active').forEach((button) => {
    button.classList.remove('active');
  });
  sendCommand('stop').catch((error) => showError(error.message));
}

// The Pi stops after 0.35 seconds unless a held control refreshes its command.
async function refreshDrive() {
  if (!heldDrive || commandPending || !online) return;

  commandPending = true;
  try {
    await sendCommand('drive', { ...heldDrive, duration: 0.35 });
  } catch (error) {
    heldDrive = null;
    showError(error.message);
  } finally {
    commandPending = false;
  }
}

document.querySelectorAll('[data-direction]').forEach((button) => {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    if (!online) return;

    heldDrive = {
      direction: Number(button.dataset.direction),
      steer: Number(button.dataset.steer),
    };
    button.setPointerCapture(event.pointerId);
    button.classList.add('active');
    refreshDrive();
  });
  button.addEventListener('pointerup', stop);
  button.addEventListener('pointercancel', stop);
});

const keys = {
  w: { direction: 1, steer: 0 },
  ArrowUp: { direction: 1, steer: 0 },
  s: { direction: -1, steer: 0 },
  ArrowDown: { direction: -1, steer: 0 },
  a: { direction: 1, steer: 30 },
  ArrowLeft: { direction: 1, steer: 30 },
  d: { direction: 1, steer: -30 },
  ArrowRight: { direction: 1, steer: -30 },
};

window.addEventListener('keydown', (event) => {
  if (event.code === 'Space') {
    event.preventDefault();
    stop();
  } else if (keys[event.key]) {
    event.preventDefault();
    if (online && !event.repeat) {
      heldDrive = keys[event.key];
      refreshDrive();
    }
  }
});
window.addEventListener('keyup', (event) => {
  if (keys[event.key]) stop();
});

// Losing focus stops manual driving, but must not cancel autonomous navigation.
window.addEventListener('blur', () => {
  if (heldDrive) stop();
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden && heldDrive) stop();
});
element('stop').onclick = stop;

for (const action of ['scan', 'reset']) {
  element(action).onclick = () => {
    heldDrive = null;
    sendCommand(action).catch((error) => showError(error.message));
  };
}
for (const side of ['left', 'right']) {
  element(side + 'Goal').onclick = () => {
    if (!online || !statusData.robot.calibrated) return;

    heldDrive = null;
    const goal = statusData.config['goal_' + side];
    sendCommand('navigate', { goal }).catch((error) =>
      showError(error.message),
    );
  };
}

function formatReading(value, unit) {
  return value == null ? 'Unavailable' : Number(value).toFixed(1) + ' ' + unit;
}

function renderStatus() {
  const { robot, vision, navigation } = statusData;
  element('connection').textContent = 'Connected over Wi-Fi';
  element('leftGoal').disabled = !robot.calibrated;
  element('rightGoal').disabled = !robot.calibrated;
  element('distance').textContent =
    robot.range_state === 'beyond_range'
      ? 'Beyond 400 cm'
      : formatReading(robot.distance_cm, 'cm');
  element('temp').textContent = formatReading(robot.temperature_c, '°C');
  element('grayscale').textContent = robot.grayscale.join(' · ');
  element('reason').textContent = robot.reason;

  const [forward, left, heading] = robot.pose_estimated;
  element('pose').textContent =
    `Estimated pose: ${forward.toFixed(1)} cm forward, ` +
    `${left.toFixed(1)} cm left, ${((heading * 180) / Math.PI).toFixed(1)}° heading`;
  element('navState').textContent = navigation.state;
  element('fps').textContent = formatReading(vision.fps, 'FPS');
  const labels = vision.detections.map((detection) => {
    return `${detection.label} (${Math.round(detection.score * 100)}%)`;
  });
  element('detections').textContent =
    vision.error ||
    labels.join(' · ') ||
    'No objects above confidence threshold';
  drawMap();
}

async function refreshStatus() {
  try {
    const response = await fetch('/api/status', {
      signal: AbortSignal.timeout(1200),
    });
    if (!response.ok) throw new Error('Server unavailable');

    statusData = await response.json();
    online = true;
    renderStatus();
  } catch {
    online = false;
    heldDrive = null;
    element('leftGoal').disabled = true;
    element('rightGoal').disabled = true;
    element('connection').textContent = 'Disconnected · controls stopped';
  }
  setTimeout(refreshStatus, 250);
}

// Poll again only after each request finishes; do not accumulate pending requests.
async function refreshMap() {
  try {
    if (element('lab1b').open) {
      const response = await fetch('/api/map', {
        signal: AbortSignal.timeout(1800),
      });
      if (!response.ok) throw new Error('Map unavailable');
      mapSnapshot = await response.json();
      drawMap();
    }
  } catch {
    // Status polling reports a lost connection; retry the map on the next cycle.
  }
  setTimeout(refreshMap, 900);
}

function drawMap() {
  if (!element('lab1b').open || !mapSnapshot || !statusData) return;

  const canvas = element('map');
  const context = canvas.getContext('2d');
  const grid = mapSnapshot.map;
  const [xmin, xmax, ymin, ymax] = grid.bounds;
  const scale = Math.min(
    (canvas.width - 50) / (ymax - ymin),
    (canvas.height - 50) / (xmax - xmin),
  );
  // Robot coordinates: x forward, y left. Canvas coordinates: x right, y down.
  const pixel = (x, y) => [
    25 + (ymax - y) * scale,
    canvas.height - 25 - (x - xmin) * scale,
  ];
  context.clearRect(0, 0, canvas.width, canvas.height);

  const cellSize = grid.cell_cm * scale;
  for (let x = 0; x < grid.occupied.length; x++) {
    for (let y = 0; y < grid.occupied[x].length; y++) {
      let color = '#13202f';
      if (grid.occupied[x][y]) color = '#ff657d';
      else if (grid.inflated[x][y]) color = '#74414e';
      else if (grid.observed[x][y]) color = '#203b46';

      const [px, py] = pixel(
        xmin + x * grid.cell_cm,
        ymin + (y + 1) * grid.cell_cm,
      );
      context.fillStyle = color;
      context.fillRect(px, py - cellSize, cellSize + 0.4, cellSize + 0.4);
    }
  }

  // Planned route starts at the current estimated pose.
  const [x, y, heading] = statusData.robot.pose_estimated;
  const car = pixel(x, y);
  context.strokeStyle = '#58d2c0';
  context.lineWidth = 3;
  context.beginPath();
  context.moveTo(...car);
  for (const step of mapSnapshot.route) {
    context.lineTo(...pixel(...step.pose));
  }
  context.stroke();

  if (mapSnapshot.goal) {
    const [gx, gy] = pixel(...mapSnapshot.goal);
    context.strokeStyle = '#f6c76a';
    context.beginPath();
    context.moveTo(gx - 7, gy - 7);
    context.lineTo(gx + 7, gy + 7);
    context.moveTo(gx + 7, gy - 7);
    context.lineTo(gx - 7, gy + 7);
    context.stroke();
  }

  // A circle marks the car; the short line shows its heading.
  const ahead = pixel(x + 13 * Math.cos(heading), y + 13 * Math.sin(heading));
  context.fillStyle = '#eef7ff';
  context.beginPath();
  context.arc(...car, 7, 0, Math.PI * 2);
  context.fill();
  context.strokeStyle = '#fff';
  context.beginPath();
  context.moveTo(...car);
  context.lineTo(...ahead);
  context.stroke();
  context.fillStyle = '#b6c9dd';
  context.font = '12px sans-serif';
  context.fillText('↑ forward (x)    ← left (+y)    5 cm grid', 25, 16);
}

function refreshCamera() {
  if (!element('lab1b').open) {
    setTimeout(refreshCamera, 500);
    return;
  }
  const camera = element('camera');
  camera.onload = camera.onerror = () => setTimeout(refreshCamera, 200);
  camera.src = '/camera.jpg?t=' + Date.now();
}

setInterval(refreshDrive, 120);
refreshStatus();
refreshMap();
refreshCamera();

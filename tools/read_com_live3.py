#!/usr/bin/env python3
import argparse
import json
import math
import random
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

# ==============================================================================
# ГЛОБАЛЬНЫЙ ПОТОКОБЕЗОПАСНЫЙ БУФЕР ДАННЫХ И ССЫЛКА НА WORKER
# ==============================================================================
MAX_POINTS = 200
data_lock = threading.Lock()
telemetry_buffer = deque(maxlen=MAX_POINTS)
ACTIVE_WORKER = None

# ==============================================================================
# HTML5 + VANILLA JS ВЕБ-ОСЦИЛЛОГРАФ + ПУЛЬТ УПРАВЛЕНИЯ WASD
# ==============================================================================
HTML_PAGE = """<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<title>Robotics Telemetry Oscilloscope &amp; Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background-color: #1e1e1e;
    color: #e0e0e0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
    display: flex;
    flex-direction: column;
    height: 100vh;
    overflow: hidden;
    user-select: none;
  }
  header {
    background-color: #252526;
    padding: 12px 20px;
    border-bottom: 1px solid #333;
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 15px;
  }
  .title { font-weight: bold; font-size: 16px; color: #fff; }
  .legend { display: flex; gap: 20px; font-size: 14px; flex-wrap: wrap; }
  .legend-item { display: flex; align-items: center; gap: 6px; }
  .dot { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
  .val { font-weight: bold; min-width: 70px; display: inline-block; text-align: right; }
  #canvas-container { flex: 1; position: relative; width: 100%; height: 100%; }
  canvas { display: block; width: 100%; height: 100%; }

  /* Панель настройки PID в левом нижнем углу */
  #pid-overlay {
    position: absolute;
    bottom: 25px;
    left: 65px;
    background: rgba(37, 37, 38, 0.88);
    border: 1px solid #444;
    border-radius: 10px;
    padding: 12px 14px;
    display: flex;
    flex-direction: column;
    gap: 8px;
    backdrop-filter: blur(4px);
  }
  .pid-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    font-size: 13px;
  }
  .pid-row label {
    min-width: 68px;
    color: #ccc;
    font-weight: bold;
  }
  .pid-row input {
    width: 85px;
    background: #1e1e1e;
    border: 1px solid #555;
    border-radius: 4px;
    color: #fff;
    padding: 4px 6px;
    font-family: monospace;
    font-size: 13px;
    text-align: right;
  }
  .pid-row input:focus {
    outline: none;
    border-color: #2196f3;
  }
  .btn-set {
    background: #2e7d32;
    border: 1px solid #4caf50;
    border-radius: 4px;
    color: #fff;
    padding: 4px 10px;
    font-size: 12px;
    font-weight: bold;
    cursor: pointer;
    transition: background 0.1s;
  }
  .btn-set:hover { background: #388e3c; }
  .btn-set:active { background: #1b5e20; }

  /* Панель управления движением в правом нижнем углу */
  #controls-overlay {
    position: absolute;
    bottom: 25px;
    right: 30px;
    background: rgba(37, 37, 38, 0.88);
    border: 1px solid #444;
    border-radius: 10px;
    padding: 12px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
    backdrop-filter: blur(4px);
  }
  .ctrl-title { font-size: 11px; color: #aaa; margin-bottom: 4px; }
  .ctrl-row { display: flex; gap: 6px; }
  .btn {
    width: 44px;
    height: 44px;
    background: #333337;
    border: 1px solid #555;
    border-radius: 6px;
    color: #fff;
    font-weight: bold;
    font-size: 14px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: background 0.1s;
  }
  .btn:hover { background: #45454a; }
  .btn.active { background: #2196f3; border-color: #64b5f6; color: #fff; }
  .btn-stop { width: 144px; height: 32px; font-size: 12px; background: #4a2525; border-color: #7a3b3b; }
  .btn-stop:hover { background: #633030; }
</style>
</head>
<body>
<header>
  <div class="title">PID &amp; IMU Real-Time Scope</div>
  <div class="legend">
    <div class="legend-item">
      <span class="dot" style="background:#ff9800;"></span>
      <span>Acc_angle:</span>
      <span id="val-acc" class="val" style="color:#ff9800;">0.00°</span>
    </div>
    <div class="legend-item">
      <span class="dot" style="background:#2196f3;"></span>
      <span>robot_angle:</span>
      <span id="val-ang" class="val" style="color:#2196f3;">0.00°</span>
    </div>
    <div class="legend-item">
      <span class="dot" style="background:#4caf50;"></span>
      <span>Cycle / Gyro:</span>
      <span id="val-gyr" class="val" style="color:#4caf50;">0.00</span>
    </div>
    <div class="legend-item">
      <span class="dot" style="background:#ba68c8;"></span>
      <span>MotorPower:</span>
      <span id="val-pid" class="val" style="color:#ba68c8;">0.00 %</span>
    </div>
  </div>
</header>
<div id="canvas-container">
  <canvas id="scope"></canvas>

  <!-- Панель настройки коэффициентов PID -->
  <div id="pid-overlay">
    <div class="ctrl-title">Настройка PID (Enter / Set)</div>
    <div class="pid-row">
      <label for="inp-kp">Kp:</label>
      <input type="number" id="inp-kp" step="0.01" value="0.2" onkeydown="handlePidKey(event, 'p', 'inp-kp')">
      <button class="btn-set" onclick="sendPid('p', 'inp-kp')">Set</button>
    </div>
    <div class="pid-row">
      <label for="inp-ki">Ki (Kc):</label>
      <input type="number" id="inp-ki" step="0.0001" value="0.0002" onkeydown="handlePidKey(event, 'i', 'inp-ki')">
      <button class="btn-set" onclick="sendPid('i', 'inp-ki')">Set</button>
    </div>
    <div class="pid-row">
      <label for="inp-kd">Kd:</label>
      <input type="number" id="inp-kd" step="0.1" value="4.17" onkeydown="handlePidKey(event, 'k', 'inp-kd')">
      <button class="btn-set" onclick="sendPid('k', 'inp-kd')">Set</button>
    </div>
    <div class="pid-row">
      <label for="inp-kv">Kv:</label>
      <input type="number" id="inp-kv" step="0.005" value="0.02" onkeydown="handlePidKey(event, 'v', 'inp-kv')">
      <button class="btn-set" onclick="sendPid('v', 'inp-kv')">Set</button>
    </div>
    <div class="pid-row">
      <label for="inp-off">Offset:</label>
      <input type="number" id="inp-off" step="0.1" value="-12.0" onkeydown="handlePidKey(event, 'o', 'inp-off')">
      <button class="btn-set" onclick="sendPid('o', 'inp-off')">Set</button>
    </div>
  </div>

  <!-- Панель управления движением -->
  <div id="controls-overlay">
    <div class="ctrl-title">Движение: WASD / Стрелки</div>
    <div class="ctrl-row">
      <button class="btn" id="btn-w" onmousedown="startCmd('w')" onmouseup="stopCmd()" onmouseleave="stopCmd()">W</button>
    </div>
    <div class="ctrl-row">
      <button class="btn" id="btn-a" onmousedown="startCmd('a')" onmouseup="stopCmd()" onmouseleave="stopCmd()">A</button>
      <button class="btn" id="btn-s" onmousedown="startCmd('s')" onmouseup="stopCmd()" onmouseleave="stopCmd()">S</button>
      <button class="btn" id="btn-d" onmousedown="startCmd('d')" onmouseup="stopCmd()" onmouseleave="stopCmd()">D</button>
    </div>
    <div class="ctrl-row">
      <button class="btn btn-stop" onclick="sendOnce('x')">STOP (Space)</button>
    </div>
  </div>
</div>

<script>
const canvas = document.getElementById('scope');
const ctx = canvas.getContext('2d');
const MAX_POINTS = 200;
const Y_MIN = -90;
const Y_MAX = 90;

let points = [];

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function mapY(val, height, padTop, padBottom) {
  const clamped = Math.max(Y_MIN, Math.min(Y_MAX, val));
  const usableHeight = height - padTop - padBottom;
  return padTop + usableHeight * (1 - (clamped - Y_MIN) / (Y_MAX - Y_MIN));
}

function drawGrid(width, height, padLeft, padRight, padTop, padBottom) {
  ctx.clearRect(0, 0, width, height);
  ctx.font = '12px monospace';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';

  for (let deg = Y_MIN; deg <= Y_MAX; deg += 30) {
    const y = mapY(deg, height, padTop, padBottom);
    ctx.beginPath();
    ctx.moveTo(padLeft, y);
    ctx.lineTo(width - padRight, y);

    if (deg === 0) {
      ctx.strokeStyle = '#757575';
      ctx.lineWidth = 1.5;
    } else {
      ctx.strokeStyle = '#2d2d30';
      ctx.lineWidth = 1;
    }
    ctx.stroke();

    ctx.fillStyle = deg === 0 ? '#ffffff' : '#9e9e9e';
    ctx.fillText((deg > 0 ? '+' : '') + deg, padLeft - 10, y);
  }

  const usableWidth = width - padLeft - padRight;
  ctx.strokeStyle = '#2d2d30';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    const x = padLeft + (usableWidth / 10) * i;
    ctx.beginPath();
    ctx.moveTo(x, padTop);
    ctx.lineTo(x, height - padBottom);
    ctx.stroke();
  }
}

function drawSeries(key, color, width, height, padLeft, padRight, padTop, padBottom) {
  if (points.length < 2) return;
  const usableWidth = width - padLeft - padRight;
  const stepX = usableWidth / (MAX_POINTS - 1);
  const startIdx = MAX_POINTS - points.length;

  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';

  for (let i = 0; i < points.length; i++) {
    const x = padLeft + (startIdx + i) * stepX;
    const y = mapY(points[i][key], height, padTop, padBottom);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}
function render() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;
  const padLeft = 55;
  const padRight = 20;
  const padTop = 20;
  const padBottom = 20;

  drawGrid(width, height, padLeft, padRight, padTop, padBottom);

  drawSeries('acc', '#ff9800', width, height, padLeft, padRight, padTop, padBottom);
  drawSeries('ang', '#2196f3', width, height, padLeft, padRight, padTop, padBottom);
  drawSeries('gyr', '#4caf50', width, height, padLeft, padRight, padTop, padBottom);
  drawSeries('pid', '#ba68c8', width, height, padLeft, padRight, padTop, padBottom);

  if (points.length > 0) {
    const last = points[points.length - 1];
    document.getElementById('val-acc').textContent = last.acc.toFixed(2) + '°';
    document.getElementById('val-ang').textContent = last.ang.toFixed(2) + '°';
    document.getElementById('val-gyr').textContent = last.gyr.toFixed(2);
    document.getElementById('val-pid').textContent = last.pid.toFixed(2) + ' %';
  }
}

async function pollData() {
  try {
    const resp = await fetch('/data');
    if (resp.ok) {
      points = await resp.json();
      render();
    }
  } catch (e) {
  } finally {
    setTimeout(pollData, 35);
  }
}
pollData();

// ============================================================================
// ОТПРАВКА КОЭФФИЦИЕНТОВ PID
// ============================================================================
function sendPid(prefix, inputId) {
  const inp = document.getElementById(inputId);
  const val = parseFloat(inp.value);
  if (!isNaN(val)) {
    sendOnce(prefix + val);
    inp.blur();
  }
}

function handlePidKey(e, prefix, inputId) {
  e.stopPropagation(); // Не запускать WASD при вводе цифр
  if (e.key === 'Enter') {
    e.preventDefault();
    sendPid(prefix, inputId);
  }
}

// ============================================================================
// ЛОГИКА УПРАВЛЕНИЯ ДВИЖЕНИЕМ (WASD / СТРЕЛКИ / МЫШЬ)
// ============================================================================
let activeMoveCmd = null;

function sendOnce(cmd) {
  fetch('/cmd', { method: 'POST', body: cmd }).catch(() => {});
}

function highlightBtn(cmd) {
  ['w', 'a', 's', 'd'].forEach(k => {
    const el = document.getElementById('btn-' + k);
    if (el) el.classList.toggle('active', k === cmd);
  });
}

function startCmd(cmd) {
  if (activeMoveCmd !== cmd) {
    activeMoveCmd = cmd;
    highlightBtn(cmd);
    sendOnce(cmd);
  }
}

function stopCmd() {
  if (activeMoveCmd !== null) {
    activeMoveCmd = null;
    highlightBtn(null);
    sendOnce('x');
  }
}

setInterval(() => {
  if (activeMoveCmd) {
    sendOnce(activeMoveCmd);
  }
}, 90);

const KEY_MAP = {
  'w': 'w', 'ц': 'w', 'arrowup': 'w',
  's': 's', 'ы': 's', 'arrowdown': 's',
  'a': 'a', 'ф': 'a', 'arrowleft': 'a',
  'd': 'd', 'в': 'd', 'arrowright': 'd'
};

window.addEventListener('keydown', (e) => {
  if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === ' ') {
    e.preventDefault();
    stopCmd();
    return;
  }
  if (KEY_MAP[k]) {
    e.preventDefault();
    startCmd(KEY_MAP[k]);
  }
});

window.addEventListener('keyup', (e) => {
  if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
  const k = e.key.toLowerCase();
  if (KEY_MAP[k] && KEY_MAP[k] === activeMoveCmd) {
    stopCmd();
  }
});

window.addEventListener('blur', stopCmd);
</script>
</body>
</html>
"""

# ==============================================================================
# ВЕБ-СЕРВЕР ТЕЛЕМЕТРИИ И ПРИЕМА КОМАНД УПРАВЛЕНИЯ
# ==============================================================================
class TelemetryHTTPHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            content = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/data":
            with data_lock:
                payload = json.dumps(list(telemetry_buffer)).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/cmd":
            length = int(self.headers.get("Content-Length", 0))
            cmd = self.rfile.read(length).decode("utf-8", errors="ignore").strip()
            if cmd and ACTIVE_WORKER:
                if hasattr(ACTIVE_WORKER, "send_command"):
                    ACTIVE_WORKER.send_command(cmd)
                elif hasattr(ACTIVE_WORKER, "handle_command"):
                    ACTIVE_WORKER.handle_command(cmd)
            self.send_response(200)
            self.send_header("Content-Length", "0")
            self.end_headers()
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def start_web_server(port: int):
    server = ThreadingHTTPServer(("127.0.0.1", port), TelemetryHTTPHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


# ==============================================================================
# ЛОГИРОВАНИЕ В ФАЙЛ
# ==============================================================================
class DebugLogger:
    def __init__(self, filename: str = "com_debug.log"):
        self.filename = filename
        self.lock = threading.Lock()

    def log(self, direction: str, message: str):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"[{ts}] [{direction}] {message.strip()}\n"
        with self.lock:
            with open(self.filename, "a", encoding="utf-8") as f:
                f.write(line)


# ==============================================================================
# ФИЗИЧЕСКИЙ СИМУЛЯТОР ОБРАТНОГО МАЯТНИКА (--sim)
# ==============================================================================
class PendulumSimulator:
    def __init__(self, logger: DebugLogger):
        self.logger = logger
        self.lock = threading.Lock()
        self.running = True

        self.Kp = 0.85
        self.Ki = 4.50
        self.Kd = 0.035
        self.Gyro_amount = 0.996

        self.dt = 0.004
        self.com_offset = 1.5
        self.MAX_VOLTS = 9.6
        self.INTEGRAL_MAX = 4.0

        self.true_angle = 0.5
        self.angular_vel = 0.0
        self.robot_angle = 0.5
        self.acc_angle = 0.5
        self.gyro_y = 0.0
        self.integral_sum = 0.0
        self.target_voltage = 0.0
        self.vertical = True
        self.external_kick = 0.0
        self.drive_bias = 0.0

    def reset(self):
        with self.lock:
            self.true_angle = 15.0
            self.angular_vel = 0.0
            self.robot_angle = 15.0
            self.integral_sum = 0.0
            self.target_voltage = 0.0
            self.vertical = False
            self.drive_bias = 0.0
        msg = "SIM: Сброс маятника в наклон 15.0°"
        print(msg)
        self.logger.log("SIM", msg)

    def kick(self):
        with self.lock:
            self.external_kick = random.choice([-1.0, 1.0]) * 180.0
        msg = "SIM: Приложен внешний импульс толчка!"
        print(msg)
        self.logger.log("SIM", msg)

    def handle_command(self, cmd: str):
        cmd = cmd.strip()
        if not cmd:
            return
        self.logger.log("TX_SIM", cmd)

        low = cmd.lower()
        if low == "reset":
            self.reset()
            return
        if low == "kick":
            self.kick()
            return

        with self.lock:
            if low == "w":
                self.drive_bias = 2.0
                return
            if low == "s":
                self.drive_bias = -2.0
                return
            if low in ("x", "a", "d"):
                self.drive_bias = 0.0
                return

            try:
                if low.startswith("kp"):
                    self.Kp = float(low[2:])
                    print(f"SIM: Kp = {self.Kp}")
                elif low.startswith("ki"):
                    self.Ki = float(low[2:])
                    self.integral_sum = 0.0
                    print(f"SIM: Ki = {self.Ki}")
                elif low.startswith("kd"):
                    self.Kd = float(low[2:])
                    print(f"SIM: Kd = {self.Kd}")
                elif low.startswith("g"):
                    val = float(low[1:])
                    if 0.0 <= val <= 1.0:
                        self.Gyro_amount = val
                        print(f"SIM: Gyro_amount = {self.Gyro_amount}")
                else:
                    print(f"SIM: Неизвестная команда '{cmd}'. Доступны: kp, ki, kd, g, kick, reset, w/a/s/d/x")
            except ValueError:
                print(f"SIM: Ошибка формата числа в команде '{cmd}'")

    def run(self):
        step_counter = 0
        next_step_time = time.perf_counter()

        while self.running:
            next_step_time += self.dt
            with self.lock:
                self.acc_angle = self.true_angle + random.gauss(0.0, 2.5)
                self.gyro_y = self.angular_vel + random.gauss(0.0, 0.4)

                self.robot_angle = (
                    (self.robot_angle + self.gyro_y * self.dt) * self.Gyro_amount
                    + self.acc_angle * (1.0 - self.Gyro_amount)
                )

                abs_ang = abs(self.robot_angle)
                if not self.vertical and abs_ang < 1.0:
                    self.vertical = True
                elif self.vertical and abs_ang > 40.0:
                    self.vertical = False

                if self.vertical:
                    err = self.robot_angle - self.drive_bias
                    p_term = self.Kp * err
                    self.integral_sum += self.Ki * err * self.dt
                    self.integral_sum = max(-self.INTEGRAL_MAX, min(self.INTEGRAL_MAX, self.integral_sum))
                    d_term = -self.Kd * self.gyro_y

                    raw_v = p_term + self.integral_sum + d_term
                    self.target_voltage = max(-self.MAX_VOLTS, min(self.MAX_VOLTS, raw_v))
                else:
                    self.integral_sum = 0.0
                    self.target_voltage = 0.0

                gravity_torque = 38.0 * math.sin(math.radians(self.true_angle + self.com_offset))
                motor_torque = 11.5 * self.target_voltage
                damping = 0.35 * self.angular_vel

                angular_acc = gravity_torque - motor_torque - damping + self.external_kick
                self.external_kick = 0.0

                self.angular_vel += angular_acc * self.dt
                self.true_angle += self.angular_vel * self.dt

                if self.true_angle > 85.0:
                    self.true_angle = 85.0
                    self.angular_vel = 0.0
                elif self.true_angle < -85.0:
                    self.true_angle = -85.0
                    self.angular_vel = 0.0

                step_counter += 1
                if step_counter >= 10:
                    step_counter = 0
                    pwr_pct = (abs(self.target_voltage) / self.MAX_VOLTS) * 100.0
                    point = {
                        "ang": round(self.robot_angle, 2),
                        "acc": round(self.acc_angle, 2),
                        "gyr": round(self.gyro_y, 2),
                        "pid": round(pwr_pct, 2),
                    }
                    with data_lock:
                        telemetry_buffer.append(point)

            sleep_time = next_step_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_step_time = time.perf_counter()


# ==============================================================================
# МЕНЕДЖЕР COM-ПОРТА С АВТОПЕРЕПОДКЛЮЧЕНИЕМ И РАЗБОРОМ STX-ПАКЕТОВ
# ==============================================================================
class SerialWorker:
    def __init__(self, port: str, baudrate: int, logger: DebugLogger):
        self.port = port
        self.baudrate = baudrate
        self.logger = logger
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True

    
    def send_command(self, cmd: str):
        cmd_clean = cmd.strip()
        if not cmd_clean:
            return
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                try:
                    # Для однобуквенных команд движения (w/a/s/d/x) отправляем без лишнего перевода строки
                    if len(cmd_clean) == 1 and cmd_clean.lower() in ("w", "a", "s", "d", "x"):
                        payload = cmd_clean.encode("utf-8")
                    else:
                        payload = (cmd_clean + "\n").encode("utf-8")
                    self.ser.write(payload)
                    self.logger.log("TX", cmd_clean)
                except serial.SerialException as e:
                    print(f"[SERIAL] Ошибка отправки: {e}")
            else:
                print("[SERIAL] Порт не подключен. Команда не отправлена.")

    def _parse_stx_packet(self, payload: bytes):
        """
        Разбирает пакет формата:
        <STX> robot_angle <0x01> Acc_angle <0x04> MotorPower% <0x05> gyro_rate <ETX>
        """
        try:
            part1, rest = payload.split(b"\x01", 1)
            part2, rest = rest.split(b"\x04", 1)
            part3, part4 = rest.split(b"\x05", 1)

            robot_ang = float(part1.decode("ascii").strip())
            acc_ang = float(part2.decode("ascii").strip())
            motor_pwr_pct = float(part3.decode("ascii").strip())
            gyro_rate = float(part4.decode("ascii").strip())

            point = {
                "ang": round(robot_ang, 4),
                "acc": round(acc_ang, 4),
                "gyr": round(gyro_rate, 2),
                "pid": round(motor_pwr_pct, 2),
            }
            with data_lock:
                telemetry_buffer.append(point)

            self.logger.log(
                "RX_PKT",
                f"ang={robot_ang:.4f}, acc={acc_ang:.4f}, pwr={motor_pwr_pct:.1f}%, gyro={gyro_rate:.2f}°/s",
            )
        except Exception as e:
            err_msg = f"[PKT_ERR] Битый пакет {payload!r}: {e}"
            print(err_msg)
            self.logger.log("ERR_PKT", err_msg)

    def run(self):
        rx_buf = bytearray()

        while self.running:
            try:
                print(f"[SERIAL] Подключение к {self.port} ({self.baudrate} бод)...")
                ser_conn = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.05,
                    dsrdtr=False,
                    rtscts=False,
                )
                with self.ser_lock:
                    self.ser = ser_conn
                print(f"[SERIAL] Успешно подключено к {self.port}")
                self.logger.log("SYS", f"Connected to {self.port} at {self.baudrate}")

                while self.running:
                    chunk = ser_conn.read(ser_conn.in_waiting or 1)
                    if not chunk:
                        continue
                    rx_buf.extend(chunk)

                    while rx_buf:
                        if rx_buf.startswith(b"\x02"):
                            etx_idx = rx_buf.find(b"\x03")
                            if etx_idx == -1:
                                if len(rx_buf) > 128:
                                    rx_buf.pop(0)
                                break
                            packet_body = bytes(rx_buf[1:etx_idx])
                            del rx_buf[: etx_idx + 1]
                            self._parse_stx_packet(packet_body)
                        else:
                            nl_idx = rx_buf.find(b"\n")
                            stx_idx = rx_buf.find(b"\x02")

                            if stx_idx != -1 and (nl_idx == -1 or stx_idx < nl_idx):
                                text_bytes = bytes(rx_buf[:stx_idx])
                                del rx_buf[:stx_idx]
                            elif nl_idx != -1:
                                text_bytes = bytes(rx_buf[: nl_idx + 1])
                                del rx_buf[: nl_idx + 1]
                            else:
                                if len(rx_buf) > 256:
                                    rx_buf.clear()
                                break

                            line = text_bytes.decode("utf-8", errors="replace").strip()
                            if line:
                                self.logger.log("RX", line)
                                print(f"[MCU] {line}")

            except (serial.SerialException, OSError) as e:
                with self.ser_lock:
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                rx_buf.clear()
                print(f"[SERIAL] Связь потеряна ({e}). Повтор через 1.5 с...")
                self.logger.log("ERR", f"Serial disconnected: {e}")
                time.sleep(1.5)


# ==============================================================================
# ТОЧКА ВХОДА И ИНТЕРАКТИВНЫЙ ТЕРМИНАЛ
# ==============================================================================
def main():
    global ACTIVE_WORKER

    parser = argparse.ArgumentParser(
        description="COM-терминал, веб-осциллограф и пульт управления роботом"
    )
    parser.add_argument("-p", "--port", type=str, help="Путь к COM-порту (например, /dev/tty.usbserial-... или COM3)")
    parser.add_argument("-b", "--baud", type=int, default=230400, help="Скорость порта (по умолчанию 230400)")
    parser.add_argument("-w", "--web-port", type=int, default=8000, help="Порт веб-сервера (по умолчанию 8000)")
    parser.add_argument("--sim", action="store_true", help="Запуск в режиме физической симуляции без платы")
    args = parser.parse_args()

    if not args.sim and not args.port:
        parser.error("Укажите COM-порт через флаг -p /dev/tty... или запустите в режиме симуляции --sim")

    logger = DebugLogger("com_debug.log")
    start_web_server(args.web_port)
    print(f"[WEB] Осциллограф и пульт WASD запущены: http://127.0.0.1:{args.web_port}")

    sim_worker = None
    serial_worker = None

    if args.sim:
        sim_worker = PendulumSimulator(logger)
        ACTIVE_WORKER = sim_worker
        t = threading.Thread(target=sim_worker.run, daemon=True)
        t.start()
        print("[SIM] Режим симуляции активен. Управление: WASD в браузере или команды в консоли.")
    else:
        serial_worker = SerialWorker(args.port, args.baud, logger)
        ACTIVE_WORKER = serial_worker
        t = threading.Thread(target=serial_worker.run, daemon=True)
        t.start()
        print("[TERM] Подключено. Управление: WASD в браузере, либо команды (например: o8.5) в консоли.")

    session = PromptSession()
    with patch_stdout():
        while True:
            try:
                user_input = session.prompt("> ")
                if user_input.strip().lower() in ("exit", "quit"):
                    break
                if args.sim and sim_worker:
                    sim_worker.handle_command(user_input)
                elif serial_worker:
                    serial_worker.send_command(user_input)
            except (KeyboardInterrupt, EOFError):
                break

    print("\nЗавершение работы...")
    if sim_worker:
        sim_worker.running = False
    if serial_worker:
        serial_worker.running = False
        with serial_worker.ser_lock:
            if serial_worker.ser and serial_worker.ser.is_open:
                serial_worker.ser.close()


if __name__ == "__main__":
    main()

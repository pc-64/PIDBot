#!/usr/bin/env python3
import argparse
import json
import math
import random
import re
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

# ==============================================================================
# ГЛОБАЛЬНЫЙ ПОТОКОБЕЗОПАСНЫЙ БУФЕР ДАННЫХ
# ==============================================================================
MAX_POINTS = 200
data_lock = threading.Lock()
telemetry_buffer = deque(maxlen=MAX_POINTS)

# Регулярное выражение для парсинга телеметрии с контроллера BGC 1.3
TELEMETRY_REGEX = re.compile(
    r"Угол \(robot_angle\):\s*([+-]?\d+(?:\.\d+)?).*?"
    r"Акселерометр:\s*([+-]?\d+(?:\.\d+)?).*?"
    r"Скор\. гиро:\s*([+-]?\d+(?:\.\d+)?).*?"
    r"PID:\s*([+-]?\d+(?:\.\d+)?)"
)

# ==============================================================================
# HTML5 + VANILLA JS ВЕБ-ОСЦИЛЛОГРАФ (БЕЗ ВНЕШНИХ CDN)
# ==============================================================================
HTML_PAGE = """<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<title>Robotics Telemetry Oscilloscope</title>
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
  .legend { display: flex; gap: 20px; font-size: 14px; }
  .legend-item { display: flex; align-items: center; gap: 6px; }
  .dot { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
  .val { font-weight: bold; min-width: 65px; display: inline-block; text-align: right; }
  #canvas-container { flex: 1; position: relative; width: 100%; height: 100%; }
  canvas { display: block; width: 100%; height: 100%; }
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
      <span>gyroY:</span>
      <span id="val-gyr" class="val" style="color:#4caf50;">0.00 °/с</span>
    </div>
    <div class="legend-item">
      <span class="dot" style="background:#ba68c8;"></span>
      <span>PID:</span>
      <span id="val-pid" class="val" style="color:#ba68c8;">0.00 В</span>
    </div>
  </div>
</header>
<div id="canvas-container">
  <canvas id="scope"></canvas>
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

  // Горизонтальные линии сетки с шагом 30° от -90° до +90°
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
    ctx.fillText((deg > 0 ? '+' : '') + deg + '°', padLeft - 10, y);
  }

  // Вертикальные линии сетки
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
    document.getElementById('val-gyr').textContent = last.gyr.toFixed(2) + ' °/с';
    document.getElementById('val-pid').textContent = last.pid.toFixed(2) + ' В';
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
    // Игнорируем временные ошибки при перезапуске сервера
  }
}

setInterval(pollData, 80);
</script>
</body>
</html>
"""

# ==============================================================================
# ВЕБ-СЕРВЕР ТЕЛЕМЕТРИИ
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

    def log_message(self, format, *args):
        # Подавляем стандартный вывод HTTP-запросов, чтобы не засорять терминал
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

        # Параметры ПИД и фильтра (идентичны прошивке BGC 1.3)
        self.Kp = 0.85
        self.Ki = 4.50
        self.Kd = 0.035
        self.Gyro_amount = 0.996

        # Физические константы маятника
        self.dt = 0.004                 # Шаг симуляции: 0.004 с (250 Гц)
        self.com_offset = 1.5           # Смещение центра масс (°)
        self.MAX_VOLTS = 9.6            # Ограничение напряжения моторов (±9.6 В)
        self.INTEGRAL_MAX = 4.0

        # Состояние системы
        self.true_angle = 0.5
        self.angular_vel = 0.0
        self.robot_angle = 0.5
        self.acc_angle = 0.5
        self.gyro_y = 0.0
        self.integral_sum = 0.0
        self.target_voltage = 0.0
        self.vertical = True
        self.external_kick = 0.0

    def reset(self):
        with self.lock:
            self.true_angle = 15.0
            self.angular_vel = 0.0
            self.robot_angle = 15.0
            self.integral_sum = 0.0
            self.target_voltage = 0.0
            self.vertical = False
        msg = "SIM: Сброс маятника в наклон 15.0° (ожидание прохождения через 0°)"
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
                    print(f"SIM: Неизвестная команда '{cmd}'. Доступны: kp, ki, kd, g, kick, reset")
            except ValueError:
                print(f"SIM: Ошибка формата числа в команде '{cmd}'")

    def run(self):
        step_counter = 0
        next_step_time = time.perf_counter()

        while self.running:
            next_step_time += self.dt
            with self.lock:
                # 1. Имитация показаний датчиков с заданным гауссовым шумом
                self.acc_angle = self.true_angle + random.gauss(0.0, 3.0)
                self.gyro_y = self.angular_vel + random.gauss(0.0, 0.4)

                # 2. Комплементарный фильтр
                self.robot_angle = (
                    (self.robot_angle + self.gyro_y * self.dt) * self.Gyro_amount
                    + self.acc_angle * (1.0 - self.Gyro_amount)
                )

                # 3. Логика активации и защиты от падения
                abs_ang = abs(self.robot_angle)
                if not self.vertical and abs_ang < 1.0:
                    self.vertical = True
                elif self.vertical and abs_ang > 40.0:
                    self.vertical = False

                # 4. Расчет ПИД-регулятора
                if self.vertical:
                    p_term = self.Kp * self.robot_angle
                    self.integral_sum += self.Ki * self.robot_angle * self.dt
                    self.integral_sum = max(-self.INTEGRAL_MAX, min(self.INTEGRAL_MAX, self.integral_sum))
                    d_term = -self.Kd * self.gyro_y

                    raw_v = p_term + self.integral_sum + d_term
                    self.target_voltage = max(-self.MAX_VOLTS, min(self.MAX_VOLTS, raw_v))
                else:
                    self.integral_sum = 0.0
                    self.target_voltage = 0.0

                # 5. Физика обратного маятника (уравнение движения с демпфированием и смещением ЦМ)
                gravity_torque = 38.0 * math.sin(math.radians(self.true_angle + self.com_offset))
                motor_torque = 11.5 * self.target_voltage
                damping = 0.35 * self.angular_vel

                angular_acc = gravity_torque - motor_torque - damping + self.external_kick
                self.external_kick = 0.0

                self.angular_vel += angular_acc * self.dt
                self.true_angle += self.angular_vel * self.dt

                # Ограничение механического упора пола (±85°)
                if self.true_angle > 85.0:
                    self.true_angle = 85.0
                    self.angular_vel = 0.0
                elif self.true_angle < -85.0:
                    self.true_angle = -85.0
                    self.angular_vel = 0.0

                # Публикация точки телеметрии каждые 10 шагов (25 Гц -> 40 мс)
                step_counter += 1
                if step_counter >= 10:
                    step_counter = 0
                    point = {
                        "ang": round(self.robot_angle, 2),
                        "acc": round(self.acc_angle, 2),
                        "gyr": round(self.gyro_y, 2),
                        "pid": round(self.target_voltage, 2),
                    }
                    with data_lock:
                        telemetry_buffer.append(point)

            sleep_time = next_step_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_step_time = time.perf_counter()


# ==============================================================================
# МЕНЕДЖЕР COM-ПОРТА С АВТОПЕРЕПОДКЛЮЧЕНИЕМ
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
                    payload = (cmd_clean + "\n").encode("utf-8")
                    self.ser.write(payload)
                    self.logger.log("TX", cmd_clean)
                except serial.SerialException as e:
                    print(f"[SERIAL] Ошибка отправки: {e}")
            else:
                print("[SERIAL] Порт не подключен. Команда не отправлена.")

    def _parse_stx_packet(self, payload: bytes):
        try:
            part1, rest = payload.split(b"\x01", 1)
            part2, rest = rest.split(b"\x04", 1)
            part3, part4 = rest.split(b"\x05", 1)

            robot_ang = float(part1.decode("ascii").strip())
            acc_ang = float(part2.decode("ascii").strip())
            motor_pwr_pct = float(part3.decode("ascii").strip())
            loop_ms = float(part4.decode("ascii").strip())

            point = {
                "ang": round(robot_ang, 4),
                "acc": round(acc_ang, 4),
                "gyr": round(loop_ms, 3),
                "pid": round(motor_pwr_pct, 2),
            }
            with data_lock:
                telemetry_buffer.append(point)

            self.logger.log(
                "RX_PKT",
                f"ang={robot_ang:.4f}, acc={acc_ang:.4f}, pwr={motor_pwr_pct:.1f}%, loop={loop_ms:.3f}ms",
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
    parser = argparse.ArgumentParser(
        description="COM-терминал, веб-осциллограф и симулятор обратного маятника"
    )
    parser.add_argument("-p", "--port", type=str, help="Путь к COM-порту (например, /dev/ttyUSB0 или COM3)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Скорость порта (по умолчанию 115200)")
    parser.add_argument("-w", "--web-port", type=int, default=8000, help="Порт веб-сервера (по умолчанию 8000)")
    parser.add_argument("--sim", action="store_true", help="Запуск в режиме физической симуляции без платы")
    args = parser.parse_args()

    if not args.sim and not args.port:
        parser.error("Укажите COM-порт через флаг -p /dev/tty... или запустите в режиме симуляции --sim")

    logger = DebugLogger("com_debug.log")
    start_web_server(args.web_port)
    print(f"[WEB] Локальный осциллограф запущен: http://127.0.0.1:{args.web_port}")

    sim_worker = None
    serial_worker = None

    if args.sim:
        sim_worker = PendulumSimulator(logger)
        t = threading.Thread(target=sim_worker.run, daemon=True)
        t.start()
        print("[SIM] Режим симуляции активен. Команды: kp<val>, ki<val>, kd<val>, g<val>, kick, reset, exit")
    else:
        serial_worker = SerialWorker(args.port, args.baud, logger)
        t = threading.Thread(target=serial_worker.run, daemon=True)
        t.start()
        print("[TERM] Введите команду для отправки на плату (например: p1.2, i4.5, d0.03, g0.996) или 'exit'")

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


if __name__ == "__main__":
    main()

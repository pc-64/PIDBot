import argparse
import json
import logging
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import serial
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
 
# Кольцевой буфер для хранения последних 200 измерений (старые точки автоматически удаляются)
history = deque(maxlen=200)
# Блокировка (Mutex) для безопасного доступа к списку history из разных потоков одновременно
history_lock = threading.Lock()
# Счетчик порядкового номера отсчета (ось X на графике)
step_counter = 0
 
# Регулярное выражение для извлечения трех чисел из строки вида:
# "Угол (robot_angle): -9.16° | Акселерометр: -0.7° | Скор. гиро: -81.9 °/с"
telemetry_re = re.compile(
    r"robot_angle\):\s*([-\d.]+).*?Акселерометр:\s*([-\d.]+).*?Скор\. гиро:\s*([-\d.]+)"
)
 
# Встроенная HTML-страница с JavaScript-отрисовкой графика на элементе <canvas>
 
HTML_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>BGC 1.3 MPU6050</title></head>
<body style="font-family:sans-serif;padding:20px;background:#1e1e1e;color:#eee;">
<h3>Телеметрия MPU6050 (живой график)</h3>
 
<!-- Легенда с живыми числовыми значениями -->
<div style="margin-bottom:12px;font-size:15px;font-family:monospace;background:#262626;padding:10px 15px;border-radius:6px;display:inline-block;border:1px solid #3a3a3a;">
  <span style="color:#4da6ff;font-weight:bold;">■ robot_angle: <span id="val_robot">0.00</span>°</span> &nbsp;&nbsp;|&nbsp;&nbsp;
  <span style="color:#ffaa00;font-weight:bold;">■ Acc_angle: <span id="val_acc">0.0</span>°</span> &nbsp;&nbsp;|&nbsp;&nbsp;
  <span style="color:#55dd55;font-weight:bold;">■ gyroY: <span id="val_gyro">0.0</span> °/с</span>
</div>
 
<br>
<canvas id="c" width="900" height="400" style="background:#111;border:1px solid #444;"></canvas>
 
<div style="max-width:870px;margin-top:15px;padding:15px;background:#262626;border:1px solid #3a3a3a;border-radius:6px;line-height:1.6;font-size:14px;">
  <ul style="margin:0;padding-left:20px;">
    <li><strong style="color:#ffaa00;">Оранжевая линия (Acc_angle):</strong> Сырой угол по акселерометру сильно реагирует на вибрации и линейные ускорения при тряске стола или корпуса, давая резкие пики вверх и вниз.</li>
    <li><strong style="color:#4da6ff;">Синяя линия (robot_angle):</strong> Итоговый отфильтрованный угол остается ровным и плавным, полностью отсекая вибрационные шумы акселерометра.</li>
    <li><strong style="color:#55dd55;">Зеленая линия (gyroY):</strong> Угловая скорость гироскопа четко отражает реальные покачивания без лишних выбросов.</li>
  </ul>
</div>
 
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const scaleAngle = 2.0; // Масштаб: 2 пикселя = 1 градус (диапазон +-100°)
const leftMargin = 50;  // Отступ слева под числовые подписи оси Y
 
// Отрисовка контрастной координатной сетки и подписей
function drawGrid() {
  const midY = canvas.height / 2;
  ctx.font = 'bold 12px monospace';
  ctx.textBaseline = 'middle';
  ctx.textAlign = 'right';
 
  for (let deg = -90; deg <= 90; deg += 30) {
    const y = midY - (deg * scaleAngle);
    
    ctx.beginPath();
    if (deg === 0) {
      ctx.strokeStyle = '#888888'; // Яркая нулевая линия
      ctx.lineWidth = 1.5;
    } else {
      ctx.strokeStyle = '#333333'; // Контрастные линии сетки
      ctx.lineWidth = 1;
    }
    ctx.moveTo(leftMargin, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
 
    // Подпись значения слева от вертикальной оси
    ctx.fillStyle = (deg === 0) ? '#ffffff' : '#bbbbbb';
    const label = (deg > 0 ? '+' : '') + deg + '°';
    ctx.fillText(label, leftMargin - 8, y);
  }
 
  // Вертикальная разделительная линия оси Y
  ctx.strokeStyle = '#555555';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(leftMargin, 0);
  ctx.lineTo(leftMargin, canvas.height);
  ctx.stroke();
}
 
// Отрисовка кривой графика
function drawLine(data, key, color, scaleY) {
  if (data.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  
  const plotWidth = canvas.width - leftMargin;
  const stepX = plotWidth / 200;
  const midY = canvas.height / 2;
  
  data.forEach((pt, idx) => {
    const x = leftMargin + (idx * stepX);
    const y = midY - (pt[key] * scaleY);
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}
 
setInterval(async () => {
  try {
    const res = await fetch('/data');
    const d = await res.json();
    
    if (d.length > 0) {
      const last = d[d.length - 1];
      document.getElementById('val_robot').textContent = last.robot.toFixed(2);
      document.getElementById('val_acc').textContent = last.acc.toFixed(1);
      document.getElementById('val_gyro').textContent = last.gyro.toFixed(1);
    }
 
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawGrid();
 
    drawLine(d, 'acc', '#ffaa00', scaleAngle);
    drawLine(d, 'robot', '#4da6ff', scaleAngle);
    drawLine(d, 'gyro', '#55dd55', 0.8);
  } catch (e) {}
}, 80);
</script></body></html>"""
 
class WebHandler(BaseHTTPRequestHandler):
    """Обработчик HTTP-запросов от браузера"""
 
    def do_GET(self):
        if self.path == "/data":
            # Отдаем браузеру накопленный массив точек в формате JSON
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with history_lock:
                self.wfile.write(json.dumps(list(history)).encode("utf-8"))
        else:
            # На любой другой запрос отдаем саму HTML-страницу с графиком
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
 
    def log_message(self, format, *args):
        # Подавляем вывод стандартных логов HTTP-сервера, чтобы не засорять терминал
        pass
 
 
def start_web_server(web_port):
    """Запуск локального HTTP-сервера в фоновом потоке"""
    server = HTTPServer(("127.0.0.1", web_port), WebHandler)
    server.serve_forever()
 
 
def parse_args():
    """Разбор аргументов командной строки при запуске скрипта"""
    parser = argparse.ArgumentParser(description="COM-терминал с prompt, логом и графиком")
    parser.add_argument("-p", "--port", required=True, help="Имя порта (например, /dev/tty.usbserial-588F0264801)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Скорость в бодах (по умолчанию: 115200)")
    parser.add_argument("-l", "--log", default="com_debug.log", help="Имя файла для сохранения лога")
    parser.add_argument("-r", "--reconnect", type=float, default=2.0, help="Пауза перед переподключением в сек")
    parser.add_argument("-w", "--web-port", type=int, default=8000, help="Порт локального веб-сервера (по умолчанию: 8000)")
    parser.add_argument("--eol", default="\\n", help="Символ конца строки при отправке: \\n, \\r\\n или none")
    return parser.parse_args()
 
 
def serial_reader(args, ser_holder, stop_event):
    """Фоновый поток для непрерывного чтения COM-порта и автоматического переподключения"""
    global step_counter
    while not stop_event.is_set():
        try:
            # Открываем порт; dsrdtr=False предотвращает лишнюю перезагрузку некоторых плат при подключении
            with serial.Serial(args.port, args.baud, timeout=0.5, dsrdtr=False, rtscts=False) as ser:
                ser_holder["ser"] = ser
                logging.info(f"Подключено к {args.port} ({args.baud} baud)")
                
                while not stop_event.is_set():
                    # Читаем строку до символа перевода строки и декодируем в UTF-8
                    data = ser.readline().decode("utf-8", errors="replace").strip()
                    if data:
                        logging.debug(f"<< {data}")
                        # Ищем в строке числа углов и скорости гироскопа
                        m = telemetry_re.search(data)
                        if m:
                            with history_lock:
                                history.append({
                                    "x": step_counter,
                                    "robot": float(m.group(1)),
                                    "acc": float(m.group(2)),
                                    "gyro": float(m.group(3))
                                })
                                step_counter += 1
        except (serial.SerialException, OSError) as e:
            # Если кабель отошел или плата перепрошивается — ждем и пробуем подключиться снова
            ser_holder["ser"] = None
            if not stop_event.is_set():
                logging.warning(f"Связь потеряна: {e}. Повтор через {args.reconnect} сек...")
                time.sleep(args.reconnect)
 
 
def main():
    args = parse_args()
    # Преобразуем строковый флаг окончания строки в реальные байты
    eol_map = {"\\n": b"\n", "\\r\\n": b"\r\n", "\\r": b"\r", "none": b""}
    eol_bytes = eol_map.get(args.eol, b"\n")
 
    # Настраиваем одновременный вывод логов и в файл, и в окно терминала
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(args.log, encoding="utf-8"),
            logging.StreamHandler()
        ]
    )
 
    # Запускаем веб-сервер в режиме daemon (он автоматически закроется при выходе из main)
    threading.Thread(target=start_web_server, args=(args.web_port,), daemon=True).start()
 
    ser_holder = {"ser": None}       # Словарь для передачи активного объекта порта между потоками
    stop_event = threading.Event()   # Флаг для корректной остановки фоновых потоков
 
    # Запускаем фоновый поток чтения данных из платы
    reader_thread = threading.Thread(target=serial_reader, args=(args, ser_holder, stop_event), daemon=True)
    reader_thread.start()
 
    session = PromptSession()
 
    # patch_stdout удерживает строку ввода '>' внизу экрана, пока сверху бегут входящие логи
    with patch_stdout():
        logging.info(f"График доступен в браузере: http://127.0.0.1:{args.web_port}")
        logging.info("Введите команду после '>' (Ctrl+C или 'exit' для выхода)")
        try:
            while not stop_event.is_set():
                cmd = session.prompt("> ")
                if cmd.strip().lower() in ("exit", "quit"):
                    break
                if not cmd.strip():
                    continue
 
                # Отправляем введенную команду в плату BGC 1.3
                ser = ser_holder.get("ser")
                if ser and ser.is_open:
                    try:
                        ser.write(cmd.encode("utf-8") + eol_bytes)
                        logging.debug(f">> {cmd}")
                    except (serial.SerialException, OSError) as e:
                        logging.error(f"Ошибка отправки: {e}")
                else:
                    logging.warning("Порт закрыт, команда не отправлена")
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            # Сигнализируем всем потокам о завершении работы
            stop_event.set()
            logging.info("Работа завершена")
 
 
if __name__ == "__main__":
    main()
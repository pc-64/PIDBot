#include <Wire.h>
 
#define MPU6050 0x68              // Адрес датчика по шине I2C
#define CONFIG_REG 0x1A           // Регистр настройки цифрового фильтра (DLPF)
#define GYRO_CONFIG 0x1B          // Регистр настройки гироскопа
#define ACCEL_CONFIG 0x1C         // Регистр настройки акселерометра
#define PWR_MGMT_1 0x6B           // Регистр управления питанием
 
// Масштабирование датчика
#define accSens 0                 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1                // 0 = 250 deg/s, 1 = 500 deg/s (чувствительность 65.5)
 
int16_t AcX, AcZ, GyY;
float gyroY;
 
// Смещения (офсеты) для калибровки нуля
int16_t AcX_offset = -60;
int16_t AcZ_offset = 300;
int16_t GyY_offset = 0;
int32_t GyY_offset_sum = 0;
 
float robot_angle = 0;
float Acc_angle = 0;
bool vertical = false;
 
float Gyro_amount = 0.996;        // Коэффициент комплементарного фильтра
 
uint32_t loop_timer;
uint32_t prev_calc_time;
uint32_t print_timer;
 
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  Serial.println("Инициализация MPU6050...");
  Serial.println("Не двигайте плату во время калибровки!");
  
  angle_setup();
  
  Serial.println("Калибровка завершена. Наклоняйте датчик.");
  Serial.println("Команда для смены фильтра: g0.95 или g0.996");
  
  loop_timer = micros();
  prev_calc_time = micros();
  print_timer = millis();
}
 
void loop() {
  // Прием новой настройки Gyro_amount из терминала (например: g0.95)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'g') {
      float val = Serial.parseFloat();
      if (val >= 0.0 && val <= 1.0) {
        Gyro_amount = val;
        Serial.print(">>> Новый Gyro_amount = ");
        Serial.println(Gyro_amount, 4);
      }
    }
  }
 
  // Целевая частота опроса ~1000 Гц (1000 мкс), но даже при задержках угол не уплывет благодаря расчету реального dt
  if (micros() - loop_timer >= 1000) {
    loop_timer = micros();
    
    angle_calc();
    gyroY = GyY / 65.5;
 
    // Вывод в порт каждые 100 мс (10 раз в секунду)
    if (millis() - print_timer >= 100) {
      print_timer = millis();
      
      Serial.print("Угол (robot_angle): ");
      Serial.print(robot_angle, 2);
      Serial.print("°  |  Акселерометр: ");
      Serial.print(Acc_angle, 1);
      Serial.print("°  |  Скор. гиро: ");
      Serial.print(gyroY, 1);
      Serial.print(" °/с  |  Статус: ");
      Serial.println(vertical ? "БАЛАНС АКТИВЕН" : "УПАЛ (>40°)");
    }
  }
}
 
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}
 
void angle_setup() {
  Wire.begin();
  Wire.setClock(400000);                          // Ускоряем шину I2C до 400 кГц (Fast Mode)
  delay(100);
  
  writeTo(MPU6050, PWR_MGMT_1, 0);                // Выводим датчик из сна
  writeTo(MPU6050, CONFIG_REG, 0x03);             // Включаем аппаратный ФНЧ (DLPF = 44 Гц) от вибраций моторов
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);   // Настраиваем диапазон акселерометра
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);   // Настраиваем диапазон гироскопа
  delay(100);
 
  // Калибровка дрейфа гироскопа (1024 замера)
  for (int i = 0; i < 1024; i++) {
    read_raw_mpu();
    GyY_offset_sum += GyY;
    delay(2);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = ");
  Serial.println(GyY_offset);
 
  // Инициализируем стартовый угол по текущему положению акселерометра,
  // чтобы фильтр не ждал 3-5 секунд при старте под наклоном
  read_raw_mpu();
  AcX += AcX_offset;
  AcZ += AcZ_offset;
  Acc_angle = atan2(-AcX, AcZ) * 57.2958;
  robot_angle = Acc_angle;
}
 
void read_raw_mpu() {
  // Пакетное чтение всех 14 байт данных за одну транзакцию I2C (экономит время цикла)
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                               // Начальный регистр ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 14, true);
 
  if (Wire.available() >= 14) {
    AcX = Wire.read() << 8 | Wire.read();         // 0x3B, 0x3C: Accel X
    Wire.read(); Wire.read();                     // 0x3D, 0x3E: Accel Y (пропускаем)
    AcZ = Wire.read() << 8 | Wire.read();         // 0x3F, 0x40: Accel Z
    Wire.read(); Wire.read();                     // 0x41, 0x42: Температура (пропускаем)
    Wire.read(); Wire.read();                     // 0x43, 0x44: Gyro X (пропускаем)
    GyY = Wire.read() << 8 | Wire.read();         // 0x45, 0x46: Gyro Y
    Wire.read(); Wire.read();                     // 0x47, 0x48: Gyro Z (пропускаем)
  }
}
 
void angle_calc() {
  uint32_t now = micros();
  float dt = (now - prev_calc_time) / 1000000.0;  // Точное время шага в секундах
  prev_calc_time = now;
 
  read_raw_mpu();
 
  AcX += AcX_offset;
  AcZ += AcZ_offset;
  GyY -= GyY_offset;
 
  // Интегрируем угловую скорость с учетом фактического dt
  robot_angle += (GyY / 65.5) * dt;                      
  Acc_angle = atan2(-AcX, AcZ) * 57.2958;    
  
  // Комплементарный фильтр
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
 
  // Используем fabs() для корректного сравнения чисел с плавающей точкой (float)
  if (fabs(robot_angle) > 40.0) vertical = false;
  if (fabs(robot_angle) < 1.0) vertical = true;
}
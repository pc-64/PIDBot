 /*
 * "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * By Matt_M
 * This code controls a two wheel balancing robot using the controller and motors from a GoPro sized gimbal
 * stabilizer. The board is based around a mega328p and uses two L6234 brushless motor control ICs. It uses
 * Silabs CP2102 USB to serial chip and the 6 axis IMU MPU6050.
 * HC-05 Bluetooth setup for 230400 baud on the TX and RX pins
 * Joystick BT Commander Version 5.2
 * Button #1: "ON", Button #2: "Fast"
 * Data field #1: "Bat. V", Data field #2: "Power", Data field #3: "Cycle (ms)"
 * note: cycle time must not exceed 4ms
 */


#pragma GCC optimize ("3")
#include <math.h>

//main loop--------------------------------------------------------
#define LEDPIN   A1

bool power = true;            //run when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running


//Bluetooth input stuff------------------------------------------
#define    STX          0x02
#define    ETX          0x03

int16_t joyY, joyX;                                         //smartphone joystick input values

uint8_t cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
uint8_t buttonStatus = 0;                                  // first Byte sent to Android device


//Battery stuff---------------------------------------------------
uint16_t Bat_Voltage = 0;
bool Bat_Discharged = false;      //true when (Bat_Voltage < dischargeVoltage)

#define dischargeVoltage  2*3.0*50.76    //number of cells * lipo cell discharged voltage / ((5/1023)*(13.3/3.3))
#define resetVoltage  2*3.7*50.76        //voltage to reset Bat_Discharged to false

#define FREQ_MASK 0x3FF
char rx_buf[16];
uint8_t rx_idx = 0;
//angle calculations from MPU-6050--------------------------------------------------
#include <Wire.h>

float kc = 0.0002;
float kv = 0.02;
float kp = 0.2;
float kd = 0.025 / (2 * 0.003);
#define MPU6050 0x68              //Device address
#define ACCEL_CONFIG 0x1C         //Accelerometer configuration address
#define GYRO_CONFIG 0x1B          //Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C


//Sensor output scaling
#define accSens 1            // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define RESOLUTION 255

int16_t  AcX, AcY, GyZ;

//IMU offset values
int16_t  AcX_offset = 0;
int16_t  AcY_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

int32_t GyZ_filter[32];
uint8_t filter_count = 0;
int32_t  GyZ_F;
float robot_angle;
float Acc_angle;            //angle calculated from acc. measurments

bool vertical = false;      //is the robot vertical enough to run

bool GyZ_filter_on = true;  //apply simple average filter to Z gyro reading


#define Gyro_amount 0.997   //percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s


//motor control-------------------------------------------------------
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

// Space Vector PWM lookup table
// using uint8_t overflow for stepping
int8_t pwmSinMotor[] = {0,  5,  10, 16, 21, 27, 32, 37, 43, 48, 53, 59, 64, 69, 74, 79, 84, 89, 94, 99, 104,  109,
111,  112,  114,  115,  116,  118,  119,  120,  121,  122,  123,  123,  124,  125,  125,  126,  126,  126,  127,
127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  123,  122,  121,  120,  119,  118,  116,
115,  114,  112,  111,  110,  112,  113,  114,  116,  117,  118,  119,  120,  121,  122,  123,  124,  124,  125,
125,  126,  126,  126,  127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  124,  123,  122,  121,
120,  119,  118,  117,  116,  114,  113,  112,  110,  106,  101,  97, 92, 87, 82, 77, 72, 66, 61, 56, 51, 45, 40,
35, 29, 24, 18, 13, 8,  2,  -2, -8, -13,  -18,  -24,  -29,  -35,  -40,  -45,  -51,  -56,  -61,  -66,  -72,  -77,
-82,  -87,  -92,  -97,  -101, -106, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122, -123, -124,
-124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -124, -123,
-122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110, -111, -112, -114, -115, -116, -118, -119, -120,
-121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -126, -126, -126,
-125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -116, -115, -114, -112, -111, -109, -104, -99,  -94,
-89,  -84,  -79,  -74,  -69,  -64,  -59,  -53,  -48,  -43,  -37,  -32,  -27,  -21,  -16,  -10,  -5, 0};

uint16_t MotorPower = 200; // 0 to 255, 255==100% power

uint8_t idl_motor_power = 100;

uint8_t resp_rate = 1; //bluetooth remote joystick multiplier

//motor numbers
#define L_Motor 0
#define R_Motor 1

//motor pole angle, 0->255 overflow to loop after >>8 shift
uint16_t R_MotorStep = 0;
uint16_t L_MotorStep = 0;


float robot_speed;

//rotation speed for turning
int8_t rot_Speed = 0;

// speed of motors, -127 to 127
int16_t R_Speed = 0;
int16_t L_Speed = 0;


//PID stuff---------------------------------------------------------------
int32_t robot_position = 0; //offset from corect position


//loop tuning values
//#define kc 0.0002          //position feedback gain, (reposition robot after disturbance)0.0002
//#define kv 0.02             //velocity feedback gain 0.02
//#define kp 0.2             //angle gain 0.2
//#define kd 0.020/(2*0.003) //angle velocity gain 0.025
float angle_offset = 0;
float drive_cmd = 0.0;
int16_t turn_cmd = 0;
uint8_t cmd_timeout = 0;
#define _PWM_FREQUENCY 32000
#define _PWM_FREQUENCY_MAX 32000
#define _PWM_FREQUENCY_MIN 4000
//Assign Pins for Motor Control
#define PID_out_max 20
#define PID_out_min -25

float PID_in, last_PID_in, Last_last_PID_in, PID_out;


//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

void setup()
{
  pinMode (LEDPIN, OUTPUT);

  // Start Serial Port
  Serial.begin(115200);
  Serial.println("Bal_Code_v8_with_bluetooth_working_SVPWM" );
  Serial.println("place robot on front or back side withing 5sec. of power for gyroscope calibration");

  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //setup robot angle calculation
  Serial.println("calibrating gyroscope...........");
  angle_setup();

  //test battery voltage
  battery_test();
  Serial.print("Battery voltage = " );
  Serial.println(Bat_Voltage * 0.019698);


  //Setup brushless motor Controller
   Bl_Setup();
  Serial.println("place robot vertical to run");

  while (Serial.available())  Serial.read();        // empty RX buffer
}

void loop()
{
  // Безопасное чтение младшего байта (атомарно на 8-бит AVR для масок до 0xFF)
  if ((freqCounter & 0x07f) == 0)
  {
    oldfreqCounter = freqCounter;

    // Отправка телеметрии (для 115200 бод: 0x3FF -> ~30.6 Гц)
    if ((freqCounter & FREQ_MASK) == 0)
    {
      sendBlueToothData();
    }

    // Расчёт угла наклона
    angle_calc();

    // Неблокирующий приём команд движения и PID
      while (Serial.available() > 0)
    {
      char c = Serial.read();

      // 1. Мгновенные однобуквенные команды срабатывают ВСЕГДА (сбрасывая rx_idx)
      if (c == 'w' || c == 'W')      { rx_idx = 0; drive_cmd = 55.0;  cmd_timeout = 60; continue; }
      else if (c == 's' || c == 'S') { rx_idx = 0; drive_cmd = -55.0; cmd_timeout = 60; continue; }
      else if (c == 'a' || c == 'A') { rx_idx = 0; turn_cmd = -10;    cmd_timeout = 60; continue; }
      else if (c == 'd' || c == 'D') { rx_idx = 0; turn_cmd = 10;     cmd_timeout = 60; continue; }
      else if (c == 'x' || c == 'X') { rx_idx = 0; drive_cmd = 0.0;   turn_cmd = 0; cmd_timeout = 0; continue; }
      else if (c == '0' && rx_idx == 0)
      {
        // '0' (когда это не часть числа вроде p0.2): полная остановка и калибровка гироскопа
        motorPowerOff();
        vertical = false;
        Last_last_PID_in = 0;
        last_PID_in = 0;
        MotorPower = 0;
        robot_speed = 0;
        robot_position = 0;
        drive_cmd = 0.0;
        turn_cmd = 0;

        Serial.println("STOP (0) -> Re-initializing gyro...");
        angle_setup();
        Serial.println("Gyro ready. Place vertical.");
        while (Serial.available()) Serial.read();
        continue;
      }

      // 2. Конец строки: разбор PID-коэффициента (например, p0.25\n)
      if (c == '\n' || c == '\r')
      {
        rx_buf[rx_idx] = '\0';
        if (rx_idx > 1)
        {
          char key = *rx_buf;
          float val = atof(rx_buf + 1);
          if (key == 'o' || key == 'O')      angle_offset = val;
          else if (key == 'p' || key == 'P') kp = val;
          else if (key == 'i' || key == 'I') kc = val;
          else if (key == 'k' || key == 'K') kd = val;
          else if (key == 'v' || key == 'V') kv = val;
        }
        rx_idx = 0;
      }
      // 3. Накопление символов числа (включая '0' внутри числа p0.25)
      else if (c != ' ')
      {
        if (rx_idx < sizeof(rx_buf) - 1) rx_buf[rx_idx++] = c;
        else rx_idx = 0;
      }
    }

    // Автоматическая остановка при отпускании клавиши (~245 мс)
    if (cmd_timeout > 0) {
      cmd_timeout--;
    } else {
      drive_cmd = 0.0;
      turn_cmd = 0;
    }

    if (power == true) PID_and_motor_comd();
    else
    {
      motorPowerOff();
      Last_last_PID_in = 0;
      last_PID_in = 0;
      MotorPower = 0;
      robot_speed = 0;
      robot_position = 0;
    }

    if (freqCounter > oldfreqCounter)
    {
      if (loop_time < (freqCounter - oldfreqCounter)) loop_time = freqCounter - oldfreqCounter;
    }
  }
}

//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA


//-----------------------------------------------------------------------------------
//bluetooth between smartphone and robot---------------------------------------------
//-----------------------------------------------------------------------------------

void bluetooth()
{
  if (Serial.available())   // data received from smartphone
  {
      Serial.print("Угол (robot_angle): ");
      Serial.print(robot_angle, 2);
      Serial.print("°  |  Акселерометр: ");
      Serial.print(Acc_angle, 1);
      Serial.print("°  |  Скор. гиро: ");
      Serial.print(robot_position, 1);
      Serial.print(" °/с  |  Статус: ");
      Serial.println(vertical ? "БАЛАНС АКТИВЕН" : "УПАЛ (>40°)");
    Serial.print( "command" );
    cmd[0] =  Serial.read();
    joyY =  ((int) cmd[0] )* 10 ;
     Serial.write((char)(cmd[0]));
  /*  if (cmd[0] == (char)STX)
    {
      int i = 1;
      while (Serial.available())
      {
        cmd[i] = Serial.read();
      Serial.print( (char)(cmd[i] ));
        if (cmd[i] > 127 || i > 7)                   break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      if      (i == 2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)        {  getJoystickState(cmd);
       Serial.print( "sudi" );
      }// 6 Bytes  ex: < STX "200" "180" ETX >
    }*/
  }
}


void sendBlueToothData()
{
  // Data frame transmitted back from Arduino to Android device:
  // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
  // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

  Serial.print((char)STX);                                             // Start of Transmission
  Serial.print(robot_angle, 4);    Serial.print((char)0x1);   // buttons status feedback
   Serial.print(Acc_angle, 4);     Serial.print((char)0x4);   // datafield #1
  Serial.print(MotorPower * 0.3921);        Serial.print((char)0x5);   // datafield #2
  Serial.print(GyZ / 65.536, 2);                                    // datafield #3
  Serial.print((char)ETX);                                             // End of Transmission




  loop_time = 0;
}


String getButtonStatusString()  {
  String bStatus = "";
  for (int i = 0; i < 6; i++)  {
    if (buttonStatus & (B100000 >> i))      bStatus += "1";
    else                                    bStatus += "0";
  }
  return bStatus;
}


void getJoystickState(byte data[8])    {
  joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)     return; // commmunication error
}


void getButtonState(int bStatus)  {
  switch (bStatus) {
    // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON

      power = true;
 Serial.print("MotorPower");
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF

      power = false;

      break;

    // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON

      resp_rate = 2;

      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      resp_rate = 1;

      break;

/*

        // -----------------  BUTTON #3  -----------------------
         case 'E':
          //buttonStatus |= B000100;        // ON
           idl_motor_power += 5;

           break;
         case 'F':
           buttonStatus &= B111011;      // OFF
           // your code...

           break;

        // -----------------  BUTTON #4  -----------------------
         case 'G':
          //buttonStatus |= B001000;       // ON
           idl_motor_power -= 5;

           break;
         case 'H':
          buttonStatus &= B110111;    // OFF
           // your code...

          break;

        // -----------------  BUTTON #5  -----------------------
         case 'I':           // configured as momentary button
          buttonStatus |= B010000;        // ON
           // your code...

           break;
         case 'J':
          buttonStatus &= B101111;        // OFF
           // your code...

           break;

        // -----------------  BUTTON #6  -----------------------
         case 'K':
          buttonStatus |= B100000;        // ON
           // your code...

          break;
         case 'L':
          buttonStatus &= B011111;        // OFF
           // your code...

           break;
      */
  }
}


//--------------------------------------------------------------------------------------
//angle PID calculation and motor command signal generation-----------------------------
//--------------------------------------------------------------------------------------
void PID_and_motor_comd()
{
 //   Serial.print(robot_angle);
  if (vertical == true)
  {
    //calculate loop output, try to set robot_angle (error) to zero
    PID_in = robot_angle + angle_offset;

    float dInput = (3 * PID_in - 4 * last_PID_in + Last_last_PID_in); //robot angle derivative
    Last_last_PID_in = last_PID_in;
    last_PID_in = PID_in;

    PID_out = kp * PID_in + kd * dInput + kc * robot_position + kv * robot_speed;  //add loop components with gain

    if (PID_out > PID_out_max) PID_out = PID_out_max;
    else if (PID_out < PID_out_min) PID_out = PID_out_min;


    robot_speed += PID_out; //integrate acceleration for velocity

    //stop motors when speed too large
    if ( robot_speed > 2000) vertical = false;

    //robot_position += robot_speed  - 1.5 * resp_rate * joyY; //integrate velocity for displacement and add offset to move robot


         robot_position += (int32_t)robot_speed - (int32_t)drive_cmd;
    if (robot_position > 30000L)  robot_position = 30000L;
    if (robot_position < -30000L) robot_position = -30000L;
    R_Speed =  map(robot_speed, -768, 767, -200, 200)- turn_cmd;
    L_Speed =  map(robot_speed, -768, 767, -200, 200)+ turn_cmd;

/*
    R_Speed = robot_speed;
    L_Speed = robot_speed;
    //adjust motors to turn robot
    rot_Speed = - 0.3 * joyX;
    R_Speed += rot_Speed;
    L_Speed -= rot_Speed;
*/
    //ajust motor power with respect to needed acc. and velocity
    if (PID_out >= 0 ) MotorPower = (idl_motor_power + PID_out * 100) * (0.000035 * robot_speed * robot_speed + 1);
    if (PID_out < 0 ) MotorPower = (idl_motor_power - PID_out * 100) * (0.000035 * robot_speed * robot_speed + 1);

    if (MotorPower > 235) MotorPower = 235;

    //run motors
    MoveMotors(R_Motor, (uint8_t)(R_MotorStep >> 8), MotorPower);
    MoveMotors(L_Motor, (uint8_t)(L_MotorStep >> 8), MotorPower);
  }

  //if not vertical turn everything off
  else
  {
    motorPowerOff();
    Last_last_PID_in = 0;
    last_PID_in = 0;
    MotorPower = 0;
    robot_speed = 0;
    robot_position = 0;
  }
}


//--------------------------------------------------------------------
//Motor control stuff-------------------------------------------------
//--------------------------------------------------------------------
void Bl_Setup()
{
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(LEDPIN, HIGH);

  cli();//stop interrupts

  //timer setup for 31.250KHZ phase correct PWM
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // enable Timer 1 interrupt
  TIMSK1 = 0;
  TIMSK1 |= _BV(TOIE1);
  // disable arduino standard timer interrupt
  TIMSK0 &= ~_BV(TOIE1);

  sei(); // Start Interrupt

  //turn off all PWM signals
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5

  // switch off PWM Power
  motorPowerOff();
}



// switch off motor power
void motorPowerOff() {
  MoveMotors(L_Motor, 0, 0);
  MoveMotors(R_Motor, 0, 0);
  R_Speed = 0;
  L_Speed = 0;
}



void MoveMotors(uint8_t motorNumber, uint8_t posStep, uint16_t power)
{
  uint16_t pwm_a;
  uint16_t pwm_b;
  uint16_t pwm_c;
/*
  pwm_a = (int16_t) 128+128*sin(2*M_PI*(double)(posStep)/RESOLUTION);
  pwm_b = (int16_t) 128+128*sin(2*M_PI*(0.33+(double)(posStep)/RESOLUTION));
  pwm_c = (int16_t) 128+128*sin(2*M_PI*(0.66+(double)(posStep)/RESOLUTION));

  pwm_a *= power/255;
  pwm_b *= power/255;
  pwm_c *= power/255;
  */

  // lookup sine values from table with 120deg. offsets
  pwm_a = (int16_t) 128 + pwmSinMotor[(uint8_t)posStep];
  pwm_b = (int16_t) 128 + pwmSinMotor[(uint8_t)(posStep + 85)];
  pwm_c = (int16_t) 128 + pwmSinMotor[(uint8_t)(posStep + 170)];


  // scale motor power
  pwm_a = power * pwm_a;
  pwm_a = pwm_a >> 8;
 // pwm_a += 128;

  pwm_b = power * pwm_b;
  pwm_b = pwm_b >> 8;
//  pwm_b += 128;

  pwm_c = power * pwm_c;
  pwm_c = pwm_c >> 8;
 // pwm_c += 128;

  // set motor pwm variables
  if (motorNumber == 0)
  {
    PWM_A_MOTOR0 = (uint8_t)pwm_a;
    PWM_B_MOTOR0 = (uint8_t)pwm_b;
    PWM_C_MOTOR0 = (uint8_t)pwm_c;
  }

  if (motorNumber == 1)
  {
    PWM_A_MOTOR1 = (uint8_t)pwm_a;
    PWM_B_MOTOR1 = (uint8_t)pwm_b;
    PWM_C_MOTOR1 = (uint8_t)pwm_c;
  }
}



//-----------------------------------------------------------------
//battery stuff----------------------------------------------------
//-----------------------------------------------------------------
void battery_test()
{
  Bat_Voltage = analogRead(A2);

  if (Bat_Voltage < dischargeVoltage)
  {
    Bat_Discharged = true;
    digitalWrite(LEDPIN, HIGH);    //LED on when battery discharged
  }
  else
  {
    if (Bat_Voltage > resetVoltage) Bat_Discharged = false;

    if (Bat_Discharged == false) digitalWrite(LEDPIN, !digitalRead(LEDPIN));  //flash LED on/off every 2sec when battery charged
  }
}



//------------------------------------------------------------------
//Robot angle calculations------------------------------------------
//------------------------------------------------------------------

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}




//setup MPU6050
void angle_setup()
{
  // Пробуждение MPU6050 и настройка диапазонов (обязательно!)
  writeTo(MPU6050, PWR_MGMT_1, 0x00);          // Выход из спящего режима
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // ±4g (1g = 8192)
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // 250 deg/s
  delay(50);

  int32_t AcX_sum = 0, AcY_sum = 0, GyZ_sum = 0;
  uint16_t valid_samples = 0;
  const uint16_t target_samples = 512;

  for (uint16_t i = 0; i < target_samples; i++)
  {
    // Чтение акселерометра (X, Y)
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050, 4, true) == 4)
    {
      int16_t rawAcX = (Wire.read() << 8) | Wire.read();
      int16_t rawAcY = (Wire.read() << 8) | Wire.read();

      // Чтение гироскопа (Z)
      Wire.beginTransmission(MPU6050);
      Wire.write(0x47);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050, 2, true) == 2)
      {
        int16_t rawGyZ = (Wire.read() << 8) | Wire.read();

        AcX_sum += rawAcX;
        AcY_sum += rawAcY;
        GyZ_sum += rawGyZ;
        valid_samples++;
      }
    }
    delay(2);
  }

  if (valid_samples == 0)
  {
    vertical = false;
    Serial.println("ERR: MPU6050 not responding!");
    return;
  }

  int16_t AcX_avg = AcX_sum / valid_samples;
  int16_t AcY_avg = AcY_sum / valid_samples;
  int16_t GyZ_avg = GyZ_sum / valid_samples;

  // 1. Калибровка дрейфа нуля гироскопа (Z)
  GyZ_offset = -GyZ_avg;

  // 2. Калибровка акселерометра (X - назад, Y - вверх, ±4g -> 1g = 8192)
  AcX_offset = -AcX_avg;
  AcY_offset = 8192 - AcY_avg;

  // 3. Начальный угол (0° в вертикальном положении)
  robot_angle = atan2((float)(AcX_avg + AcX_offset), (float)(AcY_avg + AcY_offset)) * 57.2958f;

  for (uint8_t i = 0; i < 16; i++)
  {
    GyZ_filter[i] = 0;
  }
  filter_count = 0;
}

//calculate robot tilt angle
void angle_calc()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  if (GyZ_filter_on == true)
  {
    GyZ_filter[filter_count] = GyZ;
    filter_count++;
    if (filter_count > 15) filter_count = 0;

    GyZ_F = 0;
    for (int i = 0; i < 16; i++)
    {
      GyZ_F += GyZ_filter[i];
    }
    GyZ = GyZ_F >> 4;
  }

  AcX += AcX_offset;
  AcY += AcY_offset;
  GyZ += GyZ_offset;

  // Знак "-" согласует ось Z (влево) с наклоном вперёд (+) / назад (-)
  robot_angle -= GyZ * 6.07968E-5;

  // atan2(AcX, AcY) даёт 0° при вертикальном положении (Y вверх, X назад)
  Acc_angle = atan2((float)AcX, (float)AcY) * 57.2958f;
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0f - Gyro_amount);

  if (robot_angle > 65 || robot_angle < -65) vertical = false;


  if (fabs(robot_angle + angle_offset) < 1.0f) vertical = true;

}



//--------------------------------------------------------------
// code loop timing---------------------------------------------
//--------------------------------------------------------------
// minimize interrupt code length
// is called every 31.875us (510 clock cycles)  ???????
ISR( TIMER1_OVF_vect )
{
  //every 32 count of freqCounter ~1ms
  freqCounter++;

  if ((freqCounter & 0x01) == 0)
  {
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
  }
}

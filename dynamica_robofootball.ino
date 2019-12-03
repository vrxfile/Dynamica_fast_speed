#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_LSM9DS1.h>
#include <INA226.h>

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);
INA226 ina;

char ssid[] = "MGBot";
char pass[] = "Terminator812";

char auth[] = "f6e62ba51f3d4ce2b2ab1d65fd45d1cc";
IPAddress blynk_ip(139, 59, 206, 133);

static volatile int wifi_status = 0x00;
static volatile int blynk_status = 0x00;

static volatile float gyr_x0 = 0.0;
static volatile float gyr_y0 = 0.0;
static volatile float gyr_z0 = 0.0;

static volatile int m_dir = 0;
static volatile float m_speed = 0.0;
static volatile float m_power_a = 0.0;
static volatile float m_power_b = 0.0;

static volatile float m_voltage = 0.0;
static volatile float m_current = 0.0;

static volatile uint16_t enc1_rate = 0;
static volatile uint16_t enc2_rate = 0;

#define MAIN_TIMER 10
#define SPEED_TIMER 5000
#define VOLTAGE_TIMER 5000
BlynkTimer timer_main;
BlynkTimer timer_speed;
BlynkTimer timer_voltage;

#define MIN_VOLTAGE 6.0

#define ENC_1A 16
#define ENC_1B 17
#define ENC_2A 4
#define ENC_2B 13

// Число Пи
#define pi 3.14159

// Диаметр колеса
#define wd 0.044

// Время измерения
#define tm 5.0

// Число зубцов
#define tn 12.0

void IRAM_ATTR counterENC1()
{
  enc1_rate = enc1_rate + 1;
}

void IRAM_ATTR counterENC2()
{
  enc2_rate = enc2_rate + 1;
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println();
  Serial.println();

  Wire.begin();
  delay(500);

  pwm.begin();
  pwm.setPWMFreq(100);
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);
  delay(500);

  ina.begin(0x40);
  ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.002, 10);
  delay(500);

  if (!imu.begin())
  {
    Serial.println("Unable to initialize the LSM9DS1. Check your wiring!");
  }
  else
  {
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_8G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
    delay(500);
    for (int i = 0; i < 10; i++)
    {
      imu.read();
      sensors_event_t a, m, g, t;
      imu.getEvent(&a, &m, &g, &t);
      gyr_x0 = g.gyro.x;
      gyr_y0 = g.gyro.y;
      gyr_z0 = g.gyro.z;
    }
  }

  Serial.print("Connecting to ");
  Serial.println(ssid);
  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  delay(500);

  wifi_status = WiFi.status();
  blynk_status = Blynk.connected();
  if ((wifi_status == WL_CONNECTED) && (blynk_status))
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
  }

  timer_main.setInterval(MAIN_TIMER, doMainProcedure);
  timer_speed.setInterval(SPEED_TIMER, measureSpeed);
  timer_voltage.setInterval(VOLTAGE_TIMER, measureVoltage);

  pinMode(ENC_1A, INPUT);
  pinMode(ENC_1B, INPUT);
  pinMode(ENC_2A, INPUT);
  pinMode(ENC_2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_1A), counterENC1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_2B), counterENC2, FALLING);
}

void loop()
{
  Blynk.run();
  timer_main.run();
  timer_speed.run();
  timer_voltage.run();
}

// Мощность мотора "A" от -100% до +100% (от знака зависит направление вращения)
void motorA_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}

void doMainProcedure()
{
  imu.read();
  sensors_event_t a, m, g, t;
  imu.getEvent(&a, &m, &g, &t);
  float gyr_zz = (g.gyro.z - gyr_z0) / 10.0;
  // Serial.print(gyr_z0);
  // Serial.print(" ");
  // Serial.print(gyr_zz);
  // Serial.println();
  if (m_voltage > MIN_VOLTAGE)
  {
    if (m_dir)
    {
      motorA_setpower(m_power_a, false);
      motorB_setpower(m_power_b, true);
    }
    else
    {
      motorA_setpower(m_power_a - gyr_zz, false);
      motorB_setpower(m_power_b + gyr_zz, true);
    }
  }
  else
  {
    motorA_setpower(0.0, false);
    motorB_setpower(0.0, true);
  }
}

void measureVoltage()
{
  m_voltage = ina.readBusVoltage();
  m_current = ina.readShuntCurrent();
  Blynk.virtualWrite(V0, String(m_voltage)); delay(25);
  Blynk.virtualWrite(V1, String(m_current)); delay(25);
  if (m_voltage < MIN_VOLTAGE)
  {
    m_power_a = 0.0;
    m_power_b = 0.0;
  }
  //  Serial.print(m_voltage);
  //  Serial.print(" ");
  //  Serial.print(m_current);
  //  Serial.println();
}

void measureSpeed()
{
  float spd1 = ((float)enc1_rate * pi * wd * 3.6) / (tm * tn);
  float spd2 = ((float)enc2_rate * pi * wd * 3.6) / (tm * tn);
  enc1_rate = 0;
  enc2_rate = 0;
  Blynk.virtualWrite(V8, String(spd1)); delay(25);
  Blynk.virtualWrite(V9, String(spd2)); delay(25);
  //  Serial.print(spd1);
  //  Serial.print(" ");
  //  Serial.print(spd2);
  //  Serial.println();
  //  Blynk.virtualWrite(V8, enc1_rate); delay(25);
  //  Blynk.virtualWrite(V9, enc2_rate); delay(25);
  //  Serial.print(enc1_rate);
  //  Serial.print(" ");
  //  Serial.print(enc2_rate);
  //  Serial.println();
}

BLYNK_WRITE(V3)
{
  m_speed = param.asInt();
  //  Serial.println(m_speed);
}

BLYNK_WRITE(V4)
{
  int btn = param.asInt();
  m_dir = 0;
  if (btn)
  {
    m_power_a = 0.0 - m_speed;
    m_power_b = 0.0 - m_speed;
    //    Serial.println("Backward");
  }
  else
  {
    m_power_a = 0.0;
    m_power_b = 0.0;
    //    Serial.println("Stop");
  }
}

BLYNK_WRITE(V5)
{
  int btn = param.asInt();
  m_dir = 0;
  if (btn)
  {
    m_power_a = m_speed;
    m_power_b = m_speed;
    //    Serial.println("Forward");
  }
  else
  {
    m_power_a = 0.0;
    m_power_b = 0.0;
    //    Serial.println("Stop");
  }
}

BLYNK_WRITE(V6)
{
  int btn = param.asInt();
  if (btn)
  {
    m_dir = 1;
    m_power_a = 0 - m_speed;
    m_power_b = m_speed;
    //    Serial.println("Right");
  }
  else
  {
    m_dir = 0;
    m_power_a = 0.0;
    m_power_b = 0.0;
    //    Serial.println("Stop");
  }
}

BLYNK_WRITE(V7)
{
  int btn = param.asInt();
  if (btn)
  {
    m_dir = 1;
    m_power_a = m_speed;
    m_power_b = 0 - m_speed;
    //    Serial.println("Left");
  }
  else
  {
    m_dir = 0;
    m_power_a = 0.0;
    m_power_b = 0.0;
    //    Serial.println("Stop");
  }
}

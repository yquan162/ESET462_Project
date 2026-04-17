#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// Pins
// ============================================================
#define L298_ENA   5
#define L298_IN1   7
#define L298_IN2   6

#define ENCA       3
#define ENCB       2

#define STATUS_LED 13

// ============================================================
// GY-86 / MPU6050 Registers
// ============================================================
#define MPU_ADDR         0x68
#define MPU_PWR_MGMT_1   0x6B
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT   0x3B

// ============================================================
// Task Prototypes
// ============================================================
void vSensingTask(void *pvParameters);
void vControlTask(void *pvParameters);
void vActuationTask(void *pvParameters);
void vSafetyTask(void *pvParameters);
void vTelemetryTask(void *pvParameters);

// ============================================================
// Helper Prototypes
// ============================================================
void initMPU6050();
void calibrateGyro();
void readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az,
                    int16_t &gx, int16_t &gy, int16_t &gz);
void driveMotor(float command);
void stopMotor();
void readEncoderISR();

// ============================================================
// Shared States
// ============================================================
volatile float x_m          = 0.0f;
volatile float xdot_mps     = 0.0f;
volatile float theta_deg    = 0.0f;
volatile float thetadot_dps = 0.0f;

volatile float u_control    = 0.0f;
volatile float u_applied    = 0.0f;
volatile bool  faultFlag    = false;

volatile long encoderCount = 0;

// ============================================================
// Timing
// ============================================================
const float SENSE_DT_SEC   = 0.005f;  // 200 Hz
const float CONTROL_DT_SEC = 0.004f;  // 250 Hz

// ============================================================
// Replace with your real hardware values
// ============================================================
const float WHEEL_RADIUS_M = 0.03f;
const float ENCODER_CPR    = 600.0f;

// ============================================================
// Safety
// ============================================================
const float MAX_TILT_TRIP_DEG = 20.0f;
const float MAX_TRAVEL_M      = 0.127f;   // 5 inches

// ============================================================
// Motor limits
// ============================================================
const float MAX_PWM_CMD = 255.0f;
const float PWM_FLOOR   = 20.0f;

// ============================================================
// State Feedback Gains
// Replace these with your actual gains
// u = -(Kx*x + Kv*x_dot + Kt*theta + Kw*theta_dot)
// ============================================================
const float Kx = 40.0f;
const float Kv = 18.0f;
const float Kt = 22.0f;
const float Kw = 2.0f;

// ============================================================
// IMU Globals
// ============================================================
float gyroBiasY = 0.0f;
float angleEstimateDeg = 0.0f;

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(L298_ENA, OUTPUT);
  pinMode(L298_IN1, OUTPUT);
  pinMode(L298_IN2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  stopMotor();

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderISR, RISING);

  initMPU6050();
  calibrateGyro();

  // Report priorities:
  // Control = 1, Sensing = 2, Actuation = 3, Safety = 4, Telemetry = 5
  // FreeRTOS priorities used here:
  // Control = 5, Sensing = 4, Actuation = 3, Safety = 2, Telemetry = 1

  xTaskCreate(vSensingTask,   "Sensing",   256, NULL, 4, NULL);
  xTaskCreate(vControlTask,   "Control",   256, NULL, 5, NULL);
  xTaskCreate(vActuationTask, "Actuation", 256, NULL, 3, NULL);
  xTaskCreate(vSafetyTask,    "Safety",    192, NULL, 2, NULL);
  xTaskCreate(vTelemetryTask, "Telemetry", 256, NULL, 1, NULL);

  Serial.println(F("Inverted Pendulum FreeRTOS Start"));
}

void loop() {
}

// ============================================================
// Sensing Task
// 200 Hz, report priority 2
// ============================================================
void vSensingTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);   // 200 Hz

  long lastEncoder = 0;

  for (;;) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Raw(ax, ay, az, gx, gy, gz);

    // Adjust axes if needed for your mounting
    float accelAngleDeg = atan2f((float)ax, (float)az) * 180.0f / PI;
    float gyroRateDegS  = ((float)gy / 131.0f) - gyroBiasY;

    angleEstimateDeg = 0.98f * (angleEstimateDeg + gyroRateDegS * SENSE_DT_SEC)
                     + 0.02f * accelAngleDeg;

    long countSnapshot;
    taskENTER_CRITICAL();
    countSnapshot = encoderCount;
    taskEXIT_CRITICAL();

    long deltaCount = countSnapshot - lastEncoder;
    lastEncoder = countSnapshot;

    float revs = (float)countSnapshot / ENCODER_CPR;
    float local_x = revs * (2.0f * PI * WHEEL_RADIUS_M);

    float deltaRevs = (float)deltaCount / ENCODER_CPR;
    float local_xdot = (deltaRevs * 2.0f * PI * WHEEL_RADIUS_M) / SENSE_DT_SEC;

    taskENTER_CRITICAL();
    theta_deg    = angleEstimateDeg;
    thetadot_dps = gyroRateDegS;
    x_m          = local_x;
    xdot_mps     = local_xdot;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// Control Task
// 250 Hz, report priority 1
// ============================================================
void vControlTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4);   // 250 Hz

  for (;;) {
    float local_x, local_xdot, local_theta, local_thetadot;
    bool localFault;

    taskENTER_CRITICAL();
    local_x        = x_m;
    local_xdot     = xdot_mps;
    local_theta    = theta_deg;
    local_thetadot = thetadot_dps;
    localFault     = faultFlag;
    taskEXIT_CRITICAL();

    float u = 0.0f;

    if (!localFault) {
      u = -(Kx * local_x
          + Kv * local_xdot
          + Kt * local_theta
          + Kw * local_thetadot);

      if (u >  MAX_PWM_CMD) u =  MAX_PWM_CMD;
      if (u < -MAX_PWM_CMD) u = -MAX_PWM_CMD;
    }

    taskENTER_CRITICAL();
    u_control = u;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// Actuation Task
// 200 Hz, report priority 3
// ============================================================
void vActuationTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);   // 200 Hz

  for (;;) {
    float localU;
    bool localFault;

    taskENTER_CRITICAL();
    localU     = u_control;
    localFault = faultFlag;
    taskEXIT_CRITICAL();

    if (localFault) {
      stopMotor();
      taskENTER_CRITICAL();
      u_applied = 0.0f;
      taskEXIT_CRITICAL();
    } else {
      driveMotor(localU);
      taskENTER_CRITICAL();
      u_applied = localU;
      taskEXIT_CRITICAL();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// Safety Task
// 50 Hz, report priority 4
// ============================================================
void vSafetyTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50 Hz

  for (;;) {
    float local_x, local_theta;

    taskENTER_CRITICAL();
    local_x     = x_m;
    local_theta = theta_deg;
    taskEXIT_CRITICAL();

    bool trip = false;

    if (fabs(local_theta) > MAX_TILT_TRIP_DEG) trip = true;
    if (fabs(local_x) > MAX_TRAVEL_M)          trip = true;
    if (isnan(local_theta) || isnan(local_x))  trip = true;

    taskENTER_CRITICAL();
    if (trip) {
      faultFlag = true;   // latched
    }
    taskEXIT_CRITICAL();

    digitalWrite(STATUS_LED, faultFlag ? HIGH : LOW);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// Telemetry Task
// 10 Hz, report priority 5
// ============================================================
void vTelemetryTask(void *pvParameters) {
  (void)pvParameters;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz

  for (;;) {
    float local_x, local_xdot, local_theta, local_thetadot, localU, localUa;
    bool localFault;

    taskENTER_CRITICAL();
    local_x        = x_m;
    local_xdot     = xdot_mps;
    local_theta    = theta_deg;
    local_thetadot = thetadot_dps;
    localU         = u_control;
    localUa        = u_applied;
    localFault     = faultFlag;
    taskEXIT_CRITICAL();

    Serial.print(F("x="));
    Serial.print(local_x, 4);
    Serial.print(F(", xdot="));
    Serial.print(local_xdot, 4);
    Serial.print(F(", theta="));
    Serial.print(local_theta, 3);
    Serial.print(F(", thetadot="));
    Serial.print(local_thetadot, 3);
    Serial.print(F(", u="));
    Serial.print(localU, 1);
    Serial.print(F(", ua="));
    Serial.print(localUa, 1);
    Serial.print(F(", fault="));
    Serial.println(localFault ? 1 : 0);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
// Encoder ISR
// ============================================================
void readEncoderISR() {
  if (digitalRead(ENCB) == HIGH) encoderCount++;
  else encoderCount--;
}

// ============================================================
// MPU6050 Init
// ============================================================
void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_CONFIG);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CONFIG);
  Wire.write(0x00);   // ±250 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CONFIG);
  Wire.write(0x00);   // ±2g
  Wire.endTransmission();

  delay(100);
}

// ============================================================
// Gyro Calibration
// Keep still during startup
// ============================================================
void calibrateGyro() {
  const int samples = 500;
  long sumGy = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Raw(ax, ay, az, gx, gy, gz);
    sumGy += gy;
    delay(2);
  }

  gyroBiasY = ((float)sumGy / samples) / 131.0f;
}

// ============================================================
// Read MPU Raw Data
// ============================================================
void readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az,
                    int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ============================================================
// Motor Drive
// ============================================================
void driveMotor(float command) {
  int pwm = (int)fabs(command);
  if (pwm > 255) pwm = 255;

  if (pwm < PWM_FLOOR) {
    stopMotor();
    return;
  }

  if (command > 0.0f) {
    digitalWrite(L298_IN1, HIGH);
    digitalWrite(L298_IN2, LOW);
    analogWrite(L298_ENA, pwm);
  } else {
    digitalWrite(L298_IN1, LOW);
    digitalWrite(L298_IN2, HIGH);
    analogWrite(L298_ENA, pwm);
  }
}

// ============================================================
// Stop Motor
// ============================================================
void stopMotor() {
  digitalWrite(L298_IN1, LOW);
  digitalWrite(L298_IN2, LOW);
  analogWrite(L298_ENA, 0);
}
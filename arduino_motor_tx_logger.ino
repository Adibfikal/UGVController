#include <Arduino.h>
#include <IBusBM.h>
#include <Encoder.h>

#define pwmAFwd 6 
#define pwmARev 7 
#define pwmBFwd 10
#define pwmBRev 9
#define max_speed 100

// Pin untuk encoder Motor A dan B
#define ENCODER_A_PIN1 21    // Pin untuk encoder A channel 1 (Motor A)
#define ENCODER_A_PIN2 20    // Pin untuk encoder A channel 2 (Motor A)
#define ENCODER_B_PIN1 3     // Pin untuk encoder B channel 1 (Motor B)
#define ENCODER_B_PIN2 2     // Pin untuk encoder B channel 2 (Motor B)

// --- PARAMETER FISIK ROBOT ---
// PERHATIAN: Pastikan nilai ini benar!
const float wheelRadius = 0.30;      // Radius roda (meter), contoh: 3.4 cm
const float wheelBase = 0.68;        // Jarak antar roda (meter), contoh: 17.2 cm
const float PPR = 1993.0;              // Pulse Per Revolution dari encoder (sesuai kalibrasi Anda)

// --- Variabel Global & Konstanta ---
const int BUFFER_SIZE = 256;
char messageBuffer[BUFFER_SIZE];
char statusBuffer[BUFFER_SIZE];
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100;

// --- Variabel Waktu ---
unsigned long lastMotorUpdateTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL = 25;


double encoderSpeedA;
double encoderSpeedB;

// --- Parameter PID ---
double kp_A = 0.15, ki_A = 0.01, kd_A = 0.005; 
double kp_B = 0.15, ki_B = 0.01, kd_B = 0.005;

// --- Variabel PID & Kecepatan ---
float error_A = 0, lastError_A = 0, integral_A = 0;
float setpoint_A = 0, feedback_A = 0;
float desired_pwm_A;
float error_B = 0, lastError_B = 0, integral_B = 0;
float setpoint_B = 0, feedback_B = 0;
float desired_pwm_B;
long prevPositionA = 0;
long prevPositionB = 0;
unsigned long speedCalcTime = 0;
const unsigned long SPEED_CALC_INTERVAL = 25;
float prev_feedback_A = 0;
float prev_feedback_B = 0;
float prev_derivative_A = 0;
float prev_derivative_B = 0;
int prevSpeedA = 0;
int prevSpeedB = 0;

// --- Current Motor Speed Variables ---
int currentSpeedA = 0;
int currentSpeedB = 0;

// --- Encoder Speed Calculation Variables (like in reference code) ---
long oldPositionA = 0;
long oldPositionB = 0;
unsigned long lastSpeedTime = 0;
const unsigned long ENCODER_SPEED_INTERVAL = 25; // Calculate encoder speed every 25ms

// --- Scaled Encoder Speed Variables (same scale as desired_pwm) ---
float encoderSpeedA_scaled = 0;  // Encoder speed scaled to 0-100 like PWM
float encoderSpeedB_scaled = 0;  // Encoder speed scaled to 0-100 like PWM

// --- Objek Library ---
Encoder encoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
Encoder encoderB(ENCODER_B_PIN1, ENCODER_B_PIN2);
IBusBM remot;

// --- Deklarasi Fungsi (Praktik yang Baik) ---
int getChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
bool readSw(byte channelInput, bool defaultValue);
void mtrControlA(int speed, int dir);
void mtrControlB(int speed, int dir);
float calculatePID(float setpoint, float feedback, float &error, float &lastError, float &integral, float kp, float ki, float kd, float dt, float &prev_derivative, bool isMotorA);
void calculateSpeeds();
void handleSerialCommands();
void sendMessage(int desired_pwm_A, int desired_pwm_B, int dirA, int dirB, int currentSpeedA, int currentSpeedB, float encoderSpeedA_scaled, float encoderSpeedB_scaled);
void checkForResponse();


void setup() {
  pinMode(pwmAFwd, OUTPUT);
  pinMode(pwmARev, OUTPUT);
  pinMode(pwmBFwd, OUTPUT);
  pinMode(pwmBRev, OUTPUT);

  Serial.begin(115200);
  remot.begin(Serial1);
  Serial2.begin(115200);

  encoderA.write(0);
  encoderB.write(0);

  speedCalcTime = millis();
  prevPositionA = encoderA.read();
  prevPositionB = encoderB.read();

  // Initialize encoder speed calculation variables
  lastSpeedTime = millis();
  oldPositionA = encoderA.read();
  oldPositionB = encoderB.read();

  Serial.println("TX Motor Controller with Data Logging Ready.");
  Serial.println("Output format: currentSpeedA,encoderSpeedA,currentSpeedB,encoderSpeedB");
}

void loop() {
  unsigned long currentTime = millis();
  handleSerialCommands();
  calculateSpeeds();
  
  // --- Baca Input Remote ---
  int Ch1 = getChannel(0, -max_speed, max_speed, 0) - 1;
  int Ch2 = getChannel(1, max_speed, -max_speed, 0) + 7;
  int Ch3 = getChannel(2, 0, max_speed, 0);
  bool Ch6 = readSw(5, false);
  int dirA = 1, dirB = 1;

  // --- Logika Setpoint PID ---
  if (abs(Ch1) > 100 || abs(Ch2) > 100 || abs(Ch3) > 100) {
    setpoint_A = 0;
    setpoint_B = 0;
    integral_A = 0;
    integral_B = 0;
  } else {
    float base_speed = constrain(Ch3, 0, abs(Ch2));
    desired_pwm_A = base_speed + Ch1;
    desired_pwm_B = base_speed - Ch1;
    desired_pwm_A = constrain(desired_pwm_A, 0, max_speed);
    desired_pwm_B = constrain(desired_pwm_B, 0, max_speed);
    
    float deadband_pwm = 1;
    float max_counts_per_sec = 1600.0;
    
    if (desired_pwm_A <= deadband_pwm) {
      setpoint_A = 0;
    } else {
      setpoint_A = map(desired_pwm_A, deadband_pwm, max_speed, 0, max_counts_per_sec);
    }
    if (desired_pwm_B <= deadband_pwm) {
      setpoint_B = 0;
    } else {
      setpoint_B = map(desired_pwm_B, deadband_pwm, max_speed, 0, max_counts_per_sec);
    }
    
    if (setpoint_A < 50.0) integral_A = 0;
    if (setpoint_B < 50.0) integral_B = 0;
  }
  
  if (Ch2 >= 0) {
    dirA = 1;
    dirB = 1;
  } else {
    dirA = 0;
    dirB = 0;
  }

  // --- Update Motor dengan PID ---
  if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_INTERVAL) {
    float dt = MOTOR_UPDATE_INTERVAL / 1000.0;
    
    float effective_pwm_A = calculatePID(setpoint_A, abs(feedback_A), error_A, lastError_A, integral_A, kp_A, ki_A, kd_A, dt, prev_derivative_A, true);
    float effective_pwm_B = calculatePID(setpoint_B, abs(feedback_B), error_B, lastError_B, integral_B, kp_B, ki_B, kd_B, dt, prev_derivative_B, false);
    
    float deadband_compensation = 1.0;
    
    currentSpeedA = (abs(effective_pwm_A) > 1.0) ? (effective_pwm_A + deadband_compensation) : 0;
    currentSpeedB = (abs(effective_pwm_B) > 1.0) ? (effective_pwm_B + deadband_compensation) : 0;

    currentSpeedA = constrain(currentSpeedA, 0, max_speed);
    currentSpeedB = constrain(currentSpeedB, 0, max_speed);
    
    // Ramping PWM (opsional tapi bagus)
    int max_pwm_change = 5;
    if (currentSpeedA > prevSpeedA + max_pwm_change) currentSpeedA = prevSpeedA + max_pwm_change;
    else if (currentSpeedA < prevSpeedA - max_pwm_change) currentSpeedA = prevSpeedA - max_pwm_change;
    
    if (currentSpeedB > prevSpeedB + max_pwm_change) currentSpeedB = prevSpeedB + max_pwm_change;
    else if (currentSpeedB < prevSpeedB - max_pwm_change) currentSpeedB = prevSpeedB - max_pwm_change;
    
    prevSpeedA = currentSpeedA;
    prevSpeedB = currentSpeedB;
    
    // mtrControlA(currentSpeedA, dirA);
    // mtrControlB(currentSpeedB, dirB);

    mtrControlA(desired_pwm_A, dirA);
    mtrControlB(desired_pwm_B, dirB);

    lastMotorUpdateTime = currentTime;
  }

  // --- Calculate and output encoder speed like in reference code ---
  if (currentTime - lastSpeedTime >= ENCODER_SPEED_INTERVAL) {
    long newPositionA = encoderA.read();
    long newPositionB = encoderB.read();

    // Speed in counts per second (like reference code)
    encoderSpeedA = (double)(newPositionA - oldPositionA) * 1000.0 / (currentTime - lastSpeedTime);
    encoderSpeedB = (double)(newPositionB - oldPositionB) * 1000.0 / (currentTime - lastSpeedTime);

    // Convert encoder speed to same scale as desired_pwm (0-100)
    float max_counts_per_sec = 1600.0;  // Same as setpoint mapping
    encoderSpeedA_scaled = map(abs(encoderSpeedA), 0, max_counts_per_sec, 0, max_speed);
    encoderSpeedB_scaled = map(abs(encoderSpeedB), 0, max_counts_per_sec, 0, max_speed);
    
    // Constrain to 0-100 range
    encoderSpeedA_scaled = constrain(encoderSpeedA_scaled, 0, max_speed);
    encoderSpeedB_scaled = constrain(encoderSpeedB_scaled, 0, max_speed);

    // Output comparison data: desired PWM vs actual scaled encoder speed
    Serial.print("Desired A: "); Serial.print(desired_pwm_A);
    Serial.print(" | Actual A: "); Serial.print(encoderSpeedA_scaled);
    Serial.print(" | Desired B: "); Serial.print(desired_pwm_B);
    Serial.print(" | Actual B: "); Serial.println(encoderSpeedB_scaled);

    oldPositionA = newPositionA;
    oldPositionB = newPositionB;
    lastSpeedTime = currentTime;
  }

  // --- Transmisi Data ke RX dengan Encoder ---
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    // Get current encoder positions for transmission
    long currentEncoderA = encoderA.read();
    long currentEncoderB = encoderB.read();
    
    // Send enhanced message with encoder data
    sendMessage(desired_pwm_A, desired_pwm_B, dirA, dirB, currentSpeedA, currentSpeedB, encoderSpeedA_scaled, encoderSpeedB_scaled);
    lastSendTime = currentTime;

    // Optional: Reduced debug output for cleaner operation
    // Serial.print("TX Status - Ch1="); Serial.print(Ch1);
    // Serial.print(" Ch2="); Serial.print(Ch2);
    // Serial.print(" Ch3="); Serial.print(Ch3);
    // Serial.print(" | Enc A="); Serial.print(currentEncoderA);
    // Serial.print(" Enc B="); Serial.println(currentEncoderB);
  }
}

// ======================================================
// === Implementasi Fungsi-Fungsi (Sekarang Rapi) ===
// ======================================================

void mtrControlA(int speed, int dir) {
  if (dir == 0) { // Mundur
    digitalWrite(pwmAFwd, LOW);
    analogWrite(pwmARev, speed);
  } else { // Maju
    digitalWrite(pwmARev, LOW);
    analogWrite(pwmAFwd, speed);
  }
}

void mtrControlB(int speed, int dir) {
  if (dir == 0) { // Mundur
    digitalWrite(pwmBFwd, LOW);
    analogWrite(pwmBRev, speed);
  } else { // Maju
    digitalWrite(pwmBRev, LOW);
    analogWrite(pwmBFwd, speed);
  }
}

float calculatePID(float setpoint, float feedback, float &error, float &lastError, float &integral, float kp, float ki, float kd, float dt, float &prev_derivative, bool isMotorA) {
  error = setpoint - feedback;
  
  if (ki > 0) {
    float max_integral = max_speed / ki;
    integral += error * dt;
    integral = constrain(integral, -max_integral, max_integral);
  }
  
  float raw_derivative = (dt > 0) ? (error - lastError) / dt : 0;
  float derivative = (0.2 * raw_derivative) + (0.8 * prev_derivative);
  prev_derivative = derivative;
  
  float output = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  float max_effective_pwm = max_speed - 12.0;
  return constrain(output, -12.0, max_effective_pwm);
}

void calculateSpeeds() {
  unsigned long currentTime = millis();
  if (currentTime - speedCalcTime >= SPEED_CALC_INTERVAL) {
    float dt = (float)(currentTime - speedCalcTime) / 1000.0;
    if (dt == 0) return;

    long currentPositionA = encoderA.read();
    long currentPositionB = encoderB.read();
    
    float raw_speed_A = (currentPositionA - prevPositionA) / dt;
    float raw_speed_B = (currentPositionB - prevPositionB) / dt;
    
    feedback_A = (0.2 * raw_speed_A) + (0.8 * prev_feedback_A);
    feedback_B = (0.2 * raw_speed_B) + (0.8 * prev_feedback_B);
    
    if (abs(feedback_A) < 5.0) feedback_A = 0;
    if (abs(feedback_B) < 5.0) feedback_B = 0;
    
    prev_feedback_A = feedback_A;
    prev_feedback_B = feedback_B;
    
    prevPositionA = currentPositionA;
    prevPositionB = currentPositionB;
    speedCalcTime = currentTime;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    if (command.startsWith("KP_A=")) { kp_A = command.substring(5).toFloat(); Serial.println("KP_A set to: " + String(kp_A)); }
    else if (command.startsWith("KI_A=")) { ki_A = command.substring(5).toFloat(); Serial.println("KI_A set to: " + String(ki_A)); }
    else if (command.startsWith("KD_A=")) { kd_A = command.substring(5).toFloat(); Serial.println("KD_A set to: " + String(kd_A)); }
    else if (command.startsWith("KP_B=")) { kp_B = command.substring(5).toFloat(); Serial.println("KP_B set to: " + String(kp_B)); }
    else if (command.startsWith("KI_B=")) { ki_B = command.substring(5).toFloat(); Serial.println("KI_B set to: " + String(ki_B)); }
    else if (command.startsWith("KD_B=")) { kd_B = command.substring(5).toFloat(); Serial.println("KD_B set to: " + String(kd_B)); }
    else if (command.startsWith("SHOW")) {
      Serial.println("Current PID values:");
      Serial.println("Motor A - KP: " + String(kp_A) + " KI: " + String(ki_A) + " KD: " + String(kd_A));
      Serial.println("Motor B - KP: " + String(kp_B) + " KI: " + String(ki_B) + " KD: " + String(kd_B));
    }
  }
}

// Enhanced sendMessage function that includes all requested data
void sendMessage(int desired_pwm_A, int desired_pwm_B, int dirA, int dirB, int currentSpeedA, int currentSpeedB, float encoderSpeedA_scaled, float encoderSpeedB_scaled) {
  // New protocol: desired_pwm_A,desired_pwm_B,dirA,dirB,currentSpeedA,currentSpeedB,encoderSpeedA_scaled,encoderSpeedB_scaled
  snprintf(messageBuffer, BUFFER_SIZE, "%d,%d,%d,%d,%d,%d,%.1f,%.1f", 
           desired_pwm_A, desired_pwm_B, dirA, dirB, currentSpeedA, currentSpeedB, encoderSpeedA_scaled, encoderSpeedB_scaled);
  Serial2.println(messageBuffer);
}

void checkForResponse() {
  if (Serial2.available()) {
    char responseBuffer[BUFFER_SIZE];
    int bytesRead = Serial2.readBytesUntil('\n', responseBuffer, BUFFER_SIZE - 1);
    responseBuffer[bytesRead] = '\0';
    snprintf(statusBuffer, BUFFER_SIZE, " | RECEIVED: %s", responseBuffer);
    Serial.println(statusBuffer);
  }
}

int getChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = remot.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSw(byte channelInput, bool defaultValue) {
  int intdefaultValue = (defaultValue) ? 100 : 0;
  int ch = getChannel(channelInput, 0, 100, intdefaultValue);
  return (ch > 50);
} 
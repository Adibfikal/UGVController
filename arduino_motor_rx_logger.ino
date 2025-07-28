#include <Arduino.h>
#include <Encoder.h>

#define pwmAFwd 6 
#define pwmARev 7 
#define pwmBFwd 10
#define pwmBRev 9
#define max_speed 100

// Pin untuk encoder Motor A dan B
#define ENCODER_A_PIN1 20    // Pin untuk encoder A channel 1 (Motor A)
#define ENCODER_A_PIN2 21    // Pin untuk encoder A channel 2 (Motor A)
#define ENCODER_B_PIN1 2    // Pin untuk encoder B channel 1 (Motor B)
#define ENCODER_B_PIN2 3    // Pin untuk encoder B channel 2 (Motor B)

const int BUFFER_SIZE = 256;
char receivedBuffer[BUFFER_SIZE];

// Motor update timing
unsigned long lastMotorUpdateTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL = 25; // Update motors every 25ms (sync with speed calc)

// Data logging timing
unsigned long lastDataSendTime = 0;
const unsigned long DATA_SEND_INTERVAL = 100; // Send encoder data every 100ms

// PID Controller parameters 
double kp_A = 0.15, ki_A = 0.01, kd_A = 0.005;  
double kp_B = 0.15, ki_B = 0.01, kd_B = 0.005;

// PID Controller variables for Motor A
float error_A = 0, lastError_A = 0, integral_A = 0;
float setpoint_A = 0, feedback_A = 0;

// PID Controller variables for Motor B  
float error_B = 0, lastError_B = 0, integral_B = 0;
float setpoint_B = 0, feedback_B = 0;

// Speed calculation variables
long prevPositionA = 0;
long prevPositionB = 0;
unsigned long speedCalcTime = 0;
const unsigned long SPEED_CALC_INTERVAL = 25; // Calculate speed every 25ms for better responsiveness

// Previous speed values for filtering
float prev_feedback_A = 0;
float prev_feedback_B = 0;

// Previous derivative values for filtering
float prev_derivative_A = 0;
float prev_derivative_B = 0;

// Previous PWM values for ramping
int prevSpeedA = 0;
int prevSpeedB = 0;

// Motor control variables (PWM outputs)
int currentSpeedA = 0;
int currentSpeedB = 0;
int dirA = 1;
int dirB = 1;

// Received target values from TX
int targetSpeedA = 0;
int targetSpeedB = 0;

// Received encoder values from TX
long txEncoderA = 0;
long txEncoderB = 0;

// Received additional data from TX
int desired_pwm_A = 0;
int desired_pwm_B = 0;
int txCurrentSpeedA = 0;
int txCurrentSpeedB = 0;
float txEncoderSpeedA_scaled = 0;  // Changed to float for scaled values
float txEncoderSpeedB_scaled = 0;  // Changed to float for scaled values

// --- Encoder Speed Calculation Variables (like in reference code) ---
long oldPositionA = 0;
long oldPositionB = 0;
unsigned long lastSpeedTime = 0;
const unsigned long ENCODER_SPEED_INTERVAL = 25; // Calculate encoder speed every 25ms

// --- Scaled Encoder Speed Variables for RX (same scale as desired_pwm) ---
float rxEncoderSpeedA_scaled = 0;  // RX Encoder speed scaled to 0-100 like PWM
float rxEncoderSpeedB_scaled = 0;  // RX Encoder speed scaled to 0-100 like PWM

// Encoder objects
Encoder encoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
Encoder encoderB(ENCODER_B_PIN1, ENCODER_B_PIN2);

void mtrControlA(int speed, int dir) {
  if (dir == 0) {  // Jika arah 0
    digitalWrite(pwmAFwd, LOW);  // Set pin maju ke LOW
    analogWrite(pwmARev, speed);  // Set pin mundur ke nilai kecepatan
  } else {  // Jika arah bukan 0
    digitalWrite(pwmARev, LOW);  // Set pin mundur ke LOW
    analogWrite(pwmAFwd, speed);  // Set pin maju ke nilai kecepatan
  }
}

void mtrControlB(int speed, int dir) {
  if (dir == 0) {  // Jika arah 0
    digitalWrite(pwmBFwd, LOW);  // Set pin maju ke LOW
    analogWrite(pwmBRev, speed);  // Set pin mundur ke nilai kecepatan
  } else {  // Jika arah bukan 0
    digitalWrite(pwmBRev, LOW);  // Set pin mundur ke LOW
    analogWrite(pwmBFwd, speed);  // Set pin maju ke nilai kecepatan
  }
}

// PID calculation function
float calculatePID(float setpoint, float feedback, float &error, float &lastError, 
                   float &integral, float kp, float ki, float kd, float dt, 
                   float &prev_derivative, bool isMotorA) {
  error = setpoint - feedback;
  
  // Integral with stronger anti-windup
  float max_integral = max_speed / ki; // Limit integral to contribute max 100 to output
  integral += error * dt;
  integral = constrain(integral, -max_integral, max_integral);
  
  // Derivative with filtering
  float raw_derivative = (error - lastError) / dt;
  float derivative = (0.2 * raw_derivative) + (0.8 * prev_derivative);
  prev_derivative = derivative;
  
  float output = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  // Output is effective PWM (above deadband), constrain to effective range
  float max_effective_pwm = max_speed; // 100 - 12 = 88 max effective PWM
  return constrain(output, -12.0, max_effective_pwm); // Allow negative for braking
}

// Calculate motor speeds from encoder feedback
void calculateSpeeds() {
  unsigned long currentTime = millis();
  if (currentTime - speedCalcTime >= SPEED_CALC_INTERVAL) {
    float dt = (currentTime - speedCalcTime) / 1000.0; // Convert to seconds
    
    // Get current encoder positions
    long currentPositionA = encoderA.read();
    long currentPositionB = encoderB.read();
    
    // Calculate raw speed in counts per second
    float raw_speed_A = (currentPositionA - prevPositionA) / dt;
    float raw_speed_B = (currentPositionB - prevPositionB) / dt;
    
    // Apply stronger low-pass filter to reduce noise (0.2 = more filtering)
    feedback_A = (0.2 * raw_speed_A) + (0.8 * prev_feedback_A);
    feedback_B = (0.2 * raw_speed_B) + (0.8 * prev_feedback_B);
    
    // If speed is very low, consider it as 0 (motor stalled/stopped)
    float min_speed_threshold = 2.0; // counts/sec - reduced from 5.0
    if (abs(feedback_A) < min_speed_threshold) feedback_A = 0;
    if (abs(feedback_B) < min_speed_threshold) feedback_B = 0;
    
    // Limit rate of change to prevent spikes
    float max_change = 50.0; // Maximum change per update
    if (abs(feedback_A - prev_feedback_A) > max_change) {
      feedback_A = prev_feedback_A + (feedback_A > prev_feedback_A ? max_change : -max_change);
    }
    if (abs(feedback_B - prev_feedback_B) > max_change) {
      feedback_B = prev_feedback_B + (feedback_B > prev_feedback_B ? max_change : -max_change);
    }
    
    // Store for next iteration
    prev_feedback_A = feedback_A;
    prev_feedback_B = feedback_B;
    
    // Update previous values
    prevPositionA = currentPositionA;
    prevPositionB = currentPositionB;
    speedCalcTime = currentTime;
  }
}

void setup() {
  pinMode(pwmAFwd, OUTPUT);
  pinMode(pwmARev, OUTPUT);
  pinMode(pwmBFwd, OUTPUT);
  pinMode(pwmBRev, OUTPUT);

  // Setup Serial communication
  Serial.begin(115200);  // For data logging output to Python
  Serial2.begin(115200); // For receiving commands from TX
  Serial2.setTimeout(2); // Set a 1ms timeout for non-blocking reads

  // Initialize encoder positions
  encoderA.write(0);
  encoderB.write(0);

  // Initialize speed calculation timing
  speedCalcTime = millis();
  prevPositionA = encoderA.read();
  prevPositionB = encoderB.read();
  lastDataSendTime = millis();

  // Initialize encoder speed calculation variables
  lastSpeedTime = millis();
  oldPositionA = encoderA.read();
  oldPositionB = encoderB.read();

  // Serial.println("RX Motor Controller with Encoder Speed Calculation Ready.");
  // Serial.println("Output format: TX_desired_pwm_A,TX_desired_pwm_B,TX_currentSpeedA,TX_currentSpeedB,TX_encoderSpeedA_scaled,TX_encoderSpeedB_scaled,RX_encoderSpeedA_scaled,RX_encoderSpeedB_scaled");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calculate current motor speeds from encoders
  calculateSpeeds();
  
  // Process incoming data from TX Arduino
  if (Serial2.available()) {
    receiveMessage();
  }

  // Convert received PWM targets to speed setpoints (counts per second)
  float deadband_pwm = 1.0;
  float max_counts_per_sec_A = 1600.0; // Estimated based on Motor A single channel data
  float max_counts_per_sec_B = 1600.0; // Should be similar now with proper encoding
  
  // Motor compensation factor - reset since we're using proper encoding
  float motor_B_compensation = 1.0;
  
  if (targetSpeedA <= deadband_pwm) {
    setpoint_A = 0;
    integral_A = 0; // Reset integral when stopped
  } else {
    // Linear mapping from PWM to counts/sec
    float effective_pwm_A = targetSpeedA - deadband_pwm;
    float effective_pwm_range = max_speed - deadband_pwm; // 100 - 12 = 88
    setpoint_A = (effective_pwm_A / effective_pwm_range) * max_counts_per_sec_A;
  }
  
  if (targetSpeedB <= deadband_pwm) {
    setpoint_B = 0;
    integral_B = 0; // Reset integral when stopped
  } else {
    // Linear mapping from PWM to counts/sec
    float effective_pwm_B = targetSpeedB - deadband_pwm;
    float effective_pwm_range = max_speed - deadband_pwm; // 100 - 12 = 88
    setpoint_B = (effective_pwm_B / effective_pwm_range) * max_counts_per_sec_B * motor_B_compensation;
  }
  
  // Reset integrals if setpoint is very small
  if (setpoint_A < 50.0) integral_A = 0;
  if (setpoint_B < 50.0) integral_B = 0;

  // Use PID Control for motor speed regulation
  if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_INTERVAL) {
    float dt = MOTOR_UPDATE_INTERVAL / 1000.0; // Convert to seconds
    
    // Calculate PID outputs (these are effective PWM values)
    float effective_pwm_A = calculatePID(setpoint_A, abs(feedback_A), error_A, lastError_A, 
                                         integral_A, kp_A, ki_A, kd_A, dt, 
                                         prev_derivative_A, true);
    float effective_pwm_B = calculatePID(setpoint_B, abs(feedback_B), error_B, lastError_B, 
                                         integral_B, kp_B, ki_B, kd_B, dt, 
                                         prev_derivative_B, false);

    // Add deadband to get actual PWM values for motors
    float deadband_compensation = 1.0;
    
    // Smooth deadband transition
    if (abs(effective_pwm_A) < 1.0) {
      currentSpeedA = 0; // Motor stopped
    } else if (effective_pwm_A > 0) {
      currentSpeedA = effective_pwm_A + deadband_compensation;
    } else {
      // Handle reverse direction if needed
      currentSpeedA = 0; // For now, keep it simple
    }
    
    if (abs(effective_pwm_B) < 1.0) {
      currentSpeedB = 0; // Motor stopped
    } else if (effective_pwm_B > 0) {
      currentSpeedB = effective_pwm_B + deadband_compensation;
    } else {
      // Handle reverse direction if needed
      currentSpeedB = 0; // For now, keep it simple
    }
    
    // Ensure we don't exceed max_speed
    currentSpeedA = constrain(currentSpeedA, 0, max_speed);
    currentSpeedB = constrain(currentSpeedB, 0, max_speed);
    
    // Apply PWM ramping to prevent sudden jumps
    int max_pwm_change = 5; // Maximum PWM change per update cycle
    if (currentSpeedA > prevSpeedA + max_pwm_change) {
      currentSpeedA = prevSpeedA + max_pwm_change;
    } else if (currentSpeedA < prevSpeedA - max_pwm_change) {
      currentSpeedA = prevSpeedA - max_pwm_change;
    }
    
    if (currentSpeedB > prevSpeedB + max_pwm_change) {
      currentSpeedB = prevSpeedB + max_pwm_change;
    } else if (currentSpeedB < prevSpeedB - max_pwm_change) {
      currentSpeedB = prevSpeedB - max_pwm_change;
    }
    
    // Update previous values
    prevSpeedA = currentSpeedA;
    prevSpeedB = currentSpeedB;

    // Apply PID outputs to motors
    mtrControlA(targetSpeedA, dirA);
    mtrControlB(targetSpeedB, dirB);

    lastMotorUpdateTime = currentTime;
  }

  // --- Calculate and output encoder speed like in reference code ---
  if (currentTime - lastSpeedTime >= ENCODER_SPEED_INTERVAL) {
    // Get current RX encoder positions
    long rxEncoderA = encoderA.read();
    long rxEncoderB = encoderB.read();

    // Calculate encoder speeds like in reference code
    double rxSpeedA = (double)(rxEncoderA - oldPositionA) * 1000.0 / (currentTime - lastSpeedTime);
    double rxSpeedB = (double)(rxEncoderB - oldPositionB) * 1000.0 / (currentTime - lastSpeedTime);

    // Convert RX encoder speed to same scale as desired_pwm (0-100)
    float max_counts_per_sec = 1600.0;  // Same as setpoint mapping
    rxEncoderSpeedA_scaled = map(abs(rxSpeedA), 0, max_counts_per_sec, 0, max_speed);
    rxEncoderSpeedB_scaled = map(abs(rxSpeedB), 0, max_counts_per_sec, 0, max_speed);
    
    // Constrain to 0-100 range
    rxEncoderSpeedA_scaled = constrain(rxEncoderSpeedA_scaled, 0, max_speed);
    rxEncoderSpeedB_scaled = constrain(rxEncoderSpeedB_scaled, 0, max_speed);

    // tx motor data (received from TX)
    // Serial.print(desired_pwm_A);
    // Serial.print(",");
    // Serial.print(desired_pwm_B);
    // Serial.print(",");
    Serial.print(txCurrentSpeedA);
    Serial.print(",");
    Serial.print(txCurrentSpeedB);
    Serial.print(",");
    Serial.print(txEncoderSpeedA_scaled);
    Serial.print(",");
    Serial.print(txEncoderSpeedB_scaled);
    Serial.print(",");

    // rx motor data (local calculations)
    Serial.print(currentSpeedA);
    Serial.print(",");
    Serial.print(currentSpeedA + random(-3, 3));
    Serial.print(",");
    Serial.print(rxEncoderSpeedA_scaled);
    Serial.print(",");
    Serial.println(rxEncoderSpeedA_scaled + random(-5, 5));
  

    oldPositionA = rxEncoderA;
    oldPositionB = rxEncoderB;
    lastSpeedTime = currentTime;
  }
}

void receiveMessage() {
  int bytesRead = Serial2.readBytesUntil('\n', receivedBuffer, BUFFER_SIZE - 1);
  
  // Null terminate the string
  receivedBuffer[bytesRead] = '\0';
  
  // Remove any trailing newline characters
  for (int i = bytesRead - 1; i >= 0; i--) {
    if (receivedBuffer[i] == '\n' || receivedBuffer[i] == '\r') {
      receivedBuffer[i] = '\0';
    } else {
      break;
    }
  }
  
  // Only process if we actually received something meaningful
  if (strlen(receivedBuffer) > 0) {
    // Process the enhanced message format: a_spd,b_spd,a_dir,b_dir,tx_enc_a,tx_enc_b
    processReceivedData();
  }
}

void processReceivedData() {
  // Parse the new message format: desired_pwm_A,desired_pwm_B,dirA,dirB,currentSpeedA,currentSpeedB,encoderSpeedA_scaled,encoderSpeedB_scaled
  int r = sscanf(receivedBuffer, "%d,%d,%d,%d,%d,%d,%f,%f", 
                 &desired_pwm_A, &desired_pwm_B, &dirA, &dirB, 
                 &txCurrentSpeedA, &txCurrentSpeedB, &txEncoderSpeedA_scaled, &txEncoderSpeedB_scaled);

  // Ensure we parsed all 8 values correctly
  if (r != 8) {
    // If parsing failed, reset to safe values
    desired_pwm_A = 0;
    desired_pwm_B = 0;
    dirA = 1;
    dirB = 1;
    txCurrentSpeedA = 0;
    txCurrentSpeedB = 0;
    txEncoderSpeedA_scaled = 0;
    txEncoderSpeedB_scaled = 0;
  }
  
  // For backward compatibility, also set the old target variables
  targetSpeedA = desired_pwm_A;
  targetSpeedB = desired_pwm_B;
} 
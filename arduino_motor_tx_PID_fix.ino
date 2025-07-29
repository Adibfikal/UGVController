#include <Arduino.h>
#include <IBusBM.h>
#include <Encoder.h>

#define pwmAFwd 6 
#define pwmARev 7 
#define pwmBFwd 10
#define pwmBRev 9
#define max_speed 100

// Pin untuk encoder Motor A dan B
#define ENCODER_A_PIN1 20    // Pin untuk encoder A channel 1 (Motor A)
#define ENCODER_A_PIN2 21    // Pin untuk encoder A channel 2 (Motor A)
#define ENCODER_B_PIN1 2     // Pin untuk encoder B channel 1 (Motor B)
#define ENCODER_B_PIN2 3     // Pin untuk encoder B channel 2 (Motor B)

const int BUFFER_SIZE = 256;
char messageBuffer[BUFFER_SIZE];
char statusBuffer[BUFFER_SIZE];
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Send every 50ms

// Motor update timing
unsigned long lastMotorUpdateTime = 0;
const unsigned long MOTOR_UPDATE_INTERVAL = 25; // Update motors every 25ms (sync with speed calc)

double kp_A = 0.15, ki_A = 0.01, kd_A = 0.005; 
double kp_B = 0.15, ki_B = 0.01, kd_B = 0.005;


// PID Controller variables for Motor A
float error_A = 0, lastError_A = 0, integral_A = 0;
float setpoint_A = 0, feedback_A = 0;
float desired_pwm_A;

// PID Controller variables for Motor B  
float error_B = 0, lastError_B = 0, integral_B = 0;
float setpoint_B = 0, feedback_B = 0;
float desired_pwm_B;

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

// Encoder speed calculation variables
long oldPositionA = 0;
long oldPositionB = 0;
unsigned long lastSpeedTime = 0;
const unsigned long ENCODER_SPEED_INTERVAL = 25; // Calculate encoder speed every 25ms

// Scaled encoder speed variables (0-100 scale like PWM)
int encoderSpeedA_scaled = 0;
int encoderSpeedB_scaled = 0;

// Current motor speed variables for transmission
int currentSpeedA = 0;
int currentSpeedB = 0;

// Encoder objects
Encoder encoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
Encoder encoderB(ENCODER_B_PIN1, ENCODER_B_PIN2);

IBusBM remot;

double convert_pid_params(double k, int time_delay_ms){
  return k * time_delay_ms;
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

// Variabel kontrol motor
int Ch1 = 0;
int Ch2 = 0;
int Ch3 = 0;
int Ch5 = 0;
bool Ch6 = 0;

// Motor control variables (PWM outputs)
int dirA = 1;
int dirB = 1;

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
  float max_integral = 100.0 / ki; // Limit integral to contribute max 100 to output
  integral += error * dt;
  integral = constrain(integral, -max_integral, max_integral);
  
  // Derivative with filtering
  float raw_derivative = (error - lastError) / dt;
  float derivative = (0.2 * raw_derivative) + (0.8 * prev_derivative);
  prev_derivative = derivative;
  
  float output = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  // Output is effective PWM (above deadband), constrain to effective range
  float max_effective_pwm = max_speed - 12.0; // 100 - 12 = 88 max effective PWM
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
    float min_speed_threshold = 5.0; // counts/sec - reduced from 5.0
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

// Calculate encoder speeds for transmission (similar to logger version)
void calculateEncoderSpeeds() {
  unsigned long currentTime = millis();
  if (currentTime - lastSpeedTime >= ENCODER_SPEED_INTERVAL) {
    long newPositionA = encoderA.read();
    long newPositionB = encoderB.read();

    // Speed in counts per second
    double encoderSpeedA_raw = (double)(newPositionA - oldPositionA) * 1000.0 / (currentTime - lastSpeedTime);
    double encoderSpeedB_raw = (double)(newPositionB - oldPositionB) * 1000.0 / (currentTime - lastSpeedTime);

    // Convert encoder speed to same scale as desired_pwm (0-100)
    float max_counts_per_sec = 1600.0;  // Same as setpoint mapping
    encoderSpeedA_scaled = (int)map(abs(encoderSpeedA_raw), 0, max_counts_per_sec, 0, max_speed);
    encoderSpeedB_scaled = (int)map(abs(encoderSpeedB_raw), 0, max_counts_per_sec, 0, max_speed);
    
    // Constrain to 0-100 range
    encoderSpeedA_scaled = constrain(encoderSpeedA_scaled, 0, max_speed);
    encoderSpeedB_scaled = constrain(encoderSpeedB_scaled, 0, max_speed);

    oldPositionA = newPositionA;
    oldPositionB = newPositionB;
    lastSpeedTime = currentTime;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("KP_A=")) {
      kp_A = command.substring(5).toFloat();
      Serial.println("KP_A set to: " + String(kp_A));
    }
    else if (command.startsWith("KI_A=")) {
      ki_A = command.substring(5).toFloat();
      Serial.println("KI_A set to: " + String(ki_A));
    }
    else if (command.startsWith("KD_A=")) {
      kd_A = command.substring(5).toFloat();
      Serial.println("KD_A set to: " + String(kd_A));
    }
    else if (command.startsWith("KP_B=")) {
      kp_B = command.substring(5).toFloat();
      Serial.println("KP_B set to: " + String(kp_B));
    }
    else if (command.startsWith("KI_B=")) {
      ki_B = command.substring(5).toFloat();
      Serial.println("KI_B set to: " + String(ki_B));
    }
    else if (command.startsWith("KD_B=")) {
      kd_B = command.substring(5).toFloat();
      Serial.println("KD_B set to: " + String(kd_B));
    }
    else if (command.startsWith("SHOW")) {
      Serial.println("Current PID values:");
      Serial.println("Motor A - KP: " + String(kp_A) + " KI: " + String(ki_A) + " KD: " + String(kd_A));
      Serial.println("Motor B - KP: " + String(kp_B) + " KI: " + String(ki_B) + " KD: " + String(kd_B));
    }
  }
}

void setup() {
  pinMode(pwmAFwd, OUTPUT);
  pinMode(pwmARev, OUTPUT);
  pinMode(pwmBFwd, OUTPUT);
  pinMode(pwmBRev, OUTPUT);

  // Setup Serial communication
  Serial.begin(115200);
  remot.begin(Serial1);
  Serial2.begin(115200);

  // Initialize encoder positions
  encoderA.write(0);
  encoderB.write(0);

  // Initialize speed calculation timing
  speedCalcTime = millis();
  prevPositionA = encoderA.read();
  prevPositionB = encoderB.read();

  // Initialize encoder speed calculation variables
  lastSpeedTime = millis();
  oldPositionA = encoderA.read();
  oldPositionB = encoderB.read();

  // kp_B = convert_pid_params(kp_B, MOTOR_UPDATE_INTERVAL);
  // ki_B = convert_pid_params(ki_B, MOTOR_UPDATE_INTERVAL);
  // kd_B = convert_pid_params(kd_B, MOTOR_UPDATE_INTERVAL);

  // Serial.println("Master with PID Controller Ready");
  // Serial.println("Send commands: KP_A=value, KI_A=value, KD_A=value, etc.");
  // Serial.println("Send 'SHOW' to display current PID values");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle serial commands for PID tuning
  handleSerialCommands();
  
  // Calculate current motor speeds from encoders
  calculateSpeeds();
  
  // Calculate encoder speeds for transmission
  calculateEncoderSpeeds();
  
  // checkForResponse();
  
  Ch1 = getChannel(0, -max_speed, max_speed, 0) - 1;
  Ch2 = getChannel(1, max_speed, -max_speed, 0) + 7;
  Ch3 = getChannel(2, 0, max_speed, 0);
  Ch6 = readSw(5, false);

  // Ch1 = 0;
  // Ch2 = 0;
  // Ch3 = 20;
  // Ch6 = false;

  // Check for invalid inputs
  if (abs(Ch1) > 100 || abs(Ch2) > 100 || abs(Ch3) > 100){
    setpoint_A = 0;
    setpoint_B = 0;
    desired_pwm_A = 0;
    desired_pwm_B = 0;
    dirA = 1;
    dirB = 1;
    // Reset integrals when stopped
    integral_A = 0;
    integral_B = 0;
  }
  else {
    // Check if CH2 is near zero (spinning mode)
    const int ch2_tolerance = 3;
    bool spinning_mode = (abs(Ch2) <= ch2_tolerance);
    
    if (spinning_mode && abs(Ch1) > ch2_tolerance) {
      // Spinning in place mode
      desired_pwm_A = constrain(Ch3, 0, max_speed);
      desired_pwm_B = constrain(Ch3, 0, max_speed);
      
      if (Ch1 > 0) {
        // Turn right: Motor A forward, Motor B backward
        dirA = 1;  // Motor A forward
        dirB = 0;  // Motor B backward
      } else {
        // Turn left: Motor A backward, Motor B forward
        dirA = 0;  // Motor A backward
        dirB = 1;  // Motor B forward
      }
    } else if (!spinning_mode) {
      // Normal movement mode (forward/backward with steering)
      float base_speed = constrain(Ch3, 0, max_speed);
      
      // Apply differential steering
      float steering_factor = Ch1 * 0.5; // Reduce steering sensitivity
      desired_pwm_A = constrain(base_speed + steering_factor, 0, max_speed);
      desired_pwm_B = constrain(base_speed - steering_factor, 0, max_speed);
      
      // Direction based on CH2
      if (Ch2 >= 0) {
        dirA = 1;  // Forward
        dirB = 1;  // Forward
      } else {
        dirA = 0;  // Backward
        dirB = 0;  // Backward
      }
    } else {
      // No significant input, stop motors
      desired_pwm_A = 0;
      desired_pwm_B = 0;
      dirA = 1;
      dirB = 1;
    }
    
    // Convert desired PWM to expected speed (counts/sec)
    // Using Encoder library with quadrature encoding:
    // - Motor A: Was showing ~280 counts/sec at PWM=20 with single channel
    // - Motor B: Was showing ~400+ counts/sec at PWM=20 with two channels
    // With proper quadrature encoding, both should show similar actual speeds
    // Estimate: PWM=100 should give ~1400-1600 counts/sec per motor
    
    float deadband_pwm = 1;
    float max_counts_per_sec_A = 1600.0;
    float max_counts_per_sec_B = 1600.0;
    float motor_B_compensation = 1.0;
    
    if (desired_pwm_A <= deadband_pwm) {
      setpoint_A = 0;
    } else {
      float effective_pwm_A = desired_pwm_A - deadband_pwm;
      float effective_pwm_range = max_speed - deadband_pwm;
      setpoint_A = (effective_pwm_A / effective_pwm_range) * max_counts_per_sec_A;
    }
    
    if (desired_pwm_B <= deadband_pwm) {
      setpoint_B = 0;
    } else {
      // Linear mapping from PWM to counts/sec
      float effective_pwm_B = desired_pwm_B - deadband_pwm;
      float effective_pwm_range = max_speed - deadband_pwm;
      setpoint_B = (effective_pwm_B / effective_pwm_range) * max_counts_per_sec_B * motor_B_compensation;
    }
    
    // Reset integrals if setpoint is very small
    if (setpoint_A < 50.0) integral_A = 0;
    if (setpoint_B < 50.0) integral_B = 0;
  }

  // *** Use PID Control instead of soft start ***
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
    float deadband_compensation = 12.0;
    
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
    // if(currentSpeedA <= 35 && currentSpeedB <= 35){
    //   currentSpeedA = 0;
    //   currentSpeedB = 0;
    // }

    mtrControlA(currentSpeedA, dirA);
    mtrControlB(currentSpeedB, dirB);

    lastMotorUpdateTime = currentTime;
  }

  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    sendMessage(desired_pwm_A, desired_pwm_B, dirA, dirB);
    lastSendTime = currentTime;

    Serial.print(" Ch1 = "); Serial.print(Ch1);
    Serial.print(" Ch2 = "); Serial.print(Ch2);
    Serial.print(" Ch3 = "); Serial.print(Ch3);
    Serial.print(" Ch5 = "); Serial.print(Ch5);
    Serial.print(" Ch6 = "); Serial.println(Ch6);
    
    // Print movement mode for debugging
    const int ch2_tolerance = 3;
    bool spinning_mode = (abs(Ch2) <= ch2_tolerance);
    if (spinning_mode && abs(Ch1) > ch2_tolerance) {
      Serial.print("SPINNING MODE - Direction: ");
      Serial.println(Ch1 > 0 ? "RIGHT" : "LEFT");
    } else if (!spinning_mode) {
      Serial.print("MOVEMENT MODE - Direction: ");
      Serial.println(Ch2 >= 0 ? "FORWARD" : "BACKWARD");
    }
    
    Serial.print("Desired PWM - A: "); Serial.print(desired_pwm_A);
    Serial.print(" B: "); Serial.print(desired_pwm_B);
    Serial.print(" DirA: "); Serial.print(dirA);
    Serial.print(" DirB: "); Serial.println(dirB);
    
    // Print PID status
    // Serial.println("=== PID Status ===");
    // Serial.println("Move direction: " + String(dirA));
    
    // // Debug: Print raw encoder counts
    // Serial.print("Raw Encoder Counts - A: "); Serial.print(encoderA.read());
    // Serial.print(" B: "); Serial.println(encoderB.read());
    
    // Serial.print("A: Setpoint="); Serial.print(setpoint_A);
    // Serial.print(" Feedback="); Serial.print(feedback_A);
    // Serial.print(" Error="); Serial.print(error_A);
    // Serial.print(" EffPWM="); Serial.print(currentSpeedA - 12.0);
    // Serial.print(" PWM="); Serial.print(currentSpeedA); Serial.print("/"); Serial.println(max_speed);
    // Serial.print("B: Setpoint="); Serial.print(setpoint_B);
    // Serial.print(" Feedback="); Serial.print(feedback_B);
    // Serial.print(" Error="); Serial.print(error_B);
    // Serial.print(" EffPWM="); Serial.print(currentSpeedB - 12.0);
    // Serial.print(" PWM="); Serial.print(currentSpeedB); Serial.print("/"); Serial.println(max_speed);

    // // Print setpoints after calculation for debugging
    // Serial.print(" Calculated Setpoints - A: "); Serial.print(setpoint_A);
    // Serial.print(" B: "); Serial.println(setpoint_B);
    // Serial.print(" Desired PWM: "); Serial.println(Ch3);
  }
}

void sendMessage(int a_spd, int b_spd, int a_dir, int b_dir) {
  // Enhanced message format: desired_pwm_A,desired_pwm_B,dirA,dirB,currentSpeedA,currentSpeedB,encoderSpeedA_scaled,encoderSpeedB_scaled
  snprintf(messageBuffer, BUFFER_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d", 
           a_spd, b_spd, a_dir, b_dir, currentSpeedA, currentSpeedB, encoderSpeedA_scaled, encoderSpeedB_scaled);
  
  // Send via Serial2
  Serial2.println(messageBuffer);
  
  // Print status to Serial monitor with encoder data
  snprintf(statusBuffer, BUFFER_SIZE, "SENT: %s", messageBuffer);
  Serial.println(statusBuffer);
  
  // Print TX encoder data for monitoring
  Serial.print("TX Encoder Speeds - A: "); Serial.print(encoderSpeedA_scaled);
  Serial.print(" B: "); Serial.print(encoderSpeedB_scaled);
  Serial.print(" | TX Current Speeds - A: "); Serial.print(currentSpeedA);
  Serial.print(" B: "); Serial.println(currentSpeedB);
}

void checkForResponse() {
  if (Serial2.available()) {
    char responseBuffer[BUFFER_SIZE];
    int bytesRead = 0;
    
    // Read the response
    while (Serial2.available() && bytesRead < BUFFER_SIZE - 1) {
      responseBuffer[bytesRead] = Serial2.read();
      bytesRead++;
      delay(2); // Small delay to ensure all data is received
    }
    
    // Null terminate the string
    responseBuffer[bytesRead] = '\0';
    
    // Remove any trailing newline characters
    for (int i = bytesRead - 1; i >= 0; i--) {
      if (responseBuffer[i] == '\n' || responseBuffer[i] == '\r') {
        responseBuffer[i] = '\0';
      } else {
        break;
      }
    }
    
    // Print response to Serial monitor
    snprintf(statusBuffer, BUFFER_SIZE, "RECEIVED: %s", responseBuffer);
    Serial.println(statusBuffer);
  }
}
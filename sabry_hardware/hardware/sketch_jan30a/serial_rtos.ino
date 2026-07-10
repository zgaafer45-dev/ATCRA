#include <Wire.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// ===================== CONFIGURATION =====================
#define SERIAL_BAUD 115200  // High speed for ROS 2 
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36
#define LPF_ALPHA   0.2f

// Pins (Based on your definitions)
#define STEP1 2
#define DIR1 4
#define STEP2 18
#define DIR2 3
#define STEP3 14
#define DIR3 15
#define STEP4 5
#define DIR4 6
#define STEP5 40
#define DIR5 39

# define RELAY_EXTEND 36
# define RELAY_RETRACT 35

#define SERVO_PIN 37

// Math: Steps per Radian
// For 1.8deg motor (200 steps) at 16x microstepping: (200 * 16) / (2 * PI) = 509.2958
const float FULL_STEPS_PER_REV = 200.0;
    // set to your driver config
const float STEPS_PER_REV     = FULL_STEPS_PER_REV ;
const float DEG_TO_STEP       = STEPS_PER_REV / 360.0;

// ===================== OBJECTS =====================
AccelStepper steppers[5] = {
  AccelStepper(AccelStepper::DRIVER, STEP1, DIR1),
  AccelStepper(AccelStepper::DRIVER, STEP2, DIR2),
  AccelStepper(AccelStepper::DRIVER, STEP3, DIR3),
  AccelStepper(AccelStepper::DRIVER, STEP4, DIR4),
  AccelStepper(AccelStepper::DRIVER, STEP5, DIR5)
};

// Gripper Servo Config
Servo gripper_servo;
volatile int gripper_goal = 0;     // commanded position
volatile int gripper_pos  = 0;     // feedback position

// Global shared variables
volatile float motor_speeds_rad[5] = {0, 0, 0, 0, 0};
float encoder_angles_deg[5] = {0, 0, 0, 0, 0};
float filtered_angles[5] = {0, 0, 0, 0, 0};

// command for linear motor
volatile int tool_command = 0;

// ===================== COMMUNICATION WATCHDOG =====================
unsigned long last_serial_time = 0;
const unsigned long SERIAL_TIMEOUT = 2000; // 2 seconds

// ===================== I2C HELPERS =====================
bool tca_select(uint8_t ch)
{
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);

  if (Wire.endTransmission() != 0)
    return false;

  delayMicroseconds(50); // IMPORTANT stabilization time
  return true;
}

uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // ANGLE_HIGH register
  if (Wire.endTransmission() != 0) return 0xFFFF;
  
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  return (Wire.read() << 8) | Wire.read();
}

// ===================== CORE 1: MOTOR PULSES (HIGH PRIORITY) =====================
void MotorTask(void * pvParameters) {
  for(;;) {
    // for (int i = 0; i < 5; i++) {
    //   // Set the speed based on latest ROS command
    //   steppers[i].setSpeed(motor_speeds_rad[i] * DEG_TO_STEP);
    //   // Generate pulse if due
    //   steppers[i].runSpeed();
    // }
    steppers[0].setSpeed(motor_speeds_rad[0] * DEG_TO_STEP * -1); 
    steppers[1].setSpeed(motor_speeds_rad[1] * DEG_TO_STEP * 1);
    steppers[2].setSpeed(motor_speeds_rad[2] * DEG_TO_STEP * 1);
    steppers[3].setSpeed(motor_speeds_rad[3] * DEG_TO_STEP * 1);
    steppers[4].setSpeed(motor_speeds_rad[4] * DEG_TO_STEP * -1);

    steppers[0].runSpeed();
    steppers[1].runSpeed();
    steppers[2].runSpeed();
    steppers[3].runSpeed();
    steppers[4].runSpeed();
     
    // Yield to avoid watchdog trigger, but don't delay!
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// ===================== CORE 0: COMM & ENCODERS (LOW PRIORITY) =====================
void CommTask(void * pvParameters) {
  String inputBuffer = "";
  unsigned long lastReport = 0;

  for(;;) {
    // 1. NON-BLOCKING SERIAL READ
    while (Serial.available()) {
      char c = Serial.read();
      last_serial_time = millis();
      if (c == '\n') {
        // Parse incoming: "v0,v1,v2,M1:v3,M2:v4"
        char buf[128];
        inputBuffer.toCharArray(buf, 128);
        char* ptr = strtok(buf, ",");
        int idx = 0;
        
        while (ptr != NULL) {
          if (strncmp(ptr, "M1:", 3) == 0) {
            motor_speeds_rad[3] = atof(ptr + 3);
          }
          else if (strncmp(ptr, "M2:", 3) == 0) {
            motor_speeds_rad[4] = atof(ptr + 3);
          }
          else if (strncmp(ptr, "TOOL:", 5) == 0) {
            tool_command = atoi(ptr + 5);
          }
          else if (strncmp(ptr, "GRIP:", 5) == 0) {
            gripper_goal = atoi(ptr + 5);
          }
          else if (idx < 3) {
            motor_speeds_rad[idx++] = atof(ptr);
          }

          ptr = strtok(NULL, ",");
        }
        inputBuffer = ""; 
        // ===== TOOL CONTROL =====
        if (tool_command == 1) {
          digitalWrite(RELAY_RETRACT, HIGH);
          digitalWrite(RELAY_EXTEND, LOW);
        }
        else if (tool_command == 2) {
          digitalWrite(RELAY_EXTEND, HIGH);
          digitalWrite(RELAY_RETRACT, LOW);
        }
        else {
          // Optional: stop both
          digitalWrite(RELAY_EXTEND, HIGH);
          digitalWrite(RELAY_RETRACT, HIGH);
        }
        // ===== GRIPPER CONTROL =====
        gripper_goal = constrain(gripper_goal, 0, 180);

        // avoid unnecessary writes (reduces jitter)
        if (abs(gripper_goal - gripper_pos) > 1) {
          gripper_servo.write(gripper_goal);
        }
      } else {
        inputBuffer += c;
      }
    }
    // ===================== SERIAL TIMEOUT SAFETY =====================
    if (millis() - last_serial_time > SERIAL_TIMEOUT) {

      // Stop all stepper motors
      for (int i = 0; i < 5; i++) {
        motor_speeds_rad[i] = 0;
      }

      // Stop linear actuator
      tool_command = 0;

      digitalWrite(RELAY_EXTEND, HIGH);
      digitalWrite(RELAY_RETRACT, HIGH);

      // Optional gripper safety
      gripper_goal = 0;
    }

    // 2. READ ENCODERS (Slow I2C)
    for (int i = 0; i < 5; i++) {

      // Select TCA channel
      if (!tca_select(i)) {
        // Serial.printf("TCA select failed on CH %d\n", i);
        continue;
      }

      delayMicroseconds(300); // small settling delay

      // Check if AS5600 exists on this channel
      Wire.beginTransmission(AS5600_ADDR);

      if (Wire.endTransmission() == 0) {

        // Encoder exists -> read normally
        uint16_t raw = readAS5600();

        if (raw != 0xFFFF) {

          float angle = raw * 360.0f / 4096.0f;
          
          // Low pass filter
          filtered_angles[i] =
              (LPF_ALPHA * angle) +
              ((1.0f - LPF_ALPHA) * filtered_angles[i]);
        }
        else {
          // filtered_angles[i] = 0.0;
        }
      }
      else {
        
      }
    }
    gripper_pos = gripper_servo.read();

    // 3. SEND FEEDBACK TO ROS (30Hz)
    if (millis() - lastReport > 33) {
      Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,GRIP:%d\n", 
              filtered_angles[0], filtered_angles[1], filtered_angles[2], 
              filtered_angles[3], filtered_angles[4],
              gripper_pos);
      lastReport = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // Relax Core 0 slightly
  }
}

// ===================== SETUP =====================
void setup() {
  
  // Pin Setup
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(STEP4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(STEP5, OUTPUT);
  pinMode(DIR5, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1);

  last_serial_time = millis();
  
  // I2C Setup
  Wire.begin(9, 8); // SDA=9, SCL=8
  Wire.setClock(100000); // 400kHz Fast Mode

  pinMode(RELAY_EXTEND, OUTPUT);
  pinMode(RELAY_RETRACT, OUTPUT);

  // Default OFF
  digitalWrite(RELAY_EXTEND, HIGH);
  digitalWrite(RELAY_RETRACT, HIGH);

  gripper_servo.attach(SERVO_PIN);
  gripper_servo.write(0);

  // Stepper Setup
  for (int i = 0; i < 5; i++) {
    steppers[i].setMaxSpeed(4000); 
    steppers[i].setAcceleration(500);
  }

  // Task Creation
  // MotorTask: Priority 3 (High), Core 1
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL, 3, NULL, 1);
  
  // CommTask: Priority 1 (Low), Core 0
  xTaskCreatePinnedToCore(CommTask, "CommTask", 4096, NULL, 1, NULL, 0);
  
  // Serial.println("System Ready");
}

void loop() {
  // Empty - FreeRTOS handles the tasks
}
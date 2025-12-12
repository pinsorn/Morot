#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>

// ==========================================
// 1. PIN & HARDWARE CONFIGURATION
// ==========================================

// --- Serial Configuration ---
//HardwareSerial SerialAUX(2); // Use UART2
const int S3_TX_PIN = 12; 
const int S3_RX_PIN = 11; 

// --- NeoPixel Configuration ---
#define NEOPIXEL_PIN 48 
#define NUM_PIXELS   1 
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Pin Configuration ---
#define RXD1_PIN 17 
#define TXD1_PIN 18 
#define EN_PIN   21 
#define STEP_PIN 16 
#define DIR_PIN  15 

#define RXD2_PIN RXD1_PIN
#define TXD2_PIN TXD1_PIN //Shared
#define EN2_PIN EN_PIN
#define STEP2_PIN 7
#define DIR2_PIN 6

#define RXD3_PIN RXD1_PIN
#define TXD3_PIN TXD1_PIN //Shared
#define EN3_PIN EN_PIN
#define STEP3_PIN 3
#define DIR3_PIN 8

// --- Stepper Driver Configuration ---
#define R_SENSE 0.11f
#define SERIAL_ADDRESS 1
#define SERIAL_ADDRESS_2 0
#define SERIAL_ADDRESS_3 2

// --- Motor Configuration ---
#define MOTOR_CURRENT_RMS 1300
#define MICROSTEPS 0 // 0 means full step in some libs, but usually 0 is invalid for TMC, assuming user logic handles this.
// Note: TMC2209 usually takes 0 as 256 microsteps depending on mres, check library docs.

// --- Calculation ---
#define STEPS_PER_REVOLUTION (200 * (MICROSTEPS > 0 ? MICROSTEPS : 1))
#define MAX_SPEED STEPS_PER_REVOLUTION*3 
#define MAX_ACCEL STEPS_PER_REVOLUTION

// ==========================================
// 2. GLOBAL VARIABLES & ENUMS
// ==========================================

boolean anyMotorRunning = false;
float LIMIT_COMPENSATION_RATIO = 1.0f;
boolean isErrorState = false;

// Enum สำหรับ Report
enum ReportType {
  INFO,
  WARNING,
  ERROR
};

// Enum สำหรับ Serial Target
enum SerialTarget {
  MAIN,  // Serial ปกติ (USB)
  AUX    // Serial2
};

// Struct สำหรับ Queue
struct LogPackage {
  SerialTarget target; 
  char* message;    
};

QueueHandle_t printQueue; 

// ==========================================
// 3. FORWARD DECLARATIONS (หัวใจสำคัญ!)
// ==========================================
// ประกาศชื่อฟังก์ชันไว้ก่อน เพื่อให้ Class เรียกใช้ได้ โดยไม่ต้องสนใจลำดับ

void asyncPrint(SerialTarget target, String msg);
void asyncPrint(SerialTarget target, long num);
void displayJSON(ReportType type, String message, String motorName = "", int code = 0);
void displayJSON(ReportType type, String message, int code);

// ==========================================
// 4. TMC DRIVER OBJECTS
// ==========================================
// ประกาศ Driver ไว้ก่อน เพราะ Class ต้องใช้ Pointer ชี้มาหา

TMC2209Stepper driver(&Serial1, R_SENSE, SERIAL_ADDRESS);
TMC2209Stepper driver2(&Serial1, R_SENSE, SERIAL_ADDRESS_2);
TMC2209Stepper driver3(&Serial1, R_SENSE, SERIAL_ADDRESS_3);

// ==========================================
// 5. STEPPER MOTOR CLASS
// ==========================================

class StepperMotor {
private:
  TMC2209Stepper driver;
  AccelStepper stepper;
  uint8_t enPin;
  uint8_t limitLeftPin;
  uint8_t limitRightPin;
  long stepsPerRev;
  float maxSpeed, maxAccel;
  bool movementComplete;
  bool limitEnabled;
  bool lastLeftState, lastRightState;
  String motorName;

public:
  struct Config {
    HardwareSerial &serialPort;
    uint8_t rxPin;
    uint8_t txPin;
    uint8_t enPin;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitLeftPin;
    uint8_t limitRightPin;
    float rSense;
    uint8_t serialAddress;
    uint16_t motorCurrentRMS;
    uint16_t microsteps;
    String name;
  };

  StepperMotor(const Config &cfg)
      : driver(&cfg.serialPort, cfg.rSense, cfg.serialAddress),
        stepper(AccelStepper::DRIVER, cfg.stepPin, cfg.dirPin),
        enPin(cfg.enPin),
        limitLeftPin(cfg.limitLeftPin),
        limitRightPin(cfg.limitRightPin),
        movementComplete(true),
        limitEnabled(cfg.limitLeftPin != 0 || cfg.limitRightPin != 0),
        lastLeftState(HIGH),
        lastRightState(HIGH),
        motorName(cfg.name) {
    pinMode(cfg.enPin, OUTPUT);
    digitalWrite(cfg.enPin, LOW);

    if (cfg.limitLeftPin) {
      pinMode(cfg.limitLeftPin, INPUT_PULLUP);
      displayJSON(INFO, "Left limit switch enabled on pin " + String(cfg.limitLeftPin), motorName,101);
    }
    if (cfg.limitRightPin) {
      pinMode(cfg.limitRightPin, INPUT_PULLUP);
      displayJSON(INFO, "Right limit switch enabled on pin " + String(cfg.limitRightPin), motorName,102);
    }
   
    stepsPerRev = STEPS_PER_REVOLUTION;
    maxSpeed = MAX_SPEED;
    maxAccel = MAX_ACCEL;
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(maxAccel);

    displayJSON(INFO, "Motor setup complete on EN pin " + String(enPin), motorName,100);
  }

  void moveTo(long target) {
    stepper.moveTo(target);
    movementComplete = false;
  }

  void move(long steps) {
    stepper.move(steps);
    movementComplete = false;
  }

  void update() {
    if (limitEnabled) {
      bool leftState = limitLeftPin ? digitalRead(limitLeftPin) : HIGH;
      bool rightState = limitRightPin ? digitalRead(limitRightPin) : HIGH;

      if (limitLeftPin && leftState == HIGH && lastLeftState == LOW) {
        if (isRunning() && stepper.speed() < 0) {
          emergencyStop();
          displayJSON(WARNING, "LEFT LIMIT SWITCH TRIGGERED - STEPPING BACK", motorName,411);
          stepper.move(STEPS_PER_REVOLUTION * LIMIT_COMPENSATION_RATIO);
          isErrorState = true;
          displayPosition();
        }
      }

      if (limitRightPin && rightState == HIGH && lastRightState == LOW) {
        if (isRunning() && stepper.speed() > 0) {
          emergencyStop();
          displayJSON(WARNING, "RIGHT LIMIT SWITCH TRIGGERED - STEPPING BACK", motorName,412);
          stepper.move(-STEPS_PER_REVOLUTION * LIMIT_COMPENSATION_RATIO);
          isErrorState = true;
          displayPosition();
        }
      }

      lastLeftState = leftState;
      lastRightState = rightState;
    }
    
    if (stepper.distanceToGo() != 0) {
      stepper.run();
    } else if (!movementComplete) {
      movementComplete = true;
      displayJSON(INFO, "Target reached!", motorName,211);
      displayPosition();
    }
  }

  bool isRunning() { return stepper.distanceToGo() != 0; }

  bool isLeftPressed() {
    return limitLeftPin && digitalRead(limitLeftPin) == LOW;
  }

  bool isRightPressed() {
    return limitRightPin && digitalRead(limitRightPin) == LOW;
  }

  void printLimitStatus() {
    if (!limitEnabled) {
      displayJSON(INFO, "Limit switches disabled", motorName,404);
      return;
    }
    
    String output = "{\"motor\":\"";
    output += motorName;
    output += "\",\"leftPressed\":";
    output += isLeftPressed() ? "true" : "false";
    output += ",\"rightPressed\":";
    output += isRightPressed() ? "true" : "false";
    output += ",\"code\":405}";
    asyncPrint(MAIN, output);
  }

  void stop() {
    stepper.stop();
    displayJSON(INFO, "Motor stopped (decelerating)", motorName,212);
  }

  void emergencyStop() {
    stepper.setCurrentPosition(stepper.currentPosition());
    displayJSON(INFO, "Emergency stop executed", motorName,213);
  }

  long getCurrentPosition() { return stepper.currentPosition(); }

  void displayPosition() {
    long pos = stepper.currentPosition();
    String output = "{\"motor\":\"";
    output += motorName;
    output += "\",\"position\":";
    output += String(pos);
    output += ",\"revolutions\":";
    output += String((float)pos / stepsPerRev, 2);
    output += ",\"status\":\"";
    output += isRunning() ? "MOVING" : "IDLE";
    output += "\"";
    if (limitEnabled) {
      output += ",\"limitLeft\":";
      output += isLeftPressed() ? "true" : "false";
      output += ",\"limitRight\":";
      output += isRightPressed() ? "true" : "false";
    }
    int code = 200; // Default
    bool isMoving = isRunning();
    bool limitPressed = isLeftPressed() || isRightPressed();
    
    if (isMoving && limitPressed) code = 202;
    else if (!isMoving && !limitPressed) code = 201;
    else if (isMoving && !limitPressed) code = 203;
    // 200 is already default for !isMoving && !limitPressed

    output += ",\"code\":" + String(code) + "}";
    asyncPrint(MAIN, output);
  }

  void setHome() {
    stepper.setCurrentPosition(0);
    displayJSON(INFO, "Home position set", motorName, 206);
  }

  void enable() {
    digitalWrite(enPin, LOW);
    displayJSON(INFO, "Motor enabled", motorName, 207);
  }

  void disable() {
    digitalWrite(enPin, HIGH);
    displayJSON(INFO, "Motor disabled", motorName, 208);
  }
  
  void setSpeed(float speed) { 
    stepper.setMaxSpeed(speed*stepsPerRev); 
    displayJSON(INFO, "Max speed set to: " + String(speed), motorName, 205);
  }
  
  void setAcceleration(float accel) { 
    stepper.setAcceleration(accel*stepsPerRev); 
    displayJSON(INFO, "Acceleration set to: " + String(accel), motorName, 209);
  }
  String getName() { return motorName; }
};

// ==========================================
// 6. MOTOR INSTANCES CONFIGURATION
// ==========================================

StepperMotor::Config M1 = {
  Serial1,   // UART port
  RXD1_PIN, TXD1_PIN,    // RX, TX
  EN_PIN,        // Enable pin
  STEP_PIN, DIR_PIN,    // Step, Dir
  1,         // Left limit pin
  2,         // Right limit pin
  0.11f,     // Rsense
  SERIAL_ADDRESS,         // UART address
  MOTOR_CURRENT_RMS,       // Motor current (mA)
  0,         // Microsteps (full step)
  "Motor1"   // Name
};

StepperMotor::Config M2 = {
  Serial1,   // UART port
  RXD2_PIN, TXD2_PIN,    // RX, TX
  EN2_PIN,        // Enable pin
  STEP2_PIN, DIR2_PIN,    // Step, Dir
  41,        // Left limit pin
  42,        // Right limit pin
  0.11f,     // Rsense
  SERIAL_ADDRESS_2,         // UART address
  MOTOR_CURRENT_RMS,       // Motor current (mA)
  0,         // Microsteps (full step)
  "Motor2"   // Name
};

StepperMotor::Config M3 = {
  Serial1,   // UART port
  RXD3_PIN, TXD3_PIN,    // RX, TX
  EN3_PIN,        // Enable pin
  STEP3_PIN, DIR3_PIN,    // Step, Dir
  39,        // Left limit pin
  40,        // Right limit pin
  0.11f,     // Rsense
  SERIAL_ADDRESS_3,         // UART address
  MOTOR_CURRENT_RMS,       // Motor current (mA)
  0,         // Microsteps (full step)
  "Motor3"   // Name
};

// สร้าง Instance จริงๆ ตรงนี้
StepperMotor motorX(M1);
StepperMotor motorY(M2);
StepperMotor motorZ(M3);

// ==========================================
// 7. HELPER FUNCTIONS IMPLEMENTATION
// ==========================================

// ฟังก์ชัน asyncPrint ตัวจริง
void asyncPrint(SerialTarget target, String msg) {
  // 🛡️ Safety: ถ้าคิวยังไม่มี อย่าเพิ่งทำอะไร
  if (printQueue == NULL) return;

  LogPackage pkg;
  pkg.target = target;
  
  // ⭐ จอง RAM และก๊อปปี้ข้อความลงไป (ยาวเท่าไหร่ก็ได้)
  pkg.message = strdup(msg.c_str()); 

  // ส่ง Pointer เข้าคิว
  // ⚠️ สำคัญมาก: ถ้าคิวเต็ม (ส่งไม่ผ่าน) เราต้อง free ทิ้งทันที ไม่งั้น Memory Leak!
  if (xQueueSend(printQueue, &pkg, 0) != pdTRUE) {
    free(pkg.message); // ทิ้งของที่จองไว้ เพราะส่งไม่ได้
  }
}

void asyncPrint(SerialTarget target, long num) {
  asyncPrint(target, String(num));
}

// ฟังก์ชัน displayJSON ตัวจริง
void displayJSON(ReportType type, String message, String motorName, int code) {
    String typeStr;
      switch(type) {
      case INFO: typeStr = "INFO"; break;
      case WARNING: typeStr = "WARNING"; break;
      case ERROR: typeStr = "ERROR"; break;
      }
      
      String output = "{\"type\":\"";
      output += typeStr;
      output += "\",\"message\":\"";
      output += message;
      output += "\"";
      if (motorName.length() > 0) {
      output += ",\"motor\":\"";
      output += motorName;
      output += "\"";
      }
  output += ",\"code\":";
  output += String(code);
  output += "}";
  asyncPrint(MAIN, output);
}
void displayJSON(ReportType type, String message, int code) {
    displayJSON(type, message, "", code);
}
// ฟังก์ชัน processCommand
void processCommand(String input, boolean motorStatus) {
  input.trim();

  StepperMotor* targetMotor = nullptr;
  bool bothMotors = false;
  String command = input;

  if (input.startsWith("1:")) {
    targetMotor = &motorX;
    command = input.substring(2);
  } else if (input.startsWith("2:")) {
    targetMotor = &motorY;
    command = input.substring(2);
  } else if (input.startsWith("3:")) {
    targetMotor = &motorZ;
    command = input.substring(2);
  } else {
    bothMotors = true;
  }

  if (command.equalsIgnoreCase("s")) {
    if(!motorStatus) { displayJSON(ERROR, "Nothing to stop, motors are idle.",406); return; }
    if (bothMotors) { motorX.stop(); motorY.stop(); motorZ.stop(); }
    else targetMotor->stop();
  }
  else if (command.equalsIgnoreCase("e")) {
    if(!motorStatus) { displayJSON(ERROR, "Nothing to emergency stop, motors are idle.",407); return; }
    if (bothMotors) { motorX.emergencyStop(); motorY.emergencyStop(); motorZ.emergencyStop(); }
    else targetMotor->emergencyStop();
  }
  else if (command.equalsIgnoreCase("h")) {
    if(motorStatus) { displayJSON(ERROR, "Cannot set home while motors are running.",406); return; }
    if (bothMotors) { motorX.setHome(); motorY.setHome(); motorZ.setHome(); }
    else targetMotor->setHome();
  }
  else if(command.startsWith("x")){
    if(motorStatus) { displayJSON(ERROR, "Cannot change speed while motors are running.",406); return; }
      float speed = command.substring(1).toFloat();
      if(bothMotors){
          motorX.setSpeed(speed); motorY.setSpeed(speed); motorZ.setSpeed(speed);
      } else {
          targetMotor->setSpeed(speed);
      }
  }
  else if (command.startsWith("m")) {
    asyncPrint(AUX, command.substring(1));
    displayJSON(INFO, "Forwarded command to AUX: " + command.substring(1), 300);
  }
  else if(command.startsWith("a")){
    if(motorStatus) { displayJSON(ERROR, "Cannot change acceleration while motors are running.",406); return; }
      float accel = command.substring(1).toFloat();
      if(bothMotors){
          motorX.setAcceleration(accel); motorY.setAcceleration(accel); motorZ.setAcceleration(accel);
      } else {
          targetMotor->setAcceleration(accel);
      }
  }
  else if (command.equalsIgnoreCase("p")) {
    if (bothMotors) {
      String output = "{\"motors\":[";
      output += "{\"motorName\":\"" + motorX.getName() + "\",\"position\":" + String(motorX.getCurrentPosition()) + "},";
      output += "{\"motorName\":\"" + motorY.getName() + "\",\"position\":" + String(motorY.getCurrentPosition()) + "},";
      output += "{\"motorName\":\"" + motorZ.getName() + "\",\"position\":" + String(motorZ.getCurrentPosition()) + "}";
      output += "],\"code\":210}";
      asyncPrint(MAIN, output);
    } else {
      String output = "{\"motorName\":\"" + targetMotor->getName() + "\",\"position\":" + String(targetMotor->getCurrentPosition()) + ",\"code\":210}";
      asyncPrint(MAIN, output);
    }
  }
  else if (command.equalsIgnoreCase("d")) {
    if (bothMotors) { motorX.displayPosition(); motorY.displayPosition(); motorZ.displayPosition(); }
    else targetMotor->displayPosition();
  }
  else if (command.startsWith("i")){
    if(motorStatus) { displayJSON(ERROR,"Error: Cannot change limit compensation ratio while motors are running.",406); return; }
      LIMIT_COMPENSATION_RATIO = command.substring(1).toFloat();
      displayJSON(INFO, "Limit Compensation Ratio set to: " + String(LIMIT_COMPENSATION_RATIO), 300);
  }
  else if (command.equalsIgnoreCase("l")) {
    if (bothMotors) { motorX.printLimitStatus(); motorY.printLimitStatus(); motorZ.printLimitStatus(); }
    else targetMotor->printLimitStatus();
  }
  else if (command.equalsIgnoreCase("on")) {
    if(motorStatus) { displayJSON(ERROR, "Motors are already running.",406); return; }
    if (bothMotors) { motorX.enable(); motorY.enable(); motorZ.enable(); }
    else targetMotor->enable();
  }
  else if (command.equalsIgnoreCase("off")) {
    if(motorStatus) { displayJSON(ERROR, "Cannot disable motors while they are running.",406); return; }
    if (bothMotors) { motorX.disable(); motorY.disable(); motorZ.disable(); }
    else targetMotor->disable();
  }
  else if (command.startsWith("+") && targetMotor) {
    if(motorStatus) { displayJSON(ERROR, "Cannot move motors while they are running.",406); return; }
    targetMotor->move(command.substring(1).toInt());
  }
  else if (command.startsWith("-") && targetMotor) {
    if(motorStatus) { displayJSON(ERROR, "Cannot move motors while they are running.",406); return; }
    targetMotor->move(command.toInt());
  }
  else if (command.length() > 0 && targetMotor) {
    if(motorStatus) { displayJSON(ERROR, "Cannot move motors while they are running.",406); return; }
    targetMotor->moveTo(command.toInt());
  }
  else if (command.length() > 0 && bothMotors) {
    if(motorStatus) { displayJSON(ERROR, "Cannot move motors while they are running.",406); return; }
    int commaIndex = command.indexOf(",");
    if(commaIndex != -1){
      String cmdX = command.substring(0, commaIndex);
      String remaining = command.substring(commaIndex + 1);
      int secondCommaIndex = remaining.indexOf(",");
      
      if(secondCommaIndex != -1) {
        String cmdY = remaining.substring(0, secondCommaIndex);
        String cmdZ = remaining.substring(secondCommaIndex + 1);
        motorX.moveTo(cmdX.toInt());
        motorY.moveTo(cmdY.toInt());
        motorZ.moveTo(cmdZ.toInt());
      } else {
        motorX.moveTo(cmdX.toInt());
        motorY.moveTo(remaining.toInt());
      }
    } else {
      displayJSON(ERROR, "Both motors command requires comma-separated values", 403);
    }
  }
  
  else if (command.length() > 0) {
    displayJSON(ERROR, "Unknown command format", 403);
  }
}

// ==========================================
// 8. CORE 0 TASK (SERIAL WORKER)
// ==========================================

void SerialTask(void * parameter) {
  // เริ่มต้น Serial
  Serial.begin(115200);
  
  Serial2.begin(9600, SERIAL_8N1, S3_RX_PIN, S3_TX_PIN); 

  LogPackage receivedPkg;
  static String serialBuffer = "";
  static bool commandReady = false;
  static String auxBuffer = "";

  for(;;) {
    // -----------------------------------------------------------
    // ส่วนที่ 1: งานปริ้นจากระบบ (Outgoing from Queue)
    // -----------------------------------------------------------
    // ใช้ timeout น้อยๆ เพื่อให้วนลูปไปทำอย่างอื่นได้ไว
    if (xQueueReceive(printQueue, &receivedPkg, 5 / portTICK_PERIOD_MS) == pdTRUE) {
       
       // เช็ค Pointer ว่ามีข้อมูลจริงไหม
       if (receivedPkg.message != NULL) {
           if (receivedPkg.target == MAIN) {
              Serial.println(receivedPkg.message);
           } else if (receivedPkg.target == AUX) {
              Serial2.println(receivedPkg.message);
           }

           // 🗑️ สำคัญ: ใช้เสร็จแล้วต้องทิ้ง (คืน RAM)
           free(receivedPkg.message); 
       }
    }

    // -----------------------------------------------------------
    // ส่วนที่ 2: รับคำสั่งจากคอมพิวเตอร์ (Incoming from USB)
    // -----------------------------------------------------------
    while (Serial.available() > 0) {
      char inChar = (char)Serial.read();
      //Serial.print(inChar); // Echo

      if (inChar == '\n') {
        commandReady = true;
      } 
      else if (inChar != '\r') { 
        serialBuffer += inChar;
      }
    }
    
    // -----------------------------------------------------------
    // ⭐ ส่วนที่ 3 (ใหม่): รับจาก Serial2 -> ส่งให้ Serial (Bridge)
    // -----------------------------------------------------------
    // ถ้า Serial2 (อุปกรณ์ต่อพ่วง) พูดอะไรมา ให้ส่งบอกคอมพิวเตอร์ด้วย
    while (Serial2.available() > 0) {
      char inChar = (char)Serial2.read();
      //Serial.print(inChar); // Echo
      if (inChar == '\n') {
        // พอจบประโยค (เจอ Enter) ให้ปริ้นรวดเดียวเลย
        String output = "{\"type\":\"AUX\",\"message\":" + auxBuffer + ",\"code\":301}";
        Serial.println(output);
        
        // เคลียร์ถุงเตรียมรับประโยคใหม่
        auxBuffer = ""; 
      } 
      else if (inChar != '\r') {
        // เก็บสะสมตัวอักษรใส่ถุงไว้ก่อน
        auxBuffer += inChar;
      }
    }
    // -----------------------------------------------------------
    // ส่วนที่ 4: ประมวลผลคำสั่ง (Process Command)
    // -----------------------------------------------------------
    if (commandReady) {
      Serial.println(); 
      processCommand(serialBuffer, anyMotorRunning); 

      serialBuffer = "";
      commandReady = false;
    }
  }
}

// ==========================================
// 9. SETUP & LOOP
// ==========================================



void setup() {
  // 1. สร้างคิว
  printQueue = xQueueCreate(20, sizeof(LogPackage));

  // 2. สร้าง Task Core 0
  xTaskCreatePinnedToCore(
    SerialTask,   "SerialWorker", 
    4096,         NULL, 
    1,            NULL, 
    0 // Core 0
  );

  // 3. Setup Hardware
  pixels.begin();
  pixels.clear(); 
  pixels.setPixelColor(0, pixels.Color(255, 100, 0)); // Orange
  pixels.show();   
  delay(1000); 
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
  pixels.show();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 

  // 4. Setup TMC (Core 1 ทำหน้าที่ Setup HW หลัก)
  Serial1.begin(115200, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
  
  driver.begin();
  driver.rms_current(MOTOR_CURRENT_RMS);
  driver.microsteps(MICROSTEPS);
  driver.toff(5);
  driver.pdn_disable(true);

  driver2.begin();
  driver2.rms_current(MOTOR_CURRENT_RMS);
  driver2.microsteps(MICROSTEPS);
  driver2.toff(5);
  driver2.pdn_disable(true);

  driver3.begin();
  driver3.rms_current(MOTOR_CURRENT_RMS);
  driver3.microsteps(MICROSTEPS);
  driver3.toff(5);
  driver3.pdn_disable(true);
  
  // แจ้ง Core 0 ให้ปริ้นข้อความต้อนรับ
  asyncPrint(MAIN, "Driver configured via UART for " + String(MICROSTEPS) + " microsteps.");
  asyncPrint(MAIN, "\n=== Triple Motor Non-Blocking Stepper Controller ===");
  // ... (ข้อความเมนูยาวๆ นายท่านใส่เพิ่มได้เลยค่ะ) ...
  
  motorX.displayPosition();
  motorY.displayPosition();
  motorZ.displayPosition();
}

void loop() {
  // Priority สูงสุด: สั่งมอเตอร์ทำงาน (Run ที่ Core 1)
  motorX.update();
  motorY.update();
  motorZ.update();
  
  // อัปเดตสถานะ (เขียนค่าลงตัวแปร Global)
  anyMotorRunning  = motorX.isRunning() || motorY.isRunning() || motorZ.isRunning();
  
  // LED Status
  if(anyMotorRunning) {
    if(isErrorState){
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // RED
    }else{
      pixels.setPixelColor(0, pixels.Color(255, 100, 0)); // ORANGE
    }
  }else{
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // GREEN
    isErrorState = false;
  }
  
  // สำคัญ: loop นี้จะวิ่งเร็วมาก ไม่ควรใส่ delay เยอะๆ
  if(anyMotorRunning) {
      pixels.show(); // การเรียก pixels.show บ่อยเกินไปอาจทำให้ stepper สะดุดได้ ถ้ารู้สึกว่าสะดุด ให้ใส่ millis() ครอบตรงนี้ค่ะ
  } else {
      // update LED นานๆ ทีก็ได้ถ้าหยุดหมุนแล้ว
      static unsigned long lastPixel = 0;
      if(millis() - lastPixel > 100) {
          pixels.show();
          lastPixel = millis();
      }
  }
}
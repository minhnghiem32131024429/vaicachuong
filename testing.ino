#include <AccelStepper.h>
#include <Servo.h>  

// --- Chân kết nối ---
#define ENABLE_PIN   8
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Y_STEP_PIN   3
#define Y_DIR_PIN    6
#define SERVO_PIN    12  

// --- Thông số động cơ ---
#define STEPS_PER_DEG 0.56      // 200 bước/vòng với 1.8° và microstepping 8

// --- Thông số Servo ---
#define PEN_UP_ANGLE   90
#define PEN_DOWN_ANGLE 40
bool isPenDown = false;       

// --- Biến trạng thái ---
float currentX = 0.0;
float currentY = 0.0;
bool isProcessingGcode = false;
bool commandCompleted = true;
bool absolutePositioning = true;

Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { IDLE, MOVE, HOME, TEST } mode = IDLE;

// --- Hàm di chuyển tối ưu ---
void moveToAngle(AccelStepper &stepper, float targetAngle) {
  long currentSteps = stepper.currentPosition();
  long targetSteps = (long)(targetAngle * STEPS_PER_DEG);
  
  // Tính toán số bước cần di chuyển
  long diff = targetSteps - currentSteps;
  
  // Tìm đường đi ngắn nhất (giải quyết vấn đề quay 360°)
  long fullRotation = (long)(360.0 * STEPS_PER_DEG);
  if (diff > fullRotation / 2) {
    diff -= fullRotation;
  } else if (diff < -fullRotation / 2) {
    diff += fullRotation;
  }
  
  // Di chuyển tương đối
  stepper.move(diff);
}

// --- Hàm kiểm tra động cơ ---
void testSteppers() {
  Serial.println(F("Test motor X+"));
  for (int i = 0; i < 200; i++) {
    digitalWrite(X_DIR_PIN, HIGH);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(500);
  
  Serial.println(F("Test motor X-"));
  for (int i = 0; i < 200; i++) {
    digitalWrite(X_DIR_PIN, LOW);
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(500);
  
  Serial.println(F("Test motor Y+"));
  for (int i = 0; i < 200; i++) {
    digitalWrite(Y_DIR_PIN, HIGH);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(500);
  
  Serial.println(F("Test motor Y-"));
  for (int i = 0; i < 200; i++) {
    digitalWrite(Y_DIR_PIN, LOW);
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  Serial.println(F("Motors test complete"));
}

// --- Hàm kích hoạt động cơ ---
void enableMotors() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  Serial.println(F("MOTORS_ENABLED"));
}

void disableMotors() {
  digitalWrite(ENABLE_PIN, HIGH);
  Serial.println(F("MOTORS_DISABLED"));
}

// --- Hàm điều khiển bút ---
void penUp() {
  penServo.write(PEN_UP_ANGLE);
  isPenDown = false;
  Serial.println(F("PEN_UP"));
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  isPenDown = true;
  Serial.println(F("PEN_DOWN"));
}

void setup() {
  Serial.begin(9600);
  
  // Khởi tạo pin
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Kích hoạt động cơ
  enableMotors();

  // Thiết lập servo
  penServo.attach(SERVO_PIN);
  penUp();
  delay(300);

  // Khởi tạo stepper
  stepperX.setMaxSpeed(300);
  stepperX.setAcceleration(100);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(300); 
  stepperY.setAcceleration(100);
  stepperY.setCurrentPosition(0);
  
  Serial.println(F("SCARA_READY"));
}

void loop() {
  // Xử lý lệnh từ Serial
  if (Serial.available()) {
    char buffer[32];
    uint8_t len = Serial.readBytesUntil('\n', buffer, 31);
    buffer[len] = '\0';
    
    // Xử lý lệnh gốc từ Python
    if (strcmp(buffer, "status") == 0) {
      Serial.println(commandCompleted ? F("READY") : F("BUSY"));
    }
    else if (strcmp(buffer, "test") == 0) {
      enableMotors();
      mode = TEST;
      testSteppers();
      mode = IDLE;
    }
    else if (strcmp(buffer, "enable") == 0) {
      enableMotors();
    }
    else if (strcmp(buffer, "disable") == 0) {
      disableMotors();
    }
    else if (strcmp(buffer, "u") == 0 || strcmp(buffer, "up") == 0) {
      penUp();
    }
    else if (strcmp(buffer, "d") == 0 || strcmp(buffer, "down") == 0) {
      penDown();
    }
    else if (buffer[0] == 's' && isdigit(buffer[1])) {
      // Thiết lập góc servo tùy chỉnh (s90)
      int servoAngle = atoi(buffer+1);
      if (servoAngle >= 0 && servoAngle <= 180) {
        penServo.write(servoAngle);
        Serial.print(F("SERVO_SET:"));
        Serial.println(servoAngle);
      }
    }
    else if (buffer[0] == 'h' || strcmp(buffer, "home") == 0) {
      // Home về vị trí 0,0 (góc)
      enableMotors();
      mode = HOME;
      commandCompleted = false;
      moveToAngle(stepperX, 0);
      moveToAngle(stepperY, 0);
      Serial.println(F("HOMING"));
    }
    else if (buffer[0] == 'f') {
      // Thiết lập tốc độ (f1000)
      int feedRate = atoi(buffer+1);
      if (feedRate > 0) {
        stepperX.setMaxSpeed(feedRate);
        stepperY.setMaxSpeed(feedRate);
        Serial.print(F("FEED_SET:"));
        Serial.println(feedRate);
      }
    }
    else if (buffer[0] == 'a') {
      // Thiết lập gia tốc (a300)
      int accel = atoi(buffer+1);
      if (accel > 0) {
        stepperX.setAcceleration(accel);
        stepperY.setAcceleration(accel);
        Serial.print(F("ACCEL_SET:"));
        Serial.println(accel);
      }
    }
    else if (isProcessingGcode) {
      processGcode(buffer);
    }
    else if (strcmp(buffer, "gstart") == 0) {
      isProcessingGcode = true;
      Serial.println(F("GCODE_START"));
    }
    else if (strcmp(buffer, "gend") == 0) {
      isProcessingGcode = false;
      Serial.println(F("GCODE_END"));
    }
    else {
      // Di chuyển theo góc (X,Y) - đến vị trí góc cụ thể
      char* comma = strchr(buffer, ',');
      if (comma) {
        *comma = '\0';
        float xAng = atof(buffer);
        float yAng = atof(comma+1);
        
        enableMotors();
        mode = MOVE;
        commandCompleted = false;
        moveToAngle(stepperX, xAng);
        moveToAngle(stepperY, yAng);
        
        Serial.print(F("MOVING_TO:"));
        Serial.print(xAng);
        Serial.print(F(","));
        Serial.println(yAng);
      }
    }
  }

  // Xử lý chuyển động
  bool xMoving = stepperX.distanceToGo() != 0;
  bool yMoving = stepperY.distanceToGo() != 0;
  
  if (xMoving) stepperX.run();
  if (yMoving) stepperY.run();
  
  // Thông báo khi hoàn thành chuyển động
  if ((!xMoving && !yMoving) && !commandCompleted && (mode == HOME || mode == MOVE)) {
    commandCompleted = true;
    Serial.println(F("MOVE_COMPLETE"));
    mode = IDLE;
  }
}

// Hàm xử lý G-code
void processGcode(const char* gcode) {
  commandCompleted = false;
  
  // Bỏ qua comment hoặc dòng trống
  if (strlen(gcode) < 1 || gcode[0] == ';') {
    commandCompleted = true;
    return;
  }

  // Copy để xử lý
  char gcopy[32];
  strncpy(gcopy, gcode, 31);
  gcopy[31] = '\0';
  
  // Chuyển thành chữ hoa
  for (char* p = gcopy; *p; ++p) *p = toupper(*p);
  
  // Trích xuất tham số
  float x = 0, y = 0, z = 0, f = 0;
  bool hasX = false, hasY = false, hasZ = false;
  
  // Tìm các tham số
  char *token = strtok(gcopy, " ");
  while (token != NULL) {
    if (*token == 'X') {
      x = atof(token + 1);
      hasX = true;
    } else if (*token == 'Y') {
      y = atof(token + 1);
      hasY = true;
    } else if (*token == 'Z') {
      z = atof(token + 1);
      hasZ = true;
    } else if (*token == 'F') {
      f = atof(token + 1);
    }
    token = strtok(NULL, " ");
  }
  
  // Reset để sử dụng lại cho so sánh lệnh
  strncpy(gcopy, gcode, 31);
  gcopy[31] = '\0';
  for (char* p = gcopy; *p; ++p) *p = toupper(*p);

  // Xử lý G-code
  if (strncmp(gcopy, "G0 ", 3) == 0 || strncmp(gcopy, "G1 ", 3) == 0) {
    // Xử lý Z (nâng/hạ bút)
    if (hasZ) {
      if (z <= 0) {
        penDown();
      } else {
        penUp();
      }
    }
    
    // Xử lý X, Y - Di chuyển theo góc (đã được tính toán bởi Python)
    if (hasX && hasY) {
      mode = MOVE;
      enableMotors();
      
      // Thiết lập tốc độ nếu được chỉ định
      if (f > 0) {
        float speedFactor = constrain(f / 60.0, 100, 2000);
        stepperX.setMaxSpeed(speedFactor);
        stepperY.setMaxSpeed(speedFactor);
      }
      
      // Di chuyển đến góc đã tính từ Python
      moveToAngle(stepperX, x);
      moveToAngle(stepperY, y);
      
      Serial.print(F("MOVING_TO:"));
      Serial.print(x);
      Serial.print(F(","));
      Serial.println(y);
    } 
    else if (hasZ) {
      commandCompleted = true;
    }
    else {
      Serial.println(F("ERROR:Missing coordinates"));
      commandCompleted = true;
    }
  }
  else if (strncmp(gcopy, "G4", 2) == 0) {
    // Tìm tham số P (thời gian tạm dừng - ms)
    char *p_param = strstr(gcopy, "P");
    if (p_param) {
      long delay_ms = atol(p_param + 1);
      Serial.print(F("DWELL:"));
      Serial.println(delay_ms);
      delay(delay_ms);
    }
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "G28", 3) == 0) {
    mode = HOME;
    enableMotors();
    moveToAngle(stepperX, 0);
    moveToAngle(stepperY, 0);
    Serial.println(F("HOMING"));
  }
  else if (strncmp(gcopy, "G90", 3) == 0) {
    absolutePositioning = true;
    Serial.println(F("ABS_MODE"));
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "G91", 3) == 0) {
    absolutePositioning = false;
    Serial.println(F("REL_MODE"));
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "M3", 2) == 0) {
    penDown();
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "M5", 2) == 0) {
    penUp();
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "M17", 3) == 0) {
    enableMotors();
    commandCompleted = true;
  }
  else if (strncmp(gcopy, "M84", 3) == 0 || strncmp(gcopy, "M18", 3) == 0) {
    disableMotors();
    commandCompleted = true;
  }
  else {
    Serial.print(F("ERROR:Unknown command - "));
    Serial.println(gcode);
    commandCompleted = true;
  }
}
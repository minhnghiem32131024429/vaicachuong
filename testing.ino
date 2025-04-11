#include <AccelStepper.h>
#include <Servo.h>  

// --- Định nghĩa hướng quay ---
#define X_DIRECTION 1    // 1: chiều thuận, -1: chiều ngược
#define Y_DIRECTION 1    // 1: chiều thuận, -1: chiều ngược

// --- Chân kết nối ---
#define ENABLE_PIN   8
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Y_STEP_PIN   3
#define Y_DIR_PIN    6
#define SERVO_PIN    12  

// --- Thông số động cơ ---
#define STEPS_PER_DEG 0.56      // 200 bước/vòng với 1.8° và microstepping 1/1
#define DEFAULT_SPEED 800       // Tốc độ mặc định
#define DEFAULT_ACCEL 300       // Gia tốc mặc định

// --- Thông số Servo ---
#define PEN_UP_ANGLE   90
#define PEN_DOWN_ANGLE 40
bool isPenDown = false;       

// --- Biến trạng thái ---
float currentX = 90.0;  // Khởi tạo vị trí ban đầu là 90 độ
float currentY = 90.0;  // Khởi tạo vị trí ban đầu là 90 độ
bool isProcessingGcode = false;
bool commandCompleted = true;
bool absolutePositioning = true;
unsigned long lastCommandTime = 0;
#define WATCHDOG_TIMEOUT 5000  // 5 giây

// --- Khởi tạo đối tượng ---
Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { IDLE, MOVE, HOME, TEST } mode = IDLE;

// --- Hàm kiểm tra giới hạn góc ---
bool checkAngleLimits(float theta1, float theta2) {
  // Giới hạn góc - phải khớp với Python
  const float THETA1_MIN = -120, THETA1_MAX = 150; 
  const float THETA2_MIN = 30, THETA2_MAX = 300;
  
  if (theta1 < THETA1_MIN || theta1 > THETA1_MAX) {
    Serial.print(F("ERROR:Angle1 out of range ("));
    Serial.print(THETA1_MIN);
    Serial.print(F(" to "));
    Serial.print(THETA1_MAX);
    Serial.println(F(")"));
    return false;
  }
  
  if (theta2 < THETA2_MIN || theta2 > THETA2_MAX) {
    Serial.print(F("ERROR:Angle2 out of range ("));
    Serial.print(THETA2_MIN);
    Serial.print(F(" to "));
    Serial.print(THETA2_MAX);
    Serial.println(F(")"));
    return false;
  }
  
  return true;
}

// --- Hàm di chuyển tối ưu với hỗ trợ hướng ---
void moveToAngle(AccelStepper &stepper, float targetAngle, int direction = 1) {
  long currentSteps = stepper.currentPosition();
  long targetSteps = (long)(targetAngle * STEPS_PER_DEG * direction);
  
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

// --- Hàm kích hoạt động cơ ---
void enableMotors(bool enabled = true) {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, !enabled);  // LOW = enabled, HIGH = disabled
  if (enabled) {
    Serial.println(F("MOTORS_ENABLED"));
  } else {
    Serial.println(F("MOTORS_DISABLED"));
  }
}

// --- Hàm điều khiển bút cải tiến ---
void penDown() {
  Serial.println(F("Starting pen down..."));
  
  // Không tắt động cơ bước khi điều khiển servo để tránh mất vị trí
  // Thay vào đó, di chuyển servo từ từ với bước nhỏ
  int currentPos = penServo.read();
  
  if (currentPos > PEN_DOWN_ANGLE) {
    for (int angle = currentPos; angle > PEN_DOWN_ANGLE; angle -= 2) {
      penServo.write(angle);
      delay(15);  // Đợi 15ms giữa các bước - nhẹ nhàng hơn
    }
  }
  
  penServo.write(PEN_DOWN_ANGLE);
  delay(50);  // Đợi servo ổn định
  
  isPenDown = true;
  Serial.println(F("PEN_DOWN"));
  Serial.println(F("OK"));
}

void penUp() {
  Serial.println(F("Starting pen up..."));
  
  int currentPos = penServo.read();
  
  if (currentPos < PEN_UP_ANGLE) {
    for (int angle = currentPos; angle < PEN_UP_ANGLE; angle += 2) {
      penServo.write(angle);
      delay(15);
    }
  }
  
  penServo.write(PEN_UP_ANGLE);
  delay(50);
  
  isPenDown = false;
  Serial.println(F("PEN_UP"));
  Serial.println(F("OK"));
}

// --- Chức năng hiệu chuẩn ---
void calibratePosition() {
  stepperX.setCurrentPosition((long)(90 * STEPS_PER_DEG * X_DIRECTION));
  stepperY.setCurrentPosition((long)(90 * STEPS_PER_DEG * Y_DIRECTION));
  currentX = 90.0;
  currentY = 90.0;
  
  Serial.println(F("CALIBRATED_TO_90_90"));
}

// --- Hàm home với kiểm tra vị trí hiện tại ---
void homePosition() {
  enableMotors();
  mode = HOME;
  commandCompleted = false;
  
  // Lấy vị trí hiện tại (góc)
  long currentPosX = stepperX.currentPosition();
  long currentPosY = stepperY.currentPosition();
  float currentAngX = currentPosX / (STEPS_PER_DEG * X_DIRECTION);
  float currentAngY = currentPosY / (STEPS_PER_DEG * Y_DIRECTION);
  
  // Kiểm tra nếu đã ở gần vị trí home
  if (abs(currentAngX - 90) < 2 && abs(currentAngY - 90) < 2) {
    Serial.println(F("ALREADY_AT_HOME"));
    commandCompleted = true;
  } else {
    moveToAngle(stepperX, 90, X_DIRECTION);
    moveToAngle(stepperY, 90, Y_DIRECTION);
    Serial.println(F("HOMING"));
  }
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

void setup() {
  Serial.begin(9600);
  
  // Khởi tạo pin
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Kích hoạt động cơ
  enableMotors(true);

  // Thiết lập servo
  penServo.attach(SERVO_PIN);
  penUp();
  delay(200);

  // Khởi tạo stepper
  stepperX.setMaxSpeed(DEFAULT_SPEED);
  stepperX.setAcceleration(DEFAULT_ACCEL);
  stepperX.setCurrentPosition((long)(90 * STEPS_PER_DEG * X_DIRECTION));

  stepperY.setMaxSpeed(DEFAULT_SPEED); 
  stepperY.setAcceleration(DEFAULT_ACCEL);
  stepperY.setCurrentPosition((long)(90 * STEPS_PER_DEG * Y_DIRECTION));
  
  // Reset watchdog
  lastCommandTime = millis();
  
  // Thông báo sẵn sàng
  Serial.println(F("SCARA_READY"));
}

void loop() {
  // Kiểm tra watchdog - nếu quá lâu không nhận được lệnh, reset trạng thái
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT && !commandCompleted && mode != IDLE) {
    Serial.println(F("WATCHDOG: Command timeout - resetting state"));
    commandCompleted = true;
    mode = IDLE;
  }
  
  // Xử lý lệnh từ Serial
  if (Serial.available()) {
    lastCommandTime = millis();  // Reset watchdog timer
    
    char buffer[32];
    uint8_t len = Serial.readBytesUntil('\n', buffer, 31);
    buffer[len] = '\0';
    
    // Xử lý lệnh
    processCommand(buffer);
  }

  // Xử lý chuyển động
  bool xMoving = stepperX.distanceToGo() != 0;
  bool yMoving = stepperY.distanceToGo() != 0;
  
  if (xMoving) stepperX.run();
  if (yMoving) stepperY.run();
  
  // Thông báo khi hoàn thành chuyển động
  static bool wasMoving = false;
  bool isMovingNow = (xMoving || yMoving);
  
  if (wasMoving && !isMovingNow && !commandCompleted && (mode == HOME || mode == MOVE)) {
    commandCompleted = true;
    
    // Cập nhật vị trí hiện tại
    currentX = stepperX.currentPosition() / (STEPS_PER_DEG * X_DIRECTION);
    currentY = stepperY.currentPosition() / (STEPS_PER_DEG * Y_DIRECTION);
    
    Serial.println(F("MOVE_COMPLETE"));
    mode = IDLE;
  }
  
  wasMoving = isMovingNow;
}

// --- Hàm xử lý lệnh ---
void processCommand(const char* buffer) {
  // Xử lý lệnh gốc từ Python
  if (strcmp(buffer, "status") == 0) {
    Serial.println(F("OK"));
    Serial.println(commandCompleted ? F("READY") : F("BUSY"));
  }
  else if (strcmp(buffer, "test") == 0) {
    enableMotors(true);
    mode = TEST;
    testSteppers();
    mode = IDLE;
    Serial.println(F("TEST_COMPLETE"));
  }
  else if (strcmp(buffer, "enable") == 0) {
    enableMotors(true);
  }
  else if (strcmp(buffer, "disable") == 0) {
    enableMotors(false);
  }
  else if (strcmp(buffer, "u") == 0 || strcmp(buffer, "up") == 0) {
    penUp();
  }
  else if (strcmp(buffer, "d") == 0 || strcmp(buffer, "down") == 0) {
    penDown();
  }
  else if (strcmp(buffer, "calibrate") == 0) {
    calibratePosition();
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
    // Sử dụng hàm home
    homePosition();
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
  else if (strcmp(buffer, "gstart") == 0) {
    isProcessingGcode = true;
    Serial.println(F("GCODE_START"));
  }
  else if (strcmp(buffer, "gend") == 0) {
    isProcessingGcode = false;
    Serial.println(F("GCODE_END"));
  }
  else if (isProcessingGcode) {
    processGcode(buffer);
  }
  else {
    // Di chuyển theo góc (X,Y) - đến vị trí góc cụ thể
    char* comma = strchr(buffer, ',');
    if (comma) {
      *comma = '\0';
      float theta1 = atof(buffer);
      float theta2 = atof(comma+1);
      
      // Kiểm tra giới hạn góc trước khi di chuyển
      if (checkAngleLimits(theta1, theta2)) {
        enableMotors(true);
        mode = MOVE;
        commandCompleted = false;
        moveToAngle(stepperX, theta1, X_DIRECTION);
        moveToAngle(stepperY, theta2, Y_DIRECTION);
        
        Serial.print(F("MOVING_TO:"));
        Serial.print(theta1);
        Serial.print(F(","));
        Serial.println(theta2);
      }
    }
    else {
      Serial.print(F("UNKNOWN_COMMAND:"));
      Serial.println(buffer);
    }
  }
  
  // Phản hồi cho các lệnh đơn giản
  if (mode == IDLE && commandCompleted) {
    Serial.println(F("OK"));
  }
}

// Hàm xử lý G-code
void processGcode(const char* gcode) {
  // Bỏ qua comment hoặc dòng trống
  if (strlen(gcode) < 1 || gcode[0] == ';') {
    Serial.println(F("OK"));
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
  bool hasX = false, hasY = false, hasZ = false, hasF = false;
  
  // Tìm các tham số
  char *token = strtok(gcopy, " ");
  char *command = token;  // Lưu lệnh (G0, G1, v.v.)
  
  while ((token = strtok(NULL, " "))) {
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
      hasF = true;
    }
  }
  
  // Xử lý lệnh G-code
  if (strncmp(command, "G0", 2) == 0 || strncmp(command, "G1", 2) == 0) {
    // Xử lý Z (nâng/hạ bút)
    if (hasZ) {
      if (z <= 0) {
        penDown();
      } else {
        penUp();
      }
    }
    
    // Thiết lập tốc độ nếu được chỉ định
    if (hasF) {
      float speedFactor = constrain(f / 60.0, 100, 1000);
      stepperX.setMaxSpeed(speedFactor);
      stepperY.setMaxSpeed(speedFactor);
    }
    
    // Xử lý X, Y - Di chuyển theo góc
    if (hasX && hasY) {
      if (checkAngleLimits(x, y)) {
        mode = MOVE;
        commandCompleted = false;
        moveToAngle(stepperX, x, X_DIRECTION);
        moveToAngle(stepperY, y, Y_DIRECTION);
        
        Serial.print(F("MOVING_TO:"));
        Serial.print(x);
        Serial.print(F(","));
        Serial.println(y);
      }
    } 
    else if (hasZ) {
      Serial.println(F("Z_ONLY"));
    }
    else {
      Serial.println(F("ERROR:Missing coordinates"));
    }
  }
  else if (strncmp(command, "G4", 2) == 0) {
    // Tìm tham số P (thời gian tạm dừng - ms)
    char *p_param = strstr(command, "P");
    if (p_param) {
      long delay_ms = atol(p_param + 1);
      Serial.print(F("DWELL:"));
      Serial.println(delay_ms);
      delay(delay_ms);
      Serial.println(F("DWELL_COMPLETE"));
    }
  }
  else if (strncmp(command, "G28", 3) == 0) {
    homePosition();
  }
  else if (strncmp(command, "G90", 3) == 0) {
    absolutePositioning = true;
    Serial.println(F("ABS_MODE"));
  }
  else if (strncmp(command, "G91", 3) == 0) {
    absolutePositioning = false;
    Serial.println(F("REL_MODE"));
  }
  else if (strncmp(command, "M3", 2) == 0) {
    penDown();
  }
  else if (strncmp(command, "M5", 2) == 0) {
    penUp();
  }
  else if (strncmp(command, "M17", 3) == 0) {
    enableMotors(true);
  }
  else if (strncmp(command, "M84", 3) == 0 || strncmp(command, "M18", 3) == 0) {
    enableMotors(false);
  }
  else {
    Serial.print(F("ERROR:Unknown command - "));
    Serial.println(gcode);
  }
  
  // Đảm bảo phản hồi cho mọi lệnh G-code
  if (mode == IDLE) {
    Serial.println(F("OK"));
  }
}
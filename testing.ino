#include <AccelStepper.h>
#include <Servo.h>  

// --- Định nghĩa hướng quay ---
#define X_DIRECTION 1    // 1: chiều thuận, -1: chiều ngược
#define Y_DIRECTION 1    // Đổi thành -1 nếu trục Y quay ngược

// --- Chân kết nối ---
#define ENABLE_PIN   8
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Y_STEP_PIN   3
#define Y_DIR_PIN    6
#define SERVO_PIN    12  

// --- Thông số động cơ - Đã giảm tốc độ để ổn định ---
#define STEPS_PER_DEG 0.56      // 200 bước/vòng với 1.8° và microstepping 1/1
#define DEFAULT_SPEED 800       // Giảm tốc độ từ 1200 xuống 800
#define DEFAULT_ACCEL 300       // Giảm gia tốc từ 500 xuống 300

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
#define WATCHDOG_TIMEOUT 10000  // 10 giây

Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { IDLE, MOVE, HOME, TEST } mode = IDLE;

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
  
  // In debug log
  Serial.print(F("DEBUG: Moving from steps:"));
  Serial.print(currentSteps);
  Serial.print(F(" to steps:"));
  Serial.print(targetSteps);
  Serial.print(F(" diff:"));
  Serial.println(diff);
  
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

// --- Chức năng hiệu chuẩn ---
void calibratePosition() {
  // Đặt lại vị trí hiện tại là 90°, 90° 
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
  
  Serial.print(F("DEBUG: Current X angle:"));
  Serial.print(currentAngX);
  Serial.print(F(" Y angle:"));
  Serial.println(currentAngY);
  
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

  // Khởi tạo stepper với tốc độ và gia tốc thấp hơn
  stepperX.setMaxSpeed(DEFAULT_SPEED);
  stepperX.setAcceleration(DEFAULT_ACCEL);
  
  // Thiết lập vị trí hiện tại là 90°, 90° với hướng quay
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
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT && !commandCompleted) {
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
    
    // Xử lý lệnh - phản hồi cho tất cả các lệnh
    processCommand(buffer);
    
    // Đảm bảo phản hồi sau mỗi lệnh
    if (mode == IDLE && commandCompleted) {
      Serial.println(F("OK"));
    }
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
    Serial.println(commandCompleted ? F("READY") : F("BUSY"));
    
    // In thêm thông tin debug
    Serial.print(F("DEBUG: X pos:"));
    Serial.print(stepperX.currentPosition());
    Serial.print(F(" Y pos:"));
    Serial.println(stepperY.currentPosition());
  }
  else if (strcmp(buffer, "test") == 0) {
    enableMotors();
    mode = TEST;
    testSteppers();
    mode = IDLE;
    Serial.println(F("TEST_COMPLETE"));
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
    // Sử dụng hàm home mới
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
      float xAng = atof(buffer);
      float yAng = atof(comma+1);
      
      enableMotors();
      mode = MOVE;
      commandCompleted = false;
      moveToAngle(stepperX, xAng, X_DIRECTION);
      moveToAngle(stepperY, yAng, Y_DIRECTION);
      
      Serial.print(F("MOVING_TO:"));
      Serial.print(xAng);
      Serial.print(F(","));
      Serial.println(yAng);
      
      // Cập nhật vị trí đích
      currentX = xAng;
      currentY = yAng;
    }
    else {
      Serial.print(F("UNKNOWN_COMMAND:"));
      Serial.println(buffer);
    }
  }
}

// Hàm xử lý G-code
void processGcode(const char* gcode) {
  // Bỏ qua comment hoặc dòng trống
  if (strlen(gcode) < 1 || gcode[0] == ';') {
    Serial.println(F("EMPTY_LINE"));
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
    
    // Xử lý X, Y - Di chuyển theo góc
    if (hasX && hasY) {
      mode = MOVE;
      enableMotors();
      commandCompleted = false;
      
      // Thiết lập tốc độ nếu được chỉ định
      if (f > 0) {
        float speedFactor = constrain(f / 60.0, 100, 1000);
        stepperX.setMaxSpeed(speedFactor);
        stepperY.setMaxSpeed(speedFactor);
      }
      
      // Di chuyển đến góc được chỉ định
      moveToAngle(stepperX, x, X_DIRECTION);
      moveToAngle(stepperY, y, Y_DIRECTION);
      
      Serial.print(F("MOVING_TO:"));
      Serial.print(x);
      Serial.print(F(","));
      Serial.println(y);
      
      // Cập nhật vị trí đích
      currentX = x;
      currentY = y;
    } 
    else if (hasZ) {
      Serial.println(F("Z_ONLY"));
    }
    else {
      Serial.println(F("ERROR:Missing coordinates"));
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
      Serial.println(F("DWELL_COMPLETE"));
    }
  }
  else if (strncmp(gcopy, "G28", 3) == 0) {
    // Sử dụng hàm home mới
    homePosition();
  }
  else if (strncmp(gcopy, "G90", 3) == 0) {
    absolutePositioning = true;
    Serial.println(F("ABS_MODE"));
  }
  else if (strncmp(gcopy, "G91", 3) == 0) {
    absolutePositioning = false;
    Serial.println(F("REL_MODE"));
  }
  else if (strncmp(gcopy, "M3", 2) == 0) {
    penDown();
  }
  else if (strncmp(gcopy, "M5", 2) == 0) {
    penUp();
  }
  else if (strncmp(gcopy, "M17", 3) == 0) {
    enableMotors();
  }
  else if (strncmp(gcopy, "M84", 3) == 0 || strncmp(gcopy, "M18", 3) == 0) {
    disableMotors();
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
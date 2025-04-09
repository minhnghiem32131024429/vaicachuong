#include <AccelStepper.h>
#include <Servo.h>  

// --- Chân kết nối (CNC Shield v3) ---
#define ENABLE_PIN   8
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Y_STEP_PIN   3
#define Y_DIR_PIN    6
#define SERVO_PIN    12  

// --- Thông số motor & microstep ---
const float FULL_STEP_ANGLE = 1.8;    // Góc mỗi bước đầy đủ (độ)
const int   MICROSTEPS      = 1;      // Số vi bước (1, 2, 4, 8, 16 hoặc 32)
const float STEPS_PER_REV   = (360.0 / FULL_STEP_ANGLE) * MICROSTEPS; // 200 bước/vòng với 1.8°
const float STEPS_PER_DEG   = STEPS_PER_REV / 360.0;                  // Số bước/độ

// --- Thông số Servo ---
const int PEN_UP_ANGLE   = 90;  // Góc servo khi bút ở vị trí nâng lên
const int PEN_DOWN_ANGLE = 40;  // Góc servo khi bút ở vị trí hạ xuống
bool isPenDown = false;       

// --- Thông số máy vẽ ---
const float MM_PER_DEG_X = 1.0;  // mm trên mỗi độ quay trục X (cần hiệu chỉnh)
const float MM_PER_DEG_Y = 1.0;  // mm trên mỗi độ quay trục Y (cần hiệu chỉnh)

// --- Biến vị trí và trạng thái ---
float currentX = 0.0;        // vị trí hiện tại (mm)
float currentY = 0.0;
bool isProcessingGcode = false;
bool commandCompleted = true;  // Để kiểm tra nếu lệnh đã hoàn thành

Servo penServo;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
enum Mode { RUN, HOME, CUSTOM, GCODE } mode = RUN;
int runState = 1;

void setup() {
  Serial.begin(9600);
  Serial.println(F(
    "CNC G-code Plotter - Sẵn sàng nhận lệnh:\n"
    "  a         → HOME (0°)\n"
    "  b         → RUN sync 0→45°(X)&0→90°(Y)→0\n"
    "  X,Y       → custom move (e.g. 30,60)\n"
    "  u         → Pen UP\n"
    "  d         → Pen DOWN\n"
    "  gstart    → Bắt đầu nhận G-code\n"
    "  gend      → Kết thúc nhận G-code\n"
    "  status    → Kiểm tra trạng thái\n"
  ));

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Kích hoạt stepper drivers

  penServo.attach(SERVO_PIN);
  penUp(); 

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // Điều chỉnh tốc độ tùy vào động cơ của bạn
  stepperX.setMaxSpeed(500);
  stepperX.setAcceleration(200);

  stepperY.setMaxSpeed(500);
  stepperY.setAcceleration(200);
}

void loop() {
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    
    if (in == "gstart") {
      isProcessingGcode = true;
      Serial.println("→ Bắt đầu nhận G-code. Gửi từng dòng G-code, kết thúc với 'gend'.");
    }
    else if (in == "gend") {
      isProcessingGcode = false;
      Serial.println("→ Kết thúc nhận G-code.");
    }
    else if (in == "status") {
      if (commandCompleted) {
        Serial.println("READY");
      } else {
        Serial.println("BUSY");
      }
    }
    else if (isProcessingGcode) {
      processGcode(in);
    }
    else if (in == "a") {
      mode = HOME;
      stepperX.moveTo(angleToSteps(0));
      stepperY.moveTo(angleToSteps(0));
      Serial.println("→ HOME (0°) và dừng.");
    }
    else if (in == "b") {
      mode = RUN;
      runState = 1;
      startSyncMove();
      Serial.println("→ Chạy sync 0→45°/0→90°→0.");
    }
    else if (in == "u") {
      penUp();
      Serial.println("→ Bút đã được nhấc lên.");
    }
    else if (in == "d") {
      penDown();
      Serial.println("→ Bút đã được hạ xuống.");
    }
    else if (in.length() > 0) {
      float xAng, yAng;
      int comma = in.indexOf(',');
      if (comma >= 0) {
        xAng = in.substring(0, comma).toFloat();
        yAng = in.substring(comma + 1).toFloat();
      } else {
        xAng = in.toFloat();
        yAng = xAng;
      }
      mode = CUSTOM;
      stepperX.moveTo(angleToSteps(xAng));
      stepperY.moveTo(angleToSteps(yAng));
      Serial.print("→ Custom move: X="); Serial.print(xAng);
      Serial.print("°, Y="); Serial.print(yAng); Serial.println("°");
    }
  }

  // Xử lý chuyển động
  if (mode == HOME || mode == CUSTOM || mode == GCODE) {
    bool xMoving = stepperX.distanceToGo() != 0;
    bool yMoving = stepperY.distanceToGo() != 0;
    
    if (xMoving) stepperX.run();
    if (yMoving) stepperY.run();
    
    // Báo hiệu khi chuyển động hoàn thành
    if (!xMoving && !yMoving && !commandCompleted) {
      commandCompleted = true;
      Serial.println("→ Hoàn thành di chuyển");
    }
  }
  else if (mode == RUN) {
    stepperX.run();
    stepperY.run();
    
    if (runState == 1) {
      if (stepperY.distanceToGo() == 0) {
        stepperX.moveTo(angleToSteps(0));
        stepperY.moveTo(angleToSteps(0));
        runState = 2;
      }
    }
    else if (runState == 2) {
      if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
        runState = 1;
        startSyncMove();
      }
    }
  }
}

// Chuyển từ góc (độ) sang số bước motor
long angleToSteps(float angle) {
  return lround(angle * STEPS_PER_DEG);
}

// Hàm xử lý G-code
void processGcode(String gcode) {
  commandCompleted = false;  // Đánh dấu đang xử lý lệnh
  
  gcode.toUpperCase();
  Serial.print("Xử lý G-code: "); Serial.println(gcode);
  
  if (gcode.length() < 1 || gcode[0] == ';') {
    // Bỏ qua comment hoặc dòng trống
    commandCompleted = true;
    return;
  }

  // Trích xuất tham số
  float x = parseGcodeParam(gcode, 'X');
  float y = parseGcodeParam(gcode, 'Y');
  float z = parseGcodeParam(gcode, 'Z');
  float f = parseGcodeParam(gcode, 'F');
  
  // Xử lý G-code
  if (gcode.startsWith("G0 ") || gcode.startsWith("G1 ")) {
    // Xử lý lệnh di chuyển
    bool hasX = gcode.indexOf('X') >= 0;
    bool hasY = gcode.indexOf('Y') >= 0;
    bool hasZ = gcode.indexOf('Z') >= 0;
    
    // Xử lý Z (nâng/hạ bút)
    if (hasZ) {
      if (z <= 0) {
        penDown();
        Serial.println("→ Bút đã được hạ xuống (Z <= 0)");
      } else {
        penUp();
        Serial.println("→ Bút đã được nhấc lên (Z > 0)");
      }
    }
    
    // Xử lý X, Y (chuyển từ tọa độ mm sang góc)
    if (hasX || hasY) {
      // Lấy giá trị hiện tại nếu không có trong lệnh
      float targetX = hasX ? x : currentX;
      float targetY = hasY ? y : currentY;
      
      // Chuyển từ tọa độ mm sang góc
      float xAngle = targetX / MM_PER_DEG_X;  // Chuyển X từ mm sang góc
      float yAngle = targetY / MM_PER_DEG_Y;  // Chuyển Y từ mm sang góc
      
      // Giới hạn góc an toàn
      xAngle = constrain(xAngle, -180.0, 180.0);
      yAngle = constrain(yAngle, -180.0, 180.0);
      
      // Di chuyển đến vị trí mới
      mode = GCODE;
      stepperX.moveTo(angleToSteps(xAngle));
      stepperY.moveTo(angleToSteps(yAngle));
      
      Serial.print("→ Di chuyển đến X="); Serial.print(xAngle);
      Serial.print("°, Y="); Serial.print(yAngle); Serial.println("°");
      
      // Cập nhật vị trí hiện tại
      currentX = targetX;
      currentY = targetY;
      
      // Cập nhật tốc độ nếu có
      if (f > 0) {
        float speedFactor = f / 60.0;  // Chuyển từ mm/phút sang mm/giây
        stepperX.setMaxSpeed(speedFactor * STEPS_PER_DEG);
        stepperY.setMaxSpeed(speedFactor * STEPS_PER_DEG);
      }
    } else {
      // Nếu chỉ có Z, đánh dấu lệnh đã hoàn thành
      commandCompleted = true;
    }
  }
  else if (gcode.startsWith("G28")) {
    // Về home
    mode = HOME;
    stepperX.moveTo(angleToSteps(0));
    stepperY.moveTo(angleToSteps(0));
    currentX = 0;
    currentY = 0;
    Serial.println("→ Về home (0,0)");
  }
  else if (gcode.startsWith("M3")) {
    // Hạ bút xuống
    penDown();
    Serial.println("→ Bút đã được hạ xuống (M3)");
    commandCompleted = true;
  }
  else if (gcode.startsWith("M5")) {
    // Nhấc bút lên
    penUp();
    Serial.println("→ Bút đã được nhấc lên (M5)");
    commandCompleted = true;
  }
  else {
    // Lệnh không được hỗ trợ
    Serial.print("→ Lệnh không hỗ trợ: "); Serial.println(gcode);
    commandCompleted = true;
  }
}

// Hàm trích xuất tham số từ G-code
float parseGcodeParam(String gcode, char param) {
  int idx = gcode.indexOf(param);
  if (idx < 0) return 0;  // Không tìm thấy tham số
  
  // Tìm vị trí bắt đầu của giá trị sau tham số
  idx++;
  
  // Tìm vị trí kết thúc của giá trị (khoảng trắng hoặc hết chuỗi)
  int endIdx = idx;
  while (endIdx < gcode.length() && 
        (isDigit(gcode[endIdx]) || gcode[endIdx] == '.' || gcode[endIdx] == '-')) {
    endIdx++;
  }
  
  // Trích xuất và chuyển đổi giá trị
  String value = gcode.substring(idx, endIdx);
  return value.toFloat();
}

// Các hàm điều khiển bút
void penUp() {
  penServo.write(PEN_UP_ANGLE);
  delay(200);  // Đợi servo di chuyển
  isPenDown = false;
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(200);  // Đợi servo di chuyển
  isPenDown = true;
}

// Hàm khởi tạo chuyển động đồng bộ
void startSyncMove() {
  stepperX.moveTo(angleToSteps(45.0));
  stepperY.moveTo(angleToSteps(90.0));
}
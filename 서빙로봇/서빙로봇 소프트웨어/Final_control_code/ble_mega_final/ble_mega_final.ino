#include "MPU9250.h"
#include <Wire.h>
#include "odometry.h"   // ★ 추가
#include <PS2X_lib.h>

// PS2 핀 정의
#define PS2_DAT        52  
#define PS2_CMD        51  
#define PS2_SEL        53  
#define PS2_CLK        50  

// 모터 핀 정의
#define PWMA 12    
#define DIRA1 34 
#define DIRA2 35   
#define PWMB 8      
#define DIRB1 37 
#define DIRB2 36   
#define PWMC 9      
#define DIRC1 43 
#define DIRC2 42   
#define PWMD 5     
#define DIRD1 A4 
#define DIRD2 A5   

// 엔코더 핀 정의
#define ENCA_A  18
#define ENCA_B  3
#define ENCA_C  2
#define ENCA_D  19

volatile long encA = 0, encB = 0, encC = 0, encD = 0;
float speedA = 0, speedB = 0, speedC = 0, speedD = 0;
unsigned long lastSpeedTime = 0;
unsigned long lastPidTime = 0;           // PID 제어 전용 타이머
unsigned long lastImuTime = 0;           // IMU(자이로) 읽기 전용 타이머
const unsigned long IMU_INTERVAL = 50;   // IMU 주기 (ms) — 20Hz (서빙 로봇에 충분)
const unsigned long PID_INTERVAL = 300;  // PID 주기 (ms) — I2C와 시간차를 두어 충돌 방지

unsigned long lastRecvTime = 0;          // 마지막 통신 수신 시점 (워치독용)
const unsigned long WATCHDOG_TIMEOUT = 3000; // 3초간 통신 없으면 비상 정지

float targetSpeed = 80.0;
int baseA = 120, baseB = 120, baseC = 120, baseD = 120;
int pwmA = 0, pwmB = 0, pwmC = 0, pwmD = 0;

int Motor_PWM = 100;

MPU9250 mpu;

// 각도 및 시간 변수
float roll = 0, pitch = 0, yaw = 0;
unsigned long last_ms = 0;

// 보정 및 필터 계수
float gyroZ_offset = 0;
const float alpha = 0.96;
const float deadzone = 0.25;

bool isCalibrated = false;
PS2X ps2x;
int ps2_error = 0;
byte ps2_type = 0;
byte vibrate = 0;
bool wasPs2Controlled = false;
bool isTurning = false;
float destinationAngle = 0.0;
const float ANGLE_TOLERANCE = 2.0;

// 엔코더 인터럽트 (ISR)
void isrEncA() { encA++; }
void isrEncB() { encB++; }
void isrEncC() { encC++; }
void isrEncD() { encD++; }

class SimplePID {
public:
  float kp, ki, kd;
  float error, lastError, integral;
  unsigned long lastT;
  float lastOutput;

  SimplePID() {}
  SimplePID(float _kp, float _ki, float _kd) {
    kp = _kp; ki = _ki; kd = _kd;
    error = lastError = integral = 0;
    lastT = 0;
    lastOutput = 0;
  }

  float compute(float target, float current) {
    unsigned long now = millis();
    float dt = (now - lastT) / 1000.0;
    if (dt <= 0.0) dt = 0.001;

    error = target - current;
    integral += error * dt;
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    float derivative = (error - lastError) / dt;
    float out = kp * error + ki * integral + kd * derivative;
    lastError = error;
    lastT = now;
    lastOutput = out;
    return out;
  }

  void setKp(float _kp) { kp = _kp; }
  void setKi(float _ki) { ki = _ki; }
  void setKd(float _kd) { kd = _kd; }
  void resetIntegral() { integral = 0; lastError = 0; }

  void printGains() {
    Serial.print("Kp="); Serial.print(kp, 3);
    Serial.print(" Ki="); Serial.print(ki, 3);
    Serial.print(" Kd="); Serial.print(kd, 3);
  }
};

SimplePID pidA(0.3, 0.02, 0.08);
SimplePID pidB(0.3, 0.02, 0.08);
SimplePID pidC(0.3, 0.02, 0.08);
SimplePID pidD(0.3, 0.02, 0.08);

void motorDrive(int pwmPin, int d1, int d2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    analogWrite(pwmPin, -pwm);
  } else {
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void ADVANCE() {
  motorDrive(PWMA, DIRA1, DIRA2,  pwmA);  
  motorDrive(PWMB, DIRB1, DIRB2, -pwmB);   
  motorDrive(PWMC, DIRC1, DIRC2,  pwmC);   
  motorDrive(PWMD, DIRD1, DIRD2, -pwmD);   
}

void STOP() {
  motorDrive(PWMA, DIRA1, DIRA2, 0);   
  motorDrive(PWMB, DIRB1, DIRB2, 0);
  motorDrive(PWMC, DIRC1, DIRC2, 0);   
  motorDrive(PWMD, DIRD1, DIRD2, 0);
}

void TURN_RIGHT() {
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_PWM);      
  motorDrive(PWMB, DIRB1, DIRB2,  Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2,  Motor_PWM); 
}

void TURN_LEFT() {
  motorDrive(PWMA, DIRA1, DIRA2, -Motor_PWM);      
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM); 
  motorDrive(PWMC, DIRC1, DIRC2, -Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void BACK() {
  motorDrive(PWMA, DIRA1, DIRA2, -Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2,  Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2, -Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2,  Motor_PWM);
}

void LEFT_1() {
  motorDrive(PWMA, DIRA1, DIRA2, 0);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, 0);
}

void LEFT_2() {
  motorDrive(PWMA, DIRA1, DIRA2, -Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2,  Motor_PWM);
}

void LEFT_3() {
  motorDrive(PWMA, DIRA1, DIRA2, -Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, 0);
  motorDrive(PWMC, DIRC1, DIRC2, 0);
  motorDrive(PWMD, DIRD1, DIRD2,  Motor_PWM);
}

void RIGHT_1() {
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, 0);
  motorDrive(PWMC, DIRC1, DIRC2, 0);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void RIGHT_2() {
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2,  Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2, -Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void RIGHT_3() {
  motorDrive(PWMA, DIRA1, DIRA2, 0);
  motorDrive(PWMB, DIRB1, DIRB2,  Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2, -Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, 0);
}

void LEFT_11() {
  int Motor_Low_PWM = Motor_PWM - 80;
  if (Motor_Low_PWM < 0) Motor_Low_PWM = 0;
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_Low_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_Low_PWM);
}

void LEFT_12() {
  int Motor_Low_PWM = Motor_PWM - 80;
  if (Motor_Low_PWM < 0) Motor_Low_PWM = 0;
  motorDrive(PWMA, DIRA1, DIRA2, -Motor_Low_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2,  Motor_Low_PWM);
}

void RIGHT_11() {
  int Motor_Low_PWM = Motor_PWM - 80;
  if (Motor_Low_PWM < 0) Motor_Low_PWM = 0;
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_Low_PWM);
  motorDrive(PWMC, DIRC1, DIRC2,  Motor_Low_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void RIGHT_12() {
  int Motor_Low_PWM = Motor_PWM - 80;
  if (Motor_Low_PWM < 0) Motor_Low_PWM = 0;
  motorDrive(PWMA, DIRA1, DIRA2,  Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2,  Motor_Low_PWM);
  motorDrive(PWMC, DIRC1, DIRC2, -Motor_Low_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void initStraightMode() {
  pidA.resetIntegral(); pidB.resetIntegral();
  pidC.resetIntegral(); pidD.resetIntegral();
  pidA.lastOutput = 0; pidB.lastOutput = 0;
  pidC.lastOutput = 0; pidD.lastOutput = 0;
  pwmA = baseA; pwmB = baseB; pwmC = baseC; pwmD = baseD;
  encA = 0; encB = 0; encC = 0; encD = 0;
  lastSpeedTime = millis(); 
  Serial.println("PID 초기화");
}

void print_all_data(uint32_t t) {
    String dir = "";
    if (yaw > 337.5 || yaw <= 22.5)   dir = "북(정면)";
    else if (yaw > 22.5 && yaw <= 67.5)   dir = "북동";
    else if (yaw > 67.5 && yaw <= 112.5)  dir = "동";
    else if (yaw > 112.5 && yaw <= 157.5) dir = "남동";
    else if (yaw > 157.5 && yaw <= 202.5) dir = "남";
    else if (yaw > 202.5 && yaw <= 247.5) dir = "남서";
    else if (yaw > 247.5 && yaw <= 292.5) dir = "서";
    else if (yaw > 292.5 && yaw <= 337.5) dir = "북서";

    Serial.print("Y(Heading): "); Serial.print(yaw, 1);
    Serial.print(" ["); Serial.print(dir); Serial.print("]");

    // ★ 위치 좌표 출력 추가
    /*Serial.print("  |  X: "); Serial.print(pose.x, 3);
    Serial.print("m  Y: ");   Serial.print(pose.y, 3);
    Serial.println("m");

    // ★ Nano(Serial2)로 좌표 및 각도 데이터 전송 (나노의 Serial1에 도달)
    Serial2.print("POS:");
    Serial2.print(pose.x, 3);
    Serial2.print(",");
    Serial2.print(pose.y, 3);
    Serial2.print(",");
    Serial2.println(yaw, 1);*/
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  
  delay(300); // PS2 무선 모듈 기동 대기
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  if (ps2_error == 0) {
    ps2_type = ps2x.readType();
    Serial.println("PS2 컨트롤러 연결 성공");
  } else {
    Serial.print("PS2 컨트롤러 감지 실패, 에러 코드: ");
    Serial.println(ps2_error);
  }
  
  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);
  STOP();     

  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCA_C, INPUT_PULLUP);
  pinMode(ENCA_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isrEncB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_C), isrEncC, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_D), isrEncD, RISING);

  Wire.begin();
  if (!mpu.setup(0x68)) {
    while (1) { delay(1000); }
  }

  mpu.calibrateAccelGyro();

  float sum = 0;
  int validSamples = 0;
  unsigned long startCalib = millis();
  
  Serial.print("IMU 칼리브레이션 시작");
  while (validSamples < 500 && millis() - startCalib < 8000) {
    STOP(); 
    if (mpu.update()) {
      sum += mpu.getGyroZ();
      validSamples++;
      if (validSamples % 100 == 0) Serial.print(".");
    }
    delay(10);
  }
  Serial.println();

  if (validSamples > 0) {
    gyroZ_offset = sum / (float)validSamples;
  } else {
    gyroZ_offset = 0.0;
  }
  Serial.print("Calculated Gyro Z Offset: ");
  Serial.println(gyroZ_offset, 4);
  
  while (Serial2.available() > 0) Serial2.read();

  STOP(); 
  yaw = 0;
  resetOdometry();   // ★ 위치 초기화
  last_ms = millis();
  isCalibrated = true; 
  lastRecvTime = millis(); // 워치독 타이머 초기화
  Serial.println("준비 완료");
}

void loop() {
  uint32_t now = millis();

  // ── IMU(자이로) 읽기: 50ms 주기 (20Hz) ──────────────────────
  if (now - lastImuTime >= IMU_INTERVAL) {
    lastImuTime = now;

    if (mpu.update()) { 
      float dt = (now - last_ms) / 1000.0; 
      last_ms = now;

      float ax = mpu.getAccX(); 
      float ay = mpu.getAccY(); 
      float az = mpu.getAccZ();
      float acc_roll  = atan2(ay, az) * RAD_TO_DEG;
      float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
      roll  = alpha * (roll  + mpu.getGyroX() * dt) + (1.0 - alpha) * acc_roll;
      pitch = alpha * (pitch + mpu.getGyroY() * dt) + (1.0 - alpha) * acc_pitch;
      
      static float lastValidGz = 0.0;
      float gz = mpu.getGyroZ() - gyroZ_offset; 
      if (abs(gz) < deadzone) gz = 0;

      // 센서 오류 등으로 인한 비정상적인 값(예: 500 deg/s 초과)이나 nan/inf 필터링
      if (isnan(gz) || isinf(gz) || abs(gz) > 500.0) {
        gz = lastValidGz; // 에러 발생 시 직전의 정상 각속도 값 유지하여 회전 중 오차 최소화
      } else {
        lastValidGz = gz; // 정상 데이터 백업
      }

      float next_yaw = yaw - gz * dt;
      if (!isnan(next_yaw) && !isinf(next_yaw)) {
        yaw = next_yaw;
        while (yaw < 0.0)    yaw += 360.0;
        while (yaw >= 360.0) yaw -= 360.0;
      }

      static uint32_t p_ms = 0;
      if (now > p_ms + 500) { 
        print_all_data(now);
        p_ms = now;
      }
    } else {
      // 모터 기동 시 전압 강하/노이즈로 인한 I2C 통신 끊김 복구 루틴
      static uint32_t lastResetTime = 0;
      if (now - lastResetTime >= 500) {
        lastResetTime = now;
        Serial.println("[경고] 자이로 센서 I2C 통신 오류 감지! I2C 버스 및 센서 재연결 시도...");
        Wire.end();
        delay(10);
        Wire.begin();
        mpu.setup(0x68);
      }
    }
  }

  // ── PS2 컨트롤러 처리 ──────────────────────────────────
  bool ps2Controlled = false;
  if (ps2_error == 0 && ps2_type != 2) {
    ps2x.read_gamepad(false, vibrate);
    
    bool buttonPressed = ps2x.Button(PSB_START) || ps2x.Button(PSB_PAD_UP) || 
                         ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT) || 
                         ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_SELECT) || 
                         ps2x.Button(PSB_PINK) || ps2x.Button(PSB_RED) || 
                         ps2x.Button(PSB_GREEN) || ps2x.Button(PSB_BLUE) ||
                         ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1);
                         
    if (buttonPressed) {
      ps2Controlled = true;
      wasPs2Controlled = true;
      isTurning = false;
      pwmA = 0; pwmB = 0; pwmC = 0; pwmD = 0;
      lastRecvTime = now; // 워치독 방지
      
      if (ps2x.Button(PSB_START)) {
        Motor_PWM = 90;
        pwmA = pwmB = pwmC = pwmD = Motor_PWM;
        ADVANCE();
      }
      else if (ps2x.Button(PSB_PAD_UP)) {
        Motor_PWM = 120;
        pwmA = pwmB = pwmC = pwmD = Motor_PWM;
        ADVANCE();
      }
      else if (ps2x.Button(PSB_PAD_DOWN)) {
        Motor_PWM = 120;
        BACK();
      }
      else if (ps2x.Button(PSB_PAD_LEFT)) {
        Motor_PWM = 100;
        LEFT_11();
      }
      else if (ps2x.Button(PSB_PAD_RIGHT)) {
        Motor_PWM = 100;
        RIGHT_11();
      }
      else if (ps2x.Button(PSB_SELECT)) {
        STOP();
      }
      else if (ps2x.Button(PSB_PINK)) {
        Motor_PWM = 100;
        LEFT_12();
      }
      else if (ps2x.Button(PSB_RED)) {
        Motor_PWM = 100;
        RIGHT_12();
      }
      else if (ps2x.Button(PSB_GREEN)) {
        Motor_PWM = 100;
        LEFT_3();
      }
      else if (ps2x.Button(PSB_BLUE)) {
        Motor_PWM = 100;
        RIGHT_3();
      }
      else if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
        int LY = ps2x.Analog(PSS_LY);
        int LX = ps2x.Analog(PSS_LX);
        
        if (LY < 127) {
          Motor_PWM = 1.5 * (127 - LY);
          pwmA = pwmB = pwmC = pwmD = Motor_PWM;
          ADVANCE();
        }
        else if (LY > 127) {
          Motor_PWM = 1.5 * (LY - 128);
          BACK();
        }
        else if (LX < 128) {
          Motor_PWM = 1.5 * (127 - LX);
          LEFT_1();
        }
        else if (LX > 128) {
          Motor_PWM = 1.5 * (LX - 128);
          RIGHT_3();
        }
        else {
          STOP();
        }
      }
      delay(20);
    } else {
      if (wasPs2Controlled) {
        STOP();
        wasPs2Controlled = false;
      }
    }
  }

  if (!ps2Controlled) {
    // PC 명령 수신 (중복 명령 연속 수신 방지 포함)
    static int lastCommand = -9999;        // 마지막 처리한 명령값
    static unsigned long lastCmdTime = 0;  // 마지막 명령 처리 시간
    const unsigned long CMD_COOLDOWN = 500; // 동일 명령 반복 처리 최소 간격 (ms)

    if (isCalibrated && Serial2.available()) { 
      String input = Serial2.readStringUntil('\n');
      input.trim();
      
      if (input.length() > 0) {
        // 1. 비정상적으로 긴 데이터 필터링 (최대 10자 제한 - 노이즈 방지)
        if (input.length() > 10) {
          Serial.println("[경고] 비정상적인 길이의 명령 무시 (노이즈)");
          return;
        }

        lastRecvTime = now; // 통신 정상 수신 시각 업데이트 (워치독 리셋)
        // ★ "RESET_ODO" 명령 추가 — 위치를 (0,0)으로 리셋
        if (input == "RESET_ODO") {
          resetOdometry();
          Serial.println("위치 초기화 완료 (0, 0)");
          Serial2.println("POS:0.000,0.000,0.0"); // 초기화 즉시 나노에 전송
          return;
        }

        // 유효성 검사: 수신된 문자열이 유효한 숫자인지 확인 (노이즈 방지)
        bool isValidNumber = true;
        if (input.length() == 0) {
          isValidNumber = false;
        } else {
          for (unsigned int i = 0; i < input.length(); i++) {
            if (i == 0 && input[i] == '-') continue; // 음수 기호 허용
            if (!isDigit(input[i])) {
              isValidNumber = false;
              break;
            }
          }
        }

        if (!isValidNumber) {
          // 노이즈 로그는 에러 폭풍 방지를 위해 가끔만 출력
          static unsigned long lastNoiseLog = 0;
          if (now - lastNoiseLog >= 2000) {
            lastNoiseLog = now;
            Serial.print("유효하지 않은 명령 무시 (노이즈): ");
            Serial.println(input);
          }
          return;
        }

        int target = input.toInt();

        // 동일 명령 연속 수신 방지: 같은 명령이 빠르게 들어오면 무시
        if (target == lastCommand && (now - lastCmdTime) < CMD_COOLDOWN) {
          return; // 중복 명령 무시
        }
        lastCommand = target;
        lastCmdTime = now;

        Serial.print("명령 각도 수신: "); Serial.println(target);

        if (target == -1) {
          STOP();
          isTurning = false;
          pwmA = 0; pwmB = 0; pwmC = 0; pwmD = 0;
        } 
        else if (target == 0) {
          if (isTurning || pwmA == 0) { 
            initStraightMode();
          }
          isTurning = false;
          ADVANCE();
        } 
        else {
          STOP();
          pwmA = 0; pwmB = 0; pwmC = 0; pwmD = 0;
          delay(100);
          destinationAngle = yaw + target;
          if (destinationAngle >= 360.0) destinationAngle -= 360.0;
          if (destinationAngle < 0.0)   destinationAngle += 360.0;
          isTurning = true;
        }
      }
    }

    // ── 엔코더/오도메트리: 100ms 주기 ──────────────────────
    if (!isTurning && pwmA > 0 && (now - lastSpeedTime >= 100)) {
      float dt = (now - lastSpeedTime) / 1000.0;
      if (dt <= 0.0) dt = 0.1;

      // ★ 오도메트리용으로 먼저 복사 (리셋 전에 읽어야 함)
      long cA = encA; encA = 0;
      long cB = encB; encB = 0;
      long cC = encC; encC = 0;
      long cD = encD; encD = 0;

      // ★ 위치 좌표 업데이트
      updateOdometry(cA, cB, cC, cD, yaw, isTurning);

      speedA = (cA / 10.0) / dt;
      speedB = (cB / 10.0) / dt;
      speedC = (cC / 10.0) / dt;
      speedD = (cD / 10.0) / dt;
      lastSpeedTime = now;

      ADVANCE(); 
    }

    // ── PID 제어: 비활성화 상태 ──────────────────────────────────
    /*
    if (!isTurning && pwmA > 0 && (now - lastPidTime >= PID_INTERVAL)) {
      lastPidTime = now;

      float corrA = constrain(pidA.compute(targetSpeed, speedA), -70.0, 70.0);
      float corrB = constrain(pidB.compute(targetSpeed, speedB), -70.0, 70.0);
      float corrC = constrain(pidC.compute(targetSpeed, speedC), -70.0, 70.0);
      float corrD = constrain(pidD.compute(targetSpeed, speedD), -70.0, 70.0);

      pwmA = constrain(baseA + (int)corrA, 0, 255);
      pwmB = constrain(baseB + (int)corrB, 0, 255);
      pwmC = constrain(baseC + (int)corrC, 0, 255);
      pwmD = constrain(baseD + (int)corrD, 0, 255);
    }
    */

    // 제자리 회전 시퀀스
    if (isTurning) {
      float error = destinationAngle - yaw;
      if (error > 180)  error -= 360;
      if (error < -180) error += 360;

      if (abs(error) <= ANGLE_TOLERANCE) {
        STOP(); 
        delay(150);
        isTurning = false;
        initStraightMode();
        ADVANCE();
        Serial.println("직진 시작");
      } 
      else {
        if (error > 0) TURN_RIGHT();
        else           TURN_LEFT();
      }
    }

    // ── 안전 워치독 (Safety Watchdog) ──────────────────────
    // 로봇이 구동 중(pwmA > 0 또는 회전 중)인데 3초 동안 통신 수신이 없으면 비상 정지
    /*if ((pwmA > 0 || isTurning) && (now - lastRecvTime > WATCHDOG_TIMEOUT)) {
      STOP();
      isTurning = false;
      pwmA = 0; pwmB = 0; pwmC = 0; pwmD = 0;
      lastRecvTime = now; // 연속적인 비상정지 로그 출력 방지
      Serial.println("[비상] 통신 끊김 감지! 모터 비상 정지 실행");
    }*/
  }
}

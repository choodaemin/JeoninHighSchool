#include "odometry.h" // ★ 추가
#include <PS2X_lib.h>

// PS2 핀 정의
#define PS2_DAT 52
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 50

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
#define ENCA_A 18
#define ENCA_B 3
#define ENCA_C 2
#define ENCA_D 19

volatile long encA = 0, encB = 0, encC = 0, encD = 0;
float speedA = 0, speedB = 0, speedC = 0, speedD = 0;
unsigned long lastSpeedTime = 0;

unsigned long lastRecvTime = 0; // 마지막 통신 수신 시점 (워치독용)

// ── 주행 속도 및 가감속 제어 변수 ────────────────────────────
int Motor_PWM = 125;             // 기본 주행 PWM 기준값
float targetSpeed = 125.0;       // PID 엔코더 목표 속도
int currentBasePWM = 0;          // 램프 가감속용 현재 PWM
const int RAMP_ACCEL_STEP = 20;  // 100ms당 가속 폭 (약 0.6초에 목표 속도 도달)
const int RAMP_DECEL_STEP = 30;  // 100ms당 감속 폭 (부드러운 정지)

int pwmA = 0, pwmB = 0, pwmC = 0, pwmD = 0;

// 헤딩 보정 (Heading Lock) 전용 변수
float targetHeading = 0.0;
const float KP_HEADING = 2.0;    // 각도 1도 오차당 목표 속도 보정량 (튜닝 가능)

// 각도 변수 (나노 BLE에서 YAW 수신)
float yaw = 0;

bool isCalibrated =
    true; // 나노가 부팅 시 칼리브레이션을 수행하므로 메가는 상시 참으로 시작
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

  SimplePID() {
    kp = 0;
    ki = 0;
    kd = 0;
    error = lastError = integral = 0;
    lastT = 0;
    lastOutput = 0;
  }

  SimplePID(float _kp, float _ki, float _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    error = lastError = integral = 0;
    lastT = 0;
    lastOutput = 0;
  }

  float compute(float target, float current) {
    unsigned long now = millis();

    // 첫 호출이거나 시간이 갱신되지 않은 경우 방어
    if (lastT == 0) {
      lastT = now;
      lastError = target - current;
      return 0;
    }

    float dt = (now - lastT) / 1000.0;
    // dt가 0 이하이거나 비정상적으로 긴 경우(0.5초 초과) 기본 0.1초로 제한하여
    // 적분 폭주 방지
    if (dt <= 0.0 || dt > 0.5)
      dt = 0.1;

    error = target - current;
    integral += error * dt;
    if (integral > 50)
      integral = 50;
    if (integral < -50)
      integral = -50;

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

  void reset() {
    integral = 0;
    lastError = 0;
    lastT = millis();
    lastOutput = 0;
  }

  void printGains() {
    Serial.print("Kp=");
    Serial.print(kp, 3);
    Serial.print(" Ki=");
    Serial.print(ki, 3);
    Serial.print(" Kd=");
    Serial.print(kd, 3);
  }
};

SimplePID pidA(0.3, 0.02, 0.08);
SimplePID pidB(0.3, 0.02, 0.08);
SimplePID pidC(0.3, 0.02, 0.08);
SimplePID pidD(0.3, 0.02, 0.08);

// 각 바퀴의 현재 회전 방향 (+1: 전진, -1: 후진, 0: 정지)
int dirSignA = 0, dirSignB = 0, dirSignC = 0, dirSignD = 0;

void motorDrive(int pwmPin, int d1, int d2, int pwm) {
  pwm = constrain(pwm, -255, 255);

  int sign = 0;
  if (pwm > 0)
    sign = 1;
  else if (pwm < 0)
    sign = -1;

  // 하드웨어 배선에 맞춘 바퀴별 방향 판별 (ADVANCE 전진 시 +1, BACK 후진 시 -1)
  // PWMA/PWMC는 음수 PWM이 전진, PWMB/PWMD는 양수 PWM이 전진
  if (pwmPin == PWMA)
    dirSignA = -sign;
  else if (pwmPin == PWMB)
    dirSignB = sign;
  else if (pwmPin == PWMC)
    dirSignC = -sign;
  else if (pwmPin == PWMD)
    dirSignD = sign;

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
  motorDrive(PWMA, DIRA1, DIRA2, -pwmA);
  motorDrive(PWMB, DIRB1, DIRB2, pwmB);
  motorDrive(PWMC, DIRC1, DIRC2, -pwmC);
  motorDrive(PWMD, DIRD1, DIRD2, pwmD);
}

void STOP() {
  motorDrive(PWMA, DIRA1, DIRA2, 0);
  motorDrive(PWMB, DIRB1, DIRB2, 0);
  motorDrive(PWMC, DIRC1, DIRC2, 0);
  motorDrive(PWMD, DIRD1, DIRD2, 0);
}

void TURN_RIGHT(int speed = 0) {
  int spd = (speed == 0) ? Motor_PWM : speed;
  motorDrive(PWMA, DIRA1, DIRA2, -spd);
  motorDrive(PWMB, DIRB1, DIRB2, -spd);
  motorDrive(PWMC, DIRC1, DIRC2, -spd);
  motorDrive(PWMD, DIRD1, DIRD2, -spd);
}

void TURN_LEFT(int speed = 0) {
  int spd = (speed == 0) ? Motor_PWM : speed;
  motorDrive(PWMA, DIRA1, DIRA2, spd);
  motorDrive(PWMB, DIRB1, DIRB2, spd);
  motorDrive(PWMC, DIRC1, DIRC2, spd);
  motorDrive(PWMD, DIRD1, DIRD2, spd);
}

void BACK() {
  motorDrive(PWMA, DIRA1, DIRA2, Motor_PWM);
  motorDrive(PWMB, DIRB1, DIRB2, -Motor_PWM);
  motorDrive(PWMC, DIRC1, DIRC2, Motor_PWM);
  motorDrive(PWMD, DIRD1, DIRD2, -Motor_PWM);
}

void initStraightMode() {
  pidA.reset();
  pidB.reset();
  pidC.reset();
  pidD.reset();
  targetSpeed = (float)Motor_PWM;
  currentBasePWM = 40; // 최소 기동 토크(부드러운 가속 시작점)
  pwmA = pwmB = pwmC = pwmD = currentBasePWM;
  encA = encB = encC = encD = 0;
  lastSpeedTime = millis();
  targetHeading = yaw; // ★ 직진 시작 시점의 각도를 목표 헤딩으로 기억
  Serial.print("PID/가감속/헤딩락 초기화 (목표 각도: ");
  Serial.print(targetHeading, 1);
  Serial.println("도)");
}

void print_all_data(uint32_t t) {
  String dir = "";
  if (yaw > 337.5 || yaw <= 22.5)
    dir = "북(정면)";
  else if (yaw > 22.5 && yaw <= 67.5)
    dir = "북동";
  else if (yaw > 67.5 && yaw <= 112.5)
    dir = "동";
  else if (yaw > 112.5 && yaw <= 157.5)
    dir = "남동";
  else if (yaw > 157.5 && yaw <= 202.5)
    dir = "남";
  else if (yaw > 202.5 && yaw <= 247.5)
    dir = "남서";
  else if (yaw > 247.5 && yaw <= 292.5)
    dir = "서";
  else if (yaw > 292.5 && yaw <= 337.5)
    dir = "북서";

  Serial.print("Y(Heading): ");
  Serial.print(yaw, 1);
  Serial.print(" [");
  Serial.print(dir);
  Serial.print("]");

  // ★ 위치 좌표 출력 추가
  Serial.print("  |  X: ");
  Serial.print(pose.x, 3);
  Serial.print("m  Y: ");
  Serial.print(pose.y, 3);
  Serial.println("m");

  // ★ Nano(Serial2)로 좌표 데이터 전송 (YAW는 나노가 직접 계산하므로 제외)
  Serial2.print("POS:");
  Serial2.print(pose.x, 3);
  Serial2.print(",");
  Serial2.println(pose.y, 3);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // 나노와 고속 통신 설정 (115200bps)

  delay(300); // PS2 무선 모듈 기동 대기
  ps2_error =
      ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  if (ps2_error == 0) {
    ps2_type = ps2x.readType();
    Serial.println("PS2 컨트롤러 연결 성공");
  } else {
    Serial.print("PS2 컨트롤러 감지 실패, 에러 코드: ");
    Serial.println(ps2_error);
  }

  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  STOP();

  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCA_C, INPUT_PULLUP);
  pinMode(ENCA_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isrEncB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_C), isrEncC, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_D), isrEncD, RISING);

  // I2C는 나노가 담당하므로 메가에서 제거됨

  while (Serial2.available() > 0)
    Serial2.read();

  STOP();
  yaw = 0;
  resetOdometry(); // ★ 위치 초기화
  isCalibrated = true;
  lastRecvTime = millis(); // 워치독 타이머 초기화
  Serial.println("준비 완료");
}

void loop() {
  uint32_t now = millis();

  // ── 데이터 전송 및 출력 주기 제어 ──────────────────────
  static uint32_t p_ms = 0;
  if (now - p_ms >= 500) {
    print_all_data(now);
    p_ms = now;
  }

  // ── PS2 컨트롤러 처리 ──────────────────────────────────
  bool ps2Controlled = false;
  if (ps2_error == 0 && ps2_type != 2) {
    ps2x.read_gamepad(false, vibrate);

    bool buttonPressed =
        ps2x.Button(PSB_START) || ps2x.Button(PSB_PAD_UP) ||
        ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT) ||
        ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_SELECT) ||
        ps2x.Button(PSB_PINK) || ps2x.Button(PSB_RED) ||
        ps2x.Button(PSB_GREEN) || ps2x.Button(PSB_BLUE) ||
        ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1);

    if (buttonPressed) {
      ps2Controlled = true;
      wasPs2Controlled = true;
      isTurning = false;
      pwmA = 0;
      pwmB = 0;
      pwmC = 0;
      pwmD = 0;
      lastRecvTime = now; // 워치독 방지

      if (ps2x.Button(PSB_START)) {
        Motor_PWM = 125;
        pwmA = pwmB = pwmC = pwmD = Motor_PWM;
        ADVANCE();
      } else if (ps2x.Button(PSB_PAD_UP)) {
        Motor_PWM = 125;
        pwmA = pwmB = pwmC = pwmD = Motor_PWM;
        ADVANCE();
      } else if (ps2x.Button(PSB_PAD_DOWN)) {
        Motor_PWM = 125;
        BACK();
      } else if (ps2x.Button(PSB_PAD_LEFT)) {
        Motor_PWM = 125;
        TURN_LEFT();
      } else if (ps2x.Button(PSB_PAD_RIGHT)) {
        Motor_PWM = 125;
        TURN_RIGHT();
      } else if (ps2x.Button(PSB_SELECT)) {
        STOP();
      } else if (ps2x.Button(PSB_PINK)) {
        Motor_PWM = 125;
        TURN_LEFT();
      } else if (ps2x.Button(PSB_RED)) {
        Motor_PWM = 125;
        TURN_RIGHT();
      } else if (ps2x.Button(PSB_GREEN)) {
        Motor_PWM = 125;
        pwmA = pwmB = pwmC = pwmD = Motor_PWM;
        ADVANCE();
      } else if (ps2x.Button(PSB_BLUE)) {
        Motor_PWM = 125;
        BACK();
      } else if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
        int LY = ps2x.Analog(PSS_LY);
        int LX = ps2x.Analog(PSS_LX);

        if (LY < 127) {
          Motor_PWM = 1.5 * (127 - LY);
          pwmA = pwmB = pwmC = pwmD = Motor_PWM;
          ADVANCE();
        } else if (LY > 127) {
          Motor_PWM = 1.5 * (LY - 128);
          BACK();
        } else if (LX < 128) {
          Motor_PWM = 1.5 * (127 - LX);
          TURN_LEFT();
        } else if (LX > 128) {
          Motor_PWM = 1.5 * (LX - 128);
          TURN_RIGHT();
        } else {
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

  // ── 엔코더/오도메트리/PID 제어: 100ms 주기 (상시 실행) ──
  if (now - lastSpeedTime >= 100) {
    float dt = (now - lastSpeedTime) / 1000.0;
    if (dt <= 0.0)
      dt = 0.1;

    // 인터럽트 변수 안전 복사 및 리셋
    noInterrupts();
    long rawA = encA;
    encA = 0;
    long rawB = encB;
    encB = 0;
    long rawC = encC;
    encC = 0;
    long rawD = encD;
    encD = 0;
    interrupts();

    // 모터 회전 방향(전진/후진) 부호 반영
    long cA = rawA * dirSignA;
    long cB = rawB * dirSignB;
    long cC = rawC * dirSignC;
    long cD = rawD * dirSignD;

    // ★ 위치 좌표 업데이트
    updateOdometry(cA, cB, cC, cD, yaw, isTurning);

    speedA = (abs(cA) / 10.0) / dt;
    speedB = (abs(cB) / 10.0) / dt;
    speedC = (abs(cC) / 10.0) / dt;
    speedD = (abs(cD) / 10.0) / dt;
    lastSpeedTime = now;

    // 직진 주행 중일 때만 100ms 주기로 PID 연산 및 출력 반영
    if (!ps2Controlled && !isTurning && pwmA > 0) {
      // ── 1. 가감속(Ramp) 점진적 속도 증가/감소 (목표: Motor_PWM) ──
      if (currentBasePWM < Motor_PWM) {
        currentBasePWM = min(currentBasePWM + RAMP_ACCEL_STEP, Motor_PWM);
      } else if (currentBasePWM > Motor_PWM) {
        currentBasePWM = max(currentBasePWM - RAMP_DECEL_STEP, Motor_PWM);
      }

      // ── 2. 헤딩 오차 계산 (-180 ~ +180도 정규화) ──
      float headingError = targetHeading - yaw;
      if (headingError > 180.0)
        headingError -= 360.0;
      if (headingError < -180.0)
        headingError += 360.0;

      // 헤딩 보정량 계산 (오차가 클 때 과도한 보정 방지를 위해 ±30.0으로 제한)
      float headingCorrection = constrain(headingError * KP_HEADING, -30.0f, 30.0f);

      // 좌/우 바퀴의 목표 속도 차등 적용
      // (차체가 오른쪽으로 틀어지면 headingError < 0 -> 좌측 증속, 우측 감속)
      float targetSpeedLeft  = (float)Motor_PWM - headingCorrection;
      float targetSpeedRight = (float)Motor_PWM + headingCorrection;

      float corrA = constrain(pidA.compute(targetSpeedLeft,  speedA), -70.0, 70.0);
      float corrB = constrain(pidB.compute(targetSpeedRight, speedB), -70.0, 70.0);
      float corrC = constrain(pidC.compute(targetSpeedLeft,  speedC), -70.0, 70.0);
      float corrD = constrain(pidD.compute(targetSpeedRight, speedD), -70.0, 70.0);

      pwmA = constrain(currentBasePWM + (int)corrA, 0, 255);
      pwmB = constrain(currentBasePWM + (int)corrB, 0, 255);
      pwmC = constrain(currentBasePWM + (int)corrC, 0, 255);
      pwmD = constrain(currentBasePWM + (int)corrD, 0, 255);

      ADVANCE();
    }
  }

  if (!ps2Controlled) {
    // ── 안전 워치독 (1.5초 이상 통신 두절 시 자동 비상 정지) ──
    if (now - lastRecvTime > 1500) {
      if (pwmA > 0 || isTurning) {
        currentBasePWM = 0;
        STOP();
        isTurning = false;
        pwmA = pwmB = pwmC = pwmD = 0;
        static unsigned long lastWdLog = 0;
        if (now - lastWdLog > 2000) {
          lastWdLog = now;
          Serial.println("[경고] 통신 두절로 인한 비상 정지 (Watchdog)");
        }
      }
    }

    // ── PC/나노 명령 논블로킹(Non-blocking) 수신 ──
    static String inputBuffer = "";
    static int lastCommand = -9999;       // 마지막 처리한 명령값
    static unsigned long lastCmdTime = 0; // 마지막 명령 처리 시간
    const unsigned long CMD_COOLDOWN =
        500; // 동일 명령 반복 처리 최소 간격 (ms)

    while (isCalibrated && Serial2.available() > 0) {
      char c = Serial2.read();
      if (c == '\n') {
        inputBuffer.trim();

        if (inputBuffer.length() > 0) {
          // YAW 데이터 파싱 (나노로부터 공급받음)
          if (inputBuffer.startsWith("YAW:")) {
            float tempYaw = inputBuffer.substring(4).toFloat();
            if (!isnan(tempYaw) && !isinf(tempYaw)) {
              yaw = tempYaw;
              lastRecvTime = now; // 워치독 타이머 갱신
            }
          }
          // 1. 비정상적으로 긴 데이터 필터링 (최대 10자 제한 - 노이즈 방지)
          else if (inputBuffer.length() > 10) {
            Serial.println("[경고] 비정상적인 길이의 명령 무시 (노이즈)");
          }
          // ★ "RESET_ODO" 명령 추가 — 위치를 (0,0)으로 리셋
          else if (inputBuffer == "RESET_ODO") {
            lastRecvTime = now; // 워치독 리셋
            resetOdometry();
            Serial.println("위치 초기화 완료 (0, 0)");
            Serial2.println("POS:0.000,0.000,0.0"); // 초기화 즉시 나노에 전송
          } else {
            lastRecvTime = now; // 워치독 리셋

            // 유효성 검사: 수신된 문자열이 유효한 숫자인지 확인 (노이즈 방지)
            bool isValidNumber = true;
            for (unsigned int i = 0; i < inputBuffer.length(); i++) {
              if (i == 0 && inputBuffer[i] == '-')
                continue; // 음수 기호 허용
              if (!isDigit(inputBuffer[i])) {
                isValidNumber = false;
                break;
              }
            }

            if (!isValidNumber) {
              static unsigned long lastNoiseLog = 0;
              if (now - lastNoiseLog >= 2000) {
                lastNoiseLog = now;
                Serial.print("유효하지 않은 명령 무시 (노이즈): ");
                Serial.println(inputBuffer);
              }
            } else {
              int target = inputBuffer.toInt();

              // 동일 명령 연속 수신 방지
              if (!(target == lastCommand &&
                    (now - lastCmdTime) < CMD_COOLDOWN)) {
                lastCommand = target;
                lastCmdTime = now;

                Serial.print("명령 각도 수신: ");
                Serial.println(target);

                if (target == -1) {
                  currentBasePWM = 0;
                  STOP();
                  isTurning = false;
                  pwmA = pwmB = pwmC = pwmD = 0;
                } else if (target == 0) {
                  if (isTurning || pwmA == 0) {
                    initStraightMode();
                  }
                  isTurning = false;
                  ADVANCE();
                } else {
                  STOP();
                  pwmA = pwmB = pwmC = pwmD = 0;
                  delay(100);
                  destinationAngle = yaw + target;
                  if (destinationAngle >= 360.0)
                    destinationAngle -= 360.0;
                  if (destinationAngle < 0.0)
                    destinationAngle += 360.0;
                  isTurning = true;
                }
              }
            }
          }
        }
        inputBuffer = ""; // 1줄 처리 완료 후 버퍼 비우기
      } else if (c != '\r') {
        inputBuffer += c;
        if (inputBuffer.length() > 30) inputBuffer = ""; // 노이즈 오버플로우 방어
      }
    }
    // 제자리 회전 시퀀스 (오차 비례 부드러운 감속 회전)
    if (isTurning) {
      float error = destinationAngle - yaw;
      if (error > 180.0)
        error -= 360.0;
      if (error < -180.0)
        error += 360.0;

      if (abs(error) <= ANGLE_TOLERANCE) {
        STOP();
        delay(150);
        isTurning = false;
        initStraightMode();
        ADVANCE();
        Serial.println("직진 시작");
      } else {
        // 남은 오차(2도 ~ 45도)에 따라 회전 PWM을 60 ~ Motor_PWM 사이로 비례 감속
        int turnPwm = constrain((int)(abs(error) * 2.0f + 55), 60, Motor_PWM);

        if (error > 0)
          TURN_RIGHT(turnPwm);
        else
          TURN_LEFT(turnPwm);
      }
    }
  }
}
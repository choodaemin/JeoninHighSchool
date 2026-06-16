// ============================================================
// 정사각형 주행 테스트 전용 코드 (BLE 제거 버전)
// - 전원 투입 후 30초 대기 → 자동으로 정사각형 주행 시작
// - Serial1(TX/RX)을 통해 Mega 2560에 명령 전송
// - USB Serial(시리얼 모니터)은 디버깅 로그 출력 전용
// ============================================================

// 정사각형 주행 테스트 설정
const unsigned long STARTUP_DELAY = 30000;   // 시작 대기 시간 (ms)
const unsigned long FORWARD_DURATION = 3000; // 전진 시간 (ms)
const unsigned long TURN_DURATION = 2500;    // 회전 대기 시간 (ms)

bool squareTestStarted = false;

enum SquareState {
  SQ_FORWARD_START,
  SQ_FORWARD_WAIT,
  SQ_TURN_START,
  SQ_TURN_WAIT
};
SquareState squareState = SQ_FORWARD_START;
unsigned long squareStateTime = 0;
int squareStepCount = 0;

// 카운트다운 표시용
unsigned long lastCountdownPrint = 0;

void setup() {
  Serial.begin(9600);   // USB 시리얼 모니터용 (디버깅)
  Serial1.begin(9600);  // Mega 2560과 통신할 하드웨어 시리얼 (TX/RX 핀)

  Serial.println("========================================");
  Serial.println("  정사각형 주행 테스트 (BLE 제거 버전)");
  Serial.println("========================================");
  Serial.print("전진 시간: "); Serial.print(FORWARD_DURATION); Serial.println("ms");
  Serial.print("회전 대기: "); Serial.print(TURN_DURATION); Serial.println("ms");
  Serial.print("시작 대기: "); Serial.print(STARTUP_DELAY / 1000); Serial.println("초");
  Serial.println("----------------------------------------");
  Serial.println("30초 후 자동 시작합니다...");
}

void loop() {
  unsigned long now = millis();

  // ── 시작 전 카운트다운 ──────────────────────────────────
  if (!squareTestStarted) {
    // 1초마다 남은 시간 표시
    if (now - lastCountdownPrint >= 1000) {
      lastCountdownPrint = now;
      int remaining = (STARTUP_DELAY - now) / 1000;
      if (remaining > 0) {
        Serial.print("시작까지 ");
        Serial.print(remaining);
        Serial.println("초...");
      }
    }

    // 대기 시간 완료 → 테스트 시작
    if (now >= STARTUP_DELAY) {
      squareTestStarted = true;

      // 위치 초기화 명령 전송
      Serial1.println("RESET_ODO");
      delay(100);

      squareState = SQ_FORWARD_START;
      squareStepCount = 0;

      Serial.println("========================================");
      Serial.println(">> 정사각형 테스트 시작! (위치 초기화 완료) <<");
      Serial.println("========================================");
    }
    return; // 대기 중에는 여기서 종료
  }

  // ── 정사각형 테스트 상태 머신 ──────────────────────────────
  switch (squareState) {
    case SQ_FORWARD_START:
      Serial1.println("0");
      Serial.print("[");
      Serial.print(squareStepCount + 1);
      Serial.println("번째 변] 전진 명령(0) 전송");
      squareState = SQ_FORWARD_WAIT;
      squareStateTime = now;
      break;

    case SQ_FORWARD_WAIT:
      if (now - squareStateTime >= FORWARD_DURATION) {
        squareState = SQ_TURN_START;
      }
      break;

    case SQ_TURN_START:
      Serial1.println("90");
      Serial.print("[");
      Serial.print(squareStepCount + 1);
      Serial.println("번째 변] 90도 회전 명령(90) 전송");
      squareState = SQ_TURN_WAIT;
      squareStateTime = now;
      break;

    case SQ_TURN_WAIT:
      if (now - squareStateTime >= TURN_DURATION) {
        squareStepCount++;
        Serial.print(">> ");
        Serial.print(squareStepCount);
        Serial.println("번째 변 완료! <<");

        if (squareStepCount >= 4) {
          // 정사각형 한 바퀴 완료 → 정지
          Serial1.println("-1");
          Serial.println("========================================");
          Serial.println(">> 정사각형 1바퀴 완료! 정지합니다. <<");
          Serial.println("========================================");
          squareTestStarted = false; // 다시 시작하지 않음
        } else {
          squareState = SQ_FORWARD_START;
        }
      }
      break;
  }

  // ── USB 시리얼 비상 정지 ──────────────────────────────────
  // 시리얼 모니터에서 아무 글자나 입력하면 즉시 정지
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      Serial1.println("-1");
      squareTestStarted = false;
      Serial.println("========================================");
      Serial.println(">> 비상 정지! (USB 입력 감지) <<");
      Serial.println("========================================");
    }
  }
}
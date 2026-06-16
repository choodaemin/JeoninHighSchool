#include <ArduinoBLE.h>

// [센서 1] 핀 변경 (D9, D10)
const int trigPin1 = 9;
const int echoPin1 = 10;

// [센서 2] 핀 변경 (D11, D12)
const int trigPin2 = 11;
const int echoPin2 = 12;

float pre_ch1, pre_ch2; // 이전 값
float ch1, ch2;         // 현재 값

unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 1000;

// ── 인터럽트 기반 초음파 측정 ──────────────────────────
// 센서 상태 머신
enum SensorState {
  SENSOR_IDLE,       // 대기
  SENSOR_TRIG_HIGH,  // trig HIGH 출력 중
  SENSOR_WAIT_ECHO,  // echo RISING 대기
  SENSOR_MEASURING,  // echo HIGH → FALLING 대기
  SENSOR_DONE        // 측정 완료
};

// 센서별 데이터 구조체
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  volatile unsigned long echoStart;  // echo RISING 시점 (micros)
  volatile unsigned long echoEnd;    // echo FALLING 시점 (micros)
  volatile bool echoReceived;        // 측정 완료 플래그
  SensorState state;
  unsigned long trigStartTime;       // trig 시작 시점 (micros)
  unsigned long timeoutStart;        // 타임아웃 체크용 (micros)
  float distance;                    // 최종 거리 (cm)
};

UltrasonicSensor sensor1;
UltrasonicSensor sensor2;

// 현재 측정 중인 센서 인덱스 (0: 없음, 1: 센서1, 2: 센서2)
volatile int activeSensorIndex = 0;

// ── 인터럽트 핸들러 (센서 1) ──────────────────────────
void echoISR1() {
  if (digitalRead(sensor1.echoPin) == HIGH) {
    // RISING: echo 시작
    sensor1.echoStart = micros();
  } else {
    // FALLING: echo 끝
    sensor1.echoEnd = micros();
    sensor1.echoReceived = true;
  }
}

// ── 인터럽트 핸들러 (센서 2) ──────────────────────────
void echoISR2() {
  if (digitalRead(sensor2.echoPin) == HIGH) {
    sensor2.echoStart = micros();
  } else {
    sensor2.echoEnd = micros();
    sensor2.echoReceived = true;
  }
}

// 센서 측정 시작 (trig 펄스 발사)
void startMeasurement(UltrasonicSensor &sensor) {
  sensor.echoReceived = false;
  sensor.echoStart = 0;
  sensor.echoEnd = 0;
  sensor.distance = -1;

  digitalWrite(sensor.trigPin, LOW);
  sensor.state = SENSOR_TRIG_HIGH;
  sensor.trigStartTime = micros();
}

// 비블로킹 센서 상태 업데이트
// 반환: true = 측정 완료(또는 타임아웃), false = 아직 진행 중
bool updateSensor(UltrasonicSensor &sensor) {
  unsigned long now = micros();

  switch (sensor.state) {
    case SENSOR_IDLE:
      return true;

    case SENSOR_TRIG_HIGH:
      // trig LOW 후 최소 2μs 후 HIGH
      if (now - sensor.trigStartTime >= 2) {
        digitalWrite(sensor.trigPin, HIGH);
        sensor.trigStartTime = now;
        sensor.state = SENSOR_WAIT_ECHO;
      }
      return false;

    case SENSOR_WAIT_ECHO:
      // trig HIGH 유지 10μs 후 LOW
      if (now - sensor.trigStartTime >= 10) {
        digitalWrite(sensor.trigPin, LOW);
        sensor.timeoutStart = now;
        sensor.state = SENSOR_MEASURING;
      }
      return false;

    case SENSOR_MEASURING:
      // 인터럽트에서 echoReceived가 설정되면 완료
      if (sensor.echoReceived) {
        unsigned long duration = sensor.echoEnd - sensor.echoStart;
        sensor.distance = duration * 0.034 / 2.0;
        sensor.state = SENSOR_DONE;
        return true;
      }
      // 타임아웃: 20ms
      if (now - sensor.timeoutStart > 20000) {
        sensor.distance = -1;
        sensor.state = SENSOR_DONE;
        return true;
      }
      return false;

    case SENSOR_DONE:
      return true;
  }
  return true;
}

// ── 센서 측정 시퀀서 ──────────────────────────────────
// 두 센서를 순차 비블로킹으로 측정하는 상태 머신
enum MeasurePhase {
  PHASE_IDLE,
  PHASE_SENSOR1,
  PHASE_SENSOR2,
  PHASE_COMPLETE
};

MeasurePhase measurePhase = PHASE_IDLE;

// 측정 시작 요청
void requestMeasurement() {
  if (measurePhase == PHASE_IDLE) {
    measurePhase = PHASE_SENSOR1;
    startMeasurement(sensor1);
  }
}

// 비블로킹으로 측정 진행 (loop에서 매번 호출)
// 반환: true = 두 센서 모두 측정 완료
bool updateMeasurement() {
  switch (measurePhase) {
    case PHASE_IDLE:
      return false;

    case PHASE_SENSOR1:
      if (updateSensor(sensor1)) {
        // 센서1 완료 → 센서2 시작
        measurePhase = PHASE_SENSOR2;
        startMeasurement(sensor2);
      }
      return false;

    case PHASE_SENSOR2:
      if (updateSensor(sensor2)) {
        measurePhase = PHASE_COMPLETE;
        return true;
      }
      return false;

    case PHASE_COMPLETE:
      return true;
  }
  return false;
}

// 측정 결과 확인 후 IDLE로 리셋
void finishMeasurement() {
  measurePhase = PHASE_IDLE;
  sensor1.state = SENSOR_IDLE;
  sensor2.state = SENSOR_IDLE;
}

// BLE 설정
BLEService myService("12345678-1234-1234-1234-1234567890ab");
BLEStringCharacteristic myChar("abcdefab-1234-5678-1234-abcdefabcdef", BLERead | BLEWrite | BLENotify, 100);

void setup() {
  Serial.begin(9600);   // USB 시리얼 모니터용 (디버깅)
  Serial1.begin(9600);  // Mega 2560과 통신할 하드웨어 시리얼 (TX/RX 핀)

  if (!BLE.begin()) {
    Serial.println("실패");
    while (1);
  }

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  // 센서 구조체 초기화
  sensor1.trigPin = trigPin1;
  sensor1.echoPin = echoPin1;
  sensor1.state = SENSOR_IDLE;
  sensor1.echoReceived = false;

  sensor2.trigPin = trigPin2;
  sensor2.echoPin = echoPin2;
  sensor2.state = SENSOR_IDLE;
  sensor2.echoReceived = false;

  // Echo 핀에 인터럽트 등록 (CHANGE: RISING + FALLING 모두 감지)
  attachInterrupt(digitalPinToInterrupt(echoPin1), echoISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin2), echoISR2, CHANGE);

  BLE.setLocalName("Nano_BLE");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(myChar);
  BLE.addService(myService);
  BLE.advertise();
  Serial.println("PC 연결 대기 중...");
}

void loop() {
  // ★ BLE.poll()을 매 루프마다 호출하여 BLE 스택 유지
  BLE.poll();

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("PC 연결됨: ");
    Serial.println(central.address());

    while (central.connected()) {
      // ★ 연결 유지 중에도 BLE 이벤트 처리
      BLE.poll();

      unsigned long currentMillis = millis();

      // 측정 주기 도달 → 측정 요청
      if (measurePhase == PHASE_IDLE && currentMillis - lastSensorTime >= sensorInterval) {
        lastSensorTime = currentMillis;
        pre_ch1 = ch1;
        pre_ch2 = ch2;
        requestMeasurement();
      }

      // 비블로킹 측정 업데이트
      if (updateMeasurement()) {
        ch1 = sensor1.distance;
        ch2 = sensor2.distance;
        Serial.println(ch2);

        if (pre_ch1 != ch1 || pre_ch2 != ch2) {
          myChar.writeValue(String(ch1) + "," + String(ch2) + "\n");
        }
        finishMeasurement();
      }

      // BLE 수신 처리
      if (myChar.written()) {
        String command = myChar.value();
        command.trim(); // 공백 제거

        // 디버깅용 출력
        Serial.print("수신 데이터: ");
        Serial.println(command);

        // Mega 2560으로 데이터 전달
        Serial1.println(command);
      }
    }
    Serial.println("연결 종료");
  }
}
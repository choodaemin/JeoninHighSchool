#include <ArduinoBLE.h>

// [센서 1] 핀 변경 (D9, D10)
const int trigPin1 = 9;
const int echoPin1 = 10;

// [센서 2] 핀 변경 (D11, D12)
const int trigPin2 = 11;
const int echoPin2 = 12;

float pre_ch1, pre_ch2;
float ch1, ch2;

unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 1000;

bool bleAvailable = false;

// 로봇 상태 변수
float robotX = 0.0;
float robotY = 0.0;
float robotYaw = 0.0;

// BLE 설정
BLEService myService("12345678-1234-1234-1234-1234567890ab");
BLEStringCharacteristic myChar("abcdefab-1234-5678-1234-abcdefabcdef", BLERead | BLEWrite | BLENotify, 100);

// 센서 상태 머신
enum SensorState {
  SENSOR_IDLE,
  SENSOR_TRIG_HIGH,
  SENSOR_WAIT_ECHO,
  SENSOR_MEASURING,
  SENSOR_DONE
};

struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  volatile unsigned long echoStart;
  volatile unsigned long echoEnd;
  volatile bool echoReceived;
  SensorState state;
  unsigned long trigStartTime;
  unsigned long timeoutStart;
  float distance;
};

UltrasonicSensor sensor1;
UltrasonicSensor sensor2;

volatile int activeSensorIndex = 0;

void echoISR1() {
  if (digitalRead(sensor1.echoPin) == HIGH) {
    sensor1.echoStart = micros();
  } else {
    sensor1.echoEnd = micros();
    sensor1.echoReceived = true;
  }
}

void echoISR2() {
  if (digitalRead(sensor2.echoPin) == HIGH) {
    sensor2.echoStart = micros();
  } else {
    sensor2.echoEnd = micros();
    sensor2.echoReceived = true;
  }
}

void startMeasurement(UltrasonicSensor &sensor) {
  sensor.echoReceived = false;
  sensor.echoStart = 0;
  sensor.echoEnd = 0;
  sensor.distance = -1;

  digitalWrite(sensor.trigPin, LOW);
  sensor.state = SENSOR_TRIG_HIGH;
  sensor.trigStartTime = micros();
}

bool updateSensor(UltrasonicSensor &sensor) {
  unsigned long now = micros();

  switch (sensor.state) {
    case SENSOR_IDLE:
      return true;

    case SENSOR_TRIG_HIGH:
      if (now - sensor.trigStartTime >= 2) {
        digitalWrite(sensor.trigPin, HIGH);
        sensor.trigStartTime = now;
        sensor.state = SENSOR_WAIT_ECHO;
      }
      return false;

    case SENSOR_WAIT_ECHO:
      if (now - sensor.trigStartTime >= 10) {
        digitalWrite(sensor.trigPin, LOW);
        sensor.timeoutStart = now;
        sensor.state = SENSOR_MEASURING;
      }
      return false;

    case SENSOR_MEASURING:
      if (sensor.echoReceived) {
        unsigned long duration = sensor.echoEnd - sensor.echoStart;
        sensor.distance = duration * 0.034 / 2.0;
        sensor.state = SENSOR_DONE;
        return true;
      }
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

enum MeasurePhase {
  PHASE_IDLE,
  PHASE_SENSOR1,
  PHASE_SENSOR2,
  PHASE_COMPLETE
};

MeasurePhase measurePhase = PHASE_IDLE;

void requestMeasurement() {
  if (measurePhase == PHASE_IDLE) {
    measurePhase = PHASE_SENSOR1;
    startMeasurement(sensor1);
  }
}

bool updateMeasurement() {
  switch (measurePhase) {
    case PHASE_IDLE:
      return false;

    case PHASE_SENSOR1:
      if (updateSensor(sensor1)) {
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

void finishMeasurement() {
  measurePhase = PHASE_IDLE;
  sensor1.state = SENSOR_IDLE;
  sensor2.state = SENSOR_IDLE;
}

void handleCommand(String command) {
  Serial1.println(command);
}

void sendCombinedData() {
  if (bleAvailable) {
    BLEDevice central = BLE.central();
    if (central && central.connected()) {
      String msg = String(robotX, 3) + "," + String(robotY, 3) + "," + String(robotYaw, 1) + "," + String(ch1, 1) + "," + String(ch2, 1) + "\n";
      myChar.writeValue(msg);
    }
  }
}

void readMegaSerial() {
  static String inputBuffer = "";
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        Serial.print("Mega 수신 데이터: ");
        Serial.println(inputBuffer);

        if (inputBuffer.startsWith("POS:")) {
          int comma1 = inputBuffer.indexOf(',', 4);
          int comma2 = inputBuffer.indexOf(',', comma1 + 1);
          if (comma1 != -1 && comma2 != -1) {
            float tempX   = inputBuffer.substring(4, comma1).toFloat();
            float tempY   = inputBuffer.substring(comma1 + 1, comma2).toFloat();
            float tempYaw = inputBuffer.substring(comma2 + 1).toFloat();

            if (!isnan(tempX) && !isinf(tempX) &&
                !isnan(tempY) && !isinf(tempY) &&
                !isnan(tempYaw) && !isinf(tempYaw) &&
                tempYaw >= 0.0 && tempYaw <= 360.0) {
              robotX = tempX;
              robotY = tempY;
              robotYaw = tempYaw;
              sendCombinedData();
            }
          }
        }
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

// ── BLE 수신 명령 처리 ────────────────────────────────
void handleBLECommand() {
  if (!myChar.written()) return;

  String command = myChar.value();
  command.trim();

  // 콤마 포함(결합 데이터) 필터링
  if (command.indexOf(',') != -1) return;

  // 유효 범위 필터링 (-90 ~ 90)
  int val = command.toInt();
  if (command.length() == 0 || val < -90 || val > 90) return;

  Serial.print("BLE 수신 데이터: ");
  Serial.println(command);
  handleCommand(command);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패! BLE 없이 주행 테스트를 계속 진행합니다.");
    bleAvailable = false;
  } else {
    Serial.println("BLE 초기화 성공.");
    bleAvailable = true;
  }

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  sensor1.trigPin = trigPin1;
  sensor1.echoPin = echoPin1;
  sensor1.state = SENSOR_IDLE;
  sensor1.echoReceived = false;

  sensor2.trigPin = trigPin2;
  sensor2.echoPin = echoPin2;
  sensor2.state = SENSOR_IDLE;
  sensor2.echoReceived = false;

  attachInterrupt(digitalPinToInterrupt(echoPin1), echoISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin2), echoISR2, CHANGE);

  if (bleAvailable) {
    BLE.setLocalName("Nano_BLE");
    BLE.setAdvertisedService(myService);
    myService.addCharacteristic(myChar);
    BLE.addService(myService);

    myChar.writeValue("");  // 초기값 클리어
    myChar.written();       // written() 플래그 소모

    BLE.advertise();
    Serial.println("PC 연결 대기 중...");
  } else {
    Serial.println("BLE가 비활성화된 상태로 주행 대기 중...");
  }
}

void loop() {
  if (bleAvailable) {
    BLE.poll();
  }

  if (Serial && Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      Serial.print("USB 수신 데이터: ");
      Serial.println(command);
      handleCommand(command);
    }
  }

  readMegaSerial();

  if (bleAvailable) {
    BLEDevice central = BLE.central();

    if (central) {
      Serial.print("PC 연결됨: ");
      Serial.println(central.address());

      while (central.connected()) {
        BLE.poll();

        if (Serial && Serial.available() > 0) {
          String command = Serial.readStringUntil('\n');
          command.trim();
          if (command.length() > 0) {
            Serial.print("USB 수신 데이터: ");
            Serial.println(command);
            handleCommand(command);
          }
        }

        readMegaSerial();

        unsigned long currentMillis = millis();

        if (measurePhase == PHASE_IDLE && currentMillis - lastSensorTime >= sensorInterval) {
          lastSensorTime = currentMillis;
          pre_ch1 = ch1;
          pre_ch2 = ch2;
          requestMeasurement();
        }

        if (updateMeasurement()) {
          ch1 = sensor1.distance;
          ch2 = sensor2.distance;
          Serial.println(ch2);
          sendCombinedData();
          finishMeasurement();
        }

        handleBLECommand();  // ← 별도 함수로 분리 및 필터링 적용
      }

      // 연결 종료 시 플래그 클리어
      myChar.writeValue("");
      myChar.written();
      Serial.println("연결 종료");
    }
  }
}
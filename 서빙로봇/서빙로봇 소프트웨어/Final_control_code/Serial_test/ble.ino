나노 ble 통신 코드
#include <ArduinoBLE.h> 

BLEService myService("12345678-1234-1234-1234-1234567890ab");
BLEStringCharacteristic myChar("abcdefab-1234-5678-1234-abcdefabcdef", BLERead | BLEWrite | BLENotify, 100);

bool firstSend = false; // 최초 1회 자동 전송을 위한 플래그

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // 메가와 연결된 시리얼 (Nano의 D0, D1 핀)

  if (!BLE.begin()) {
    while (1);
  }

  BLE.setLocalName("Nano_BLE");
  BLE.setAdvertisedService(myService);
  myService.addCharacteristic(myChar);
  BLE.addService(myService);
  BLE.advertise();

  Serial.println("Nano BLE Auto Bridge Ready...");
}

void loop() {
  BLEDevice central = BLE.central();

  // [최초 자동 시작] 보드가 켜지자마자 메가에게 먼저 'A'를 전송
  if (!firstSend) {
    delay(1000); // 메가가 준비될 때까지 잠시 대기
    Serial.println("[Auto Start]: Sending 'A' to Mega");
    Serial1.println("A"); 
    firstSend = true; // 다시 실행되지 않도록 차단
  }

  // [무한 루프] 메가에서 데이터가 들어오면 무조건 'A'로 답장
  if (Serial1.available()) {
    String megaMsg = Serial1.readStringUntil('\n');
    megaMsg.trim();
    
    if (megaMsg.length() > 0) {
      Serial.print("[From Mega]: ");
      Serial.println(megaMsg); // 메가가 보낸 'B' 출력

      // 너무 빠르게 전송되면 시리얼 버퍼가 터질 수 있으므로 0.1초(100ms) 대기
      delay(100); 
      
      // 메가에게 다시 'A' 전송
      Serial.println("[Nano]: Sending 'A'");
      Serial1.println("A"); 
    }
  }
}

메가 ble 통신 코드
void setup() {
  Serial.begin(9600);   // PC 디버깅용
  Serial2.begin(9600);  // 나노와 연결된 시리얼 (Mega의 TX2, RX2 핀)
  Serial.println("Mega Auto-Responder Ready...");
}

void loop() {
  // 나노(BLE)로부터 데이터가 들어오면
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n'); // '\n'까지 읽기
    msg.trim();
    
    if (msg.length() > 0) {
      Serial.print("[From Nano]: ");
      Serial.println(msg); // 나노가 보낸 'A' 출력
      
      // 나노에게 'B'를 회신 (나노가 readStringUntil로 읽도록 println 사용)
      Serial2.println("B"); 
    }
  }
}
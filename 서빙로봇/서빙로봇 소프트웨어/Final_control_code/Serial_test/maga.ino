//메가 ble 통신 코드
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
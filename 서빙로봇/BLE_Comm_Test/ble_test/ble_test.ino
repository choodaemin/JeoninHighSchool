#include <ArduinoBLE.h>

BLEService myService("12345678-1234-1234-1234-1234567890ab");

BLECharacteristic myChar(
  "abcdefab-1234-5678-1234-abcdefabcdef",
  BLERead | BLEWrite | BLENotify,
  100
);

String inputBuffer = "";

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패");
    while (1);
  }

  BLE.setLocalName("Nano33BLE");
  BLE.setAdvertisedService(myService);

  myService.addCharacteristic(myChar);
  BLE.addService(myService);

  myChar.writeValue("READY\n");
  BLE.advertise();

  Serial.println("BLE 준비 완료");
  Serial.println("엔터를 누르면 PC로 전송됩니다");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("PC 연결됨");

    while (central.connected()) {

      /* 🔹 PC → Arduino */
      if (myChar.written()) {
        const uint8_t* data = myChar.value();
        int len = myChar.valueLength();

        String msg = "";
        for (int i = 0; i < len; i++) {
          msg += (char)data[i];
        }

        msg.trim();
        Serial.print("PC → Arduino: ");
        Serial.println(msg);
      }

      /* 🔹 Arduino → PC (엔터 기준) */
      while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
          inputBuffer.trim();

          if (inputBuffer.length() > 0) {
            myChar.writeValue(inputBuffer.c_str());
            Serial.print("Arduino → PC: ");
            Serial.println(inputBuffer);

            inputBuffer = "";
            delay(50);
          }
        } else {
          inputBuffer += c;
        }
      }
    }

    Serial.println("PC 연결 해제");
  }
} //아두이노

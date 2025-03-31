#define ENA 5   // L298N Enable A
#define IN1 6   // L298N Motor A IN1
#define IN2 7   // L298N Motor A IN2
#define ENB 9   // L298N Enable B
#define IN3 10  // L298N Motor B IN3
#define IN4 11  // L298N Motor B IN4

uint8_t receivedValues[12]; // 使用 uint8_t 確保數據正確

void setup() {
    Serial.begin(9600);    // 用於 Arduino 與電腦的通訊
    Serial1.begin(9600);   // 用於 Arduino 與 CC2541 BLE 模組的通訊
    Serial.println("BLE Module Initialized");

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    stopMotors();
}

void loop() {
    if (Serial1.available() >= 12) {
        int i = 0;
        while (i < 12) {
            if (Serial1.available()) {
                receivedValues[i] = Serial1.read();
                i++;
            }
        }
        Serial.print("Received: ");
        for (int i = 0; i < 12; i++) {
            Serial.print(receivedValues[i]);
            Serial.print(" ");
        }
        Serial.println();
        controlMotors(receivedValues);
    }
    delay(200);
}

void controlMotors(uint8_t values[]) {
    digitalWrite(IN1, values[0] > 0 ? HIGH : LOW);
    digitalWrite(IN2, values[1] > 0 ? HIGH : LOW);
    digitalWrite(IN3, values[2] > 0 ? HIGH : LOW);
    digitalWrite(IN4, values[3] > 0 ? HIGH : LOW);
    analogWrite(ENA, values[4]);
    analogWrite(ENB, values[5]);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

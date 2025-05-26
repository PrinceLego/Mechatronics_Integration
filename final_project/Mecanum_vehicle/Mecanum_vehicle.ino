#define EN1 5   // L298N Enable A
#define IN1 13   // L298N Motor A IN1
#define IN2 12   // L298N Motor A IN2
#define EN2 4   // L298N Enable B
#define IN3 11  // L298N Motor B IN3
#define IN4 10  // L298N Motor B IN4
#define EN3 3   // L298N Enable A
#define IN5 9   // L298N Motor A IN1
#define IN6 8   // L298N Motor A IN2
#define EN4 2   // L298N Enable B
#define IN7 7  // L298N Motor B IN3
#define IN8 6  // L298N Motor B IN4

uint8_t receivedValues[12]; // 使用 uint8_t 確保數據正確

void setup() {
    Serial.begin(9600);    // 用於 Arduino 與電腦的通訊
    Serial1.begin(9600);   // 用於 Arduino 與 CC2541 BLE 模組的通訊
    Serial.println("BLE Module Initialized");

    pinMode(EN1, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN3, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(EN4, OUTPUT);
    pinMode(IN7, OUTPUT);
    pinMode(IN8, OUTPUT);
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
    analogWrite(EN1, values[2]);
    digitalWrite(IN3, values[3] > 0 ? HIGH : LOW);
    digitalWrite(IN4, values[4] > 0 ? HIGH : LOW);
    analogWrite(EN2, values[5]);
    digitalWrite(IN5, values[6] > 0 ? HIGH : LOW);
    digitalWrite(IN6, values[7] > 0 ? HIGH : LOW);
    analogWrite(EN3, values[8]);
    digitalWrite(IN7, values[9] > 0 ? HIGH : LOW);
    digitalWrite(IN8, values[10] > 0 ? HIGH : LOW);
    analogWrite(EN4, values[11]);
}

void stopMotors() {
    uint8_t stopValues[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    controlMotors(stopValues);
}

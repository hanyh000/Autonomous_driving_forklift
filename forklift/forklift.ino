#include <Servo.h>

Servo myServo;
const int servoPin = 11;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(0);  // 초기 상태: 0도
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    // '8' 신호가 오면 180도로 이동
    if (input == '8') {
      myServo.write(180);
      delay(800);          // 서보가 움직일 물리적 시간 (0.8초) 대기
      Serial.println("done"); // 이동 완료 후 신호 전송
    } 
    // '9' 신호가 오면 0도로 이동
    else if (input == '9') {
      myServo.write(0);
      delay(800);          // 대기
      Serial.println("done"); // 이동 완료 후 신호 전송
    }
  }
}
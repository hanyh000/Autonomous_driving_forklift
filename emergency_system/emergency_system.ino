int Button = 3;
int buzzer = 6;
int R = 8; 
int G = 9;
int B = 10;
int sound = A0;

int systemState = 0; 
int lastSystemState = 0;
bool emergencyLocked = false;

unsigned long lastUpdate = 0;
unsigned long lastButtonPress = 0;  // 디바운스용
bool toggle = false;

void setup() {
  Serial.begin(9600);
  pinMode(Button, INPUT);
  pinMode(sound, INPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  int soundValue = analogRead(sound);
  bool physicalButtonPressed = digitalRead(Button);
  unsigned long now = millis();

  // 버튼 디바운스 처리
  if (physicalButtonPressed == HIGH && (now - lastButtonPress > 300)) {
    lastButtonPress = now;
    if (emergencyLocked) {
      // 비상 상태에서 버튼 → 해제
      emergencyLocked = false;
      systemState = lastSystemState;
      noTone(buzzer);
      Serial.println("R");
    } else {
      // 일반 상태에서 버튼 → 비상 발동
      Serial.println("E");
      activateEmergency();
    }
  }

  // 사운드 센서 → 비상 발동만 (해제 없음)
  if (soundValue > 75 && !emergencyLocked) {
    Serial.println("E");
    activateEmergency();
  }

  // 시리얼 명령 처리
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'E' || cmd == 'e') {
      activateEmergency();
    }
    else if (cmd == 'R' || cmd == 'r') {
      emergencyLocked = false;
      systemState = lastSystemState;
      noTone(buzzer);
    }
    else if (cmd == 'F' || cmd == 'f') {
      emergencyLocked = false;
      systemState = 0;
      noTone(buzzer);
    }
    else if (!emergencyLocked) {
      if (cmd == 'D' || cmd == 'd') { systemState = 1; lastSystemState = 1; }
      if (cmd == 'W' || cmd == 'w') { systemState = 2; lastSystemState = 2; }
    }
  }

  handleSystemOutput();
}

void activateEmergency() {
  emergencyLocked = true;
  systemState = 3;
}

void handleSystemOutput() {
  unsigned long currentMillis = millis();
  switch (systemState) {
    case 0:
      digitalWrite(R, LOW); digitalWrite(G, LOW); digitalWrite(B, LOW);
      noTone(buzzer);
      break;
    case 1:
      digitalWrite(R, LOW); digitalWrite(G, HIGH); digitalWrite(B, LOW);
      noTone(buzzer);
      break;
    case 2:
      digitalWrite(R, LOW); digitalWrite(G, LOW); digitalWrite(B, HIGH);
      if (currentMillis - lastUpdate >= 500) {
        lastUpdate = currentMillis;
        toggle = !toggle;
        if (toggle) tone(buzzer, 1000); else noTone(buzzer);
      }
      break;
    case 3:
      digitalWrite(R, HIGH); digitalWrite(G, LOW); digitalWrite(B, LOW);
      if (currentMillis - lastUpdate >= 200) {
        lastUpdate = currentMillis;
        toggle = !toggle;
        if (toggle) tone(buzzer, 1500); else tone(buzzer, 800);
      }
      break;
  }
}


// //센서값 확인용
// int Button = 3;
// int buzzer = 6;
// int R = 8; 
// int G = 9;
// int B = 10;
// int sound = A0;

// int systemState = 0; 
// int lastSystemState = 0;
// bool emergencyLocked = false; 

// unsigned long lastUpdate = 0;
// unsigned long lastSerialPrint = 0;
// bool toggle = false;
// int peakSound = 0; // 피크값 추적용

// void setup() {
//   Serial.begin(9600);
//   pinMode(Button, INPUT);
//   pinMode(sound, INPUT);

//   pinMode(R, OUTPUT);
//   pinMode(G, OUTPUT);
//   pinMode(B, OUTPUT);

//   pinMode(buzzer, OUTPUT);
// }

// void loop() {
//   // 1. 센서 및 물리 버튼 감지
//   int soundValue = analogRead(sound);
//   bool physicalButtonPressed = digitalRead(Button);

//   //  피크값 갱신
//   if (soundValue > peakSound) peakSound = soundValue;

//   // 0.5초마다 피크값 포함 출력
//   if (millis() - lastSerialPrint >= 500) {
//     Serial.print("--- Sensor Data --- | Sound Current: ");
//     Serial.print(soundValue);
//     Serial.print(" | Sound Peak: ");
//     Serial.print(peakSound);
//     Serial.print(" | Button: ");
//     Serial.print(physicalButtonPressed ? "ON" : "OFF");
//     Serial.print(" | State: ");
//     Serial.println(systemState);
//     peakSound = 0; // 구간 리셋
//     lastSerialPrint = millis();
//   }

//   // 2. 비상 상황 발생 조건 체크 (소리 OR 버튼)
//   if (soundValue > 100 || physicalButtonPressed == HIGH) {
//     if (!emergencyLocked) {
//       Serial.println("E"); 
//       if (soundValue > 100) Serial.println("DEBUG: Sound Emergency Triggered!");
//       if (physicalButtonPressed == HIGH) Serial.println("DEBUG: Physical Button Triggered!");
//       activateEmergency();
//     }
//   }

//   // 3. 웹/시리얼 신호 처리
//   if (Serial.available() > 0) {
//     char cmd = Serial.read();
    
//     if (cmd == 'E' || cmd == 'e') {
//       activateEmergency();
//     }
//     else if (cmd == 'R' || cmd == 'r') {
//       emergencyLocked = false;
//       systemState = lastSystemState; 
//       noTone(buzzer);
//       Serial.println("DEBUG: Emergency Released");
//     }
//     else if (cmd == 'F' || cmd == 'f') {
//       emergencyLocked = false;
//       systemState = 0;
//       noTone(buzzer);
//       Serial.println("DEBUG: System Reset/Finish");
//     }
//     else if (!emergencyLocked) {
//       if (cmd == 'D' || cmd == 'd') { 
//         systemState = 1; lastSystemState = 1; 
//         Serial.println("Mode: Drive"); 
//       }
//       if (cmd == 'W' || cmd == 'w') { 
//         systemState = 2; lastSystemState = 2; 
//         Serial.println("Mode: Work"); 
//       }
//     }
//   }

//   // 4. 출력 실행
//   handleSystemOutput();
// }

// void activateEmergency() {
//   emergencyLocked = true;
//   systemState = 3;
// }

// void handleSystemOutput() {
//   unsigned long currentMillis = millis();
//   switch (systemState) {
//     case 0:
//       digitalWrite(R, LOW); digitalWrite(G, LOW); digitalWrite(B, LOW);
//       noTone(buzzer);
//       break;
//     case 1: 
//       digitalWrite(R, LOW); digitalWrite(G, HIGH); digitalWrite(B, LOW);
//       noTone(buzzer);
//       break;
//     case 2: 
//       digitalWrite(R, LOW); digitalWrite(G, LOW); digitalWrite(B, HIGH);
//       if (currentMillis - lastUpdate >= 500) { 
//         lastUpdate = currentMillis;
//         toggle = !toggle;
//         if (toggle) tone(buzzer, 1000); else noTone(buzzer);
//       }
//       break;
//     case 3: 
//       digitalWrite(R, HIGH); digitalWrite(G, LOW); digitalWrite(B, LOW);
//       if (currentMillis - lastUpdate >= 200) { 
//         lastUpdate = currentMillis;
//         toggle = !toggle;
//         if (toggle) tone(buzzer, 1500); else tone(buzzer, 800);
//       }
//       break;
//   }
// }
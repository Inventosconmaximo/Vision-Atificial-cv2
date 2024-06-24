#include <Servo.h>

Servo servoPan;   // Servo para el movimiento horizontal (pan)
Servo servoTilt;  // Servo para el movimiento vertical (tilt)

void setup() {
  Serial.begin(115200);  // Aumentar la velocidad de comunicación
  servoPan.attach(9);  // Pin donde está conectado el servo pan
  servoTilt.attach(10); // Pin donde está conectado el servo tilt
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("PAN")) {
      int angle = command.substring(3).toInt();
      servoPan.write(angle);
    } else if (command.startsWith("TILT")) {
      int angle = command.substring(4).toInt();
      servoTilt.write(angle);
    }
  }
}


#include <Servo.h>

Servo servo1;

void setup() {
servo1.attach(11);
}

void loop() {

servo1.write(0);
delay(3000);
servo1.write(180);
delay(3000);
}

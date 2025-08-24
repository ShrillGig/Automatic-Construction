#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define R_SENSE 0.11f

#define EN_PIN 8
#define RX_PIN 10 
#define TX_PIN 11

#define CLOSE 0
#define OPEN 90


SoftwareSerial TMCss(RX_PIN, TX_PIN); //UART connection
TMC2208Stepper driver(&TMCss, R_SENSE); 

Servo claw, lift;

int STEP_PIN_X = 2; int DIR_PIN_X = 5;
int STEP_PIN_Y = 3; int DIR_PIN_Y = 6;

float step_angle = 1.8f;
float steps_per_rev = 360 / step_angle;
int teeth = 20;
int tooth_pitch = 2;
int microsteps = 8;
int steps_per_mm = (steps_per_rev * microsteps) / (tooth_pitch * teeth);
int steps_per_cm = steps_per_mm * 10;


bool start = false;
bool task = false;
int value = 0;
int digit = 0;
int i = 0;
int coords[10][2];


void setup() {

Serial.begin(9600);
TMCss.begin(115200);

claw.attach(11);
lift.attach(10);

driver.begin();
driver.rms_current(1200);
driver.toff(5);  
driver.pdn_disable(true);      
driver.mstep_reg_select(true);
driver.microsteps(microsteps);
driver.pwm_autoscale(true); 
//Serial.print("conn="); Serial.println(driver.test_connection()==0 ? "OK":"FAIL");
//Serial.print("msteps="); Serial.println(driver.microsteps());  

pinMode(EN_PIN, OUTPUT);
pinMode(STEP_PIN_X, OUTPUT);
pinMode(DIR_PIN_X, OUTPUT);
pinMode(STEP_PIN_Y, OUTPUT);
pinMode(DIR_PIN_Y, OUTPUT);
digitalWrite(EN_PIN, LOW);
//digitalWrite(EN_PIN, HIGH);

lift.write(CLOSE);
delay(1000);
claw.write(OPEN);
delay(1000);


}

void Step(uint16_t n, int step_axis, int dir_axis, bool move) {

if (move == true) 
  digitalWrite(dir_axis, LOW);
if (move == false)
  digitalWrite(dir_axis, HIGH);

for (uint16_t i = 0; i < n; i++) {
    digitalWrite(step_axis, HIGH);
    delayMicroseconds(160);
    digitalWrite(step_axis, LOW);
    delayMicroseconds(160);
  }
  delay(1000);

}

void loop() {

if (task == false) {
if (Serial.available()) {
char size = Serial.read(); 
if (!(size == '\n' || size == '\r'))
{
digit = size - '0';
Serial.print(F("Digit: "));
Serial.println(digit); 
task = true;
}
}
}

if (task == true) {
if (Serial.available()) {
char symbol = Serial.read();
if (!(symbol == '\n' || symbol == '\r'))
{
if (symbol == '#' || symbol == '$') 
value = Serial.parseInt();
Serial.readStringUntil(';');

if (symbol == '#') {
  coords[i][0] = value;
  Serial.print(F("X Value: "));
  Serial.println(coords[i][0]);
}
else if (symbol == '$') {
  coords[i][1] = value;
  Serial.print(F("Y Value: "));
  Serial.println(coords[i][1]);
  i++;
}

else if (symbol == 'p') {
Serial.println("Ok");
start = true;
}
}
}
}


if (start == true) {

delay(3000);


Serial.println("READY TO START");
int check = steps_per_cm * 5;
claw.write(CLOSE);
delay(1000);
lift.write(OPEN);
delay(1000);
Step(check, STEP_PIN_X, DIR_PIN_X, true);
Step(check, STEP_PIN_Y, DIR_PIN_Y, true);

lift.write(CLOSE);
delay(1000);
claw.write(OPEN);
delay(1000);
Step(check, STEP_PIN_X, DIR_PIN_X, false);
Step(check, STEP_PIN_Y, DIR_PIN_Y, false);

for (int i = 0; i < digit; i++) {

int x_axis = steps_per_cm * coords[i][0];
int y_axis = steps_per_cm * coords[i][1];
int bias_x = steps_per_cm * 8;

lift.write(CLOSE);
delay(1000);
claw.write(CLOSE);
delay(1000);
lift.write(OPEN);
delay(1000);
Step(bias_x, STEP_PIN_X, DIR_PIN_X, true);
Step(x_axis, STEP_PIN_X, DIR_PIN_X, true);
Step(y_axis, STEP_PIN_Y, DIR_PIN_Y, true);

lift.write(CLOSE);
delay(1000);
claw.write(OPEN);
delay(1000);
lift.write(OPEN);
delay(1000);
Step(y_axis, STEP_PIN_Y, DIR_PIN_Y, false);
Step(bias_x, STEP_PIN_X, DIR_PIN_X, false);
Step(x_axis, STEP_PIN_X, DIR_PIN_X, false);

}
start = false;
}
}

#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <Servo.h>       

const int MAX_ROWS = 50;
float world_coord[MAX_ROWS][2]; 
int currentRow = 0;
bool printed = false;

// ——— Пины для шаговиков ———
const int stepPinX = 2, dirPinX = 5;
const int stepPinY = 3, dirPinY = 6;

#define RX_PIN 10  // Arduino → TMC2208 PDN_UART
#define TX_PIN 11  // Arduino ← TMC2208 PDN_UART
#define R_SENSE 0.11f

SoftwareSerial TMCss(RX_PIN, TX_PIN);
TMC2208Stepper driver(&TMCss, R_SENSE);


// ————— Серво (клешня) —————
Servo claw; 
const int CLAW_CLOSED_ANGLE = 0;
const int CLAW_OPEN_ANGLE   = 180;

// ——— Механика ———
const int   microsteps       = 1;       // единичный шаг (чем больше, тем медленне и точнее)
const float stepAngle        = 1.8;     // градусы полного шага (200 шагов)
const float leadScrewPitchMm = 5.0;     // мм за оборот винта/ремня
const float stepsPerRev      = 360.0 / stepAngle * microsteps;  // 200
const float stepsPerMm       = stepsPerRev / leadScrewPitchMm;  // 200 / 5 = 40 шагов/мм
const float stepsPerCm       = stepsPerMm * 10.0;               // 400 шагов/см

const int stepDelayMicro = 300; // задержка шагового мотора 
const int Bias = 8; //значение для смещения 
const int check = 10; //значение для проверки 

void setup() {
  Serial.begin(9600);
  Serial.println("Receiver ready");

  claw.attach(11);

  TMCss.begin(9600);
  driver.begin();
  driver.microsteps(microsteps);
  driver.rms_current(1200);  // максимум 1.5А для драйвера, но лучше 80% 

  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX,  OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY,  OUTPUT);

  int checkX = int(check * stepsPerCm + 0.5f); //сдвиг на 5 см по Х для проверки мотора 
  int checkY = int(check * stepsPerCm + 0.5f); //сдвиг на 5 см по У для проверки мотора 
  

  //Проверка захвата 
  delay(3000);
  claw.write(CLAW_CLOSED_ANGLE);
  delay(3000);
  claw.write(CLAW_OPEN_ANGLE);
  delay(200);

  //Проверка моторов 
  moveAxis(stepPinX, dirPinX, checkX, true);
  moveAxis(stepPinY, dirPinY, checkY, false);

  delay(1000);

  moveAxis(stepPinX, dirPinX, checkX, false);
  moveAxis(stepPinY, dirPinY, checkY, true);

  delay(3000);

}

void loop() {

  // 1) Приём команд a<float> и b<float>
  if (Serial.available() > 0 && !printed) {
    char key = Serial.read();
    if (key == 'a' || key == 'b') {
      float val = Serial.parseFloat();
      Serial.readStringUntil('\n');  // сбросить остаток строки

      int col = (key == 'a') ? 0 : 1;
      if (currentRow < MAX_ROWS) {
        world_coord[currentRow][col] = val;
        // Если пришла метка 'b' — завершена пара (x,y)
        if (key == 'b') {
          currentRow++;
        }
        Serial.println("OK");
      }
      return;  // сразу выходим, чтобы не одновременно обрабатывать 'p'
    }

    // если это не 'a' и не 'b', проверим, нет ли 'p'
    if (key == 'p') {
      // Отбросим любой '\r' или '\n' после 'p'
      if (Serial.peek() == '\r' || Serial.peek() == '\n') {
        Serial.read();
        if (Serial.peek() == '\n') {
          Serial.read();
        }
      }
      // Пришёл запрос печати и оправляем ответ ПК 
      Serial.println("GOT p");
      printAllPoints();
      printed = true;       // отметим, что печатали и двигаемся один раз

      //Функция запуска движения моторов по координатам 
      executeMovement();

      // Бесконечная пауза — Arduino ничего больше не делает
      while (true) {
        ; 
      }
    }

    // Если сюда попало что-то другое (нишевые символы), сбросим строку до \n
    Serial.readStringUntil('\n');
  }
}

// Печать принятых точек
void printAllPoints() {
  Serial.println("All points received:");
  for (int i = 0; i < currentRow; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(world_coord[i][0], 2); // x
    Serial.print(", ");
    Serial.println(world_coord[i][1], 2); // y
  }
}

void executeMovement() {

  for (int j = 0; j < currentRow; j++) {
    // Конвертация в целое число шагов
    int stepsX = int(world_coord[j][0] * stepsPerCm + 0.5);  // x → X-ось
    int stepsY = int(world_coord[j][1] * stepsPerCm + 0.5);  // y → Y-ось
    int stepsBiasX = int(Bias * stepsPerCm + 0.5f);

    claw.write(CLAW_CLOSED_ANGLE);
    delay(500);

    moveAxis(stepPinX, dirPinX, stepsBiasX, true); //смещение по оси Х, чтобы не задевать края 
    delay(3000);

    // ——— 1) Движение вперёд: X и Y — вперед
    moveAxis(stepPinX, dirPinX, stepsX, true);
    moveAxis(stepPinY, dirPinY, stepsY, false);

    // Подождать, чтобы убедиться, что моторы встали
    delay(200);

    // ——— 2) Открываем клешню
    claw.write(CLAW_OPEN_ANGLE);
    delay(500);  // даём серво полсекунды «провалиться» в открытое положение

    // ——— 3) Возвращаемся назад (X и Y — назад)
    moveAxis(stepPinX, dirPinX, stepsX, false);
    moveAxis(stepPinY, dirPinY, stepsY, true);

    moveAxis(stepPinX, dirPinX, stepsBiasX, false);
    delay(3000);
  }
}

// Универсальная функция: “подать X шагов на шаговый мотор”
void moveAxis(int stepPin, int dirPin, int steps, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelayMicro);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelayMicro);
  }
}

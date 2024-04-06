#include <Wire.h>
#include <VL53L0X.h>
#include <MovingAverage.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

const TickType_t motorControlDelay = 10 / portTICK_PERIOD_MS;
const TickType_t sensorControlDelay = 10 / portTICK_PERIOD_MS;




const int pin_LeftWheel1 = 17;
const int pin_LeftWheel2 = 5;

const int pin_RightWheel1 = 13;
const int pin_RightWheel2 = 15;

const int pin_rotationMotor1 = 16;
const int pin_rotationMotor2 = 4;

const int pin_sensorEnableFrontLeft = 14;
const int pin_sensorEnableFrontRight = 27;
const int pin_sensorEnableBack = 26;


const float degreePerMilliSecond = 8;
const float mmPerMilliSecond = 3.9;


// 0 = none, 1 = front wall/robot/rugby ball, 2 = back wall/robot/rugby ball, 3 = too close, 5 = found small ball
int foundObject = 0;
int dir = 0;
int ang = 40;


// The number of sensors in your system.
const uint8_t sensorCount = 3;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { 14, 27, 25 };  //,27,14

VL53L0X sensors[sensorCount];

MovingAverage<int, 1> filter[sensorCount];

int filteredReading[sensorCount] = { 0, 0, 0 };


void initWheels() {
  pinMode(pin_LeftWheel1, OUTPUT);
  pinMode(pin_LeftWheel2, OUTPUT);

  pinMode(pin_RightWheel1, OUTPUT);
  pinMode(pin_RightWheel2, OUTPUT);

  digitalWrite(pin_LeftWheel1, HIGH);
  digitalWrite(pin_LeftWheel2, HIGH);

  digitalWrite(pin_RightWheel1, HIGH);
  digitalWrite(pin_RightWheel2, HIGH);
}

void initRotor(int speed) {
  //ledcSetup(pin_rotationMotor1, freq, resolution);
  //ledcAttachPin(pin_rotationMotor1,  rotorPWMChannel);

  pinMode(pin_rotationMotor1, OUTPUT);
  pinMode(pin_rotationMotor2, OUTPUT);
  digitalWrite(pin_rotationMotor2, HIGH);
  int pwmOutput = map(speed, 0, 100, 0, 255);
  analogWrite(pin_rotationMotor1, pwmOutput);
}

void initSensors() {
  Wire.begin(12, 26);
  Wire.setClock(400000);  // use 400 kHz I2C
  delay(100);

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(100);
  /// Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++) {

    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(500);

    //sensors[i].setTimeout(500);
    bool sensorInit = 0;
    while (sensorInit == 0) {
      if (!sensors[i].init()) {
        Serial.print("Failed to detect and initialize sensor ");
        Serial.println(i);
        delay(200);

      } else {
        Serial.print("Initialized sensor ");
        Serial.println(i);
        sensorInit = 1;
        delay(200);
      }
    }
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);
    Serial.println("Address SEt");
    sensors[i].startContinuous(50);
  }
}

bool turn(int angle, int direction) {
  if (direction == 0) {
    digitalWrite(pin_LeftWheel1, HIGH);
    digitalWrite(pin_LeftWheel2, LOW);

    digitalWrite(pin_RightWheel1, LOW);
    digitalWrite(pin_RightWheel2, HIGH);
  } else if (direction == 1) {
    digitalWrite(pin_LeftWheel1, LOW);
    digitalWrite(pin_LeftWheel2, HIGH);

    digitalWrite(pin_RightWheel1, HIGH);
    digitalWrite(pin_RightWheel2, LOW);
  }

  delay(angle * degreePerMilliSecond);

  digitalWrite(pin_LeftWheel1, HIGH);
  digitalWrite(pin_LeftWheel2, HIGH);

  digitalWrite(pin_RightWheel1, HIGH);
  digitalWrite(pin_RightWheel2, HIGH);

  return 1;
}

void moveForward() {

  digitalWrite(pin_LeftWheel1, LOW);
  digitalWrite(pin_LeftWheel2, HIGH);

  digitalWrite(pin_RightWheel1, LOW);
  digitalWrite(pin_RightWheel2, HIGH);
}

void moveBackwards(int distance) {
  digitalWrite(pin_LeftWheel1, HIGH);
  digitalWrite(pin_LeftWheel2, LOW);

  digitalWrite(pin_RightWheel1, HIGH);
  digitalWrite(pin_RightWheel2, LOW);

  delay(distance * mmPerMilliSecond);

  digitalWrite(pin_LeftWheel1, HIGH);
  digitalWrite(pin_LeftWheel2, HIGH);

  digitalWrite(pin_RightWheel1, HIGH);
  digitalWrite(pin_RightWheel2, HIGH);
}

void turnRotor(int speed) {
  //speed can be 0-100
  int pwmOutput = map(speed, 0, 100, 0, 255);
  analogWrite(pin_rotationMotor1, pwmOutput);
}

void getSensors() {

  for (uint8_t i = 0; i < sensorCount; i++) {

    int sen = sensors[i].readRangeContinuousMillimeters();
    //if (sen != 0){
  filteredReading[i] = filter[i].add(sen);
    //}

    //Serial.print(filteredReading[i]);

    //Serial.print(',');
  }
 Serial.println(' ');
  if (filteredReading[0] < 250 or filteredReading[1] < 250){
    foundObject = 1;
    ang = random(45,100);
    dir = random(0,2);
  }
  /*
  else if (filteredReading[0] < 300){
    foundObject = 1;
    dir = 0;
    ang = random(90,180);
  } 
  else if (filteredReading[1] < 300)
  {
    foundObject = 1;
    dir = 1;
    ang = random(90,180);;
   */ 
  else {
    foundObject = 0;
  }
  //Serial.println(foundObject);

}
void fullStop() {
  digitalWrite(pin_LeftWheel1, HIGH);
  digitalWrite(pin_LeftWheel2, HIGH);

  digitalWrite(pin_RightWheel1, HIGH);
  digitalWrite(pin_RightWheel2, HIGH);
}




void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(motorControlTask, "Task1", 5000, NULL, 2, &Task1, 1);
  delay(500);

  xTaskCreatePinnedToCore(sensorControlTask, "Task2", 5000, NULL, 1, &Task2, 0);
  delay(500);
}

//  This task controls motors.

void motorControlTask(void* parameter) {
  bool taskFree = 1;
  Serial.print("\motorControlTask is running on core ");
  Serial.println(xPortGetCoreID());

  initRotor(30);
  initWheels();
  turn(45, 1);

  for (;;) {
    //Serial.println(foundObject);
    if (foundObject != 1 and taskFree) {
      moveForward();
    } else {
      taskFree = 0;
      taskFree = turn(ang, dir);
    }
    vTaskDelay(motorControlDelay);
  }
}


//  This task contols sensors.
void sensorControlTask(void* parameter) {
  Serial.print("\sensorControlTask is running on core ");
  Serial.println(xPortGetCoreID());

  initSensors();

  for (;;) {
    getSensors();
    vTaskDelay(sensorControlDelay);
  }
}



void loop(){};

 
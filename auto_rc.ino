#include <QTRSensors.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define DEBUG                   1  // set to 1 to print sensor data to serial output
#define NUM_SENSORS             4  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is default none connected

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// assign the motors for steering and driving forward/backward
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *steer = AFMS.getMotor(3);
Adafruit_DCMotor *drive = AFMS.getMotor(4);

void setup() {
  
  delay(500);
  calibrateSensors();
  
  AFMS.begin();
  steer->setSpeed(150);
  steer->run(RELEASE);
  
  drive->setSpeed(150);
  drive->run(RELEASE);
  
  delay(1000);
  
}


void loop() {
  
  // read calibrated sensor values and obtain a measure of the line position from 0 to 3000
  unsigned int position = qtra.readLine(sensorValues);
  
  // if DEBUG == true, print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance
  // and 1000 means minimum reflectance, followed by the line position
  if (DEBUG) {
    for (unsigned char i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    }
    Serial.println(position);
  }
  
  driveForward();
  
  if (position > 2000) {
    steerLeft();
  } else if (position < 1000) {
    steerRight();
  } else {
    steerStraight();
  }
  
}

void steerLeft() {
  steer->run(FORWARD);
}

void steerRight() {
  steer->run(BACKWARD);
}

void steerStraight() {
  steer->run(RELEASE);
}

void driveForward() {
  drive->run(FORWARD);
}

void driveBackward() {
  drive->run(BACKWARD);
}

void driveStop() {
  drive->run(RELEASE);
}

// set the thresholds for the infrared sensors
void calibrateSensors() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++) {
    qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);
  
  if (DEBUG) { // if DEBUG == true, print the sensor data via serial output
    Serial.begin(9600);
    // print the calibration minimum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
  
    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}


#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <NewPing.h>

#define leftMotorF 4
#define leftMotorR 5
#define rightMotorF 6
#define rightMotorR 7
#define rightMotorF2 8
#define rightMotorR2 9
#define leftMotorF2 10
#define leftMotorR2 11
#define TRIGGER_PIN_0 A2
#define ECHO_PIN_0 A2

int RXPin = 0;
int TXPin = 1;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
double curLat = gps.location.lat();
double curLong = gps.location.lng();
// double destLat = 28.601323598201336; //Destination Latitude
// double destLong = -81.19788219072872; //Destination Longitude
// double destLat = 28.6015202; //Destination Latitude
// double destLong = -81.1978852;
double destLat = 28.601053; //Destination Latitude
double destLong = -81.197694;

int counter=0;

QMC5883LCompass compass;
NewPing sensor1(TRIGGER_PIN_0, ECHO_PIN_0, 400);

void setup() 
{
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorR, OUTPUT);
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorR, OUTPUT);
  pinMode(rightMotorR2, OUTPUT);
  pinMode(rightMotorF2, OUTPUT);
  pinMode(leftMotorF2, OUTPUT);
  pinMode(leftMotorR2, OUTPUT);

  //Serial monitor for testing
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(2000);

  compass.init();
  compass.setMagneticDeclination(-6, 54);

  turnRight();
  Serial.println("Caliration beginning");
  compass.calibrate();
  Serial.println("Calibration done");
  stopMotors();
  Serial.println("Stop Motors");

  compass.setCalibrationOffsets(compass.getCalibrationOffset(0), compass.getCalibrationOffset(1), compass.getCalibrationOffset(2));
  compass.setCalibrationScales(compass.getCalibrationScale(0), compass.getCalibrationScale(1), compass.getCalibrationScale(2));
}


void loop() 
{
    if(gps.encode(gpsSerial.read())) 
    {
      compass.read();
      
      int startDir = compassCalc(compass.getAzimuth(), 10, 'n');
      int goalDir = gps.courseTo(gps.location.lat(), gps.location.lng(), destLat, destLong);

      if(goalDir > 180)
        goalDir = goalDir - 360;

      if(startDir != goalDir) 
      {
        int diff = goalDir - startDir;
        if(diff < 0)
          diff = diff + 360;
        stopMotors();
        compass.read();
        int a = compassCalc(compass.getAzimuth(), 20, 'n');
        int between[2];
        between[0] = compassCalc(a, (diff - 15), 'p');
        between[1] = compassCalc(a, (diff + 15), 'p');
        while(!(a > between[0] && a < between[1])) 
        {
          if(counter == 5)
          {
            turnRight();
            Serial.println("Turn Right");
            counter = 0;
          }
          compass.read();
          a = compassCalc(compass.getAzimuth(), 20, 'n');
          Serial.print("Current direction: ");
          Serial.println(a);
          Serial.println("\n");
          Serial.print("Between[0]: ");
          Serial.println(between[0]);
          Serial.println("\n");
          Serial.print("Between[1]: ");
          Serial.println(between[1]);
          Serial.print("Goal Dir: ");
          Serial.println(goalDir);
          Serial.println("\n");
          Serial.print("Diff: ");
          Serial.println(diff);
          Serial.println("\n");
          // delay(500);

          if((a > between[0] && a < between[1])) {
            Serial.println("Stop Motors");
            stopMotors();
            delay(1000);
          }
          counter++;
        }
      }
      if(gps.distanceBetween(curLat, curLong, destLat, destLong) > 2) 
      {
        // Serial.print("\nCurrent Latitude: ");
        curLat = gps.location.lat();
        Serial.println(curLat, 8);
        // Serial.print("Goal Latitude: ");
        // Serial.println(destLat, 8);

        // Serial.print("\nCurrent Longitude: ");
        curLong = gps.location.lng();
        Serial.println(curLong, 10);
        // Serial.print("Goal Longitude: ");
        // Serial.println(destLong, 10);

        // Serial.println();
        // Serial.print("Distance Between: ");
        Serial.println(gps.distanceBetween(curLat, curLong, destLat, destLong));

        moveForward();
        int distance1 = sensor1.ping_cm();
        if(distance1 < 30 && distance1 > 0) {
          avoid();
        }
        // delay(2000);
      }
    }
  // }
  if(gps.distanceBetween(curLat, curLong, destLat, destLong) < 2) 
  {
    Serial.println("Stop Motors");
    stopMotors();
  }

}

//Stop both left and right motors
void stopMotors() 
{
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorR, LOW);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorR, LOW);
  digitalWrite(rightMotorR2, LOW);
  digitalWrite(rightMotorF2, LOW);
  digitalWrite(leftMotorF2, LOW);
  digitalWrite(leftMotorR2, LOW);
}

//Move both left and right motors forward
void moveForward() 
{
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorR, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorR, LOW);
  digitalWrite(leftMotorF2, HIGH);
  digitalWrite(leftMotorR2, LOW);
  digitalWrite(rightMotorF2, HIGH);
  digitalWrite(rightMotorR2, LOW);
}

void turnRight() 
{
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorR, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorR, LOW);
  digitalWrite(leftMotorF2, LOW);
  digitalWrite(leftMotorR2, HIGH);
  digitalWrite(rightMotorF2, LOW);
  digitalWrite(rightMotorR2, HIGH);
}

void turnLeft() 
{
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorR, HIGH);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorR, HIGH);
  digitalWrite(leftMotorF2, HIGH);
  digitalWrite(leftMotorR2, LOW);
  digitalWrite(rightMotorF2, HIGH);
  digitalWrite(rightMotorR2, LOW);
}

int compassCalc(int degrees, int offset, char offsetDirection) 
{
  int degrees1 = degrees;
  if(offsetDirection == 'p') 
  {
    degrees = degrees + offset;
    if(degrees > 360)
      degrees = degrees - 360;
  }
  if(offsetDirection == 'n') 
  {
    degrees = degrees - offset;
    if(degrees < 0)
      degrees = degrees + 360;
  }
  if(degrees < -180)
    degrees = degrees + 360;
  if(degrees > 180)
    degrees = degrees - 360;
  return degrees;
}

void avoid() {
  int distance1 = sensor1.ping_cm();
  if(distance1 > 30) {
    return;
  }
  else {
    if (distance1 < 30 && distance1 > 0) {
      int a = compassCalc(compass.getAzimuth(), 20, 'n');
      int startDir = compassCalc(a, 20, 'n');
      int goalDir = compassCalc(a, 90, 'p');

      if(startDir != goalDir) {
        // Serial.println("\nStop Motors");
        stopMotors();
        compass.read();
        int a = compassCalc(compass.getAzimuth(), 20, 'n');
        int between[2];
        between[0] = compassCalc(a, 85, 'p');
        between[1] = compassCalc(a, 95, 'p');
        while(!(a > between[0] && a < between[1])) {
          // Serial.print("\nCurrent direction: ");
          // Serial.println(a);
          // Serial.print("Goal direction: ");
          // Serial.println(goalDir);
          // Serial.println("Turn Right");
          turnRight();
          compass.read();
          a = compassCalc(compass.getAzimuth(), 20, 'n');
          if((a > between[0] && a < between[1])) {
            // Serial.println("\nDone Turning");
            // Serial.println("Stop Motors");
            stopMotors();
          }
        }
      }
      moveForward();
      delay(1000);
      compass.read();
      startDir = compassCalc(compass.getAzimuth(), 20, 'n');
      goalDir = gps.courseTo(gps.location.lat(), gps.location.lng(), destLat, destLong);
      if(startDir != goalDir) {
        int diff = startDir - goalDir;
        if(diff < 0)
          diff = diff + 360;
        // Serial.println("\nStop Motors");
        stopMotors();
        compass.read();
        int a = compassCalc(compass.getAzimuth(), 20, 'n');
        int between[2];
        between[0] = compassCalc(a, (diff - 15), 'n');
        between[1] = compassCalc(a, (diff + 15), 'n');
        while(!(a < between[0] && a > between[1])) {
          // Serial.print("\nCurrent direction: ");
          // Serial.println(a);
          // Serial.print("Goal direction: ");
          // Serial.println(goalDir);
          // Serial.println("Turn Right");
          turnLeft();
          compass.read();
          a = compassCalc(compass.getAzimuth(), 20, 'n');
          if((a > between[0] && a < between[1])) {
            // Serial.println("\nDone Turning");
            // Serial.println("Stop Motors");
            stopMotors();
          }
        }
      }
    }
    avoid();
  }
}
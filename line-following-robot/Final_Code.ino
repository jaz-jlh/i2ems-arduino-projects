//------------------------------------DECLARATIONS--------------------------------//
//Include Libraries
#include <Servo.h>


// Assign Pins
#define encoderPin 2
#define pwmA 3
#define refSensorR 4
#define refSensorL 6
#define pwmB 11
#define dirA 12
#define dirB 13

//Motor Control
#define motorSpeedFactor .15
#define motorSpeedA (255 * motorSpeedFactor)
#define motorSpeedB (255 * motorSpeedFactor)
#define motorDirectionA HIGH
#define motorDirectionB LOW

//Reflectance Sensors
#define DARK true
#define LIGHT false
boolean lastLRSReading = false;           //left relfectance sensor last reading
boolean lastRRSReading = false;           //right reflectance sensor last reading
#define rightSensor 1
#define leftSensor 2
#define hys 50
//Reflectance Calibration
int maxReadingR = 0;
int minReadingR = 10000;
int maxReadingL = 0;
int minReadingL = 10000;
int avgL = 0;
int avgR = 0;

//Serial Communications & Trip Data
String order = "";
int location = 0;               //A number 0-12 indicating which road the robot is on; destination numbers match their "driveways"
int destination = 0;            //A number 0-8, (1-5 are houses, 6-8 are warehouses)
int timeElapsed = 0;
unsigned long elapsedStart = 0;
unsigned long elapsedEnd = 0;
boolean delivered = false;
int warehouseDestination = 0;
int houseDestination = 0;
//Lewin's Variables - for handling serial communications
boolean servicingOrder = false;
#define UVA_ID "ak4ns"
String uva_id;
uint16_t order_id;

//Navigation
#define HEADQUARTERS 0
#define HOUSE_1 1
#define HOUSE_2 2
#define HOUSE_3 3
#define HOUSE_4 4
#define HOUSE_5 5
#define WAREHOUSE_A 6
#define WAREHOUSE_B 7
#define WAREHOUSE_C 8
#define ROAD_H 9
#define ROAD_345 10
#define ROAD_W 11
#define ROAD_BC 12

//Encoders
#define encoderA 2
volatile unsigned long encoder_counter=0;

Servo servo;                    //Dumping Servo
//------------------------------------DECLARATIONS--------------------------------//





//------------------------------------SETUP--------------------------------//
void setup() {
  Serial.begin(115200);

  //Establish Pin Modes
  pinMode(dirB, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(pwmA, OUTPUT);

  pinMode(7, OUTPUT);     //Servo Pin
  servo.attach(7);
  servo.write(180);
  //Starting Position
  location = 0;
  destination = WAREHOUSE_A;

  attachInterrupt(0,ISRA,CHANGE); 
  Serial.println("Calibrating...");
  calibrationPeriod();
}
//------------------------------------SETUP--------------------------------//






//------------------------------------LOOP--------------------------------//
void loop() {
  lineFollow();

  if (CheckSerialForNewOrder()) {
    CompleteOrder();
  }
  if (OrderComplete()) {
    SendCompletedOrder();
  }
}
//------------------------------------LOOP--------------------------------//





//------------------------------------SERIAL Reading/Writing Methods--------------------------------//
boolean OrderComplete(void) {
  elapsedEnd = millis();
  return delivered;
}

void SendCompletedOrder(void) {  
  int distanceTravelled = (int) odometer();
  int timeElapsed = (int) (elapsedEnd - elapsedStart) / 1000;

  Serial.println("Sending completed order:");
  Serial.print("<del>");
  Serial.print(uva_id);
  Serial.print(":");
  Serial.print(order_id);
  Serial.print(":");
  Serial.print(distanceTravelled); 
  Serial.print(":");
  Serial.print(timeElapsed);
  Serial.println("</del>");

  //Reset variables
  servicingOrder = false;
  delivered = false;
  distanceTravelled = 0;
  elapsedEnd = 0;
  elapsedStart = 0;

  Serial.println("Done with sending!");
}

void CompleteOrder(void) {
  servicingOrder = true;
  destination = warehouseDestination;
  elapsedStart = millis();
}

String currSerialLine;

boolean CheckSerialForNewOrder(void) {
  boolean retVal = false;
  //Serial.println("Checking for new orders.");

  if (Serial.available()) {
    // read incoming bytes:
    char inChar = Serial.read();
    //Serial.println(inChar);
    if (inChar == '+') {
      while(true) {
        fullStop();
      }
    }
    // add incoming byte to end of line:
    if(inChar != '\n' && inChar != '\r')
      currSerialLine += inChar;
    // if you get a newline, convert the string to integer setpoint:
    if (inChar == '\n') {
      if(currSerialLine.startsWith("<order>") && currSerialLine.endsWith("</order>")) {
        Serial.println("Looks like we have a new order!");

        if(servicingOrder) {
          Serial.println("Sorry. I'm busy.");
          currSerialLine = "";
          return false;
        }

        //parse
        int colon = currSerialLine.indexOf(":");
        uva_id = currSerialLine.substring(7, colon);
        if(uva_id != String(UVA_ID)) {
          Serial.println("But not for us...");
          Serial.println(uva_id);
          Serial.println(String(UVA_ID));

          currSerialLine = "";
          return false;
        }

        int lastColon = colon;
        colon = currSerialLine.indexOf(":", lastColon + 1);
        order_id = currSerialLine.substring(lastColon + 1, colon).toInt();

        lastColon = colon;
        colon = currSerialLine.indexOf(":", lastColon + 1);
        String warehouseString = currSerialLine.substring(lastColon + 1, colon);

        if (warehouseString.equals("A")) {
          warehouseDestination = WAREHOUSE_A;
        }
        if (warehouseString.equals("B")) {
          warehouseDestination = WAREHOUSE_B;
        }
        if (warehouseString.equals("C")) {
          warehouseDestination = WAREHOUSE_C;
        }
        lastColon = colon;
        houseDestination = currSerialLine.substring(lastColon + 1).toInt();

        retVal = true;
      }
      currSerialLine = "";
    } 
  }
  return retVal;
}
//------------------------------------SERIAL Reading/Writing Methods--------------------------------//





//------------------------------------SENSOR Reading Methods--------------------------------//
// Method for reading relfectance sensors
unsigned long refSensorRead(int sensorPin) {
  pinMode(sensorPin, OUTPUT);
  digitalWrite(sensorPin, HIGH); // Drive sensor high
  delayMicroseconds(10);         // Wait for charge
  pinMode(sensorPin, INPUT);
  unsigned long startTime = micros();
  digitalWrite(sensorPin, LOW);  
  while (digitalRead(sensorPin)) {
  }
  unsigned long endTime = micros();  //Times decay
  return (endTime - startTime);
}

//Converts sensor readings into booleans
boolean checkSensor(int sensor) {
  
  if (sensor == rightSensor) {
    int rightRead = refSensorRead(refSensorR);
    if(rightRead > (avgR + hys) && lastRRSReading == LIGHT) {
      lastRRSReading = rightRead;
      Serial.println("Right = dark");
      return DARK;               //reading tape
    } 
    else if (rightRead < (avgR - hys) && lastRRSReading == DARK){ 
      lastRRSReading = rightRead;
      Serial.println("Right = light");
      return LIGHT;              //reading floor
    }
    lastRRSReading = rightRead;
  }
  else if (sensor == leftSensor) {
    int leftRead = refSensorRead(refSensorL);
    if(leftRead > (avgL + hys) && lastLRSReading == LIGHT) {
      lastLRSReading = leftRead;
      Serial.println("left = dark");
      return DARK;               //reading tape
    } 
    else if (leftRead < (avgL - hys) && lastLRSReading == DARK){ 
      lastLRSReading = leftRead;
      Serial.println("Left = light");
      return LIGHT;              //reading floor
    }
    lastLRSReading = leftRead;
  }
}
//------------------------------------SENSOR Reading Methods--------------------------------//




//------------------------------------BASIC LINE FOLLOWING LOGIC--------------------------------//
//Method that contains logic for following lines
void lineFollow() {
  digitalWrite(dirA, motorDirectionA);
  digitalWrite(dirB, motorDirectionB);
  analogWrite(pwmA, motorSpeedA);
  analogWrite(pwmB, motorSpeedB);

  //Intersection
  if(checkSensor(rightSensor) == DARK && checkSensor(leftSensor) == DARK) {
    //    digitalWrite(dirA, !motorDirectionA);
    //    digitalWrite(dirB, !motorDirectionB);
    //    analogWrite(pwmA, motorSpeedA);
    //    analogWrite(pwmB, motorSpeedB);
    //    delay(50);
    //    digitalWrite(dirA, motorDirectionA);
    //    digitalWrite(dirB, motorDirectionB);
    //    analogWrite(pwmA, 0.25 * motorSpeedA);
    //    analogWrite(pwmB, 0.25 * motorSpeedA);
    //    delay(150);
    analogWrite(pwmA, 0);
    analogWrite(pwmB, 0);
    delay(500);
    //turnRight(); 
    //    Serial.println("Done turning left");
    handleIntersection();
  }
  //Turning Left
  else if (checkSensor(rightSensor) == DARK && checkSensor(leftSensor) == LIGHT) {
    analogWrite(pwmA, 0);
    analogWrite(pwmB, motorSpeedB); //correct by turning right
  }

  //Turning Right
  else if(checkSensor(rightSensor) == LIGHT && checkSensor(leftSensor) == DARK) {
    analogWrite(pwmA, motorSpeedA); //correct by turning left
    analogWrite(pwmB, 0);
  }

  //Straight
  else if(checkSensor(rightSensor) == LIGHT && checkSensor(leftSensor) == LIGHT) {
    analogWrite(pwmA, motorSpeedA);
    analogWrite(pwmB, motorSpeedB);
  }
}
//------------------------------------BASIC LINE FOLLOWING LOGIC--------------------------------//





//------------------------------------TURNING METHODS--------------------------------//
void turnRight() {
  Serial.println("Turn Right");
  analogWrite(pwmA, 0);
  analogWrite(pwmB, motorSpeedB);  //Turn right
  //  while (checkSensor(leftSensor) == LIGHT) {
  //  }
  //  while (checkSensor(leftSensor)checkSensor(leftSensor) == DARK) {
  //  }
  //  while (checkSensor(leftSensor) == LIGHT) {
  //  }
  //  analogWrite(pwmB, 0);
  delay(1200);
}
void turnLeft() {
  Serial.println("Turn Left");
  analogWrite(pwmA, motorSpeedA);
  analogWrite(pwmB, 0);  //Turn left
  //  while (checkSensor(rightSensor) == LIGHT) {
  //  }
  //  while (checkSensor(rightSensor) == DARK) {
  //  }
  //  while (checkSensor(rightSensor) == LIGHT) {
  //  }
  //  analogWrite(pwmA, 0);
  //  analogWrite(pwmB, motorSpeedB);
  delay(1250);
}
void driveStraight() {
  analogWrite(pwmA, motorSpeedA);
  analogWrite(pwmB, motorSpeedB);
  while (checkSensor(rightSensor) == DARK && checkSensor(leftSensor) == DARK) {
  }
  while (checkSensor(rightSensor) == LIGHT && checkSensor(leftSensor) == LIGHT) {
  }
}
void turnAround() {
  Serial.println("Turn Around");
  digitalWrite(dirA, !motorDirectionA);
  analogWrite(pwmA, motorSpeedA);
  analogWrite(pwmB, motorSpeedB);
  while (checkSensor(leftSensor) == LIGHT) {
  }
  while (checkSensor(leftSensor) == DARK) {
  }
  while (checkSensor(leftSensor) == LIGHT) {
  }
  while (checkSensor(leftSensor) == DARK) {
  }
  while (checkSensor(leftSensor) == LIGHT) {
  }
  digitalWrite(dirA, motorDirectionA);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}

void turnInPlace90() {
  digitalWrite(dirA, !motorDirectionA);
  analogWrite(pwmA, motorSpeedA);
  analogWrite(pwmB, motorSpeedB);
  while (checkSensor(leftSensor) == LIGHT) {
  }
  while (checkSensor(leftSensor) == DARK) {
  }
  while (checkSensor(leftSensor) == LIGHT) {
  }
  digitalWrite(dirA, motorDirectionA);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}

void fullStop() {        // Brings robot to a stop
  Serial.println("Full Stop");
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}
//------------------------------------TURNING METHODS--------------------------------//



//------------------------------------NAVIGATION STATE MACHINE--------------------------------//
//Contains logic for navigation after encountering an intersection
void handleIntersection() {
  switch(location) {
    //-----------ROADS---------------------------------------------------
  case ROAD_W:
    if (destination == WAREHOUSE_A) {
      driveStraight();
      location = WAREHOUSE_A;
    }
    if (destination == WAREHOUSE_B || destination == WAREHOUSE_C) {
      turnLeft();
      location = ROAD_BC;
    }
    else {
      driveStraight();
      location = ROAD_H;
    }
    break;
  case ROAD_BC:
    if (destination == WAREHOUSE_B) {
      turnRight();
      location = WAREHOUSE_B;
    }
    if (destination == WAREHOUSE_C) {
      driveStraight();
      location = WAREHOUSE_A;
    }
    else {
      turnRight();
      location = ROAD_W;
    }
    break;
  case ROAD_H:
    if (destination == HOUSE_1) {
      turnLeft();
      location = HOUSE_1;
    }
    if (destination == HOUSE_2) {
      turnRight();
      location = HOUSE_2;
    }
    if (destination == HOUSE_3 || destination == HOUSE_4 || destination == HOUSE_5) {
      driveStraight();
      location = ROAD_345;
    }
    else if (destination == HEADQUARTERS) {
      turnLeft();
      location = HEADQUARTERS;
    }
    else {
      driveStraight();
      location = ROAD_W;
    }
    break;
  case ROAD_345:
    if (destination == HOUSE_3) {
      turnLeft();
      location = HOUSE_3;
    }
    if (destination == HOUSE_4) {
      turnRight();
      location = HOUSE_4;
    }
    if (destination == HOUSE_5) {
      driveStraight();
      location = HOUSE_5;
    }
    else {
      driveStraight();
      location = ROAD_H;
    }
    break;
    //------------HOUSES----------------------------------------------
  case HOUSE_1:
    if (destination == HOUSE_1) {
      deliver();
    } 
    else {
      turnRight();
      location = ROAD_H;
    }
  case HOUSE_2:
    if (destination == HOUSE_2) {
      deliver();
    } 
    else {
      turnLeft();
      location = ROAD_H;
    }
    break;
  case HOUSE_3:
    if (destination == HOUSE_3) {
      deliver();
    } 
    else {
      turnRight();
      location = ROAD_345;
    }
    break;
  case HOUSE_4:
    if (destination == HOUSE_4) {
      deliver();
    } 
    else {
      turnLeft();
      location = ROAD_345;
    }
    break;
  case HOUSE_5:
    if (destination == HOUSE_5) {
      deliver();
    } 
    else {
      driveStraight();
      location = ROAD_345;
    }
    break;
    //--------WAREHOUSES-------------------------------
  case WAREHOUSE_A:
    if (destination == WAREHOUSE_A) {
      pickup();
    } 
    else {
      driveStraight();
      location = ROAD_W;
    }
    if (destination == WAREHOUSE_B) {
      pickup();
    } 
    else {
      turnLeft();
      location = ROAD_BC;
    }
    if (destination == WAREHOUSE_C) {
      pickup();
    } 
    else {
      driveStraight();
      location = ROAD_BC;
    }
    break;
    //---------HEADQUARTERS----------------------------
  case HEADQUARTERS:
    if (destination == HEADQUARTERS) {
      turnAround();
      while (!servicingOrder) {
        fullStop();
      }
    }
    else {
      turnLeft();
      location = ROAD_W;
    }
    break;
  }
  Serial.println(location);
}
//------------------------------------NAVIGATION STATE MACHINE--------------------------------//




//------------------------------------Pickup / Deliver--------------------------------//
void deliver() {
  turnInPlace90();
  Serial.println("Delivering");
  servo.write(50);
  delay(1000);
  turnInPlace90();
  destination = HEADQUARTERS;
  delivered = true;
  servo.write(180);
}

void pickup() {
  Serial.println("Picking up");
  servo.write(180);
  delay(5000);
  turnAround();
  destination = houseDestination;
}
//------------------------------------Pickup / Deliver--------------------------------//





//------------------------------------ENCODER--------------------------------//
double odometer() {
  double distance = 0;
  int secondaryCount;
  noInterrupts();
  secondaryCount = encoder_counter;
  interrupts();
  //Motor Speed Math
  distance = (abs(secondaryCount)) * (1 / 3592.) * 2 * PI * 0.042;
  encoder_counter = 0;
  return distance;
}

//ISR for A
void ISRA() {
  encoder_counter++;
}
//------------------------------------ENCODER--------------------------------//




//------------------------------------SENSOR CALIBRATION-------------------------------//
void calibrateL() {
  int current = refSensorRead(refSensorL);
  if (current > maxReadingL) {
    maxReadingL = current;
  }
  if (current < minReadingL) {
    minReadingL = current;
  }  
}

void calibrateR() {
  int current = refSensorRead(refSensorR);
  if (current > maxReadingR) {
    maxReadingR = current;
  }
  if (current < minReadingR) {
    minReadingR = current;
  }  
}

void calibrationPeriod() {
  unsigned long endTime = millis() + 5000;
  while (millis() < endTime) {
    calibrateL();
    calibrateR();
  }
  avgR = (maxReadingR + minReadingR) / 2;
  avgL = (maxReadingL + minReadingL) / 2;
  Serial.print("Done Calibrating. AvgL = ");
  Serial.print(avgL);
  Serial.print(" AvgR = ");
  Serial.println(avgR);
}
//------------------------------------SENSOR CALIBRATION-------------------------------//







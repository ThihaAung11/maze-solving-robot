#include <NewPing.h>

#define TRIGGER_PINL  4 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     5  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  6  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     7  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  2  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     3  // Arduino pin tied to echo pin on ping sensor.





int dir;
     

#define STOP 100
#define FORWARD 101
#define BACKWARD 102
#define LEFT 103
#define RIGHT 104 



float P = 1.5 ;
float D = 0.8 ;
float I = 0.4 ;
float oldErrorP = 0;
float totalError = 0;

int wall_threshold = 16;
int front_threshold = 12;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;


int en1 =  24; //l
int en2 = 25;

int en3 =  23;//r
int en4 =  22;

int enA =  12;   //r
int enB =  13;   //l

int baseSpeed = 90 ;

int RMS ;
int LMS ;



NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

//int TestNUM = 1  ;



void setup() {

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


  for (int i = 8; i <= 13; i++) //For Motor Shield
    pinMode(i, OUTPUT);
    
  for (int i = 22; i <= 25; i++) //For Motor Shield
    pinMode(i, OUTPUT);
}

void loop() {


  //========================================START========================================//


  ReadSensors();

  walls();

  
  if (rightwall == false && frontwall == true) {
    pid_turnLeft(false);
  }
  
  else if (rightwall == true && frontwall == false ) {
    pid_forward();
  }
  
  else if (leftwall == false && rightwall == true && frontwall == true ) {
    pid_turnLeft(true) ;

  }
  /*else{
    analogWrite(enA,240);
    analogWrite(enB,230);
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
    delay(730);
    }*/
  
  
  // read sensors & print the result to the serial monitor //


  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  //measure error & print the result to the serial monitor


  Serial.print("error=");
  Serial.println(totalError);
}

//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, LOW );   // Left wheel forward
    digitalWrite(en2, HIGH );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, HIGH );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

  //leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
  //rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
  //frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

  //leftSensor = sonarLeft.convert_cm(leftSensor);
  //rightSensor = sonarRight.convert_cm(rightSensor);
  //frontSensor = sonarFront.convert_cm(frontSensor);

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

//---------------------------------------------------------------------------//


//--------------------------------- control ---------------------------------//

void pid_forward() {
  float errorP= 0.0, errorD= 0.0, errorI= 0.0;
  
  if(leftSensor > rightSensor){
    errorP = leftSensor - rightSensor ;  
  }

  if(leftSensor < rightSensor){
    errorP = rightSensor - leftSensor ;
  }
  errorD = errorP - oldErrorP;
  errorI = errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;
  
  oldErrorP = errorP ;
  
  if(leftSensor > rightSensor){
   
   RMS = baseSpeed + totalError+10;
   LMS = baseSpeed -totalError+40; 
   analogWrite(enA , RMS);
   analogWrite(enB , LMS);
  }

  if(leftSensor < rightSensor){
    RMS = baseSpeed - totalError ;
    LMS = baseSpeed + totalError+50 ;
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
  }
    setDirection(FORWARD);
}


//----------------------------- wall follow  control -------------------------------//

void pid_turnLeft( boolean left ) {

  if (left == true ) {
    RMS = 195;
    LMS = 180; 
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(LEFT); 
    delay(710);
    analogWrite(enA,85);
    analogWrite(enB,85);
    setDirection(FORWARD);
    delay(575);
   }
   
  else {
      RMS = 170;
      LMS = 215; 
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(RIGHT);
      delay(710);
      analogWrite(enA, 85);
      analogWrite(enB, 85);
      setDirection(FORWARD);
      delay(650);
    }      

}

//--------------------------- wall detection --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;
  }
  
  if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}

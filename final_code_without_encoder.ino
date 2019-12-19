#include <NewPing.h>
#include <Servo.h>

Servo rotateServo,baseServo,armServo;

#define S0 36
#define S1 34
#define S2 38
#define S3 40
#define sensorOut 42

#define TRIGGER_PINL  A1 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A2//hjui8 Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A3 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A4 // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A6  // Arduino pin tied to echo pin on ping sensor.

int dir;
     

#define STOP 100
#define FORWARD 101
#define BACKWARD 102
#define LEFT 103
#define RIGHT 104 



float P = 4;
float D = 1;
float I = 0.5;
float oldErrorP = 0;
float totalError = 0;

int wall_threshold = 15;
int front_threshold = 10;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;


int en1 = 3; //l
int en2 = 4;

int en3 =  6;//r
int en4 =  5;

int enA =  2;   //l
int enB =  7;   //r

int baseSpeed = 200;

int RMS ;
int LMS ;
int frequencyR = 0,frequencyG=0,frequencyB=0;
int yellow = 0, blue = 0, green = 0, brown=0;
int y=0, bl=65, gr=110, br=170;

NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

float oldLeftSensor, oldRightSensor,leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

void setup() {

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


  for (int i = 2; i <= 7; i++) //For Motor Shield
    pinMode(i, OUTPUT);
    
  for (int i =34; i <= 40 ; i+=2) //For Motor Shield
    pinMode(i, OUTPUT);
       
  pinMode(sensorOut, INPUT);
      
      // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  rotateServo.attach(19);
  rotateServo.write(40);
  baseServo.attach(20);
  baseServo.write(110);
  armServo.attach(21);
  armServo.write(50);
}

void loop() {

  ReadSensors();
  walls();
  colorDetection();
  
   if(frequencyR > 240 && frequencyG > 240 && frequencyB < 210){
    adjustment();
    servoMovement(yellow,y);
    Serial.println("yellow");
    yellow++;
  }
   else if( frequencyR < 85 && frequencyG < 90 && frequencyB < 130){
    adjustment();
    Serial.println("blue");
    servoMovement(blue,bl);
    blue++;
  }
  else if((frequencyR > 120 && frequencyR < 190 )&&(frequencyG > 40 && frequencyG < 100 )&& frequencyB < 95){
    adjustment();
    Serial.println("brown");
    servoMovement(brown,br);
    brown++;
  }
   else if(frequencyR < 105 &&(frequencyG < 210 && frequencyG > 180  )&&frequencyB < 120){
    adjustment();
   servoMovement(green,gr);
   Serial.println("green");
   green++;
  }  
  else if(frequencyR < 60 &&frequencyG < 60 && frequencyB < 60){
    Serial.println("stop");
    setDirection(STOP);
    delay(20000);
  }
  
  if((rightwall == false || leftwall == false) && frontwall == false){
    analogWrite(enA,170);
    analogWrite(enB,170);
    setDirection(FORWARD);
  }
  else if (rightwall == false && frontwall == true) {
    pid_turnLeft(false);
  }
  
  else if (frontwall == false && rightwall == true) {
    pid_forward();
  }
  
  else if (leftwall == false && rightwall == true && frontwall == true ) {
    pid_turnLeft(true) ;

  }
  else if(frontwall == true && rightwall == true && leftwall == true){
    analogWrite(enA,250);
    analogWrite(enB,250);
    if(leftSensor > rightSensor){
      setDirection(RIGHT);
    }
    else{
      setDirection(LEFT);
    }
    delay(700);
    
    }
  
  
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
   
   RMS = baseSpeed +totalError;
   LMS = baseSpeed -totalError; 
   analogWrite(enB, RMS);
   analogWrite(enA , LMS+15);
  }

  if(leftSensor < rightSensor){
    RMS = baseSpeed - totalError ;
    LMS = baseSpeed + totalError;
    analogWrite(enB , RMS);
    analogWrite(enA , LMS+15);
  }
    setDirection(FORWARD);
}

void pid_turnLeft( boolean left ) {

  if (left == true ) {
    
    RMS = 250;
    LMS = 250;
    analogWrite(enB,RMS);
    analogWrite(enA,LMS);
    setDirection(LEFT);
    delay(680);
    setDirection(STOP); 
    delay(1000);
    analogWrite(enA, 190);
    analogWrite(enB, 180);
    setDirection(FORWARD); 
    delay(700);
  }
   
  else {
    
     RMS = 250;
     LMS = 250; 
     analogWrite(enB , RMS);
     analogWrite(enA , LMS);
     setDirection(RIGHT);
     delay(650);
     setDirection(STOP);
     delay(1000);
     analogWrite(enA, 180);
     analogWrite(enB, 180);
     setDirection(FORWARD);
     delay(700);
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

void adjustment(){
  while(rightwall==false && frontSensor > 10){
    setDirection(FORWARD);
    analogWrite(enA,170);
    analogWrite(enB,170);
    ReadSensors();
  }
  
  while(leftwall == false && frontSensor > 10){
    setDirection(FORWARD);
    analogWrite(enA,170);
    analogWrite(enB,170);
    ReadSensors();
  }
  
  if (rightwall == false && frontwall < 10) {
    pid_turnLeft(false);
    setDirection(STOP);
  }
  
  else if (frontwall == false && rightwall == true) {
    analogWrite(enA,190);
    analogWrite(enB,180);
    setDirection(FORWARD);
    delay(500);
    setDirection(STOP);
  }
  
  else if (leftwall == false && rightwall == true && frontwall == true ) {
    pid_turnLeft(true);
    setDirection(STOP);
  }
  else if(frontwall == true && rightwall == true && leftwall == true){
    analogWrite(enA,250);
    analogWrite(enB,250);
    if(leftSensor > rightSensor){
      setDirection(RIGHT);
    }
    else{
      setDirection(LEFT);   
    delay(700);
    setDirection(STOP);
    }
  }
}

void colorDetection(){
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);                                     
  frequencyR = pulseIn(sensorOut, LOW);                     // Reading the output frequency
  frequencyR= map(frequencyR, 5,38,255,0);                  //Remaping the value of the frequency to the RGB Model of 0 to 255
  Serial.print("R= ");//printing name
  Serial.print(frequencyR);//printing RED color frequency
  Serial.print("  ");
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequencyG = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequencyG = map(frequencyG, 6,39,255,0);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequencyG);//printing RED color frequency
  Serial.print("  ");
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequencyB = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequencyB = map(frequencyB, 4,29,255,0);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequencyB);//printing RED color frequency
  Serial.println("  ");
}

void servoMovement(int count,int degree){
 int d=0;
 int p=40,pos=40; 
 if(degree < 40){
  for(p = 40; p > degree;p-=1){
    rotateServo.write(p);              
    delay(15);
    d = 1;
  }
 }
 else{
   for(pos = 40; pos < degree; pos += 1) { 
     rotateServo.write(pos);              
     delay(15);                       
    } 
  
  }
 
  if(count == 0){
    baseServo.write(90);
    delay(150);
    }
  else{
    baseServo.write(70);
    delay(150);  
  }
  armServo.write(0);
  delay(150);
  armServo.write(50);
  if(d==1){
    for(p=0 ; p < 40;p+=1){
    rotateServo.write(p);              
    delay(15);
  }
  }
  else{
    for(pos=degree; pos > 40;pos-=1){
    rotateServo.write(pos);              
    delay(15);
    }
  }
  delay(150);
}

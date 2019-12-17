#define S0 36
#define S1 34
#define S2 38
#define S3 40
#define sensorOut 42
int frequencyR = 0,frequencyG=0,frequencyB=0;
int yellow = 0, blue = 0, green = 0, brown=0;
void setup() {
  // put your setup code here, to run once:
 for (int i =34; i <= 40 ; i+=2) //For Motor Shield
    pinMode(i, OUTPUT);
       
  pinMode(sensorOut, INPUT);
      
      // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequencyR = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequencyR= map(frequencyR, 5,33,255,0);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequencyR);//printing RED color frequency
  Serial.print("  ");
  delay(15);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequencyG = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequencyG = map(frequencyG, 6,35,255,0);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequencyG);//printing RED color frequency
  Serial.print("  ");
  delay(15);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequencyB = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  frequencyB = map(frequencyB, 4,26,255,0);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequencyB);//printing RED color frequency
  Serial.println("  ");
  delay(15);
   if(yellow < 2 && frequencyR > 240 && frequencyG > 240 && frequencyB < 240){
    setDirection(STOP);
    delay(100);
    adjustment();
    //servoMovement();
    Serial.println("yellow");
    yellow++;
  }
  else if(blue < 2 && frequencyR < 40 && frequencyG < 40 && frequencyB < 200){
    setDirection(STOP);
    delay(100);
    adjustment();
    Serial.println("blue");
    //servoMovement();
    blue++;
  }
  else if(brown < 2 && (frequencyR < 140 && frequencyR > 180 )&&(frequencyG < 60 && frequencyG > 90 )&&frequencyB < 100){
    setDirection(STOP);
    delay(100);
    adjustment();
    Serial.println("brown");
    //servoMovement();
    brown++;
  }
  else if(green < 2 && frequencyR < 60&&(frequencyG < 150 && frequencyG > 200 )&&frequencyB < 60){
    setDirection(STOP);
    delay(100);
    adjustment();
   // servoMovement();
   Serial.println("green");
   green++;
  }  
  else if(frequencyR < 60 &&frequencyG < 60 && frequencyB < 60){
    Serial.println("stop");
    setDirection(STOP);
    delay(20000);
  }
  
}

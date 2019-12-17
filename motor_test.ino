int en1 = 3;//l
int en2 = 4;

int en3 =  6;//r
int en4 =  5;

int enA =  2;   //r
int enB =  7; 
void setup() {
  // put your setup code here, to run once:
  for (int i = 2; i <= 7; i++) //For Motor Shield
    pinMode(i, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    analogWrite(enA,220);
    analogWrite(enB,200);
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
    /*analogWrite(enA,240);
    analogWrite(enB,230);
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
    delay(730);
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, LOW); 
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, LOW);
    delay(3000);*/
}

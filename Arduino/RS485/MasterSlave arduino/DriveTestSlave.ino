#include <Servo.h>
byte j;
byte message[7];
int  x[10];
int  k=10;
boolean f0 = true;  //true when arduino is waiting for data.
boolean f1 = false; //indicates that x needs to be filled with values
const int redPin =  9;
const int yellowPin = 13;


void setup() {
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(5, HIGH);
  Serial1.begin(9600);
  //Serial1.setTimeout(15);
  pinMode(redPin,    OUTPUT); digitalWrite(redPin,    LOW);
  pinMode(yellowPin, OUTPUT); digitalWrite(yellowPin, LOW);

  while (!Serial1) {;}
  //Serial.println("System ON");
}

void loop() {
  if (f0){
   for (int i=0;i<7;i++){         //read in data for each element of a
     while(Serial1.available()<2);  //wait until the buffer contains two bytes
     j = Serial1.read();      //read the first byte (index)
     message[int(j)] = Serial1.read();      //read the second byte (actual value)
     Serial1.flush();   //flush the output buffer
     Serial1.write(j);  //tell arduino that item j was received
   }
    if (message[0]==1 & message[1]==0 & message[2]==0 & message[3]==20){
     f0=false;
     f1=true;
   }
 }
 if (f1){  //encode a pattern of flashes in x, (1 is red, 2 is yellow)
   x[0]=1; 
   x[1]=2; 
   x[2]=1;
   x[3]=2;
   x[4]=1;
   x[5]=2;
   x[6]=1;
   x[7]=2;
   x[8]=1;
   x[9]=2;
   f1=false;
   k=0;
 }
 
 if(k<10){
   if(x[k]==1){        
     //flash the red pin
     digitalWrite(redPin,    HIGH); delay(100);
     digitalWrite(redPin,    LOW);  delay(500);
   }else if (x[k]==2){ 
     //flash the yellow pin
     digitalWrite(yellowPin, HIGH); delay(100);
     digitalWrite(yellowPin, LOW);  delay(500);
   }
   k=k+1;
 }else if (k==10){
   f0=true;
 }
}


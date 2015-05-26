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
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, LOW);
  Serial.begin(57600);
  Serial1.begin(9600);
  //Serial1.setTimeout(15);

  while (!Serial) {;}
  Serial.println("System ON");
}

void loop() {
   for (int i=0;i<7;i++){         //read in data for each element of a
     if(Serial1.available()==2) {  //wait until the buffer contains two bytes
     Serial.println("inside the if statement");
     j = Serial1.read();      //read the first byte (index)
     message[int(j)] = Serial1.read();      //read the second byte (actual value)
     Serial1.flush();   //flush the output buffer
     Serial.println(message[j]);  //tell arduino that item j was received
     }
   }
}


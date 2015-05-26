#include <Servo.h>
byte boardAddress = 31;
byte j;
byte message[7];
byte address;
byte function;
byte argumentLo;
byte argumentMid1;
byte argumentMid2;
byte argumentHi;
byte termination;
int messageComponents = 7;

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
  if (Serial1.read() == 167)
    readMessage();
  
  //if this Board is the chosen one
  if((address > 30) && (address < 39)){
    
    //If the message was correctly terminated
    if (termination == 255){
      processMessage(); 
    }
  }
}

void readMessage(){
 for (int i=0;i<messageComponents;i++){         //read in data for each element of a
     if(Serial1.available()==2) {  //wait until the buffer contains two bytes
     Serial.println("inside the if statement");
     j = Serial1.read();      //read the first byte (index)
     message[int(j)] = Serial1.read();      //read the second byte (actual value)
     Serial1.flush();   //flush the output buffer
     Serial.println(message[j]);  //tell arduino that item j was received
     }
   }
   
  address = message[0];
  function = message[1];
  argumentLo = message[2];
  argumentMid1 = message[3];
  argumentMid2 = message[4];
  argumentHi = message[5];
  termination = message[6];
}

void processMessage(){
  if ((function = 1) && (address = 31))
    enableServo1();
    
  else if ((function = 1) && (address = 32))
    enableServo2();
    
  else if ((function = 1) && (address = 33))
    enableServo3();
    
  else if ((function = 1) && (address = 34))
    enableServo4();
    
  else if ((function = 1) && (address = 35))
    enableServo5();
    
  else if ((function = 1) && (address = 36))
    enableServo6();
    
  else if ((function = 1) && (address = 37))
    enableServo7();
    
  else if ((function = 1) && (address = 38))
    enableServo8();

  else if ((function = 2) && (address = 31))
    disableServo1();
    
  else if ((function = 2) && (address = 32))
    disableServo2();
    
  else if ((function = 2) && (address = 33))
    disableServo3();
    
  else if ((function = 2) && (address = 34))
    disableServo4();
    
  else if ((function = 2) && (address = 35))
    disableServo5();
    
  else if ((function = 2) && (address = 36))
    disableServo6();
    
  else if ((function = 2) && (address = 37))
    disableServo7();
    
  else if ((function = 2) && (address = 38))
    disableServo8();
    
  else if ((function = 5) && (address = 31))
    setServo1Angle();
    
  else if ((function = 5) && (address = 32))
    setServo2Angle();
    
  else if ((function = 5) && (address = 33))
    setServo3Angle();
    
  else if ((function = 5) && (address = 34))
    setServo4Angle();
    
  else if ((function = 5) && (address = 35))
    setServo5Angle();
    
  else if ((function = 5) && (address = 36))
    setServo6Angle();
    
  else if ((function = 5) && (address = 37))
    setServo7Angle();
    
  else if ((function = 5) && (address = 38))
    setServo8Angle();
    
  else{}
}

void enableServo1(){
  //put your enable servo code here
}

void enableServo2(){
  //put your enable servo code here
}

void enableServo3(){
  //put your enable servo code here
}

void enableServo4(){
  //put your enable servo code here
}

void enableServo5(){
  //put your enable servo code here
}

void enableServo6(){
  //put your enable servo code here
}

void enableServo7(){
  //put your enable servo code here
}

void enableServo8(){
  //put your enable servo code here
}

void disableServo1(){
  //put your disable servo code here
}

void disableServo2(){
  //put your disable servo code here
}

void disableServo3(){
  //put your disable servo code here
}

void disableServo4(){
  //put your disable servo code here
}

void disableServo5(){
  //put your disable servo code here
}

void disableServo6(){
  //put your disable servo code here
}

void disableServo7(){
  //put your disable servo code here
}

void disableServo8(){
  //put your disable servo code here
}

void setServo1Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo2Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo3Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo4Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo5Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo6Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo7Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

void setServo8Angle(){
  int angle = decodeLongArgument();
  //put your disable servo code here
}

int decodeLongArgument(){
  return argumentHi + (argumentMid2 << 8) + (argumentMid1 << 16) + (argumentLo << 24);
}


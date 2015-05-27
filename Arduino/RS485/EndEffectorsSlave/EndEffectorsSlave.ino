#include <Servo.h>
byte boardAddress = 21;
byte j;
byte message[7];
byte address;
byte function;
byte argumentLo;
byte argumentMid1;
byte argumentMid2;
byte argumentHi;
byte termination;

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
  if(Serial1.read() == 167)
    readMessage();
  
  //if this Board is the chosen one
  if((address = boardAddress) || (address = boardAddress + 1)){
    
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
  if ((function = 1) && (address = boardAddress))
    enableClaw();
    
  else if ((function = 1) && (address = boardAddress + 1))
    enableScienceEnclosure();
    
  else if ((function = 2) && (address = boardAddress))
    disableClaw();
    
  else if ((function = 2) && (address = boardAddress + 1))
    disableScienceEnclosure();
    
  else if ((function = 3) && (address = boardAddress))
    setClawSpeed();
    
  else if ((function = 3) && (address = boardAddress + 1))
    setScienceEnclosureSpeed();
    
  else if ((function = 4) && (address = boardAddress))
    clawBrake();
    
  else if ((function = 4) && (address = boardAddress + 1))
    scienceEnclosureBrake();
    
  else if ((function = 5) && (address = boardAddress))
    setClawAngle;
    
  else if ((function = 5) && (address = boardAddress + 1))
    setScienceEnclosureAngle;
    
  else{}
}

void enableClaw(){
  //insert enable motor code here
}

void enableScienceEnclosure(){
  //insert enable motor code here
}

void disableClaw(){
  //insert disable motor code here
}

void disableScienceEnclosure(){
  //insert disable motor code here
}

//5000 is 0 speed, 9000 is max forward and 1000 is max backward
void setClawSpeed(){
  int speed = decodeLongArgument();
  //insert code for setting speed here
}

void setScienceEnclosureSpeed(){
  int speed = decodeLongArgument();
  //insert code for setting speed here
}

void clawBrake(){
  //insert code to brake motors here
}

void scienceEnclosureBrake(){
  //insert code to brake motors here
}

void setClawAngle(){
  int angle = decodeLongArgument();
  //insert claw angle adjustment code here
}

void setScienceEnclosureAngle(){
  int angle = decodeLongArgument();
  //insert science enclosure angle adjustment code here
}

int decodeLongArgument(){
  return argumentHi + (argumentMid2 << 8) + (argumentMid1 << 16) + (argumentLo << 24);
}

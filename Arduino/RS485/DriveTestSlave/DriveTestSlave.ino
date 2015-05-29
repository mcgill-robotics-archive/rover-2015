#include <Servo.h>
byte boardAddress = 1;
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
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
  Serial.begin(9600);
  Serial1.begin(9600);
  //Serial1.setTimeout(15);

  while (!Serial) {;}
  Serial.println("System ON");
}

void loop() {
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
 while (! Serial1.available()) delay(5);
  if (Serial1.read() == 167)
  {
    for (int i = 0; i < 7; i++)
    {
      while( !(Serial1.available() > 2)) delay(5);
      if (Serial1.read() != '$');
      else
      {
        j = Serial1.read();      //read the first byte (index)
//        Serial.print ("index: ");
//        Serial.print(j);
        message[int(j)] = Serial1.read();      //read the second byte (actual value)
//        Serial.print(" message: ");
//        Serial.println(message[j]);  //tell arduino that item j was received
      }
    }
  }
  else
    delay(10);
   
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
    enableLeftMotor();
    
  else if ((function = 1) && (address = boardAddress + 1))
    enableRightMotor();
    
  else if ((function = 2) && (address = boardAddress))
    disableLeftMotor();
    
  else if ((function = 2) && (address = boardAddress + 1))
    disableRightMotor();
    
  else if ((function = 3) && (address = boardAddress))
    setLeftMotorSpeed();
    
  else if ((function = 3) && (address = boardAddress + 1))
    setRightMotorSpeed();
    
  else if ((function = 4) && (address = boardAddress))
    leftMotorBrake();
    
  else if ((function = 4) && (address = boardAddress + 1))
    rightMotorBrake();  
    
  else{}
}

void enableLeftMotor(){
  //insert enable motor code here
  Serial.print ("enabling left motor");
}

void enableRightMotor(){
  //insert enable motor code here
}

void disableLeftMotor(){
  //insert disable motor code here
}

void disableRightMotor(){
  //insert disable motor code here
}

//5000 is 0 speed, 9000 is max forward and 1000 is max backward
void setLeftMotorSpeed (){
  int speed = decodeLongArgument();
  //insert code for setting speed here
}

void setRightMotorSpeed (){
  int speed = decodeLongArgument();
  //insert code for setting speed here
}

void leftMotorBrake(){
  //insert code to brake motors here
}

void rightMotorBrake(){
  //insert code to brake motors here
}

int decodeLongArgument(){
  return argumentHi + (argumentMid2 << 8) + (argumentMid1 << 16) + (argumentLo << 24);
}


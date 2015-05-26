#include <Servo.h>
byte driveBoardAddress = 1;
byte j;
byte message[8];

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
  readMessage();
  
  //if this Board is the chosen one
  if((message[1] = driveBoardAddress) || (message[1] = driveBoardAddress + 1)){
    
    //If the message was correctly terminated
    if (message[7] != 255){
      processMessage(); 
    }
    
//    else{
//      waitForTermination();
//    }

  }
}

void readMessage(){
 for (int i=0;i<8;i++){         //read in data for each element of a
     if(Serial1.available()==2) {  //wait until the buffer contains two bytes
     Serial.println("inside the if statement");
     j = Serial1.read();      //read the first byte (index)
     message[int(j)] = Serial1.read();      //read the second byte (actual value)
     Serial1.flush();   //flush the output buffer
     Serial.println(message[j]);  //tell arduino that item j was received
     }
   } 
}

void processMessage(){
  if ((message[2] = 1) && (message[1] = driveBoardAddress))
    enableLeftMotor();
    
  else if ((message[2] = 1) && (message[1] = driveBoardAddress + 1))
    enableRightMotor();
    
  else if ((message[2] = 2) && (message[1] = driveBoardAddress))
    disableLeftMotor();
    
  else if ((message[2] = 2) && (message[1] = driveBoardAddress + 1))
    disableRightMotor();
    
  else if ((message[2] = 3) && (message[1] = driveBoardAddress))
    setLeftMotorSpeed();
    
  else if ((message[2] = 3) && (message[1] = driveBoardAddress + 1))
    setRightMotorSpeed();
    
  else if ((message[2] = 4) && (message[1] = driveBoardAddress))
    leftMotorBrake();
    
  else if ((message[2] = 4) && (message[1] = driveBoardAddress + 1))
    rightMotorBrake();  
    
  else{}
}

void enableLeftMotor(){
  //insert enable motor code here
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
  return message[6] + (message[5] << 8) + (message[4] << 16) + (message[3] << 24);
}

void waitForTermination(){
  while (message[7] != 255){}
}



#include <Servo.h>
byte boardAddress = 41;
byte firstBatteryAddress = 41;
byte secondBatteryAddress = 42;
byte thirdBatteryAddress = 43;
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
  if(Serial1.read() == 167)
    readMessage();
  
  //if this Board is the chosen one
  if((address == firstBatteryAddress)||(address == secondBatteryAddress)||(address == thirdBatteryAddress)){
    
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
  if ((function = 6) && (address = firstBatteryAddress))
    setBattery1RelayOn();
    
  else if ((function = 6) && (address = secondBatteryAddress))
    setBattery2RelayOn();
    
  else if ((function = 6) && (address = thirdBatteryAddress))
    setBattery3RelayOn();
    
  else if ((function = 7) && (address = firstBatteryAddress))
    setBattery1RelayOff();
    
  else if ((function = 7) && (address = secondBatteryAddress))
    setBattery2RelayOff();
    
  else if ((function = 7) && (address = thirdBatteryAddress))
    setBattery3RelayOff();
    
  else{}
}

void setBattery1RelayOn(){
  
}

void setBattery2RelayOn(){
  
}

void setBattery3RelayOn(){
  
}

void setBattery1RelayOff(){
  
}

void setBattery2RelayOff(){
  
}

void setBattery3RelayOff(){
  
}


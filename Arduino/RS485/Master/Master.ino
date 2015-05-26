byte address = 1;
byte function = 0;
byte argumentLo = 0;
byte argumentMid1 = 20;
byte argumentMid2 = 0;
byte argumentHi = 67;
byte termination = 255;
int messageComponents = 7;
int state = 2;
int last;
byte message[]= {address, function, argumentLo, argumentMid1, argumentMid2, argumentHi, termination};     //Constructing the message

/*
 * The main idea here is to encode/decode the messages we're going to be sending over 
 * the RS485 communications protocol.
 * We'll be sending 7 bytes, 1 for address, 1 for function, 
 * 4 for argument and 1 for termination which will always have a value of 255
 */
 
void setup(){  
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A5, LOW);
  digitalWrite(A4, HIGH);
  Serial.begin(57600);          //USB Com port instantiation
  Serial1.begin(9600);          //RS-485 Com port instantiation
  while(!Serial){;}
  Serial.println("System ON");
}

void loop(){
    Serial.println("System ON");
    Serial1.flush();
    Serial1.write(byte(0));
    Serial1.write(message[0]);
    delay(40);
    
    for (int i=1; i<messageComponents; i++){
     //while (Serial1.available()<2){
       Serial1.flush();
       Serial1.write(byte(i));
       Serial1.write(message[i]);
       delay(40);
     // }
    }
}

void enableMotor(byte address){
  setAddress(address);
  setFunction(1);
}

void disabelMotor(byte address){
  setAddress(address);
  setFunction(2);
}

//5000 is 0 speed, 9000 is max forward and 1000 is max backward
void setMotorSpeed (byte address, int speed){
  setAddress(address);
  setFunction(3);
  sendLongArgument(speed);
}

void motorBrake(byte address){
  setAddress(address);
  setFunction(4);
}

void setAngle(byte address, int angle){
  setAddress(address);
  setFunction(5);
  sendLongArgument(angle);
}

void setRelayOn(byte address){
  setAddress(address);
  setFunction(6);
}

void setRelayOff(byte address){
  setAddress(address);
  setFunction(7);
}


//Construct Message
void constructMessage (byte message[]){
  setAddress(message[0]);
  setFunction(message[1]);
  setArgumentLo(message[2]);
  setArgumentMid1(message[3]);
  setArgumentMid2(message[4]);
  setArgumentHi(message[5]);
  setTermination();  
}

void setAddress (byte address){
  ::address = address;
}

void setFunction (byte function){
  ::function = function;
}

void setArgumentLo (byte argumentLo){
  ::argumentLo = argumentLo;
}

void setArgumentMid1 (byte argumentMid1){
  ::argumentMid1 = argumentMid1;
}

void setArgumentMid2 (byte argumentMid2){
  ::argumentMid2 = argumentMid2;
}

void setArgumentHi (byte argumentHi){
  ::argumentHi = argumentHi;
}

void setTermination(){
  ::termination = 255;
}

void sendLongArgument(int argument){
  argumentHi = argument & 0b00000000000000000000000011111111;
  argumentMid2 = (argument & 0b00000000000000001111111100000000) >> 8;
  argumentMid1 = (argument & 0b00000000111111110000000000000000) >> 16;
  argumentLo = (argument & 0b11111111000000000000000000000000) >> 24;
}




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

//Sends the angle*10 for increased precision
void setAngle(byte address, int angle){
  setAddress(address);
  setFunction(5);
  sendLongArgument(angle*10);
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
}

void setAddress (byte addressByte){
  address = addressByte;
}

void setFunction (byte functionByte){
  function = functionByte;
}

void setArgumentLo (byte argumentLoByte){
  argumentLo = argumentLoByte;
}

void setArgumentMid1 (byte argumentMid1Byte){
  argumentMid1 = argumentMid1Byte;
}

void setArgumentMid2 (byte argumentMid2Byte){
  argumentMid2 = argumentMid2Byte;
}

void setArgumentHi (byte argumentHiByte){
  argumentHi = argumentHiByte;
}

//puts a big argument in the 4 argument bytes
void sendLongArgument(int argument){
  argumentHi = argument & 0b00000000000000000000000011111111;
  argumentMid2 = (argument & 0b00000000000000001111111100000000) >> 8;
  argumentMid1 = (argument & 0b00000000111111110000000000000000) >> 16;
  argumentLo = (argument & 0b11111111000000000000000000000000) >> 24;
}


byte address = 1;
byte function = 0;
byte argumentLo = 0;
byte argumentMid1 = 20;
byte argumentMid2 = 0;
byte argumentHi = 67;
byte termination = 255;
int messageComponents = 7;
int state = 1;
int last;

/*
 * The main idea here is to encode/decode the messages we're going to be sending over 
 * the RS485 communications protocol.
 * We'll be sending 7 bytes, 1 for address, 1 for function, 
 * 4 for argument and 1 for termination
 */
 
void setup(){
  byte message[]= {addres, function, argumentLo, argumentMid1, argumentMid2, argumentHi, termination};     //Constructing the message
  
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(5, HIGH);
  Serial.begin(57600);          //USB Com port instantiation
  Serial1.begin(9600);          //RS-485 Com port instantiation
  while(!Serial){;}
  Serial.println("System ON");
}

void loop(){
  if(state == 2){
    Serial1.flush();
    Serial1.write(byte(0));
    Serial1.write(message[0]);
    delay(40);
    
    for (int i=1; i<messageComponents; i++){
     while (Serial1.available()<1);
     last = Serial1.read();
     if (int (last) == i-1){ //Confirm that the last element was received
       Serial1.flush();
       Serial1.write(byte(i));
       Serial1.write(message[i]);
       delay(40);
     }
    }
    state = 1;
  }
}





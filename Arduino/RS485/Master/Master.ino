byte address = 1;
byte function = 0;
byte argumentLo = 0;
byte argumentMid1 = 20;
byte argumentMid2 = 0;
byte argumentHi = 67;
byte termination = 254;
int messageComponents = 7;
int state = 2;
int last;
byte message[]= {address, function, argumentLo, argumentMid1, argumentMid2, argumentHi, termination};     //Constructing the message

/*
 * The main idea here is to encode/decode the messages we're going to be sending over 
 * the RS485 communications protocol.
 * We'll be sending 7 bytes, 1 for address, 1 for function, 
 * 4 for argument and 1 for termination
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





char mode;
int DataIn=0;
String ModeString[7] = {
  "RW0 = ", "RW1 = ", "ADD0 = ", "ADD1 = ", "ENPOL = ", "DIRPOL = ", "SPEED = 3"};
boolean ModeSelect = false;
byte RW0 =0;
byte RW1 = 0;
byte ADD0 = 0;
byte ADD1 = 0;

void setup(){
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){
  }
}

void parse(char mode){
  switch(mode){

    case '0':
      if(Serial.available() > 0){
        DataIn = Serial.parseInt();
        bitWrite(RW0,0,bitRead(DataIn,0));
        Serial.print(ModeString[0]);
        Serial.println(String(RW0, BIN));
        ModeSelect=false;
      }

    case '1':
      if(Serial.available() > 0){
        DataIn = Serial.parseInt();
        bitWrite(RW1,0,bitRead(DataIn,0));
        Serial.print(ModeString[1]);
        Serial.println(String(RW1, BIN));
        ModeSelect=false;
      }

    case '2':
      if(Serial.available() > 0){
        DataIn = Serial.parseInt();
        for (int i=0 ; i < 8; i++){
          bitWrite(ADD0,i,bitRead(DataIn,i));
        }
        Serial.print(ModeString[2]);
        Serial.println(String(ADD0, BIN));
        ModeSelect=false;
      }
      
       case '3':
      if(Serial.available() > 0){
        DataIn = Serial.parseInt();
        for (int i=0 ; i < 8; i++){
          bitWrite(ADD1,i,bitRead(DataIn,i));
        }
        Serial.print(ModeString[3]);
        Serial.println(String(ADD1, BIN));
        ModeSelect=false;
      }
}
}
void loop(){

  if(ModeSelect == false){
    if(Serial.available() > 0){
      char inByte = Serial.read();
      mode = inByte;
      ModeSelect = true;
      Serial.println("Mode Selected");
      Serial.println(ModeString[mode-'0']);
    }
  }

  if(ModeSelect == true){
    parse(mode);
    
  }


}









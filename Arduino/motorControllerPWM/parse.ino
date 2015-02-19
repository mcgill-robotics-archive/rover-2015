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
        for (int i=0 ; i < 7; i++){
          bitWrite(ADD0,i,bitRead(DataIn,i));
        }
        Serial.print(ModeString[2]);
        Serial.println(String(ADD0, BIN));
        ModeSelect=false;
      }
      
    case '3':
      if(Serial.available() > 0){
        DataIn = Serial.parseInt();
        for (int i=0 ; i < 7; i++){
          bitWrite(ADD1,i,bitRead(DataIn,i));
        }
        Serial.print(ModeString[3]);
        Serial.println(String(ADD1, BIN));
        ModeSelect=false;
      }
  }
}
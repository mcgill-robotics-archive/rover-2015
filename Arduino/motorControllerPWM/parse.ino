#include "register.h"

void update(int * value){
  if (Serial.available() > 0){
    Serial.println("1112");
    int dataIn = Serial.parseInt();
    *value = dataIn;
    ModeSelect = false;
    Serial.println("2");
  }
}

void parse(int mode){
  switch(mode){
    case 0:
    Serial.println("0");
    while(!Serial.available()){delay(10);}
    if(Serial.available() > 0){
      Serial.println("AG_SETPT");
      update(&Values::AG_SETPT);
      int bin = buildReg00();
      Serial.println(String(bin,BIN));
      }
      break; //end register update
     
     
     
     
     //default:
       //Serial.println("default");
       //break;
     
    case 1:
  Serial.println("1");
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        Serial.println("ENPOL");
        update(&Values::ENPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      }
     break; //end register update
     case 2:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        Serial.println("DIRPOL");
        update(&Values::DIRPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case 3:
      while(!Serial.available()){delay(10);}
    if(Serial.available() > 0){
        update(&Values::BRKPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
      /*
    case 4:
      if(Serial.available() > 0){
        update(&Values::SYNRECT);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case PWMF:
      if(Serial.available() > 0){
        update(&Values::PWMF);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case SPDMODE:
      if(Serial.available() > 0){
        update(&Values::SPDMODE);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case FGSEL:
      if(Serial.available() > 0){
        update(&Values::FGSEL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case BRKMOD:
      if(Serial.available() > 0){
        update(&Values::BRKMOD);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case RETRY:
      if(Serial.available() > 0){
        update(&Values::RETRY);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  ADVANCE:
      if(Serial.available() > 0){
        update(&Values::ADVANCE);
        int bin = buildReg01();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  SPDREVS:
      if(Serial.available() > 0){
        update(&Values::SPDREVS);
        int bin = buildReg02();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case MINSPD:
      if(Serial.available() > 0){
        update(&Values::MINSPD);
        int bin = buildReg02();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  BASIC:
      if(Serial.available() > 0){
        update(&Values::BASIC);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case SPEDTH:
      if(Serial.available() > 0){
        update(&Values::SPEDTH);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case MOD120:
      if(Serial.available() > 0){
        update(&Values::MOD120);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  LRTIME:
      if(Serial.available() > 0){
        update(&Values::LRTIME);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case HALLRST:
      if(Serial.available() > 0){
        update(&Values::HALLRST);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case DELAY:
      if(Serial.available() > 0){
        update(&Values::DELAY);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case AUTOADV:
      if(Serial.available() > 0){
        update(&Values::AUTOADV);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case AUTOGAIN:
      if(Serial.available() > 0){
        update(&Values::AUTOGAIN);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case ENSINE:
      if(Serial.available() > 0){
        update(&Values::ENSINE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case TDRIVE:
      if(Serial.available() > 0){
        update(&Values::TDRIVE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case DTIME:
      if(Serial.available() > 0){
        update(&Values::DTIME);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case IDRIVE:
      if(Serial.available() > 0){
        update(&Values::IDRIVE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  INTCLK:
      if(Serial.available() > 0){
        update(&Values::INTCLK);
        int bin = buildReg05();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case SPDGAIN:
      if(Serial.available() > 0){
        update(&Values::SPDGAIN);
        int bin = buildReg05();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  HALLPOL:
      if(Serial.available() > 0){
        update(&Values::HALLPOL);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case BYPFILT:
      if(Serial.available() > 0){
        update(&Values::BYPFILT);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case FILK1:
      if(Serial.available() > 0){
        update(&Values::FILK1);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  FILK2:
      if(Serial.available() > 0){
        update(&Values::FILK2);
        int bin = buildReg07();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  BYPCOMP:
      if(Serial.available() > 0){
        update(&Values::BYPCOMP);
        int bin = buildReg08();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case COMK1:
      if(Serial.available() > 0){
        update(&Values::COMK1);
        int bin = buildReg08();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  AA_SETPT:
      if(Serial.available() > 0){
        update(&Values::AA_SETPT);
        int bin = buildReg09();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case COMK2:
      if(Serial.available() > 0){
        update(&Values::COMK2);
        int bin = buildReg09();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case  OCPDEG:
      if(Serial.available() > 0){
        update(&Values::OCPDEG);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case OCPTH:
      if(Serial.available() > 0){
        update(&Values::OCPTH);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case OVTH:
      if(Serial.available() > 0){
        update(&Values::OVTH);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case VREG_EN:
      if(Serial.available() > 0){
        update(&Values::VREG_EN);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case LOOPGAIN:
      if(Serial.available() > 0){
        update(&Values::LOOPGAIN);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }

    case SPED:
      if(Serial.available() > 0){
        update(&Values::SPED);
        int bin = buildReg0B();
        Serial.println(String(bin,BIN));
      break; //end register update
      }

    case RLOCK:
      if(Serial.available() > 0){
        update(&Values::RLOCK);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case VMOV:
      if(Serial.available() > 0){
        update(&Values::VMOV);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case CPFAIL:
      if(Serial.available() > 0){
        update(&Values::CPFAIL);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case UVLO:
      if(Serial.available() > 0){
        update(&Values::UVLO);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case OTS:
      if(Serial.available() > 0){
        update(&Values::OTS);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case CPOC:
      if(Serial.available() > 0){
        update(&Values::CPOC);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    case OCP:
      if(Serial.available() > 0){
        update(&Values::OCP);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }*/
    }
}

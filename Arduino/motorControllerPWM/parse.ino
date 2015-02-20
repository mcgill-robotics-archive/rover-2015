#include "register.h"

void update(int * value){
  if (Serial.available() > 0){
    int dataIn = Serial.parseInt();
    *value = dataIn;
    ModeSelect = false;
  }
}

void parse(Registers mode){
  switch(mode){
    
    case AG_SETPT:
    while(!Serial.available()){delay(10);}
    if(Serial.available() > 0){
      Serial.println("AG_SETPT");
      update(&Values::AG_SETPT);
      int bin = buildReg00();
      Serial.println(String(bin,BIN));
      }
      break; //end register update
    
    case ENPOL:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        Serial.println("ENPOL");
        update(&Values::ENPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      }
     break; //end register update
    
     case DIRPOL:
     while(!Serial.available()){delay(10);}
     if(Serial.available() > 0){
        Serial.println("DIRPOL");
        update(&Values::DIRPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case BRKPOL:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::BRKPOL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case SYNRECT:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SYNRECT);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case PWMF:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::PWMF);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case SPDMODE:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SPDMODE);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case FGSEL:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::FGSEL);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case BRKMOD:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::BRKMOD);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case RETRY:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::RETRY);
        int bin = buildReg00();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  ADVANCE:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::ADVANCE);
        int bin = buildReg01();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  SPDREVS:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SPDREVS);
        int bin = buildReg02();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case MINSPD:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::MINSPD);
        int bin = buildReg02();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  BASIC:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::BASIC);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case SPEDTH:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SPEDTH);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case MOD120:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::MOD120);
        int bin = buildReg03();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  LRTIME:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::LRTIME);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case HALLRST:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::HALLRST);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case DELAY:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::DELAY);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case AUTOADV:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::AUTOADV);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case AUTOGAIN:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::AUTOGAIN);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case ENSINE:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::ENSINE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case TDRIVE:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::TDRIVE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case DTIME:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::DTIME);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case IDRIVE:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::IDRIVE);
        int bin = buildReg04();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  INTCLK:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::INTCLK);
        int bin = buildReg05();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case SPDGAIN:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SPDGAIN);
        int bin = buildReg05();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  HALLPOL:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::HALLPOL);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case BYPFILT:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::BYPFILT);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case FILK1:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::FILK1);
        int bin = buildReg06();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  FILK2:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::FILK2);
        int bin = buildReg07();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  BYPCOMP:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::BYPCOMP);
        int bin = buildReg08();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case COMK1:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::COMK1);
        int bin = buildReg08();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  AA_SETPT:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::AA_SETPT);
        int bin = buildReg09();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case COMK2:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::COMK2);
        int bin = buildReg09();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case  OCPDEG:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::OCPDEG);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case OCPTH:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::OCPTH);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case OVTH:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::OVTH);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case VREG_EN:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::VREG_EN);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case LOOPGAIN:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::LOOPGAIN);
        int bin = buildReg0A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }

    
    case SPED:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::SPED);
        int bin = buildReg0B();
        Serial.println(String(bin,BIN));
      break; //end register update
      }

    
    case RLOCK:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::RLOCK);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case VMOV:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::VMOV);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case CPFAIL:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::CPFAIL);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case UVLO:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::UVLO);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case OTS:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::OTS);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case CPOC:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::CPOC);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    
    case OCP:
    while(!Serial.available()){delay(10);}
      if(Serial.available() > 0){
        update(&Values::OCP);
        int bin = buildReg2A();
        Serial.println(String(bin,BIN));
      break; //end register update
      }
    }
}

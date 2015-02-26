/** 
 * Register parameters for DRV8308
 */

#include "register.h"

int buildReg00(){
    return Values::AG_SETPT << 12 | Values::ENPOL << 11 | Values::DIRPOL << 10 | Values::BRKPOL << 9 | Values::SYNRECT << 8 | Values::PWMF << 6 | Values::SPDMODE << 4 | Values::FGSEL << 2 | Values::BRKMOD << 1 | Values::RETRY;
}

int buildReg01 (){
    return Values::ADVANCE;
}

int buildReg02(){
    return Values::SPDREVS << 8 | Values::MINSPD;
}

int buildReg03(){
    return Values::BASIC << 15 | Values::SPEDTH << 12 | Values::MOD120;
}

int buildReg04(){
    return Values::LRTIME << 14 | Values::HALLRST << 12 | Values::DELAY << 11 | Values::AUTOADV << 10 | Values::AUTOGAIN << 9 | Values::ENSINE << 8 | Values::TDRIVE << 6 | Values::DTIME << 3 | Values::IDRIVE;
}

int buildReg05(){
    return Values::INTCLK << 12 | Values::SPDGAIN;
}

int buildReg06(){
    return Values::HALLPOL<< 15 | Values::BYPFILT << 12 | Values::FILK1;
}

int buildReg07(){
    return Values::FILK2;
}

int buildReg08(){
    return Values::BYPCOMP << 12 | Values::COMK1;
}

int buildReg09(){
    return Values::AA_SETPT << 12 | Values::COMK2;
}

int buildReg0A(){
    return Values::OCPDEG << 14 | Values::OCPTH << 12 | Values::OVTH << 11 | Values::VREG_EN << 10 | Values::LOOPGAIN;
}

int buildReg0B(){
    return Values::SPED;
}

int buildReg2A(){
    return Values::RLOCK << 6 | Values::VMOV << 5 | Values::CPFAIL << 4 | Values::UVLO << 3 | Values::OTS << 2 | Values::CPOC << 1 | Values::OCP;
}


void parseReg0B(int read){
    Values::SPED = read;
}

void parseReg2A(int read){
    Values::RLOCK = (read >> 6) & 1;
    Values::VMOV = (read >> 5) & 1;
    Values::CPFAIL = (read >> 4) & 1;
    Values::UVLO = (read >> 3) & 1;
    Values::OTS = (read >> 2) & 1;
    Values::CPOC = (read >> 1) & 1;
    Values::OCP = (read & 1);
}

void parseReg00(int read){
    Values::AG_SETPT = (read >> 12) & 15;
    Values::ENPOL = (read >> 11) & 1;
    Values::DIRPOL = (read >> 10) & 1;
    Values::BRKPOL = (read >> 9) & 1;
    Values::SYNRECT  =(read >> 8) & 1;
    Values::PWMF = (read >> 6) & 3;
    Values::SPDMODE = (read >> 4) & 3;
    Values::FGSEL = (read >> 2) & 3;
    Values::BRKMOD = (read >> 1) & 1;
    Values::RETRY = read & 1;

}

void parseReg01 (int read){
     Values::ADVANCE = read;
}

void parseReg02(int read){
     Values::SPDREVS  = (read >> 8 ) & 255;
     Values::MINSPD = (read ) & 255;
}

void parseReg03(int read){
    Values::BASIC = (read >> 15) & 1;
    Values::SPEDTH = (read >> 12) & 7;
    Values::MOD120= read & 4095;
}

void parseReg04(int read){
    Values::LRTIME = (read >> 14) & 3;
    Values::HALLRST = (read >> 12) & 3;
    Values::DELAY = (read >> 11) & 1;
    Values::AUTOADV = (read >> 10) & 1;
    Values::AUTOGAIN = (read >> 9) & 1;
    Values::ENSINE = (read >> 8) & 1;
    Values::TDRIVE = (read >> 6) & 3;
    Values::DTIME = (read >> 3) & 7;
    Values::IDRIVE = (read & 7);
}

void parseReg05(int read){
    Values::INTCLK = (read >> 12) & 7;
    Values::SPDGAIN= (read ) & 1023;
}

void parseReg06(int read){
    Values::HALLPOL= (read >> 15) & 1;
    Values::BYPFILT = (read >> 12) & 1;
    Values::FILK1= (read ) & 1023;
}

void parseReg07(int read){
    Values::FILK2 = (read ) ;
}

void parseReg08(int read){
    Values::BYPCOMP  = (read >> 12) & 1;
    Values::COMK1 = (read ) & 1023;
}

void parseReg09(int read){
    Values::AA_SETPT  = (read >> 12) & 15;
    Values::COMK2 = (read ) & 1023;
}

void parseReg0A(int read){
    Values::OCPDEG  = (read >> 14) & 3;
    Values::OCPTH  = (read >> 12) & 3;
    Values::OVTH  = (read >> 11) & 1;
    Values::VREG_EN  = (read >> 10) & 1;
    Values::LOOPGAIN = (read ) & 255;
}

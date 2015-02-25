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



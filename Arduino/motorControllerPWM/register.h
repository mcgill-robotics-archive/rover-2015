/** 
 * Register parameters for DRV8308
 */
#ifndef REGISTERS
#define REGISTERS

namespace Values {
    // 0x00
    int AG_SETPT;
    int ENPOL;
    int DIRPOL;
    int BRKPOL;
    int SYNRECT;
    int PWMF;
    int SPDMODE;
    int FGSEL;
    int BRKMOD;
    int RETRY;
    // 0x01;
    int ADVANCE;
    //0x02;
    int SPDREVS;
    int MINSPD;
    // 0x03
    int BASIC;
    int SPEDTH;
    int MOD120;
    // 0x04;
    int LRTIME;
    int HALLRST;
    int DELAY;
    int AUTOADV;
    int AUTOGAIN;
    int ENSINE;
    int TDRIVE;
    int DTIME;
    int IDRIVE;
    // 0x05;
    int INTCLK;
    int SPDGAIN;
    // 0x06
    int HALLPOL;
    int BYPFILT;
    int FILK1;
    // 0x07
    int FILK2;
    // 0x08
    int BYPCOMP;
    int COMK1;
    // 0x09
    int AA_SETPT;
    int COMK2;
    // 0x0A
    int OCPDEG;
    int OCPTH;
    int OVTH;
    int VREG_EN;
    int LOOPGAIN;
    // 0x0B
    int SPEED;
    // 0x2A
    int RLOCK;
    int VMOV;
    int CPFAIL;
    int UVLO;
    int OTS;
    int CPOC;
    int OCP;
}
/** 
 * Register parameters for DRV8308
 */
int buildReg00();
int buildReg01();
int buildReg02();
int buildReg03();
int buildReg04();
int buildReg05();
int buildReg06();
int buildReg07();
int buildReg08();
int buildReg09();
int buildReg0A();
int buildReg0B();
int buildReg2A();

#endif

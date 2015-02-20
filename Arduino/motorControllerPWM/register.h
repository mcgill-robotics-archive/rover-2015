/** 
 * Register parameters for DRV8308
 */
#ifndef REGISTERS
#define REGISTERS

enum Registers
{
       // 0x00
    AG_SETPT,
    ENPOL,
    DIRPOL,
    BRKPOL,
    SYNRECT,
    PWMF,
    SPDMODE,
    FGSEL,
    BRKMOD,
    RETRY,

    ADVANCE,

    SPDREVS,
    MINSPD,

    BASIC,
    SPEDTH,
    MOD120,

    LRTIME,
    HALLRST,
    DELAY,
    AUTOADV,
    AUTOGAIN,
    ENSINE,
    TDRIVE,
    DTIME,
    IDRIVE,

    INTCLK,
    SPDGAIN,

    HALLPOL,
    BYPFILT,
    FILK1,

    FILK2,

    BYPCOMP,
    COMK1,

    AA_SETPT,
    COMK2,

    OCPDEG,
    OCPTH,
    OVTH,
    VREG_EN,
    LOOPGAIN,

    SPED,

    RLOCK,
    VMOV,
    CPFAIL,
    UVLO,
    OTS,
    CPOC,
    OCP
}registers;


namespace Values {
    // 0x00
    int AG_SETPT = 0;
    int ENPOL = 0;
    int DIRPOL = 0;
    int BRKPOL = 0;
    int SYNRECT = 0;
    int PWMF = 0;
    int SPDMODE = 0;
    int FGSEL = 0;
    int BRKMOD = 0;
    int RETRY = 0;
    // 0x01;
    int ADVANCE = 0;
    //0x02;
    int SPDREVS = 0;
    int MINSPD = 0;
    // 0x03
    int BASIC = 0;
    int SPEDTH = 0;
    int MOD120 = 0;
    // 0x04;
    int LRTIME = 0;
    int HALLRST = 0;
    int DELAY = 0;
    int AUTOADV = 0;
    int AUTOGAIN = 0;
    int ENSINE = 0;
    int TDRIVE = 0;
    int DTIME = 0;
    int IDRIVE = 0;
    // 0x05;
    int INTCLK = 0;
    int SPDGAIN = 0;
    // 0x06
    int HALLPOL = 0;
    int BYPFILT = 0;
    int FILK1 = 0;
    // 0x07
    int FILK2 = 0;
    // 0x08
    int BYPCOMP = 0;
    int COMK1 = 0;
    // 0x09
    int AA_SETPT = 0;
    int COMK2 = 0;
    // 0x0A
    int OCPDEG = 0;
    int OCPTH = 0;
    int OVTH = 0;
    int VREG_EN = 0;
    int LOOPGAIN = 0;
    // 0x0B
    int SPED = 0;
    // 0x2A
    int RLOCK = 0;
    int VMOV = 0;
    int CPFAIL = 0;
    int UVLO = 0;
    int OTS = 0;
    int CPOC = 0;
    int OCP = 0;
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

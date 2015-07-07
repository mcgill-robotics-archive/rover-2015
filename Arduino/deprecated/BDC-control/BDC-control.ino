#include <SPI.h> //instantiate SPI library

//Register Addresses
const byte AR00 = B00000000;
const byte AR01 = B00000001;
const byte AR02 = B00000010;
const byte AR03 = B00000011;
const byte AR04 = B00000100;
const byte AR05 = B00000101;
const byte AR06 = B00000110;
const byte AR07 = B00000111;
const byte AR08 = B00001000;
const byte AR09 = B00001001;
const byte AR0A = B00001010;
const byte AR0B = B00001011;
const byte AR2A = B00101010;

//Read/Write Commands
const byte WRT = B10000000; 
const byte RED = B00000000;

//Register Values
int DR00 = 0;
int DR01 = 0;
int DR02 = 0;
int DR03 = 0;
int DR04 = 0;
int DR05 = 0;
int DR06 = 0;
int DR07 = 0;
int DR08 = 0;
int DR09 = 0;
int DR0A = 0;
int DR0B = 0;
int DR2A = 0;

//Control Pins
const int ENB = 4; //pin 25 - Active High
const int CLK = 5; //31 - Frequency Matching Clock Input
const int RST = 10; //30 - Active High
const int DIR = 9 //29 - 0 Is Forward / 1 Is Reverse
const int BRK = 8 //28 - Active High
const int LCK = 6 //27 - Input Only, Low When Speed Is Locked
const int FLT = 12 //26 - Input Only, Fault Indication
const int SCS = 7; //1 - Slave Select, Active High

//Setting Values
byte Retry = B00000000;
byte BRKMOD = B00000000;
byte FGSEL = B00000000;
byte SPDMODE = B00000000;
byte PWMF = B00000000;
byte SYNRECT = B00000000;
byte BRKPOL = B00000000;
byte DIRPOL = B00000000;
byte ENPOL = B00000000;
byte AG_SETPT = B00000000;
byte ADVANCE = B00000000;
byte MINSPD = B00000000;
byte SPDREVS = B00000000;
byte SPEEDTH = B00000000;
byte BASIC = B00000000;
byte IDRIVE = B00000000;
byte DTIME = B00000000;
byte TDRIVE = B00000000;
byte ENSINE = B00000000;
byte AUTOGAIN = B00000000;
byte AUTOADVANCE  = B00000000;
byte DELAY = B00000000;
byte HALLRST = B00000000;
byte LRTIME = B00000000;
byte INTCLK = B00000000;
byte BYPFILT = B00000000;
byte HALLPOL = B00000000;
byte BYPCOMP = B00000000;
byte AA_SETPT = B00000000;
byte VREG_EN = B00000000;
byte OVTH = B00000000;
byte OCPTH = B00000000;
byte OCPDEG = B00000000;
byte OCP = B00000000;
byte CPOC = B00000000;
byte OTS = B00000000;
byte UVLO = B00000000;
byte CPFAIL = B00000000;
byte VMOV = B00000000;
byte RLOCK = B00000000;
int MOD120 = 0;
int SPDGAIN = 0;
int FILK1 = 0;
int FILK2 = 0;
int COMK1 = 0;
int COMK2 = 0;
int LOOPGAIN = 0;
int SPEED = 0;





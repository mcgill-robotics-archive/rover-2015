#include <Servo.h> 
#include <SPI.h>
#include <SoftwareSerial.h>
#include "L6470.h"  // include the register and bit definitions


#define rx 5
#define tx 6

#define SLAVE_SELECT_PIN 50  // Wire this to the CSN pin
#define dSPIN_RESET      53  // Wire this to the STBY line
#define dSPIN_BUSYN      52  // Wire this to the BSYN line

#define PH_STEPS 20
#define BIO_STEPS 30
#define BIO_READING_STEPS 10

#define SAMPLE_ITERATIONS 5.0

#define BIO_COLLECTION_TIME_ms 1000

Servo base; 
char junk = ' ';
SoftwareSerial pH_probe(rx,tx);
int home = 0;
double dry_1 = 360/14, wet_1 = 2*360/14, wet_2 = 3*360/14, wet_3 = 4*360/14, dry_3 = 5*360/14, dry_2 = 6*360/14;
int pos = 0; 

//copypasta from sparkfun website
char ph_data[20];              
char computerdata[20];         
byte received_from_computer=0; 
byte received_from_sensor=0;   
byte arduino_only=0;          
byte startup=0;                
float ph=0;                    
byte string_received=0;        

void setup() { 
  Serial.begin(9600);
  pH_probe.begin(9600);
  setupStepperScience();
  base.write(home);
} 

void setupStepperScience()
{
    
  dSPIN_init();
  dSPIN_SetParam(dSPIN_STEP_MODE,
                 !dSPIN_SYNC_EN |
                 dSPIN_STEP_SEL_1_8 |
                 dSPIN_SYNC_SEL_1);
  dSPIN_SetParam(dSPIN_MAX_SPEED, MaxSpdCalc(750));
  dSPIN_SetParam(dSPIN_FS_SPD, 0x3FF);
  dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_6000mA);
  dSPIN_SetParam(dSPIN_CONFIG,
                 dSPIN_CONFIG_PWM_DIV_1 |
                 dSPIN_CONFIG_PWM_MUL_2 |
                 dSPIN_CONFIG_SR_180V_us |
                 dSPIN_CONFIG_OC_SD_ENABLE |
                 dSPIN_CONFIG_VS_COMP_DISABLE |
                 dSPIN_CONFIG_SW_HARD_STOP |
                 dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(dSPIN_KVAL_HOLD, 0x2F);
  dSPIN_SetParam(dSPIN_KVAL_RUN, 0x43);
  dSPIN_SetParam(dSPIN_KVAL_ACC, 0x43);
  dSPIN_SetParam(dSPIN_KVAL_DEC, 0x43);
  dSPIN_SetParam(dSPIN_ST_SLP, 0x1B);
  dSPIN_SetParam(dSPIN_INT_SPD, 0x39E6);
  dSPIN_SetParam(dSPIN_FN_SLP_ACC, 0x2E);
  dSPIN_SetParam(dSPIN_FN_SLP_DEC, 0x2E);
  dSPIN_GetStatus();
}

void loop() { 
  if (Serial.available() > 0)
  {
     char cmd = Serial.read();
     if (cmd == 'a')
     {
      acidityTests();
     }
     else if (cmd == 'b')
     {
      while (Serial.available() < 1);
      int unit = Serial.parseInt();
      bioTest(unit);
     }
     else if (cmd == 'm')
     {
      moistureTest();
     }
     else if (cmd == 'r')
     {
      returnHome();
     }
     else if (cmd == 'd')
     {
      while (Serial.available() < 1);
      int unit = Serial.parseInt();
      dry(unit);
     }
     else if (cmd == 'w')
     {
      while (Serial.available() < 1);
      int unit = Serial.parseInt();
      wet(unit);
     }
  }
  else delay(10);
}

void returnHome(){
  base.write(home);
}

void dry(int i)
{
  if (i == 1)
  {
    base.write(dry_1);
  }
  else if (i == 2)
  {
    base.write(dry_2);
  }
  else if (i == 3)
  {
    base.write(dry_3);
  }
}

void wet(int i)
{
  if (i == 1)
  {
    base.write(wet_1);
  }
  else if (i == 2)
  {
    base.write(wet_2);
  }
  else if (i == 3)
  {
    base.write(wet_3);
  }
}

void moistureTest(){
  int sensorValue = analogRead(A0);
  Serial.println(sensorValue);
  delay(100);
}


void positionWriterTest(){
  for(int posi = 0; posi < 360; posi+=10){
    base.write(posi);
    delay(500);
  }
}

void collectSamples(){
  //Note to debuggers: While-loops are doubled because program was skipping every other,
  //limited understanding of arduino forced me to make this dirty hack
  
  //Dry sample site #1
  while (Serial.available() == 0);
  base.write(dry_1);
  Serial.println("moved to dry base 1");
  while (Serial.available() != 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(dry_1);
  Serial.println("moved to dry base 1");
  while (Serial.available() != 0){
    junk = Serial.read(); 
  }
  
  //Wet sample site #1
  while (Serial.available() == 0);
  base.write(wet_1);
  Serial.println("moved to wet base 1");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(wet_1);
  Serial.println("moved to wet base 1");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  //Home for traveling
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }

  //Dry sample site #2
  while (Serial.available() == 0);
  base.write(dry_2);
  Serial.println("moved to dry base 2");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(dry_2);
  Serial.println("moved to dry base 2");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  //Wet sample site #2
  while (Serial.available() == 0);
  base.write(wet_2);
  Serial.println("moved to wet base 2");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(wet_2);
  Serial.println("moved to wet base 2");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  //Home
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  //Dry sample site #3
  while (Serial.available() == 0);
  base.write(dry_3);
  Serial.println("moved to dry base 3");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(dry_3);
  Serial.println("moved to dry base 3");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  //Wet sample site #3
  while (Serial.available() == 0);
  base.write(wet_3);
  Serial.println("moved to wet base 3");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(wet_3);
  Serial.println("moved to wet base 3");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  
  
  //Home
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
  while (Serial.available() == 0);
  base.write(home);
  Serial.println("moved home");
  while (Serial.available() > 0){
    junk = Serial.read(); 
  }
}

void acidityTests(){
  while (Serial.available() == 0);
  base.write(wet_1);
  dSPIN_Move(FWD, PH_STEPS);
  float pH_1 = getAveragedPH();
  Serial.println((String)pH_1);
  dSPIN_Move(REV, PH_STEPS);
  
  base.write(wet_2);
  dSPIN_Move(FWD, PH_STEPS);
  float pH_2 = getAveragedPH();
  Serial.println((String)pH_2);
  dSPIN_Move(REV, PH_STEPS);
  
  base.write(wet_3);
  dSPIN_Move(FWD, PH_STEPS);
  float pH_3 = getAveragedPH();
  Serial.println((String)pH_3);
  dSPIN_Move(REV, PH_STEPS);
  
}

float getAveragedPH(){
  float pH = 0;
  for(int i = 0; i < SAMPLE_ITERATIONS; i++){
    if(pH_probe.available() > 0){
      received_from_sensor=pH_probe.readBytesUntil(13,ph_data,20);
      ph_data[received_from_sensor]=0;
      string_received=1;
      float current_pH = atof(ph_data);
      pH += current_pH;
    }   
    delay(200);
  }
  return (pH / SAMPLE_ITERATIONS);
}

void bioTest(int input){
  if(input == 1){
    base.write(wet_1);
  } else if (input == 2){
    base.write(wet_2);
  } else if (input == 3){
    base.write(wet_3);
  } else {
    Serial.println("bad input");
  }
  
  dSPIN_Move(FWD, BIO_STEPS);
  delay(BIO_COLLECTION_TIME_ms);
  dSPIN_Move(REV, BIO_STEPS);
  base.write(home);
  dSPIN_Move(FWD, BIO_STEPS);
  delay(4000);
  dSPIN_Move(REV, BIO_STEPS);
  
}

void cal_s(){                        //calibrate to a pH of 7
  pH_probe.print("cal,mid,7\r");}    //send the "cal,mid,7" command to calibrate to a pH of 7.00


void cal_f(){                       //calibrate to a pH of 4 
  pH_probe.print("cal,low,4\r");}     //send the "cal,low,4" command to calibrate to a pH of 4.00 


void cal_t(){                      //calibrate to a pH of 10.00
  pH_probe.print("cal,high,10\r");}  //send the "cal,high,10" command to calibrate to a pH of 10.00  


void phFactoryDefault(){           //factory defaults the pH circuit
  pH_probe.print("X\r");}          //send the "X" command to factory reset the device 


void read_info(){                  //get device info
    pH_probe.print("I\r");}        //send the "I" command to query the information


void phSetLEDs(byte enabled)      //turn the LEDs on or off
{
  if(enabled)                     //if enabled is > 0 
    pH_probe.print("L,1\r");      //the LED's will turn ON 
  else                            //if enabled is 0        
    pH_probe.print("L,0\r");      //the LED's will turn OFF
}


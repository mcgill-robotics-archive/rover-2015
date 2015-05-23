#include <Servo.h> 
#include <Servo.Serial>
#include <Stepper.h>
#include <SoftwareSerial.h>

#define BASE_ROTATE 9

#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4

#define rx 5
#define tx 6

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
Stepper prober = Stepper(720,STEP_1,STEP_2,STEP_3,STEP_4);

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
  pinMode(STEP_1,OUTPUT);
  pinMode(STEP_2,OUTPUT);
  pinMode(STEP_3,OUTPUT);
  pinMode(STEP_4,OUTPUT);
  
  base.attach(BASE_ROTATE); // attaches the servo on pin 9 to the servo object 
  base.write(0);
} 

void loop() { 
  //positionWriterTest();
  //turnBySection();
  //moistureTest();
  
  
  
  collectSamples();
  
  /*acidityTests();
  
  Serial.println("Which site's sample would you like to test? (Enter 1-3)");
  while (Serial.available() == 0);
  int input = Serial.parseInt();
  
  bioTest(input);
  */
  
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
  prober.step(PH_STEPS); //TODO: Find out if the program waits until movement is finished to start next line
  float pH_1 = getAveragedPH();
  prober.step(-PH_STEPS);
  
  base.write(wet_2);
  prober.step(PH_STEPS);
  float pH_2 = getAveragedPH();
  prober.step(-PH_STEPS);
  
  base.write(wet_3);
  prober.step(PH_STEPS);
  float pH_3 = getAveragedPH();
  prober.step(-PH_STEPS);
  
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
  
  prober.step(BIO_STEPS);
  delay(BIO_COLLECTION_TIME_ms);
  prober.step(-BIO_STEPS);
  base.write(home);
  prober.step(BIO_READING_STEPS);
  delay(4000);
  prober.step(-BIO_READING_STEPS);
}

void cal_s(){                        //calibrate to a pH of 7
  myserial.print("cal,mid,7\r");}    //send the "cal,mid,7" command to calibrate to a pH of 7.00


void cal_f(){                       //calibrate to a pH of 4 
  myserial.print("cal,low,4\r");}     //send the "cal,low,4" command to calibrate to a pH of 4.00 


void cal_t(){                      //calibrate to a pH of 10.00
  myserial.print("cal,high,10\r");}  //send the "cal,high,10" command to calibrate to a pH of 10.00  


void phFactoryDefault(){           //factory defaults the pH circuit
  myserial.print("X\r");}          //send the "X" command to factory reset the device 


void read_info(){                  //get device info
    myserial.print("I\r");}        //send the "I" command to query the information


void phSetLEDs(byte enabled)      //turn the LEDs on or off
{
  if(enabled)                     //if enabled is > 0 
    myserial.print("L,1\r");      //the LED's will turn ON 
  else                            //if enabled is 0        
    myserial.print("L,0\r");      //the LED's will turn OFF
}


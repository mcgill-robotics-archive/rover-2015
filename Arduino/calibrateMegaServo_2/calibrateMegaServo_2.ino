
//Accepts a number between 800 and 2200 via serial and sends it to the servo
//I've explained the mapping of these numbers to angles at the bottom

#include <Servo.h> 

//actual servos
Servo lr;
Servo lf;
Servo rf;
Servo rr;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  //myservo.attach(9);

//these are just sent to defaults so we dont get in the way of the servos  
  analogWrite(6, 0);  //suspension actuators extended ('up')
  analogWrite(7, 0);  //dumpActuator retracted 'down'
  //analogWrite(8, 10);    //auger stopped
  analogWrite(13, 255);  //door actuators retracted, 'closed'

          
  // servo pins valid Wed May 8 night
  
  rf.attach(3);
  rr.attach(5);
  lr.attach(4);
  lf.attach(2);
  
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    int val = Serial.parseInt();
    
    if (val == 1){          // This is a 1 degree of rotation
    mCommand(2200);
    delay(2600);
    mCommand(1500);  
    } else if(val >2200){    // If written more then 2200, by writting 10000 = 1second, 20000 = 2 second and ... of delay time.
    mCommand(2200);
    delay(val);
    mCommand(1500); 
    } else{
    mCommand(val);
    }
  }
  delay(10);
}

void mCommand(int num){
    
    Serial.println(num);
    
    int RF_OFFSET = 0;
    int RR_OFFSET = 0;    
    int LR_OFFSET = 0;
    int LF_OFFSET = 0;    
    
    int rf_cmd = num+RF_OFFSET;
    int rr_cmd = num+RR_OFFSET;
    int lr_cmd = num+LR_OFFSET;
    int lf_cmd = num+LF_OFFSET;
    
    rf.writeMicroseconds(rf_cmd);
    rr.writeMicroseconds(rr_cmd);
    lr.writeMicroseconds(lr_cmd);
    lf.writeMicroseconds(lf_cmd);
  
}

void timeDelay(int time){
  
}
/*

HOW WRITEMICROSECONDS() WORKS WITH MEGASERVOS
---------------------------------------------


servo.attach() writes 93º, or approx 1500 usec
writemicroseconds() takes an argument sort of between 0 and 3000

Numbers between approx 900 and 2100 correspond to 0 and 180 physical degrees, respectively, but they write differently if you query Servo.read() you get something like 44 to 151 degrees

numbers just outside of that range correspond to continuous rotation in opposite directions.

800 will rotate continuously CCW, 2200 will rotate continuously CW

If spinning CCW after an 800 command, 900 will stop it in spot (with a small delay for communication)
if spinning CW after a 2200 command, 2100 will stop it in spot

If you mix and match the above, it will spin 180 deg. and then stop.

Numbers just outside of the 800-2100 range correspond to slow continuous rotation

*/








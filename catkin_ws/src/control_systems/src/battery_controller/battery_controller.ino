//This is a general battery controller
int voltage = 0;

//Pin for the reading of the battery voltage
int batteryPin = 10;


void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){}
  
  //Setup for raeding battery
  pinMode(batteryPin,INPUT);
}

void loop()
{
  //Listen until message is passed
  if (Serial.available())
  {
    //Check a message
    if(Serial.read()==167)
    {
      ///////////
      //May be a source of error in the future
      //once the 
      ///////////
      char message[13];
      message[0] = Serial.read();
      message[1] = Serial.read();
      message[2] = Serial.read();
      message[3] = Serial.read();
      message[4] = Serial.read();
      message[5] = Serial.read();
      message[6] = Serial.read();
      message[7] = Serial.read();
      message[8] = Serial.read();
      message[9] = Serial.read();
      message[10] = Serial.read();
      message[11] = Serial.read();
      //termination character
      message[12] = Serial.read();
      //Make sure message has correct structure
      if (message[0] == 0 && message[2] == 1 &&
          message[4] == 2 && message[6] == 3 &&
          message[8] == 4 && message[10]== 5)
      {
      //Check if address is for batteries
      if (message[1] == 41)
      {
        //Switch for the functions 
        switch(message[3])
        {
         case  5: //set angle function
           int result = 0;
           for (int n = 0; n < 4 ;n++)
           {
             result = (result << 8) + message[5+2*n];
           }
           //bound check
           double angle = (double)result/10.;
           //final bound check
           if (angle <= top3 && angle >= bottom3)
           {
             setpoint3 = angle;
           }
           break;
        }
        //Perform some calculation to calculate angle from the message
        //Check bounds
      }
      else if (message[1] == 14)
      {
        //Switch for the functions 
        switch(message[3])
        {
         case  5: //set angle function
           int result = 0;
           for (int n = 0; n < 4 ;n++)
           {
             result = (result << 8) + message[5+2*n];
           }
           //bound check
           double angle = (double)result/10.;
           //final bound check
           if (angle <= top4 && angle >= bottom4)
           {
             setpoint4 = angle;
           }
           break;
        }
      }
      }
      
    }
  }
  
}

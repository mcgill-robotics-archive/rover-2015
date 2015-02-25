
int d = 0;            // pwm value

void setup()

{
  //  16000000 / (1 * 200 * 2 ) = 40 KHz
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){
    delay(1);
  }

  TCCR1A = _BV (WGM10) | _BV (WGM11) | _BV (COM1B1);  // Phase Correct
  TCCR1B = _BV (WGM13) | _BV (CS10);                  // Phase Correct / Prescale 1
  OCR1A = 200;                                        // Sets Top to correspond to frequency 
  OCR1B = 0;                                          // Sets duty cycle (duty cycle = 0CR1B/OCR1A)			
//OCR1B is the scaled speed we want out of 200
  pinMode(10, OUTPUT);           


}  

void loop()

{ 
  if (Serial.available()>0){
    OCR1B = Serial.parseInt();

  }

}


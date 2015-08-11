

int InA1 = 40;
int InB1 = 41;
int InA2 = 46;
int InB2 = 47;
int PWM1 = 2;  
int PWM2 = 3;  
int PWM1_val = 255; //(25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM2_val = 255; //(25% = 64; 50% = 127; 75% = 191; 100% = 255)

void setup() {
Serial.begin(9600);
pinMode(InA1, OUTPUT);
pinMode(InB1, OUTPUT);
pinMode(InA2, OUTPUT);
pinMode(InB2, OUTPUT);
pinMode(PWM1, OUTPUT);
pinMode(PWM2, OUTPUT);
}
void loop() {
  if(Serial.available() > 0){
    String inputC = Serial.readString();
    if(inputC.charAt(0) == 'L'){
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    }else if(inputC.charAt(0) == 'R'){
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    }else {
      Serial.print("Wrong command!");
    }
    
analogWrite(PWM1, PWM1_val);
analogWrite(PWM2, PWM2_val);
  }
}

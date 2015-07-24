String numberStep;
int steps = 0;
int speeds = 0;

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  Serial.println("Start with 'R' for Right and 'L' for Left.");
  Serial.println("Step from 000 to 900.");
  Serial.println("Speed from 0200 to 6000");

}

void loop() {
  if (Serial.available() > 0){
    //Reads number of count wanted
    numberStep = Serial.readString();
    
    //Change the number step string into int value
    if(numberStep.charAt(1) == '0'){
      if(numberStep.charAt(2) == '0'){
          steps = numberStep.substring(3,4).toInt();
      } else {
        steps = numberStep.substring(2,4).toInt();
      }
    } else {
        steps = numberStep.substring(1,4).toInt();
    }
    if(numberStep.charAt(4) == '0'){
    speeds = numberStep.substring(5,8).toInt();
    } else{
    speeds = numberStep.substring(4,8).toInt();
    }  
    
    
    //Command lines
    if(numberStep.charAt(0) == 'L'){
      digitalWrite(8, LOW);
      Serial.print("LEFT; ");
      Serial.print(steps);
      Serial.print("; ");
      Serial.println(speeds);
      
    } else if(numberStep.charAt(0) == 'R'){
      digitalWrite(8, HIGH);
      Serial.print("RIGHT; ");
      Serial.print(steps);
      Serial.print("; ");
      Serial.println(speeds);
      
    } else{
      Serial.print("Error ");
    }
    
    for (int i=0; i < steps; i++){
      digitalWrite(9, HIGH);
      delayMicroseconds(speeds);
      digitalWrite(9, LOW);
      delayMicroseconds(speeds);
    }
  }
  /*
      digitalWrite(9, HIGH);
      delayMicroseconds(speeds);
      digitalWrite(9, LOW);
      delayMicroseconds(speeds);
  */

}

void setup()
{
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  delay(50);
}

void loop()
{
  digitalWrite(8, LOW);
  delayMicroseconds(1000);
  digitalWrite(8, HIGH);
  delayMicroseconds(1000);  
}

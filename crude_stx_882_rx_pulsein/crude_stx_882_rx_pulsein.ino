

void setup() {
  Serial.begin(9600);
  pinMode(10, INPUT);
  

}

void loop() {
  

  //Serial.println(analogRead(10));
unsigned long val=pulseIn(10,HIGH);
Serial.println(val);
delay(200);
}

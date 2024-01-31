
#define TX_PIN 9
#define SIG_T 2000uL
void setup() {
pinMode(TX_PIN,OUTPUT);
}

void loop() {

  digitalWrite(TX_PIN,HIGH);

  delayMicroseconds(SIG_T);
  digitalWrite(TX_PIN,LOW);
  delayMicroseconds(SIG_T);
}

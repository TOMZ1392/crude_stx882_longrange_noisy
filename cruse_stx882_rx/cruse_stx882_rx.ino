
const int interruptPin = 10;
volatile bool fallDetected=0;
uint64_t time_past,interval;
static void signalIntrptHdlr()
{
  interval=micros()-time_past;
  fallDetected=1;
  time_past=micros();
}
void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin),  signalIntrptHdlr, FALLING);
time_past=micros();
}

void loop() {
  
if(fallDetected)
{
  fallDetected=0;
  Serial.print("R :");
  Serial.println((uint32_t)interval);
  }
  //Serial.println(analogRead(10));
  unsigned long val=pulseIn(10,HIGH);
Serial.println(val);
}



#include <Wire.h>
//#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MOTOR_RELAY_PIN  7
#define BUZZER_PIN   9

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

const int interruptPin = 2;
volatile bool fallDetected=0,startAvgCalc=0;
volatile uint8_t intBuf[20];
volatile uint8_t bufWrIdx;

uint64_t time_past,interval;

static void signalIntrptHdlr()
{
 
  interval=micros()-time_past;
  if(bufWrIdx <20)
  {
    intBuf[bufWrIdx]=(interval>>10) + 1;
    bufWrIdx++;
  }
  else
  {
    startAvgCalc=1;
    }
 
  fallDetected=1;
  time_past=micros();
}

void showDataNumeric(uint32_t val,uint8_t col)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(col, 0);
  
  display.print(val);
   display.display();
}
void showDataStr(String val,uint8_t col)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(col, 0);
  
  display.print(val);
   display.display();
}

void setup() {
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.print(65535);
  display.display();
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin),  signalIntrptHdlr, FALLING);
  pinMode(BUZZER_PIN,OUTPUT);
pinMode(MOTOR_RELAY_PIN,OUTPUT);

digitalWrite(MOTOR_RELAY_PIN,1);
delay(2000);
digitalWrite(MOTOR_RELAY_PIN,0);
time_past=micros();

}

volatile long motorRunTimer=0;
volatile bool motorTimerIsRunning=0;
volatile uint16_t runDurnSec=0;
#define TIMEOUT_S  2000
void startMotorRunTmr()
{
  motorRunTimer=millis();
  runDurnSec=0;
  motorTimerIsRunning=1;
}

bool monitorMotorTimer()
{
  bool retVal=true;
  if(motorTimerIsRunning)
  {
    if(millis() - motorRunTimer >=1000)
    {
      motorRunTimer=millis();
      runDurnSec++; 
      showDataNumeric(runDurnSec,20);
      
      }
    if(runDurnSec >= TIMEOUT_S)
    {
    retVal=false;
    motorTimerIsRunning=0;
    runDurnSec=0;
    }
  }
    return retVal;
}

#define BUZZER_TONE_INTERVAL   700
#define BUZZER_HIGH_TIME_1 300
#define BUZZER_LOW_TIME_1 50
#define BUZZER_HIGH_TIME_2 200
#define BUZZER_LOW_TIME_2 50
/* Let everyone know when the water pump turn on is requested*/
void generateAlarm()
{
  static bool hightone = false;
  static long startTime = millis();
  /* switch between tones every BUZZER_TONE_INTERVAL ms*/
  if (millis() - startTime <= BUZZER_TONE_INTERVAL) {
    if (hightone) {
      digitalWrite(BUZZER_PIN, 1);
      delayMicroseconds(BUZZER_HIGH_TIME_1);
      digitalWrite(BUZZER_PIN, 0);
      delayMicroseconds(BUZZER_LOW_TIME_1);

    }
    else {
      digitalWrite(BUZZER_PIN, 1);
      delayMicroseconds(BUZZER_HIGH_TIME_2);
      digitalWrite(BUZZER_PIN, 0);
      delayMicroseconds(BUZZER_LOW_TIME_2);

    }

  }
  else {
    if (hightone) {
      hightone = false;
    } else {
      hightone = true;
    }
    startTime = millis();
  }

}
void stopAlarm()
{
  analogWrite(BUZZER_PIN,0);
}

uint32_t lostctr=0;

void startMotor()
{
  digitalWrite(MOTOR_RELAY_PIN,1);
}

void stopMotor()
{
  digitalWrite(MOTOR_RELAY_PIN,0);
}

void loop() 
{
  static bool motorState=0;

  long durn=pulseIn(interruptPin,LOW);
  Serial.println(durn);
  if(motorState)
  {
    showDataStr("Motor ON",10);
    generateAlarm(); 
    if(!monitorMotorTimer())
    {
      motorState=false;
      stopAlarm();
      stopMotor();
      
      }
    }
    else{
      
      showDataStr("Motor OFF",10);
      
      }
    

 
if(fallDetected)
{
  fallDetected=0;
  //Serial.print("R :");
  //Serial.println((uint32_t)interval);
  showDataNumeric((uint32_t)interval,0);
  }
  
  if(startAvgCalc)
  {
    lostctr=0;
    uint8_t i=0;
    uint32_t sum=0;
    uint8_t ctr_h=0;
    uint8_t ctr_l=0;
    for(;i<20;i++)
    {
      uint8_t tmpVal=intBuf[i];
     // Serial.print(tmpVal);
      showDataNumeric(tmpVal,0);
     // Serial.print(" ");
      if(tmpVal>17 && tmpVal<23)
      {
        ctr_l++;
        }
        if(tmpVal>8 && tmpVal<12)
      {
        ctr_h++;
        }
      }
     // Serial.println("");
    if(ctr_l>10 || ctr_h>10 )
    {
        showDataStr("Sigood",10);
        

      }
      else
      {
         showDataStr("SigBad",10);
        }

        if(ctr_l>10)
        {
          motorState=0;
          stopMotor();
          stopAlarm();
          }

          if(ctr_h>10)
          {
            motorState=1;
            startMotor();
            startMotorRunTmr();
           }
    startAvgCalc=0;
    bufWrIdx=0;
    time_past=micros();
    }
    else{
      lostctr++;
      }
      if(lostctr > 1000000){
        //showDataStr("SigLost");
        }
 
}

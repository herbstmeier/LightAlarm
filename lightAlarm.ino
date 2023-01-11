#include "RTClib.h"
#include <EEPROM.h>

#define ADDRESS 0

//Initializing Pins
#define alarmEn 5       //global alarm on off switch
#define lightEn 10      //turn on light
#define lightDim A7     //dimming amount input
#define ledPwm 9        //led is controlled by pwm pin for dimming
#define buttonPin 2     //button of rotary encoder
#define rotaryA 3       //rotary encoder A phase
#define rotaryB 4       //rotary encoder B phase
#define shiftD 7        //shift register data
#define shiftCLK 6      //shift register clock
#define shiftSTR 8      //shift register strobe
#define numA A0         //enable first digit
#define numB A1         //enable second digit
#define numC A2         //enable third digit
#define numD A3         //enable fourth digit

#define ledOffset 100

int nums[4] = {A0,A1,A2,A3};

byte seg[] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};

DateTime now;

RTC_DS3231 rtc;

//alarm time
volatile int aHour = 8;    //displayed hour
volatile int aMinute = 0;  //displayed minutes
//real alarm time is 30mins earlier
volatile int rHour = 0;    //real hour
volatile int rMinute = 0;  //real minutes

DateTime fadeEnd;

//status: 0 = off, 1 = fading in, 2 = static
volatile int alarmState = 0;
//status: 0 = off, 1 = show time, 2 = show alarm, 3 = edit hours, 4 = edit minutes
volatile int displayState = 0;
int oldDS = 0;

boolean daylightSavingsState = false;

DateTime hm2dt(int h, int m) {
  return DateTime(now.year(), now.month(), now.day(), h, m, 0);
}

boolean isDaylightSavings(DateTime t, byte tzHours)
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and timezoneHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{
  int year = t.year();
  int month = t.month();
  int hour = t.hour();
  int day = t.day();
  
  if (month < 3 || month > 10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
  if (month > 3 && month < 10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
  if ((month == 3 && (hour + 24 * day) >= (1 + tzHours + 24 * (31 - (5 * year / 4 + 4) % 7))) || (month == 10 && (hour + 24 * day) < (1 + tzHours + 24 * (31 - (5 * year / 4 + 1) % 7))))
    return true;
  else
    return false;
}

void calcAlarm() {
  rHour = aHour;
  rMinute = aMinute - 30;
  if (rMinute < 0) {
    rMinute = 60 + rMinute;
    rHour--;
  }

  //Serial.print("New alarm time: " + rHour);
  //Serial.println("." + rMinute);
}

void printTime(DateTime dt) {
  Serial.print("current time: ");
  Serial.print(dt.hour(), DEC);
  Serial.print(".");
  Serial.println(dt.minute(), DEC);
}

void changeState() {  
  if (alarmState != 0) {
    Serial.println("resetting alarm.");
    alarmState = 0; //if button pressed -> reset to alarm off
    displayState = oldDS; //set display to state before alarm
  } else {
    switch(displayState) {
      
      case 2: //show alarm
        displayState = 0;
        break;

      case 4: //edit minutes
        displayState = 2;
        EEPROM.update(ADDRESS,aHour);
        EEPROM.update(ADDRESS+1,aMinute);
        calcAlarm();
        break;
      
      default:
        displayState++;
        break;
    }
  }
  /*Serial.print("alarmEn: ");
  Serial.print(digitalRead(alarmEn));
  Serial.print(", lightEn: ");
  Serial.print(digitalRead(lightEn));
  Serial.print(" | displayMachine status changed: ");
  Serial.println(displayState, DEC);*/
}

void getEncoderDir() {
  if (displayState > 2) {  
    //get rotary encoder states
    int stateA = digitalRead(rotaryA);
    int stateB = digitalRead(rotaryB);
    //Serial.print("A = ");Serial.print(stateA);Serial.print(" | B = ");Serial.println(stateB);

    if (stateA != stateB) {
      //ccw
      if (displayState == 3) {
        aHour--;
        if (aHour < 0)
          aHour = 23;
      } else {
        aMinute--;
        if (aMinute < 0)
          aMinute = 59;
      }
    } else {
      //cw
      if (displayState == 3) {
        aHour++;
        if (aHour > 23)
          aHour = 0;
      } else {
        aMinute++;
        if (aMinute > 59)
          aMinute = 0;
      }
    }
  } else if (displayState == 2) {
    displayState = 3; //change to edit mode
  }
}

void alarmMachine() {
  int nhour = now.hour();

  if (daylightSavingsState) {
    nhour++;
    if (nhour > 23) {
      nhour = nhour - 24;
    }
  }

  switch (alarmState) {

    case 0: //off
      //check for alarm
      //Serial.println("STATUS 0");
      if ((nhour == rHour) && (now.minute() == rMinute) && (now.second() == 0)) {
        Serial.println("alarm ongoing.");
        alarmState = 1; //alarm -> switch to fading
        oldDS = displayState;
        displayState = 1; //show time
        fadeEnd = hm2dt(aHour, aMinute);
      } else {
        //no alarm -> leds off
        digitalWrite(ledPwm, LOW);
      }
      break;

    case 1: //fading in
      //Serial.println("STATUS 1");
      // check for fade end
      if ((nhour == aHour) && (now.minute() == aMinute) && (now.second() == 0)) {
        alarmState = 2; //fade end -> switch to static
      } else {
        //no fade end -> continue fade in
        double frac = (1800 - fadeEnd.unixtime() + now.unixtime()) / 1800.0;
        //Serial.println(frac);
        analogWrite(ledPwm, frac * 255);
      }
      break;

    case 2: //static
      //Serial.println("STATUS 2");
      digitalWrite(ledPwm, HIGH);
      break;

    default: //leds off
      digitalWrite(ledPwm, LOW);
      break;
  }
}

void pushToReg(int digit, bool dot) {
  byte out = seg[digit];
  if (dot) {
    //Serial.println(out,BIN);
    bitSet(out,7); //add dot to digit byte
    //Serial.println(out,BIN);
  }
  
  //strobe low so outputs dont change during pushing
  digitalWrite(shiftSTR,LOW);
  shiftOut(shiftD,shiftCLK,MSBFIRST,~out); // invert bits to pull proper pins low -> connect led cathode to ground
  //strobe high to send register content to outputs
  digitalWrite(shiftSTR,HIGH);
}

/*
 * handles the multiplexing of the seven segment display
 * h: the hour to display
 * m: the minute to display
 * dot: 0 = center dot, 1 = hour dots, 2 = minute dots
 */
void outputDisplay(int h, int m, int dot) {
  for (int i = 0; i < 4; i++) {
    switch(i) {
      case 0:
        pushToReg(h/10,dot==1 ? true : false);
        break;
      case 1:
        pushToReg(h%10,dot==2 ? false : true);
        break;
      case 2:
        pushToReg(m/10,dot==2 ? true : false);
        break;
      case 3:
        pushToReg(m%10,dot==2 ? true : false);
        break;
    }
    if (i != 0 || h/10 != 0 || displayState == 3) {
      digitalWrite(nums[i],LOW);
      delay(4);
      digitalWrite(nums[i],HIGH);
    }
  }
}

void displayMachine() {  
  //initializing outputs to default values
  digitalWrite(shiftD,LOW);
  digitalWrite(shiftCLK,LOW);
  digitalWrite(shiftSTR,LOW);
  for (int i = 0; i < 4; i++) {
    digitalWrite(nums[i],HIGH);
  }
  
  if (displayState == 1) {
      outputDisplay(now.hour() + (daylightSavingsState ? 1 : 0), now.minute(),0); //add 1 to hour if daylight savings time
  } else if (displayState == 2) {
      outputDisplay(aHour,aMinute,0);
  } else if (displayState == 3) {
      outputDisplay(aHour,aMinute,1);
  } else if (displayState == 4) {
      outputDisplay(aHour,aMinute,2);
  } else {
      //delay(10);
  }
}

void setup() {
  //TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz at pin9+10
  
  //declaring pins
  pinMode(alarmEn, INPUT_PULLUP);
  pinMode(lightEn, INPUT_PULLUP);
  pinMode(lightDim, INPUT);
  pinMode(ledPwm, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(rotaryA, INPUT);
  pinMode(rotaryB, INPUT);
  pinMode(shiftD, OUTPUT);
  pinMode(shiftCLK, OUTPUT);
  pinMode(shiftSTR, OUTPUT);
  pinMode(numA, OUTPUT);
  pinMode(numB, OUTPUT);
  pinMode(numC, OUTPUT);
  pinMode(numD, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), changeState, FALLING); //interrupt for reset button
  attachInterrupt(digitalPinToInterrupt(rotaryA), getEncoderDir, RISING); //interrupt for rotary encoder A phase

  Serial.begin(9600);
  while (! Serial); // Wait untilSerial is ready - Leonardo

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (rtc.lostPower()) {
    Serial.println("RTC lost power!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  aHour = EEPROM.read(ADDRESS);
  aMinute = EEPROM.read(ADDRESS+1);
  calcAlarm();
}

void loop() {
  now = rtc.now();
  //daylightSavingsState = isDaylightSavings(now,1);
  daylightSavingsState = false;
  if (digitalRead(alarmEn) == LOW) {
    alarmMachine();
  } else if (digitalRead(lightEn) == LOW) {
    analogWrite(ledPwm, map(ledOffset+analogRead(lightDim), 0, ledOffset+1023, 0, 255));
  } else {
    alarmState = 0; 
    digitalWrite(ledPwm, LOW);
  }
  displayMachine();
}

/*
DS3231_GPS.ino

DS3231 Aging from GPS PPS

This sketch compares the SQW output of a DS3231 to the PPS output of a GPS module,
and adjusts the RTC's Aging register so the RTC clock is running at the same speed as GPS.
Comparisons are done every five minutes.  Results are valid only at the current temperature
and power supply voltage.

This runs on an ATMega328P Arduino (Uno, Nano, Pro Mini) with the PPS line of a GPS module
tied to D8, and the SQW output of a DS3231 RTC tied to D2.  GND, SDA and SCL as usual. The
RTC's VCC pin can be connected to the Arduino's 5V pin, or to D4.  D4 power turns on only
during I2C activity to power the pullup resistors, so at all other times the RTC runs off
its 3V coin cell.  This requires disabling any on-module pullup resistor on the SQW pin.

After bootup, wait for the GPS to begin one-second flashing, then press
any key to begin.

When Aging is where you want it, enter 'Q' to quit, or 'T' to set the RTC to a new date and
time you will enter.  Enter 'An' at any time to set Aging to n, then start over (-128 through
127 permitted).

There is a PDF on this project on the Github repo.
*/

#include <Wire.h>
#define flagsREG EIFR                     // ATMega328P interrupt flags register
const bool MondayOne = true;              // Day of the week: Monday = day 1. False if Sunday is
const int SNdivisor = 264;                // Divisor for SN parts for 5 minutes @ 16MHz
const int Mdivisor = 576;                 // Divisor for M parts for 5 minutes @ 16MHz
const int Period = 300;                   // number of seconds between adjustments (5 minutes)
const byte numToAvg = 16;                 // Number of readings to average
const byte RTCpwr = 4;                    // Power for RTC Vcc pin provided by D4
const byte SQWpin = 2;                    // RTC SQW pin = D2
const byte PPSpin = 8;                    // GPS PPS pin = D8
const byte secReg = 0;                    // DS3231 registers
const byte ctlReg = 0x0E;
const byte statReg = 0x0F;
const byte agingReg = 0x10;

volatile bool Capture = false;            // Capture interrupt has occurred - GPS
volatile bool captureLast = true;         // GPS and RTC pulses should alternate
volatile bool missingGPS = false;         // Multiple RTC pulses without intervening GPS pulse         
volatile bool Started = false;            // Square wave interrupt has occurred - RTC
volatile byte MSBtimer;                   // MS byte of 24-bit timer
bool isSN = true, Pending = false;        // true if SN, false if M; forced conv pending
bool fallEdge = true;                     // SQW edge used to interrupt
unsigned long GPTlow, GPThi;              // timer counts on GPS interrupt
long Diff, oldDiff = 0, deltaDiff;        // clock cycles from RTC to GPS edges
long deltaMillis;                         // used to calculate elapsed time
int Divisor;                              // Aging +/-1 adjustment = count difference over 5 minutes
int Count = 32, newAge;                   // down counter - seconds until next calc, manual Aging value
int8_t Aging, deltaAging;                 // contents of RTC Aging register
byte Seconds, Minutes,Control, Status, j; // other RTC registers' contents
byte oldControl, oldStatus;               // original contents - restored at termination
long batch[numToAvg];                     // last numToAvg readings
char buff[20];                            // serial input buffer
byte buffSize;                            // length of input string
char in;                                  // serial input character
byte t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};  //Used to calculate day of week

void setup() {
  pinMode(RTCpwr,OUTPUT);                 // power up RTC Vcc
  pwrUP();
  Serial.begin(115200);
  delay(2000);
  Wire.begin();
  pinMode(A4,INPUT);                      // Disable internal pullups on SDA and SCL
  pinMode(A5,INPUT);
  pinMode(PPSpin,INPUT);                  // Timer1 capture input - from GPS PPS
  pinMode(SQWpin,INPUT_PULLUP);           // Hardware interrupt - from RTC

  Serial.println("Wait for GPS to begin 1-sec flashes,");
  Serial.println("  then enter any key to begin");
  Serial.println();
  int key1 = 0;
  while ((key1 != 10) && (key1 != 13)) {
    key1 = Serial.read();
    delay (100);
  }
  key1 = Serial.read();                   // in case CR/LF

  Wire.beginTransmission(0x68);           // read Control, Status and Aging registers
  Wire.write(ctlReg);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 3);
  oldControl = Wire.read() & 0b11011111;  // mask out CONV bit
  oldStatus = Wire.read() & 0b00001000;   // restore only the 32KHz setting
  Aging = Wire.read();                    // starting value of the Aging register

  // Clear /EOSC, CONV, RS2, INTCN, A2E, A1E.   Set BBSQW, RS1. (SQW freq = 1KHz)
  Control = 0b01001000;
  updateReg(ctlReg);                      // update Control
  Status = 0;
  updateReg(statReg);                     // update Status
  delay(1000);
  
  while(!digitalRead(SQWpin));            // determine if SN or M
  while(digitalRead(SQWpin));             // calculate time of 1KHz half-wave
  deltaMillis = millis();
  while(!digitalRead(SQWpin));
  deltaMillis = millis() - deltaMillis;
  if(deltaMillis < 250) {                 // SN will be 0
    Serial.println("This is a DS3231SN.");
    Divisor = SNdivisor;
  }
  else {                                  // M will be about 500 ms
    Serial.println("This is a DS3231M."); //  because M can't only does 1Hz
    isSN = false;
    Divisor = Mdivisor;
  }
  Serial.println();
  Control &= 0b11110111;                  // change squarewave to 1Hz
  updateReg(ctlReg);
  if (F_CPU == 8000000) Divisor /= 2;     // if 8MHz Pro Mini
   
  while(!digitalRead(SQWpin));            // wait for SQW high, then
  while(digitalRead(SQWpin));             // wait for SQW low - beginning of second
  deltaMillis = millis();
  if (digitalRead(PPSpin)) {              // is PPS already high, wait for low, then---
    while(digitalRead(PPSpin));
  }
  while(!digitalRead(PPSpin));            // wait for high
  deltaMillis = millis() - deltaMillis;   // difference between the two edges
  if((deltaMillis < 250) || (deltaMillis > 750)) fallEdge = false;  // if too close, use other phase

  cli();
  TIMSK0 = 0;                             // Disable Timer0 interrupts (millis)
  TCCR0A = 0;
  TCCR0B = 0;
  TIFR0  = 0xFF;

  TCCR1A = 0;                             // set up Timer1
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT0  = 0;                             // clear Timer1
  TIFR1  = 0xFF;                          // clear flags
  TIMSK1 = 0b00100001;                    // enable capture and overflow interrupt (GPS)
  TIFR1  = 0xFF;                          // clear flags
  TCCR1A = 0b00000000;                    // Normal mode, no output, WGM #0
  TCCR1B = 0b01000001;                    // rising edge capture, timer1 on, no prescale

  flagsREG = 3;
  if (fallEdge == true) {
    attachInterrupt(digitalPinToInterrupt(SQWpin),rtcISR, FALLING);   // ISR for D2
  }
  else {
    attachInterrupt(digitalPinToInterrupt(SQWpin),rtcISR, RISING);
  }
  flagsREG = 3;
  sei();
  Serial.println ("Enter 'A' to display current Aging register contents");
  Serial.println ("Enter 'An' to change Aging contents to n (-128 thru 127)");
  Serial.println ("Enter 'T' to display mm:ss each second");
  Serial.println ("Enter 'Tmmss' to set time synced to next PPS pulse");
  Serial.println ("Enter 'TYYMMDDhhmmss' to set date/time synced to next PPS pulse");
  Serial.println ("              hh = 24-hour.  Day of week is calculated.");
  Serial.println ("Enter 'Q' to quit"); Serial.println();
  Serial.println ("Working..."); Serial.println();
  
  pwrDN();                                // if D4, RTC runs on coin cell except when I2C-ing
}

void loop() {
  if (Capture) {                          // GPS PPS has gone high
    Capture = false;
    GPTlow = ICR1;                        // read timer values
    GPThi = MSBtimer;
    cli();
    TCNT1 = 0;                            // clear timer1
    MSBtimer = 0;
    TIFR1 = 0xFF;                         // clear flags
    sei();

    if((captureLast == false) || (oldDiff == 0)){
      if((missingGPS == false) || (oldDiff == 0)) {
        Count--;
        if(Count < numToAvg) {
          Diff = (GPThi << 16) + GPTlow;  // combine timer counts to one long value
          batch[Count] = Diff;            // collect last numToAvg values into array
        }
      }
      else {
        Serial.println("Missing GPS pulse");
        missingGPS = false;
      }
    }
    else Serial.println("Missing RTC pulse");
    captureLast = true;
    
    if (!Count) {                         // do calculation every five minutes
      Diff = 0;
      for (j = 0; j < numToAvg; j++) {
        Diff += batch[j];
      }
      Diff = (Diff + (numToAvg/2)) / numToAvg;   // average over last numToAvg seconds
      if (!oldDiff) oldDiff = Diff;       // for initial capture only
      deltaDiff = Diff - oldDiff;         // calculate new Aging
      deltaAging = deltaDiff / Divisor;
      if (deltaAging) {                   // if any change
        Aging += deltaAging;
        pwrUP();
        updateReg(agingReg);
        pwrDN();
        oldDiff = Diff;
        if (isSN) Pending = true;         // force conversion if SN
      }
      Serial.print ("Diff "); Serial.println(Diff);   // print results
      Serial.print ("deltaDiff ");
      if (deltaAging == 0) {
        Serial.print("[");
        Serial.print(deltaDiff);
        Serial.println("]");
      }
      else Serial.println(deltaDiff);
      Serial.print ("deltaAging "); Serial.println(deltaAging);
      Serial.print ("Aging "); Serial.println(Aging); Serial.println();

      Count = Period;                     // another 5 minutes
    }
  }
  if (Started) {                          // beginning of second
    if (Pending) {
      Control |= 0b00100000;              // force conversion if SN
      pwrUP();
      updateReg(ctlReg);
      pwrDN();
      Control &= 0b11011111;
      Pending = false;
    }
    Started = false;
  }
  if(Serial.available()) {                // process input from Serial Monitor
    in = Serial.read();                   // set end-line option to Newline or CR
    if ((in == 13) || (in == 10)) {
      buff[buffSize] = 0;
      parse_cmd(buff, buffSize);
      buffSize = 0;
      buff[0] = 0;
    }
    else {
      buff[buffSize] = in;
      buffSize++;
    }
  }
}

void parse_cmd(char *cmd, byte cmdsize) {

  // T
  if (((cmd[0]&0xDF)=='T') && (cmdsize == 1)) {     // "T" only - scroll RTC minutes/seconds
    disableInts();
    ticktock();
  }

  // Tmmss
  else if (((cmd[0]&0xDF)=='T')&&(cmdsize==5)) {     // "T" set minutes and seconds only
    disableInts();
    while (!digitalRead(PPSpin));
    Wire.beginTransmission(0x68);
    Wire.write(secReg);
    Wire.write(inp2bcd(cmd,3));           // seconds
    Wire.write(inp2bcd(cmd,1));           // minutes
    Wire.endTransmission();
    fallEdge = false;
    ticktock();
  }

  // TYYMMDDhhmmss
  else if (((cmd[0]&0xDF)=='T')&&(cmdsize==13)) {     // "T" new date/time
    disableInts();
    while (!digitalRead(PPSpin));         // wait for PPS rising edge
    Wire.beginTransmission(0x68);
    Wire.write(secReg);
    Wire.write(inp2bcd(cmd,11));          // seconds
    Wire.write(inp2bcd(cmd,9));           // minutes
    Wire.write(inp2bcd(cmd,7));           // hours
    
    int y = bcd2dec(inp2bcd(cmd,1)) + 2000;  // begin day of week calculation
    byte m = bcd2dec(inp2bcd(cmd,3));
    byte d = bcd2dec(inp2bcd(cmd,5));
    if ( m < 3 ) y -= 1;
    byte DOW = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
    if (MondayOne == false) DOW++;
    if (DOW == 8) DOW = 1;
    
    Wire.write(DOW);                      // day of the week
    Wire.write(inp2bcd(cmd,5));           // date of the month
    Wire.write(inp2bcd(cmd,3) | 0x80);    // month & century
    Wire.write(inp2bcd(cmd,1));           // year
    Wire.endTransmission();
    fallEdge = false;
    ticktock();
  }

  // A, An
  else if ((cmd[0]&0xDF)=='A') {          // "A" Aging
    if (cmdsize > 1) {
      newAge = atoi(&cmd[1]);             // get value of string
      if ((newAge < 128) && (newAge > -129)) {    // check for legit value
        Aging = newAge;                   // convert to signed byte
        pwrUP();
        updateReg(agingReg);              // write to Aging register
        pwrDN();
        Count = 32; oldDiff = 0;          // start over
        if (isSN) Pending = true;
      }
      else Serial.println ("Invalid Aging Value");
    }
    pwrUP();
    Wire.beginTransmission(0x68);         // "A" alone prints current value
    Wire.write(agingReg);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    Aging = Wire.read();
    pwrDN();
    Serial.print("Aging = "); Serial.println(Aging); Serial.println();
  } 

  else if ((cmd[0]&0xDF)=='Q') {          // "Q" Quit
    pwrUP();
    Control = oldControl;                 // restore Contol
    updateReg(ctlReg);
    Status = oldStatus;                   // restore Status
    updateReg(ctlReg);
    cli();
    detachInterrupt(digitalPinToInterrupt(SQWpin));
    flagsREG = 3;
    TCCR1B = 0;
    TIMSK1 = 0;
    TIFR1 = 0xFF;
    sei();
    Serial.println("Shutting down");
    while (1);
  }
}

byte inp2bcd(char *inp, byte seek) {
  return (((inp[seek]-48)<<4) + (inp[seek+1] - 48));
}

byte bcd2dec(byte n){
  n &= 0x7F;                              // mask out Century bit
  return n - 6 * (n >> 4);
}

void disableInts (){
  pwrUP();
  detachInterrupt(digitalPinToInterrupt(SQWpin));
  TIMSK1 = 0; 
}

void ticktock() {
  for (j = 0; j < buffSize; j++) {        // display entry
    Serial.print(buff[j]);
  }
  Serial.println();
  Serial.println("Enter any key to stop");
  while (!Serial.available()) {
    while (!digitalRead(SQWpin));
    while (digitalRead(SQWpin));          // Display RTC mm:ss on falling edge of SQW
    Wire.beginTransmission(0x68);
    Wire.write(secReg);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);
    Seconds = bcd2dec(Wire.read());
    Minutes = bcd2dec(Wire.read());
    if (Minutes < 10) Serial.print("0");
    Serial.print(Minutes);Serial.print(":");
    if (Seconds < 10) Serial.print("0");
    Serial.println(Seconds);
  }
  while(Serial.available()) Serial.read();
  Serial.println("Resuming..."); Serial.println();

  cli();
  flagsREG = 3;
  if (fallEdge == true) {
    attachInterrupt(digitalPinToInterrupt(SQWpin),rtcISR, FALLING);   // ISR for D2
  }
  else {
    attachInterrupt(digitalPinToInterrupt(SQWpin),rtcISR, RISING);
  }
  flagsREG = 3;
  TIMSK1 = 0b00100001;
  Count = 32; oldDiff = 0; 
  sei();
  pwrDN();      
}

void updateReg(byte addr) {
  Wire.beginTransmission(0x68);
  Wire.write(addr);
  if(addr == ctlReg) Wire.write(Control);
  else if(addr == statReg) Wire.write(Status);
  else if(addr == agingReg) Wire.write(Aging);
  Wire.endTransmission();
}

void pwrUP() {
  digitalWrite(RTCpwr,1);
}

void pwrDN() {
  digitalWrite(RTCpwr,0);
}


ISR(TIMER1_CAPT_vect) {
  TCCR1B &= 0xFE;                         // stop Timer1 clock
  Capture = true;
}

ISR(TIMER1_OVF_vect) {
  MSBtimer++;                             // increment MSB on overflow
}

void rtcISR() {
  TCCR1B |= 1;                            // start Timer1 clock
  Started = true;
  if (captureLast == false) missingGPS = true;
  captureLast = false;
}

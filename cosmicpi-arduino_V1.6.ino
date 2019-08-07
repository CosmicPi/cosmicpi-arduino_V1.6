//Cosmic Pi software for Arduino - modified for STM32,
//J. Devine
//July 2019.
//Licensed under GPL V3 or later.
//cosmicpi.org

#include <Wire.h> 
// LPS lib from here: https://github.com/pololu/lps-arduino
#include "src/LPS.h"
// LSM9DS1 lib from here: https://github.com/adafruit/Adafruit_LSM9DS1
//with some address tweaks

//#include "src/Adafruit_Sensor.h"
#include "src/Adafruit_LSM9DS1.h"
// HTU21D lib from here: https://github.com/adafruit/Adafruit_HTU21DF_Library
#include "src/SparkFunHTU21D.h"


//counter for how long the leds stay on.
int last_event_LED = 0;


// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

//declare pressure sensor
LPS ps;

//Create an instance of the object
HTU21D myHumidity;



// string used for passing data from the dump routines
static const int TXTLEN = 512;
static char txt[TXTLEN]; //internal buffer
static char txts[TXTLEN]; //sensor buffer


//pinouts
/*
   PA0 - Pin 14 - Shaped Signal 1
   PA1 - Pin 15 - Shaped Signal 2
   PA2 - Pin 16 - TX0
   PA3 - Pin 16 - RX0
   PA4 - Pin 20 - LED1 - Power/GPS
   PA5 - Pin 21 - LED2 - Event
   PA6 - Pin 22 - Injection leds
   PA7 - Pin 23 - Bias FB1
   PA8 - Pin 41 - SCL_Slave
   PA9 - Pin 42 - USB_OTG_VBUS
   PA10 - Pin 43 - GPSTX
   PA11 - Pin 44 - USBOTG DM
   PA12 - Pin 45 - USBOTG DP
   PA13 - Pin 46 - SWDIO
   PA14 - Pin 49 - SWCLK
   PA15 - Pin 50 - GPSPPS Input
   PB0 - Pin 26 - Bias FB2
   PB1 - Pin 27 - Flag to RPi
   PB2 Pin 28 - NC
   PB 3 - Pin 55 - NC
   PB4 - Pin 56 - SDA_Slave
   PB5 - Pin 57 - NC
   PB6 - Pin 58 - GPSRX
   PB7 - Pin 59 - SDA0
   PB8 - Pin 61 - SCL0
   PB9 - Pin 62 - NC
   PB10 - Pin 29 - Trigout (input to STM)
   PB12 - Pin 33 - NC
   PB13 - Pin 34 0 HVPSU SCLK (Clock to MAX1932)
   PB14, PB15 - NC
   PC0 - Pin 8 - NC
   PC1 - Pin 9 - HVPSU CL1
   PC2 - Pin 10 - HVPSU CL2
   PC3 - Pin 11 - HV PSU DIN
   PC4, PC5, PC6- NC
   PC7 - Pin 38 - HVPSU CS2
   PC8 - Pin 39 - HVPSU CS1
   PC9 - Pin 40 - Mag Interrupt
   PC10 - Pin 51 -  NC
   PC11 - Pin 52 - STRIGOUT B
   PC12 - Pin 53 - STRIGOUT A
   PC13 - Pin 2 - Baro Int
   PC14 - Pin 3 - Accelint 1
   PC15 - Pin 4 - Accelint 2.

*/

//eeprom writing table
/*
   eeprom status byte - 00
  channel 0 high byte - 01
  channel 0 low byte  - 02
  channel 1 high byte - 03
  channel 1 low byte  - 04
  HV bias ch 0 value  - 05
  HV bias ch 1 value  - 06

  if eeprom status byte = 1 then only one set of values are stored
  if =2 then a backup set are located higher in memory, not used at the moment
*/


#include <Wire.h>
#include <EEPROM.h>
static const int SERIAL_BAUD_RATE = 19200;   // Serial baud rate for version 1.5 production
static const int GPS_BAUD_RATE = 9600;        // GPS and Serial1 line

// LED pins
#define PPS_PIN PA4      // PPS (Pulse Per Second) and LED
#define EVT_PIN PA5      // Cosmic ray event detected

char numconvbuff[16];

// Leds  flag
bool leds_on = true;

// How long the event LED should light up (in ms)
static int event_LED_time = 15;

long eventCount = 0;
int eventStack = 0;
unsigned long pps_micros = 0;
unsigned long pps_micros_old = 0;
unsigned long micros_since_pps = 0;

#define maxevent 100 //we don't expect more events than this
unsigned long evttime [maxevent];

//set up serial output buffer
#define OutBuffSize 1024 //we don't expect more chars than this in the buffer
char outputbuffer[OutBuffSize]; //1024 character output buffer
int writeincounter = 0; //the place for writing to the buffer
int readoutcounter = 0; //the place for reading out from the buffer

void WriteToOutputBuff(char *outstring)
{
  int stringlengthmax = strlen(outstring);

  for (int addchar = 0;  addchar < stringlengthmax; addchar++) {
    outputbuffer[writeincounter] = outstring[addchar];
    writeincounter++;
  }
}

void ReadFromOutputBuff()
{
  char digitout[2];
  if (writeincounter > readoutcounter)
  {
    digitout[0] = outputbuffer[readoutcounter];
    digitout[1] = '\0';
    Serial.print(digitout);
    readoutcounter++;
  }
  if (readoutcounter == writeincounter) {
    readoutcounter = 0;
    writeincounter = 0;
    //Serial.println("BUFFERCLEAR");
  }
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}   



// GPS and time flags
boolean gps_ok = false;         // Chip OK flag
boolean pps_recieved = false;



// ------------------------- Arudino Functions

// Arduino setup function, initialize hardware and software
// This is the first function to be called when the sketch is started

//setup serials
HardwareSerial GPS(PA10, PB6);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print("ON");

  //init pins that aren't used for stability
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA6, OUTPUT);
  pinMode(PA7, INPUT);
  pinMode(PA8, INPUT);
  pinMode(PB0, INPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB4, INPUT);

  pinMode(PC9, INPUT);
  pinMode(PC11, INPUT);
  pinMode(PC12, INPUT);
  pinMode(PC13, INPUT);
  pinMode(PC14, INPUT);
  pinMode(PC15, INPUT);
  

  if (leds_on) {
    pinMode(EVT_PIN, OUTPUT);       // Pin for the cosmic ray event
    pinMode(PPS_PIN, OUTPUT);       // Pin for the PPS (LED pin)
  }
  if (leds_on) {
    digitalWrite(PPS_PIN, HIGH);   // Turn on led
  }

  // start the GPS
  GPS.begin(GPS_BAUD_RATE);
  GpsSetup();

  // start the i2c bus
  Wire.begin();
  Wire.setSDA(PB7);
  Wire.setSCL(PB8);

  //setup the thresholds
  //initialise to values from EEPROM, else use defaults.

  int eeprom_read = 0;

  eeprom_read = EEPROM.read(0x00);
  if (eeprom_read > 0) {

    Serial.println("Found existing settings for Thresholds and HV, these will be applied");
    Serial.print("First channel threshold settings: 0x");
    Serial.print(EEPROM.read(0x01), HEX);
    Serial.println(EEPROM.read(0x02), HEX);
    Serial.print("Second channel threshold settings: 0x");
    Serial.print(EEPROM.read(0x03), HEX);
    Serial.println(EEPROM.read(0x04), HEX);



    Wire.begin();
    Wire.beginTransmission(byte(0x60)); // transmit to device #112
    Wire.write(byte(0x00)); //sets value to the first channel
    eeprom_read = EEPROM.read(0x01);
    Wire.write(byte(eeprom_read));
    eeprom_read = EEPROM.read(0x02);
    Wire.write(byte(eeprom_read));
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(byte(0x60)); // transmit to device #112
    Wire.write(byte(0x01)); //sets value to the first channel
    eeprom_read = EEPROM.read(0x03);
    Wire.write(byte(eeprom_read));
    eeprom_read = EEPROM.read(0x04);
    Wire.write(byte(eeprom_read));
    Wire.endTransmission();    // stop transmitting
    Serial.println("Threshold settings complete");

  }
  else
  {

    Serial.println("No existing settings for Thresholds and HV found, using defaults");
    Serial.print("First channel threshold settings: 0x");
    Serial.print(0x22F, HEX);
    Serial.print("Second channel threshold settings: 0x");
    Serial.print(0x22F, HEX);


    Wire.begin();
    Wire.beginTransmission(byte(0x60)); // transmit to device #112
    Wire.write(byte(0x00)); //sets value to the first channel
    Wire.write(byte(0x02));
    Wire.write(byte(0x2F));
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(byte(0x60)); // transmit to device #112
    Wire.write(byte(0x01)); //sets value to the first channel
    Wire.write(byte(0x02));
    Wire.write(byte(0x2F));
    Wire.endTransmission();    // stop transmitting

    Serial.println("Threshold settings complete");


  }

  //set the high voltage

  //config SPI pins
  pinMode(PC7, OUTPUT);
  pinMode(PC8, OUTPUT);
  pinMode(PC3, OUTPUT);
  pinMode(PB13, OUTPUT);


  eeprom_read = EEPROM.read(0x00);
  if (eeprom_read > 0) {
    Serial.println("Applying stored values for HV channels");

    Serial.print("HV channel settings Ch1: ");
    Serial.print(EEPROM.read(0x05), HEX);
    Serial.print(" Ch2: ");
    Serial.println(EEPROM.read(0x06), HEX);

    //set HV channel 1
    eeprom_read = EEPROM.read(0x05);
    digitalWrite(PC7, LOW);
    setHV(byte(eeprom_read));
    digitalWrite(PC7, HIGH);
    //set HV channel 2
    eeprom_read = EEPROM.read(0x06);
    digitalWrite(PC8, LOW);
    setHV(byte(eeprom_read));
    digitalWrite(PC8, HIGH);

    Serial.println("HV channels set");

  }
  else
  {
    Serial.println("Applying default values for HV channels");

    Serial.print("HV channel settings Ch1: ");
    Serial.print(0xAC, HEX);
    Serial.print(" Ch2: ");
    Serial.println(0xAC, HEX);

    digitalWrite(PC7, LOW);
    setHV(0xAC);
    digitalWrite(PC7, HIGH);
    //set HV channel 2
    digitalWrite(PC8, LOW);
    setHV(0xAC);
    digitalWrite(PC8, HIGH);
    Serial.println("HV channels set");

  }



//init the sensor suite

  Serial.println("Cosmic Pi V1.6 sensor init routine");
  //init LSM9DS1
  //lsm.begin();

  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  setupSensor();
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  //init lps
  ps.init();
  ps.enableDefault();

  //init htu21d
  myHumidity.begin();

  Serial.println("INFO: Sensor setup complete\n");
  //attach the interrupts
  attachInterrupt(digitalPinToInterrupt(PA15), GPS_PPS, RISING);
  attachInterrupt(digitalPinToInterrupt(PB10), Event_Int, RISING);

  //Serial.println();
sprintf(txt, "INFO: Running\n");
    WriteToOutputBuff(txt);
}


void GPS_PPS()
{
  //now PPS is coming through, we can switch off the internal timer.
  pps_recieved = true;

  //stop listening for events while we process this interrupt
  detachInterrupt(digitalPinToInterrupt(PB10));

  //now PPS is coming through, we can switch off the internal timer.
  //pps_recieved = true;
  
  //set the pps micros value as the micro time now, and buffer the last value;
  pps_micros_old = pps_micros;
  pps_micros = micros();
  //how many microseconds since the last pps? (just in case we're running fast/slow)
  micros_since_pps = pps_micros - pps_micros_old;

  if (leds_on) {
          digitalWrite(PPS_PIN, !digitalRead(PPS_PIN));   // Toggle led
        }

  
  //Serial.println("pps");
  //process all events to the event buffer now that the second has ended
  //Serial.print("Dumping Event Buffer");
  //print out the event stack
  for (int i = 0; i < eventStack; i++) {
    sprintf(txt, "Event: sub second micros:%d/%d; Event Count:%d\n", evttime[i], micros_since_pps, (eventCount - eventStack + i));
    WriteToOutputBuff(txt);
  }
  sprintf(txt, "PPS: GPS lock:1;\n");
  WriteToOutputBuff(txt);
  

//now print the sensors
 //Serial.print("Sensor section");
  lsm.read();  /* ask it to read in the data */ 
  //debug cmd
  //sprintf(txt, "readfail;\n");
  //WriteToOutputBuff(txt);
  /* Get a new sensor event */ 
  
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  float humd = myHumidity.readHumidity();
  float temphum = myHumidity.readTemperature();

Serial.print("Pressure: ");
Serial.print(pressure, 6);
Serial.println(';');
Serial.print("Altitude: ");
Serial.print(altitude, 6);
Serial.println(';');
Serial.print("TemperatureCBaro: ");
Serial.print(temperature, 6);
Serial.println(';');

Serial.print("AccelX: ");
Serial.print(a.acceleration.x, 6);
Serial.println(';');
Serial.print("AccelY: ");
Serial.print(a.acceleration.y, 6);
Serial.println(';');
Serial.print("AccelZ: ");
Serial.print(a.acceleration.z, 6);
Serial.println(';');

Serial.print("MagX: ");
Serial.print(m.magnetic.x, 6);
Serial.println(';');
Serial.print("MagY: ");
Serial.print(m.magnetic.y, 6);
Serial.println(';');
Serial.print("MagZ: ");
Serial.print(m.magnetic.z, 6);
Serial.println(';');

Serial.print("TemperatureCHumid: ");
Serial.print(temphum, 6);
Serial.println(';');

Serial.print("Humidity: ");
Serial.print(humd, 6);
Serial.println(';');
  
  //average temp from all 3 sensors
  float out = 0;
  out = temphum + temperature;
  out = out / 2;
Serial.print("TemperatureC: ");
Serial.print(out, 6);
Serial.println(';');
//Serial.print(txts);   
 



  //now we can reset the event stack and listen for events again
  eventStack=0;
  attachInterrupt(digitalPinToInterrupt(PB10), Event_Int, RISING);
}





void loop() {
  pipeGPS();
  ReadFromOutputBuff();

  //turn off the event led after the set time
    if (millis() >= (last_event_LED + event_LED_time)){
    if (leds_on) {
      digitalWrite(EVT_PIN, LOW);
    }
  }
}


void Event_Int()
{
   detachInterrupt(digitalPinToInterrupt(PB10));


  //only count if we've got a PPS
if (pps_recieved == true){

  /*  WriteToOutputBuff("evtx:");
    ltoa(eventCount,numconvbuff,10);
    WriteToOutputBuff(numconvbuff);

    //Serial.print(name);
    Serial.print("count=");
    Serial.print(eventCount);
    Serial.print(" ");
    Serial.println(micros()-pps_micros);
  */
    eventCount++;
  //Serial.println("e");
  //write this event to the stack of times
  evttime[eventStack] = micros() - pps_micros;
  //increment the stack counter
  eventStack++;

  //turn on the event led
  last_event_LED = millis();
  if (leds_on) {
    digitalWrite(EVT_PIN, HIGH);
  }
  
  attachInterrupt(digitalPinToInterrupt(PB10), Event_Int, RISING);
}
}

byte setHV(byte _send)  // This function is what bitbangs the data
{
  if (_send > 0x5F) {
    for (int i = 0; i < 8; i++) // There are 8 bits in a byte
    {
      digitalWrite(PC3, bitRead(_send, 7 - i));  // Set MOSI
      //delay(1);
      digitalWrite(PB13, HIGH);                  // SCK high
      //bitWrite(_receive, i, digitalRead(MISO_pin)); // Capture MISO
      digitalWrite(PB13, LOW);                   // SCK low
      //digitalWrite(MOSI_pin, LOW);    // Set MOSI

    }
    //digitalWrite(SS_pin[j], HIGH);       // SS high again
  }
}
;

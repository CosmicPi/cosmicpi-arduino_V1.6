#include <Wire.h>

// configure these to configure the detector
static const int HV_DEFAULT = 0xAC;
static const int DEFAULT_DAC_THRESH = 559; //modification for V1.5 production batch, this is a generic setting to be tuned by users

// other constants
static const int HV_MAX = 89;
static const int HV_MIN = 255;
static const int DEFAULT_THRESH = 559;
static const bool USE_DAC = true;

//set up the pins to remap SPI by hand
//static const int num_devices = 2;
//static const int SS_pin[num_devices] = {14, 15};
//static const int SCK_pin = 17;
//static const int MISO_pin = 22;
//static const int MOSI_pin = 16;

// I2C adress pins
//#define MAX5387_PA0_pin A9
//#define MAX5387_PA1_pin A10
//#define MAX5387_PA2_pin A11

byte thresh1;
byte thresh2;
int bigpart;
int smallpart;

// initilizes the detector with default values
void setupDetector(){
  // setup pins
  detecSetPinModes();
  detcSetConstantPins();
  // set defaults
  if (USE_DAC){
  //  analogWrite(DAC0, DEFAULT_DAC_THRESH);
  //  analogWrite(DAC1, DEFAULT_DAC_THRESH); 
  }else{
    setThreshold(3, DEFAULT_THRESH);
  }
  setHV(HV_DEFAULT);
  sprintf(txt,"Detector threshold: %d\n", DEFAULT_DAC_THRESH);
  aSer->print(txt);
  sprintf(txt,"Detector setup finished\n");
  aSer->print(txt);
}


// sets pin modes needed for the detector
void detecSetPinModes(){
  //setup analog writemode
//  analogWriteResolution(12);
  // I2C adress pins for the MAX5387
//  pinMode(MAX5387_PA0_pin, OUTPUT);
//  pinMode(MAX5387_PA1_pin, OUTPUT);
//  pinMode(MAX5387_PA2_pin, OUTPUT);
  // HV pins
//  digitalWrite(SS, HIGH);  // Start with SS high
//  for (int i=0; i<num_devices; i++){
//    pinMode(SS_pin[i], OUTPUT);
//    digitalWrite(SS_pin[i], HIGH);  
//  }
//  pinMode(SCK_pin, OUTPUT);
  //pinMode(MISO_pin, INPUT); //this is the avalanche pin, not implemented yet
//  pinMode(MOSI_pin, OUTPUT);
}


// sets pins that don't need changing (ever)
void detcSetConstantPins(){
  // I2C adress pins for the MAX5387
//  digitalWrite(MAX5387_PA0_pin, LOW);//configure the address of the MAX5387 pot
//  digitalWrite(MAX5387_PA1_pin, LOW);//configure the address of the MAX5387 pot
//  digitalWrite(MAX5387_PA2_pin, LOW);//configure the address of the MAX5387 pot
}


// this function sets the thresholds for the MAX5387
// 1 is the first channel, 2 the second and 3 sets both at the same time
void setThreshold(int pot_channel, int value){
int address = 0x60;  // config I2C address of the DAC

//bitshifts for the two bytes to upload (10 bit value)
smallpart=byte(value);
bigpart=byte(value>>8);


  switch(pot_channel){
    case 1:
sprintf(txt,"Setting threshold on channel 1 to: %d\n", value);
      aSer->print(txt);
      
  Wire.beginTransmission(address);
  Wire.write(B00001000);              // sends five bytes
  Wire.write(bigpart);                    // sends one byte
  Wire.write(smallpart);
  Wire.endTransmission();
  break;
  case 2:
sprintf(txt,"Setting threshold on channel 2 to: %d\n", value);
      aSer->print(txt);
      
  Wire.beginTransmission(address);
  Wire.write(B00000000);              // sends five bytes
  Wire.write(bigpart);                    // sends one byte
  Wire.write(smallpart);
  Wire.endTransmission();
  break;
  case 3:
  sprintf(txt,"Setting threshold on both channels to: %d\n", value);
      aSer->print(txt);
      
  Wire.beginTransmission(address);
  Wire.write(B00001000);              // sends five bytes
  Wire.write(bigpart);                    // sends one byte
  Wire.write(smallpart);
  Wire.endTransmission();
  
  Wire.beginTransmission(address);
  Wire.write(B00000000);              // sends five bytes
  Wire.write(bigpart);                    // sends one byte
  Wire.write(smallpart);
  Wire.endTransmission();
  break;
  }
  /*
  // do a value check
  if (value > 255 || value < 1){
    return;
  } else {
    value = byte(value);
  }
  
  Wire.begin();
  Wire.beginTransmission(byte(0x28)); // transmit to device #112
  switch(pot_channel){
    case 1:
      sprintf(txt,"Setting threshold on channel 1 to: %d\n", value);
      aSer->print(txt);
      Wire.write(byte(B00010001)); //sets value to the first channel
      Wire.write(value);
      thresh1 = value;
      break;
    case 2:
      sprintf(txt,"Setting threshold on channel 2 to: %d\n", value);
      aSer->print(txt);
      Wire.write(byte(B00010010)); //sets value to the second channel
      Wire.write(value);
      thresh2 = value;
      break;
    case 3:
      sprintf(txt,"Setting threshold on channel 1&2 to: %d\n", value);
      aSer->print(txt);
      Wire.write(byte(B00010011)); //sets value to both channels
      Wire.write(value);
      thresh1 = value;
      thresh2 = value;
      break;
  }

  
  
  Wire.endTransmission();
*/
}


// set the two HV supplies
byte setHV(byte _send)  // This function is what bitbangs the data
{
    if (_send > 0x5A){ //hardlimit values

  sprintf(txt,"INFO: Setting HV 1&2 to: %d\n", _send);
  aSer->print(txt);
  //reception isn't implemented in this version. 
  //byte _receive = 0;
  
  digitalWrite(PC7, LOW);
  
  for(int i=0; i<8; i++)  // There are 8 bits in a byte
    {
      digitalWrite(PC3, bitRead(_send, 7-i));    // Set MOSI
      //delay(1);
      digitalWrite(PB13, HIGH);                  // SCK high
      //bitWrite(_receive, i, digitalRead(MISO_pin)); // Capture MISO
      digitalWrite(PB13, LOW);                   // SCK low
    //digitalWrite(MOSI_pin, LOW);    // Set MOSI
      
    }
  
  digitalWrite(PC7, HIGH);

    
  digitalWrite(PC8, LOW);
  
  for(int i=0; i<8; i++)  // There are 8 bits in a byte
    {
      digitalWrite(PC3, bitRead(_send, 7-i));    // Set MOSI
      //delay(1);
      digitalWrite(PB13, HIGH);                  // SCK high
      //bitWrite(_receive, i, digitalRead(MISO_pin)); // Capture MISO
      digitalWrite(PB13, LOW);                   // SCK low
    //digitalWrite(MOSI_pin, LOW);    // Set MOSI
      
    }
  
  digitalWrite(PC8, HIGH);
  //return _receive;        // Return the received data
    }
}

/* This code is for pushing 2 7 seg digits
 *  4 bits on the 74595 shift register (8 bits) are dedicated to the digit
 *  which is decoded into 7 seg digits by a ye olde  cd4511b
 *  There are 6 lines, 3 per digit for RGB digit1 and RGB digit2
 *  This will need to be alternated (ie multiplexed) because the ACD8143 does not have 2 sets of 7 segment wiring. 
 *  So to use both digits simultaneously its on/off etc 
 *  There is the possibility to use 4 more bits for selecting colour but you need 6 ... so I'm thinking to just use 6 pins maybe even in PWM to mix colours
 *  This code will be used on an ESP32 because it is meant for integration into the display/controls of synthesizer/organ projects of 2022
 *  pins that are available
 *  GPIO4 ADC2 - CH0
 *  GPIO2 ADC2 - CH2
 *  GPIO15 ADC - CH3
 *  
 *  Notes on the pin allocation now that I'm going to use ADC + 7SegRGB (this) + DAC
 *  board_esp32_doit.h
#define ADC_MUL_S0_PIN  33
#define ADC_MUL_S1_PIN  32
#define ADC_MUL_S2_PIN  17 (change from 13)
#define ADC_MUL_SIG_PIN 34 (change from 12) //34 tested ok for reading analogue
#define I2S_BCLK_PIN    25
#define I2S_WCLK_PIN    27
#define I2S_DOUT_PIN    26
#midiUSB  Connections via VSPI
 *  CS:                 5
 *  INT:                17 (not used)
 *  SCK:                18
 *  MISO:               19
 *  MOSI:               23
 *  If you wanted to use HSPI :  13, 12, 14, 15 would be needed

We can't output control via 34,35,36,39 but could use for buttons input
#midi Serial #define MIDI_RX2_PIN RXD2  (that would be pin 16)
Remaining to allocate smaller ESP32 board> 14,15,4,2  (17 should be ok if you don't use MIDI out serial TX)
1,3 are uart - ie USB serial comms to program (unexpected results at boot)
The ESP32 chip has the following strapping pins:
This may not matter but are strapping pins so if you can't write ... check these
GPIO 0
GPIO 2
GPIO 4
GPIO 5 (must be HIGH during boot)
GPIO 12 (must be LOW during boot)
GPIO 15 (must be HIGH during boot)
GPIO 6-11 connected to SPI flash memory not recommended 
 */
#include <Arduino.h>
                     //discovered 34-39 are input online wtf
#define latchPin      4  //green tested with UNO 5v and tested with ESP32
#define clockPin      2  //yellow
#define dataPin       15  //blue
// The cathode side of the digits on the ACD8143
// Steps - Choose PWM channel (esp32) from 0 to 15, set frequency 5000hz for LED,
//         duty cycle resolution 1 to 16 bits [8], which GPIO with ledcAttachPin(GPIO, channel)
#define ledFrequency  1000
#define ledcReso      8    //8 bits for duty cycle
#define redDigitOne   12    // gpio
#define greenDigitOne 16    //shift reg bit 5
#define blueDigitOne  32    //shift reg bit 6
#define redDigitTwo   13    //gpio
#define greenDigitTwo 64    //shift reg bit 7
#define blueDigitTwo  128   //shift reg bit 8
typedef  enum{ red, green, blue, yellow, purple, teal, white } digitColor;

digitColor  digitOneColor,digitTwoColor;
uint8_t segValue[2]; //2 digits of an RGB 7 seg display [0] is msb and [1] is lsb
bool    segDP;
static bool digit = false; //true = lsb and false = msb

void setupMplex() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);  // 74HC595 Pins
  pinMode(dataPin, OUTPUT);
  pinMode(redDigitOne, OUTPUT);
  pinMode(redDigitTwo, OUTPUT);

  Serial.println("R1: "+String(redDigitOne)+" R2: "+String(redDigitTwo));
  setDigitsColor(blue); 
  setDigits(0);

}

void setDigitsColor(uint8_t col){
  switch(col){
    case 0: //red
      digitOneColor = red;
      digitTwoColor = red;
      break;
    case 1: //green
      digitOneColor = green;
      digitTwoColor = green;
      break;
    case 2: //blue
      digitOneColor = blue;
      digitTwoColor = blue;
      break;
    case 3: //yellow = red+green
      digitOneColor = yellow;
      digitTwoColor = yellow;
      break;
    case 4: //purple = red+blue
      digitOneColor = purple;
      digitTwoColor = purple;
      break;
    case 5: //teal = green+blue
      digitOneColor = teal;
      digitTwoColor = teal;
      break;
    case 6: //white = r+g+b
      digitOneColor = white;
      digitTwoColor = white;
      break;
  }
}
void shiftOut2(uint8_t dataP, uint8_t clockP, bool mostSig, uint8_t command)  //GPIO type?
{
   for (int i = 0; i < 8; i++)
   {
       bool output = false;
       if (MSBFIRST)
       {
           output = command & 0b10000000;
           command = command << 1;
       }
       else
       {
           output = command & 0b00000001;
           command = command >> 1;
       }
       digitalWrite(dataPin, output);
       digitalWrite(clockPin, true);
       delayMicroseconds(1);
       //delay(1);
       digitalWrite(clockPin, false);
       delayMicroseconds(1);
       //delay(1);
    }
}

void setDigits(uint8_t value){ //should be less than 100 because we have only 2 digits
  //binary is fine it will be pushed into the 4511 7 seg decoder by the shift register
  
  if(value > 99){
    value = value%100;
    segDP = false; //using the decimal point to indicate a 3rd digit of 1 - false means a 0 which will open the led 
  } else
    segDP = true; //true will unlight the dp
  segValue[0] = value / 10;
  segValue[1] = value % 10;
}

uint8_t setCathodeColours(bool digitToSetOn){        //ledcWrite(redDigitOne, 0); PMW sink didn't work  //ledcWrite(redDigitTwo, 65535);
  uint8_t shiftMod = 240;      //set bits 5-8 by default means all off                                                 
  switch(digitOneColor){
    case red:
      digitalWrite(redDigitOne,digitToSetOn); //1 is off so when digitToSetOn is 0 (the first digit) 
      break;                                  //then digitOne gets 0 and sinks  - optimized out the if/then statement
    case green:
      digitalWrite(redDigitOne,true);
      if(!digitToSetOn)
        shiftMod -= greenDigitOne;
      break;
    case blue:
      digitalWrite(redDigitOne,true);
      if(!digitToSetOn)
        shiftMod -= blueDigitOne;
      break;
    case yellow:
      digitalWrite(redDigitOne,digitToSetOn);
      if(!digitToSetOn)
        shiftMod -= greenDigitOne;
      break;
    case purple:
      digitalWrite(redDigitOne,digitToSetOn);
      if(!digitToSetOn)
        shiftMod -= blueDigitOne;
      break;
    case teal:
       digitalWrite(redDigitOne,true);
       if(!digitToSetOn){
         shiftMod -= blueDigitOne;
         shiftMod -= greenDigitOne;
       }
       break;
    case white:
       digitalWrite(redDigitOne,digitToSetOn);
       if(!digitToSetOn){
         shiftMod -= blueDigitOne;
         shiftMod -= greenDigitOne;
       }
       break;
  }
  switch(digitTwoColor){
    case red:
      digitalWrite(redDigitTwo,!digitToSetOn); //1 is off so when digitToSetOn is 1 (the second digit) we flip it off with not operator
      break;                                   //then digitTwo gets 0 and sinks  - optimized out the if/then statement
    case green:
      digitalWrite(redDigitTwo,true);
      if(digitToSetOn)
         shiftMod -= greenDigitTwo;
      break;
    case blue:
      digitalWrite(redDigitTwo,true);
      if(digitToSetOn)
        shiftMod -= blueDigitTwo;
      break;
    case yellow:
      digitalWrite(redDigitTwo,!digitToSetOn);
      if(digitToSetOn)
         shiftMod -= greenDigitTwo;
      break;
    case purple:
      digitalWrite(redDigitTwo,!digitToSetOn);
      if(digitToSetOn)
        shiftMod -= blueDigitTwo;
      break;
     case teal:
       digitalWrite(redDigitTwo,true);
       if(digitToSetOn){
         shiftMod -= blueDigitTwo;
         shiftMod -= greenDigitTwo;
       }
       break;
     case white:
       digitalWrite(redDigitTwo,!digitToSetOn);
       if(digitToSetOn){
         shiftMod -= blueDigitTwo;
         shiftMod -= greenDigitTwo;
       }
       break;
  }
  return shiftMod;
}

void process7seg() {
  //each time we are called 0process a different row - multiplexing
  //moved this code into ISR (interrupt service routine)
  if(digit){
    digit = 0;
  } else {
    digit = 1;
  }
  uint8_t shiftMod = setCathodeColours(digit);
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);
  // shift out the bits of 7segValue to the 74HC595
  shiftOut2(dataPin, clockPin, MSBFIRST, segValue[digit]+shiftMod); //LSBFIRST
  
  //set latch pin high- this sends data to outputs so the LEDs will light up
  digitalWrite(latchPin, HIGH);
 
}

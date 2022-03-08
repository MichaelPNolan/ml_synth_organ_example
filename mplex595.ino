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
#define ADC_MUL_S2_PIN  17 (change from 13)  //MIDI OUT
#define ADC_MUL_SIG_PIN 34 (change from 12) //34 tested ok for reading analogue
#define I2S_BCLK_PIN    25
#define I2S_WCLK_PIN    27
#define I2S_DOUT_PIN    26
MIDI related
#define RXD2 16
#define TXD2 17
#define MIDI_PORT2_ACTIVE
#define MIDI_RX2_PIN RXD2
#midiUSB  Connections via VSPI
 *  CS:                 5
 *  INT:                17 (not used)
 *  SCK:                18
 *  MISO:               19
 *  MOSI:               23
 *  If you wanted to use HSPI :  13, 12, 14, 15 would be needed
The OLEDscreen is on 
 *  SCL 22 
 *  SDA  21
 *  AND enable 14
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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define DISPLAY_ADDRESS 0x70

             //discovered 34-39 are input online wtf

//#define DEBUG_SHIFT   //uncomment to use print binary debug info                 
#define latchPin      4  //green tested with UNO 5v and tested with ESP32
#define clockPin      2  //yellow
#define dataPin       15  //blue
// The cathode side of the digits on the ACD8143
// Steps - Choose PWM channel (esp32) from 0 to 15, set frequency 5000hz for LED,
//         duty cycle resolution 1 to 16 bits [8], which GPIO with ledcAttachPin(GPIO, channel)
#define ledFrequency  1000
#define ledcReso      8    //8 bits for duty cycle
#define redDigitOne   1    // shift reg bit 0  was previously a GPIO12
#define greenDigitOne 2    //shift reg bit 5
#define blueDigitOne  4    //shift reg bit 6
#define redDigitTwo   8    // sift reg  bit 1 was previously gpio13
#define greenDigitTwo 16    //shift reg bit 7
#define blueDigitTwo  32   //shift reg bit 8
#define monoDigitThree 64  //just set lights on for digit 3 which is not RGB

typedef  enum{ red, green, blue, yellow, purple, teal, white } digitColor;
#define enableShiftRegister 14

digitColor  digitOneColor,digitTwoColor;
uint8_t segValue[3]; //2 digits of an RGB 7 seg display [0] is msb and [1] is lsb [2] is the 3rd digit which is red only
uint8_t segCol[3];

bool    segDP;
static uint8_t digit = false; //true = lsb and false = msb


Adafruit_AlphaNum4 disp = Adafruit_AlphaNum4();

String text_to_display;
int string_pointer = 0;
char display_buffer[4];
long last_display_refresh = 0L;
bool scrollMode,needsWrite;
void setupAlpha4() // an i2c 14 segment LED display using Adafruit libraries
{
  disp.begin(DISPLAY_ADDRESS);
  disp.setBrightness(4);
  disp.clear();
  disp.writeDisplay();
  text_to_display.reserve(16);
  text_to_display = "  SUPI-ORGAN  ";
  scrollMode = true;
  needsWrite = true;
}
        
void textAlphaDisplay(String text){
    int theLen = text.length();
    if(theLen <17)
      text_to_display = text;
    else
      text_to_display = text_to_display.substring(0, 15);

    if(theLen<5)
      scrollMode = false;
    else
      scrollMode = true;
    needsWrite = true;
}

void setupMplex() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);  // 74HC595 Pins
  pinMode(dataPin, OUTPUT);
  pinMode(redDigitOne, OUTPUT);
  pinMode(redDigitTwo, OUTPUT);
  pinMode(enableShiftRegister, OUTPUT);
  

  //Serial.println("R1: "+String(redDigitOne)+" R2: "+String(redDigitTwo));
  setDigitsColor(red); 
  setDigits(0);
  setupAlpha4();

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
  segCol[0] = setCathodeColours(0);
  segCol[1] = setCathodeColours(1);
  segCol[2] = 191; //just the single digit pin low 
  #ifdef DEBUG_SHIFT
  printBinary(segCol[0]);
  printBinary(segCol[1]);
  #endif
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
       //delayMicroseconds(1);
       //delay(1);
       __asm__ __volatile__ ("nop\n\t");
       __asm__ __volatile__ ("nop\n\t");
       digitalWrite(clockPin, false);
       __asm__ __volatile__ ("nop\n\t");
       __asm__ __volatile__ ("nop\n\t");//really fast
       //delayMicroseconds(1); /faster
       //delay(1);//slow
    }
}
#ifdef DEBUG_SHIFT
void printBinary(uint8_t n){
  while (n) {
      if (n & 1)
          Serial.print("1");
      else
          Serial.print("0");
  
      n >>= 1;
  }
  Serial.println();
}
#endif

void setDigits(uint8_t value){ //should be less than 100 because we have only 2 digits
  //binary is fine it will be pushed into the 4511 7 seg decoder by the shift register
  
  if(value > 99){
    value = value%100;  //this should be a binary 1 for the second nibble which will light an individual 7seg mono
    segValue[2]  = 16;                              //segDP = false; //using the decimal point to indicate a 3rd digit of 1 - false means a 0 which will open the led 
  } else
     segValue[2]  = 0;                              //segDP = true; //true will blank the dp
  segValue[0] = (value / 10)+ segValue[2];

  segValue[1] = (value % 10)+ segValue[2] ;
  
 
  #ifdef DEBUG_SHIFT
  printBinary(segValue[0]+segBank);
  printBinary(segValue[1]+segBank);
  Serial.println("--------");
  #endif
}

uint8_t setCathodeColours(bool digitToSetOn){        //ledcWrite(redDigitOne, 0); PMW sink didn't work  //ledcWrite(redDigitTwo, 65535);
  uint8_t shiftMod = 255;      //set all bits high except 7th bit for 3rd digit by default means all off because current doesn't flow to high pins no light      
                           
  switch(digitOneColor){
    case red:
      //digitalWrite(redDigitOne,digitToSetOn); //1 is off so when digitToSetOn is 0 (the first digit) 
      if(!digitToSetOn)
        shiftMod -= redDigitOne;
      break;                                  //then digitOne gets 0 and sinks  - optimized out the if/then statement
    case green:
      //digitalWrite(redDigitOne,true); //1 means off
                                        // doing nothing with red now means off
      if(!digitToSetOn)
        shiftMod -= greenDigitOne;
      break;
    case blue:
      //digitalWrite(redDigitOne,true);
      if(!digitToSetOn)
        shiftMod -= blueDigitOne;
      break;
    case yellow:
      //digitalWrite(redDigitOne,digitToSetOn);
      if(!digitToSetOn){
        shiftMod -= greenDigitOne + redDigitOne;
      }
      break;
    case purple:
      //digitalWrite(redDigitOne,digitToSetOn);
      if(!digitToSetOn){
        shiftMod -= blueDigitOne + redDigitOne;
      }
      break;
    case teal:
       //digitalWrite(redDigitOne,true); // doing nothing with red now means off                                 
       if(!digitToSetOn){
         shiftMod -= blueDigitOne + greenDigitOne;
       }
       break;
    case white:
       //digitalWrite(redDigitOne,digitToSetOn);
       if(!digitToSetOn){
         shiftMod -= blueDigitOne + greenDigitOne + redDigitOne;

       }
       break;
  }
  switch(digitTwoColor){
    case red:
      //digitalWrite(redDigitTwo,!digitToSetOn); //1 is off so when digitToSetOn is 1 (the second digit) we flip it off with not operator
      if(digitToSetOn)
        shiftMod -= redDigitTwo;
      break;                                   //then digitTwo gets 0 and sinks  - optimized out the if/then statement
    case green:
      if(digitToSetOn)
         shiftMod -= greenDigitTwo;
      break;
    case blue:
      if(digitToSetOn)
        shiftMod -= blueDigitTwo;
      break;
    case yellow:
      if(digitToSetOn)
         shiftMod -= greenDigitTwo +redDigitTwo;
      break;
    case purple:
      if(digitToSetOn){
        shiftMod -= blueDigitTwo + redDigitTwo;
      }
      break;
    case teal:
       if(digitToSetOn){
         shiftMod -= blueDigitTwo + greenDigitTwo;
       }
       break;
     case white:
       if(digitToSetOn){
         shiftMod -= blueDigitTwo + greenDigitTwo + redDigitTwo;
       }
       break;
  }
   if (segValue[2] > 0)     
      shiftMod -= monoDigitThree; 
  return shiftMod;
}

void process7seg() {
  //each time we are called 0process a different row - multiplexing
  //moved this code into ISR (interrupt service routine)
  digitalWrite(enableShiftRegister,HIGH);
  //delayMicroseconds(1);
  if(digit>=1){ //multiplex switch colors for digit 0 on, wait a while, come back and toggle to digit 1
    digit = 0;
  } else {
    digit += 1;
  }
  //uint8_t shiftMod = setCathodeColours(digit);
 
  
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);
   //I setup "AND" gates for latch and clock so if this is HIGH they will work and if low they will be blocked low
  // shift out the bits of 7segValue to the 74HC595
  shiftOut2(dataPin, clockPin, MSBFIRST, segCol[digit]); 
  shiftOut2(dataPin, clockPin, MSBFIRST, segValue[digit]);//+segBank

  
  //set latch pin high- this sends data to outputs so the LEDs will light up
  digitalWrite(latchPin, HIGH);
  //delayMicroseconds(1); // this may be un-needed as it only takes 5-6 nanoseconds to open the gate 
  digitalWrite(enableShiftRegister,LOW);
 
}
void sleepCheck(){
  long current_millis = millis();
  if(current_millis > (last_display_refresh+50000)){
    disp.clear();
    disp.writeDisplay();
  }
}

void processAlpha()
{
  if(scrollMode){
    long current_millis = millis();
    
    if(current_millis - last_display_refresh > 300)
    {
    if(string_pointer >= text_to_display.length())
          string_pointer = 0;
  
    // Move the existing characters one position to the left
    for(int u = 0; u < 3; u++)
          display_buffer[u] = display_buffer[u + 1];
  
    // Replace the right-most character with the next
    // character from the text_to_display variable
    display_buffer[3] = text_to_display.charAt(string_pointer++);
     
    // send the text to the display
    for(int i = 0; i < 4; i++)
          disp.writeDigitAscii(i, display_buffer[i]);
      
    // display the text
    disp.writeDisplay();
  
    // update the timing variable
    last_display_refresh = current_millis;
    }
  } else if(needsWrite){
      needsWrite = false;
      for(int u = 0; u < 4; u++){
         disp.writeDigitAscii(u, text_to_display.charAt(u));
      }
      disp.writeDisplay();
      last_display_refresh = millis();
    } else
      sleepCheck();
    
}

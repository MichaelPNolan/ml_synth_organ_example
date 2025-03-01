/*
 * Copyright (c) 2022 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file ml_synth_organ_example.ino
 * @author Marcel Licence
 * @date 26.11.2021
 *
 * @brief   This is the main project file to test the ML_SynthLibrary (organ module)
 *          It should be compatible with ESP32, ESP8266, Seedstudio XIAO, PRJC Teensy 4.1, Electrosmith Daisy Seed, Raspberry Pi Pico, STM32F4
 *          #define CTRL_PERC_SWITCH    0
#define CTRL_PERC_SPEED     1
#define CTRL_PERC_NOTE      2
#define CTRL_PERC_LOUD      3
#define CTRL_PERC_POLY      4
#define CTRL_INTR_FEEDBACK  5
#define CTRL_ROTARY_ACTIVE  6
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include "config.h"


#include <Arduino.h>


/*
 * Library can be found on https://github.com/marcel-licence/ML_SynthTools
 */
#ifdef USE_ML_SYNTH_PRO
#include <ml_organ_pro.h>
#define CTRL_PERC_SWITCH    0
#define CTRL_PERC_SPEED     1
#define CTRL_PERC_NOTE      2
#define CTRL_PERC_LOUD      3
#define CTRL_PERC_POLY      4
#define CTRL_INTR_FEEDBACK  5
#define CTRL_ROTARY_ACTIVE  6
#else
#include <ml_organ.h>
#endif
#ifdef REVERB_ENABLED
#include <ml_reverb.h>
#endif
#include <ml_delay.h>
#ifdef OLED_OSC_DISP_ENABLED
#include <ml_scope.h>
#endif

#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
#include <Wire.h> /* todo remove, just for scanning */
#endif

int pitchedOn =  0;
bool percSwitch = false;
int loop0;

void setup()
{
    /*
     * this code runs once
     */



#ifdef ARDUINO_DAISY_SEED
    DaisySeed_Setup();
#endif

    delay(500);

#ifdef SWAP_SERIAL
    /* only one hw serial use this for ESP */
    Serial.begin(115200);
    delay(500);
#else
    Serial.begin(115200);
#endif

    Serial.println();


    Serial.printf("Loading data\n");

    Serial.printf("Firmware started successfully\n");



#ifdef BLINK_LED_PIN
   //Blink_Setup();
#endif

#ifdef ESP8266
    Midi_Setup();
#endif

    Serial.printf("Initialize Audio Interface\n");
    Audio_Setup();

#ifdef TEENSYDUINO
    Teensy_Setup();
#else

#ifdef ARDUINO_SEEED_XIAO_M0
    pinMode(7, INPUT_PULLUP);
    Midi_Setup();
    pinMode(LED_BUILTIN, OUTPUT);
#else

#ifndef ESP8266 /* otherwise it will break audio output */
    Midi_Setup();
#endif
#endif

#endif



#ifdef USE_ML_SYNTH_PRO
    OrganPro_Setup(&Serial, SAMPLE_RATE);
#else
    Organ_Setup(&Serial, SAMPLE_RATE);
#endif

#ifdef REVERB_ENABLED
    /*
     * Initialize reverb
     * The buffer shall be static to ensure that
     * the memory will be exclusive available for the reverb module
     */
    //static float revBuffer[REV_BUFF_SIZE];
    static float *revBuffer = (float *)malloc(sizeof(float) * REV_BUFF_SIZE);
    Reverb_Setup(revBuffer);
#endif

#ifdef MAX_DELAY
    /*
     * Prepare a buffer which can be used for the delay
     */
    //static int16_t delBuffer1[MAX_DELAY];
    //static int16_t delBuffer2[MAX_DELAY];
    static int16_t *delBuffer1 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    static int16_t *delBuffer2 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    Delay_Init2(delBuffer1, delBuffer2, MAX_DELAY);
#endif

#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif

#ifdef ESP32
    Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

    /* PSRAM will be fully used by the looper */
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
#endif

    Serial.printf("Firmware started successfully\n");

#ifdef NOTE_ON_AFTER_SETUP
#ifdef USE_ML_SYNTH_PRO
    OrganPro_NoteOn(0, 60, 127);
    OrganPro_SetLeslCtrl(127);
#else
    Organ_NoteOn(0, 60, 127);
    Serial.println("Note On test");
    Organ_SetLeslCtrl(127);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
#endif
#endif
// initial setting of percussion on
#ifdef USE_ML_SYNTH_PRO
   OrganPro_PercussionSet(CTRL_ROTARY_ACTIVE);
   OrganPro_PercussionSet(CTRL_ROTARY_ACTIVE);
   OrganPro_PercussionSet(CTRL_PERC_SWITCH);
   OrganPro_PercussionSet(CTRL_PERC_LOUD);
#else
   Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
   Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
   Organ_PercussionSet(CTRL_PERC_SWITCH);
   Organ_PercussionSet(CTRL_PERC_LOUD);
   //Organ_PercussionSet(CTRL_PERC_POLY); //not good
#endif
   setupMplex();

   
#if (defined ADC_TO_MIDI_ENABLED) ||(defined MIDI_VIA_USB_ENABLED) || (defined OLED_OSC_DISP_ENABLED)
#ifdef ESP32
    Core0TaskInit();
#else
#error only supported by ESP32 platform
#endif
#endif
}





#ifdef ESP32
/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;


inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

void Core0TaskSetup()
{
    /*
     * init your stuff for core0 here
     */

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Setup();
    Serial.println("Scope setup complete");
#endif

#ifdef ADC_TO_MIDI_ENABLED
    Serial.println("adc initialized");
    AdcMul_Init();
#endif

#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
    //-------------------inserting 7seg logic into this module
    loop0 = 0;
}

void Core0TaskLoop()
{
    /*
     * put your loop stuff for core0 here
     */
    
#ifdef MIDI_VIA_USB_ENABLED
   if(!usbFailCheck())
      UsbMidi_Loop();
#endif
    
#ifdef ADC_TO_MIDI_ENABLED
    loop0++; 
    if(loop0>67){
      AdcMul_Process();
      loop0=0;
      
    }
#endif /* ADC_TO_MIDI_ENABLED */
   process7seg();
   processAlpha();

#ifdef OLED_OSC_DISP_ENABLED
    if((loop0 % 11) == 0)
      ScopeOled_Process();
#endif
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}
#endif



void loop_1Hz()
{
#ifdef CYCLE_MODULE_ENABLED
    CyclePrint();
#endif
#ifdef BLINK_LED_PIN
   // Blink_Process();
#endif
}

void loop()
{
    static int loop_cnt_1hz = 0; /*!< counter to allow 1Hz loop cycle */

#ifdef SAMPLE_BUFFER_SIZE
    loop_cnt_1hz += SAMPLE_BUFFER_SIZE;
#else
    loop_cnt_1hz += 1; /* in case only one sample will be processed per loop cycle */
#endif
    if (loop_cnt_1hz >= SAMPLE_RATE)
    {
        loop_cnt_1hz -= SAMPLE_RATE;
        loop_1Hz();
    }

  //  if((loop_cnt_1hz % 30) == 0)
   //    process7seg();
    /*
     * MIDI processing
     */
    Midi_Process();
#ifdef MIDI_VIA_USB_ENABLED
    if(!usbFailCheck())
      UsbMidi_ProcessSync();
#endif

    /*
     * And finally the audio stuff
     */
#ifdef USE_ML_SYNTH_PRO
#if (defined ESP8266) || (defined ARDUINO_SEEED_XIAO_M0)|| (defined ARDUINO_RASPBERRY_PI_PICO)
#error Configuration is not supported
#else
    float mono[SAMPLE_BUFFER_SIZE], left[SAMPLE_BUFFER_SIZE], right[SAMPLE_BUFFER_SIZE];
    OrganPro_Process_fl(mono, SAMPLE_BUFFER_SIZE);

#ifdef INPUT_TO_MIX
    Audio_Input(left, right);
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] += (left[i] * 0.5f + right[i] * 0.5f) * 32.0f;
    }
#endif

#ifdef REVERB_ENABLED
    Reverb_Process(mono, SAMPLE_BUFFER_SIZE);
#endif

    Rotary_Process(left, right, mono, SAMPLE_BUFFER_SIZE);

#ifdef MAX_DELAY
    /*
     * post process delay
     */
    Delay_Process_Buff2(left, right, SAMPLE_BUFFER_SIZE);
#endif

    /*
     * Output the audio
     */
    Audio_Output(left, right);

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_AddSamples(left, right, SAMPLE_BUFFER_SIZE);
#endif
#endif
#else
    int32_t mono[SAMPLE_BUFFER_SIZE];
    Organ_Process_Buf(mono, SAMPLE_BUFFER_SIZE);
#ifdef REVERB_ENABLED
    float mono_f[SAMPLE_BUFFER_SIZE];
    

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono_f[i] = mono[i];
    }
    #ifdef OLED_OSC_DISP_ENABLED //added hack by Michael N for testing the Oled Scope module from the Synth library
    
    int scopeSize = SAMPLE_BUFFER_SIZE/4;
    float mono_scope[scopeSize];
    for (int i = 0; i < scopeSize; i++)
    {
        mono_scope[i] = mono[i];
    }
    #endif
    
    Reverb_Process(mono_f, SAMPLE_BUFFER_SIZE); /* post reverb */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] = mono_f[i];
    }
#endif
    Audio_OutputMono(mono);
       
    #ifdef OLED_OSC_DISP_ENABLED 
       ScopeOled_AddSamples(mono_scope, mono_scope, scopeSize);
    #endif
    

#endif

}

/*
 * MIDI via USB Host Module
 */
#ifdef MIDI_VIA_USB_ENABLED
void App_UsbMidiShortMsgReceived(uint8_t *msg)
{   
    #ifdef MIDI_TX2_PIN
    Midi_SendShortMessage(msg);
    #endif
    Midi_HandleShortMsg(msg, 8);
}
#endif

/*
 * MIDI callbacks
 */
inline void Organ_PercSetMidi(uint8_t setting, uint8_t value)
{
    if (value > 0)
    {
#ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(setting);

#else
        Organ_PercussionSet(setting);
#endif
    }
}

inline void Organ_SetDrawbarInv(uint8_t id, uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetDrawbar(id, value);
#else
    
    if(id == 1){
      Organ_SetDrawbar(0, value);
      textAlphaDisplay("Bar0+1");
    } else
      textAlphaDisplay("Bar"+String(id));
    
    Organ_SetDrawbar(id, value);
    setDigitsColor(id%7);
    setDigits(value);
    
#endif
}

inline void Organ_SetCtrl(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
    //Serial.print("LesCtr: ");
#endif
}

inline void Organ_SetLeslieSpeedNorm(uint8_t unused __attribute__((unused)), uint8_t speed)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(speed);
#else
    Organ_SetLeslCtrl(speed);
#endif
}

inline void Organ_ModulationWheel(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
#endif
}

inline void Organ_ButtonSelect(uint8_t unused __attribute__((unused)), uint8_t value)
//for use of a series of buttons in a resistor ladder that are read by the ADC module
{
  Serial.println("Button: "+String(value));
  switch(value){
    case 0 ... 16: //voltages of no button press
      break;
    case 17 ... 30:
      Organ_PercussionSet(CTRL_PERC_SWITCH);
      textAlphaDisplay("Percussion");
      break;
     case 39 ... 48:
       textAlphaDisplay("But5");
       break;
     case 57 ... 68:
       textAlphaDisplay("But4");
       break;
     case 73 ... 88:
       textAlphaDisplay("But3");
       break;
     case 90 ... 105:
       textAlphaDisplay("But2");
       break;
     case 108 ... 127:
       textAlphaDisplay("But1");
       break;

  } 
}

inline void Organ_PercussionViaPitch(uint8_t unused __attribute__((unused)), uint8_t value)
{
  if(value == 0){
    if(pitchedOn>0)
       pitchedOn--;
     else 
       if(pitchedOn<0)
         pitchedOn++;
       else
         percSwitch = false;
  } else 
    if(value == 1){
      pitchedOn++;
      if(pitchedOn>12){
        pitchedOn=0;
        #ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(CTRL_PERC_SPEED);
        #else
        Organ_PercussionSet(CTRL_PERC_SPEED);
        #endif
        
        Serial.println("speed toggle");
      }
    } else
      if(value == 255){
        pitchedOn--;
        if(pitchedOn<-12)
          pitchedOn=0;
    } else if(value == 254){
       #ifdef USE_ML_SYNTH_PRO
       OrganPro_PercussionSet(CTRL_INTR_FEEDBACK);
       #else
       Organ_PercussionSet(CTRL_INTR_FEEDBACK);
       #endif
       
       Serial.println("feedback toggle");
    }
  
  if(!percSwitch){       
    if(pitchedOn >2){
        #ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(CTRL_PERC_SWITCH);
        #else
        Organ_PercussionSet(CTRL_PERC_SWITCH);
        #endif
        
        percSwitch = true;
        Serial.println("perc toggle");
    }
    if(pitchedOn < -2){
        #ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(CTRL_PERC_LOUD);
        #else
        Organ_PercussionSet(CTRL_PERC_LOUD);
        #endif

        percSwitch = true;  //this means no more changes until pitch bender returns to mid point
        Serial.println("loud toggle");
    }
  }
}


inline void Reverb_SetLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Reverb_SetLevel(unused, val);
#endif
}

inline void Delay_SetOutputLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetOutputLevel(unused, val);
#endif
}

inline void Delay_SetFeedbackInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetFeedback(unused, val);
#endif
}

#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
void  ScanI2C(void)
{
#ifdef ARDUINO_GENERIC_F407VGTX
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();//I2C_SDA, I2C_SCL);
#else
    Wire.begin();
#endif

    byte address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        byte r_error;
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        r_error = Wire.endTransmission();

        if (r_error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (r_error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
#endif /* (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG) */

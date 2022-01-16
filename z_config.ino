/*
 * Copyright (c) 2021 Marcel Licence
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
 * @file z_config.ino
 * @author Marcel Licence
 * @date 12.05.2021
 *
 * @brief This file contains the mapping configuration
 * Put all your project configuration here (no defines etc)
 * This file will be included at the and can access all
 * declarations and type definitions
 *
 * @see ESP32 Arduino DIY Synthesizer Projects - Little startup guide to get your MIDI synth working - https://youtu.be/ZNxGCB-d68g
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#ifdef AUDIO_KIT_BUTTON_ANALOG
audioKitButtonCb audioKitButtonCallback = NULL;
#endif

/*
 * this mapping is used for the edirol pcr-800
 * this should be changed when using another controller
 */
 struct midiControllerMapping tunaWorlde[] =
{
  #ifdef USE_UNISON
    { 0x0, 0x0a, "R1", NULL, Organ_SetDrawbarInv, 0}, 
    { 0x0, 0x0b, "R2", NULL, Organ_SetDrawbarInv, 1},
#else
    { 0x0, 0x0a, "R1", NULL, Organ_SetDrawbarInv, 2},  //0x0 - 0x7, 0x10 little tiger toy midi controller  Organ_SetDrawbarInv, SYNTH_PARAM_WAVEFORM_1
    { 0x0, 0x0b, "R2", NULL, Organ_SetDrawbarInv, 3},  //Organ_SetDrawbarInv, SYNTH_PARAM_WAVEFORM_2
#endif
    { 0x0, 0x0c, "R3", NULL, Organ_SetDrawbarInv, 4},                      //Delay_SetLength, 2
    { 0x0, 0x0d, "R4", NULL, Organ_SetDrawbarInv, 5},                       //Delay_SetLevel, 3    //Delay_SetFeedback, 4

    { 0x0, 0x0e, "R5", NULL, Organ_SetDrawbarInv, 6},
    { 0x0, 0x0f, "R6", NULL, Organ_SetDrawbarInv, 7},
    { 0x0, 0x10, "R7", NULL, Organ_SetDrawbarInv, 8},
    { 0x0, 0x11, "R8", NULL, Organ_SetCtrl, 0},

    { 0x0, 0x14, "R9", NULL, Organ_SetCtrl, 1},          //Delay_SetLevel, 3    //Delay_SetFeedback, 4
    { 0x0, 0x15, "R10", NULL, Organ_SetCtrl, 2},
    { 0x0, 0x16, "R11", NULL, Organ_SetCtrl, 3},

    { 0x0, 0x17, "R12", NULL, Organ_SetCtrl, 4},
    { 0x0, 0x18, "R13", NULL, Organ_SetCtrl, 5},
    { 0x0, 0x19, "R14", NULL, Organ_SetCtrl, 6},
    { 0x0, 0x1a, "R15", NULL, Organ_SetCtrl, 7 },
    { 0x0, 0x1b, "R15", NULL, Organ_SetCtrl, 8},

    // Central slider 
   // { 0x0, 0x13, "H1", NULL, NULL, 0},
};
struct midiControllerMapping edirolMapping[] =
{
    /* general MIDI */
    { 0x0, 0x40, "sustain", NULL, Organ_SetLeslieSpeedNorm, 0},

    /* transport buttons */
    { 0x8, 0x52, "back", NULL, NULL, 0},
    { 0xD, 0x52, "stop", NULL, NULL, 0},
    { 0xe, 0x52, "start", NULL, NULL, 0},
    { 0xe, 0x52, "start", NULL, NULL, 0},
    { 0xa, 0x52, "rec", NULL, NULL, 0},

    /* upper row of buttons */
    { 0x0, 0x50, "A1", NULL, NULL, 0},
    { 0x1, 0x50, "A2", NULL, NULL, 1},
    { 0x2, 0x50, "A3", NULL, NULL, 2},
    { 0x3, 0x50, "A4", NULL, NULL, 3},

    { 0x4, 0x50, "A5", NULL, NULL, 0},
    { 0x5, 0x50, "A6", NULL, NULL, 1},
    { 0x6, 0x50, "A7", NULL, NULL, 2},
    { 0x7, 0x50, "A8", NULL, NULL, 0},

    { 0x0, 0x53, "A9", NULL, NULL, 0},

    /* lower row of buttons */
    { 0x0, 0x51, "B1", NULL, Organ_PercSetMidi, 0},
    { 0x1, 0x51, "B2", NULL, Organ_PercSetMidi, 1},
    { 0x2, 0x51, "B3", NULL, Organ_PercSetMidi, 2},
    { 0x3, 0x51, "B4", NULL, Organ_PercSetMidi, 3},

    { 0x4, 0x51, "B5", NULL, Organ_PercSetMidi, 4},
    { 0x5, 0x51, "B6", NULL, Organ_PercSetMidi, 5},
    { 0x6, 0x51, "B7", NULL, NULL, 2},
    { 0x7, 0x51, "B8", NULL, NULL, 3},

    { 0x1, 0x53, "B9", NULL, NULL, 0},

    /* pedal */
    { 0x0, 0x0b, "VolumePedal", NULL, NULL, 0},

    /* slider */
    { 0x0, 0x11, "S1", NULL, Organ_SetDrawbarInv, 0},
    { 0x1, 0x11, "S2", NULL, Organ_SetDrawbarInv, 1},
    { 0x2, 0x11, "S3", NULL, Organ_SetDrawbarInv, 2},
    { 0x3, 0x11, "S4", NULL, Organ_SetDrawbarInv, 3},

    { 0x4, 0x11, "S5", NULL, Organ_SetDrawbarInv, 4},
    { 0x5, 0x11, "S6", NULL, Organ_SetDrawbarInv, 5},
    { 0x6, 0x11, "S7", NULL, Organ_SetDrawbarInv, 6},
    { 0x7, 0x11, "S8", NULL, Organ_SetDrawbarInv, 7},

    { 0x1, 0x12, "S9", NULL, Organ_SetDrawbarInv, 8},

    /* rotary */
    { 0x0, 0x10, "R1", NULL, Organ_SetCtrl, 0},
    { 0x1, 0x10, "R2", NULL, Organ_SetCtrl, 1},
    { 0x2, 0x10, "R3", NULL, Organ_SetCtrl, 2},
    { 0x3, 0x10, "R4", NULL, Organ_SetCtrl, 3},

    { 0x4, 0x10, "R5", NULL, Organ_SetCtrl, 4},
    { 0x5, 0x10, "R6", NULL, Organ_SetCtrl, 5},
    { 0x6, 0x10, "R7", NULL, Organ_SetCtrl, 6},
    { 0x7, 0x10, "R8", NULL, Organ_SetCtrl, 7},

    { 0x0, 0x12, "R9", NULL, Organ_SetCtrl, 8},

    /* Central slider */
    { 0x0, 0x13, "H1", NULL, Organ_SetLeslieSpeedNorm, 0},
};

struct midiMapping_s midiMapping =
{
    NULL,
    Organ_NoteOn,
    Organ_NoteOff,
    NULL,
    Organ_ModulationWheel,
    tunaWorlde,
    sizeof(tunaWorlde) / sizeof(tunaWorlde[0]),
};

#ifdef MIDI_VIA_USB_ENABLED
struct usbMidiMappingEntry_s usbMidiMappingEntries[] =
{
    {
        NULL,
        App_UsbMidiShortMsgReceived,
        NULL,
        NULL,
        0xFF,
    },
};

struct usbMidiMapping_s usbMidiMapping =
{
    NULL,
    NULL,
    usbMidiMappingEntries,
    sizeof(usbMidiMappingEntries) / sizeof(usbMidiMappingEntries[0]),
};
#endif /* MIDI_VIA_USB_ENABLED */

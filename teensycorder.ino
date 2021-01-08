
//--------------------------------------------------------------------//
//--                  -----   Teensycorder   -----                  --//
//--    A MIDI baroque recorder/blockflute with responsive touch    --//
//--     input using Teensy 3.2's capacitive touch enabled pins.    --//
//--       Written for Teensy 3.2 with MIDIUSB library v1.0.4       --//
//--              Copyright (C) 2020 - Michael Koopman              --//
//--          KOOP Instruments (koopinstruments@gmail.com)          --//
//--------------------------------------------------------------------//
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
//
// Hardware Information:
// Requires Teensy 3.2
// I/O Pin requirments:
//  21 pins in total
//      12 touchRead
//       8 digital
//       1 analog
//
// Device pinout (Left/right perspective is of face containing input):
// Name                     Teensy 3.2 Pin
// ------------------------------------
// Breath Sensor                A14
// Thumb Left                   32
// Thumb Right                  33
// Finger 1                      0
// Finger 2                      1
// Finger 3                     15
// Finger 4                     16
// Finger 5                     17
// Finger 6 Left                18
// Finger 6 Right               19
// Finger 7 Left                22
// Finger 7 Right               23
// Bell Key                     25
// Top Rear UL Button            5
// Top Rear UR Button            6
// Top Rear LL Button            7
// Top Rear LR Button            8
// Bottom Rear UL Button         9
// Bottom Rear UR Button        10
// Bottom Rear LL Button        11
// Bottom Rear LR Button        12
// Meta Button                   4
// Ground                       GND
//
// Rear Button Functions:
// Normal:
//    Octave Down -> ● ● <- Octave Up
//   Program Down -> ● ● <- Program Up
//                    -
// Drone Note Set -> ● ● <- Threshold Reset
//  Velocity Down -> ● ● <- Velocity Up
//                    ○  <- [Meta Button]
//--------------------------------------
// Meta:
// Transpose Down -> ● ● <- Transpose Up
//   Channel Down -> ● ● <- Channel Up
//                    -
//     Drone Type -> ● ● <- Device Reset
//  Reverb Enable -> ● ● <- Chorus Enable
//                    ●  <- [Meta Button]
//
// Recorder Fingerings:
// Thumb = 0, Left Hand = 1-3, Right Hand = 4-7, Bell Hole = 8
// Note     - 00 | 1 2 3 | 4 5 66 77 | 8
// B3       - ●● | ● ● ● | ● ● ●● ●● | ●
// C4       - ●● | ● ● ● | ● ● ●● ●● | ○
// C#4/Db4  - ●● | ● ● ● | ● ● ●● ●○ | ○
// D4       - ●● | ● ● ● | ● ● ●● ○○ | ○
// D#4/Eb4  - ●● | ● ● ● | ● ● ●○ ○○ | ○
// E4       - ●● | ● ● ● | ● ● ○○ ○○ | ○
// F4       - ●● | ● ● ● | ● ○ ●● ●● | ○
// F#4/Gb4  - ●● | ● ● ● | ○ ● ●● ○○ | ○
// G4       - ●● | ● ● ● | ○ ○ ○○ ○○ | ○
// G#4/Ab4  - ●● | ● ● ○ | ● ● ●○ ○○ | ○
// A4       - ●● | ● ● ○ | ○ ○ ○○ ○○ | ○
// A#4/Bb4  - ●● | ● ○ ● | ○ ● ●● ○○ | ○
//            ●● | ● ○ ● | ● ○ ○○ ○○ | ○
// B4       - ●● | ○ ● ● | ○ ○ ○○ ○○ | ○
//            ●● | ● ○ ○ | ○ ○ ○○ ○○ | ○
// C5       - ●● | ○ ● ○ | ○ ○ ○○ ○○ | ○
// C#5/Db5  - ○○ | ● ● ○ | ○ ○ ○○ ○○ | ○
// D5       - ○○ | ● ● ● | ● ● ●● ●● | ○
//            ○○ | ○ ● ○ | ○ ○ ○○ ○○ | ○
// D#5/Eb5  - ○○ | ● ● ● | ● ● ●● ○○ | ○
//            ○○ | ○ ● ● | ● ● ●● ○○ | ○
// E5       - ●○ | ● ● ● | ● ● ○○ ○○ | ○
// F5       - ●○ | ● ● ● | ● ○ ●● ○○ | ○
// F#5/Gb5  - ●○ | ● ● ● | ○ ● ○○ ○○ | ○
// G5       - ●○ | ● ● ● | ○ ○ ○○ ○○ | ○
// G#5/Ab5  - ●○ | ● ● ○ | ● ○ ○○ ○○ | ○
// A5       - ●○ | ● ● ○ | ○ ○ ○○ ○○ | ○
// A#5/Bb5  - ●○ | ● ● ○ | ○ ● ●● ●○ | ○
//            ●○ | ● ● ○ | ○ ● ●● ○○ | ○
// B5       - ●○ | ● ● ○ | ● ● ○○ ○○ | ○
// C6       - ●○ | ● ○ ○ | ● ● ○○ ○○ | ○
// C#6/Db6  - ●○ | ● ○ ● | ● ○ ●● ●○ | ●
//            ●○ | ● ○ ● | ● ○ ●● ○○ | ●
// D6       - ●○ | ● ○ ● | ● ○ ●● ●○ | ○
//            ●○ | ● ○ ● | ● ○ ●● ○○ | ○
// D#6/Eb6  - ●○ | ○ ● ● | ○ ● ●● ●○ | ○
//            ●○ | ○ ● ● | ○ ● ●● ○○ | ○
//            ●○ | ○ ● ● | ○ ○ ○○ ○○ | ○
// E6       - ●○ | ○ ● ● | ○ ● ●● ●○ | ●
//          - ●○ | ○ ● ● | ○ ● ●● ○○ | ●
// F6       - ●○ | ● ● ○ | ● ● ○○ ○○ | ●
//            ●○ | ● ○ ● | ● ● ○○ ○○ | ●
// F#6/Gb6  - ●○ | ● ○ ● | ● ● ○○ ○○ | ○
// G6       - ●○ | ● ○ ○ | ● ○ ○○ ○○ | ○

// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// START OF PROGRAM

// Define Teensy 3.x hardware restart variables for use in the runResetController() function
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

#include "MIDIUSB.h"

// Pin setup
const byte breathPin        = A14;  // Breath Sensor
const byte NUM_TOUCH_PINS   = 12;   // Total number of capacitive touchRead pins
const byte NUM_PUSH_BUTTONS = 9;    // Total number of digital push buttons

// Look up table of pins in order of assignment
const byte touchPins[NUM_TOUCH_PINS]    = { 25, 23, 22, 19, 18, 17, 16, 15, 1, 0, 33, 32 };
const byte buttonPins[NUM_PUSH_BUTTONS] = { 5, 6, 7, 8, 9, 10, 11, 12, 4 };

// Default velocity
byte velocity = 95;                     // Default value, but can be adjusted by the user

// Note value offset variables
int8_t octave;                          // Octave offset is applied to note value in user adjustable steps of 12
int8_t transpose;                       // Transpose offset is applied to note value in user adjustable steps of 1

// MIDI program
byte midiProgram[16];                   // Array to store program for all 16 MIDI channels

// MIDI Channels (range of 0-15)
byte midiChannel = 15;                  // Default to 15 (MIDI channel 16)
byte droneChannel = 14;                 // Default to 14 (MIDI channel 15)

// MIDI CC variables
byte reverbActive;                      // Toggle variable for reverb activation
byte chorusActive;                      // Toggle variable for chorus activation
byte expression;                        // Value used to determine expression and pitchBend levels
byte previousExpression;                // Comparison value to minimze controlChange event sending

// Digital button variables
unsigned int debounceTime = 40;                     // Debounce time in milliseconds
unsigned long previousButtonTime[NUM_PUSH_BUTTONS]; // Previous activation time
int pendingButtonState[NUM_PUSH_BUTTONS];          // Pre-debounce candidate states
int buttonState[NUM_PUSH_BUTTONS];                 // Post-debounce stable states
int previousButtonState[NUM_PUSH_BUTTONS];         // Previous values for comparison

// Bitwise variables for storing touchRead button states and comparison values
int rawTouchStates;                     // The raw readings from the sensors
int pendingTouchStates;                 // Pending candidate waiting for smoothing time threshold
int stableTouchStates;                  // The stabalized variable sent to the note hopper

// Breath sensor variables
unsigned long previousBreathTime;       // Previous activation time (shares noteSmoothingTime threshold to keep breath in sync with fingering)
byte rawBreathState;                    // The raw reading from the sensor
byte pendingBreathState;                // Pending candidate waiting for smoothing time threshold
byte stableBreathState;                 // The stabalized state
byte previousBreathState;               // Comparison value used to send note closings

// touchRead activation threshold
int touchMinVal[NUM_TOUCH_PINS];                // Minimum value for calculations
int touchMaxVal[NUM_TOUCH_PINS];                // Maximum value for calculations
int touchActivationThreshold[NUM_TOUCH_PINS];   // Calculated activation threshold

// Breath activation threshold
int breathBaseVal;                      // Neutral breath reading
int breathActivationThreshold;          // Calculated activation threshold
int breathMaxVal;                       // Maximum value for mapping calculations

// Note input smoothing variables
const int noteSmoothingTime = 10000;    // touchRead notes need to be active for >= N μs to play.  This filters out unintentional blips when changing fingerings, but does introduce latency.
                                        // Adjust to suit user preference between fast responsive playing with a sprinkling of 10ms squaks, or sluggish playing with smooth consistent notes.
                                        // Unfortunately, it seems the more pins that are touched, the longer the touchRead function takes to complete, adding about 3-4 ms per touched pin.
                                        // This makes fast play of lower notes a little difficult, and the added delay only exacerbates this problem.  To counteract this, a latency offset
                                        // is calculated and subtracted from the threshold in the readCapacitivePins() function.
signed int latencyOffsetInitial;        // Starting time for the latency offset calculation
signed int latencyOffset;               // Total latency offset this program loop
unsigned long previousTouchTime;        // Variable to store the time since last touch input change

// Active pitches
byte activePitch;                       // The current active pitch used to send a noteOn
byte previousActivePitch;               // Previous active pitch for comparison and to send noteOff

// Drone
byte droneNoteActive;                   // Toggle variable for drone note activation
byte droneNoteValue;                    // Saved value to send a noteOff when disabled
byte droneNoteHold;                     // Variable for sending drone only on breath (drone flute), or constantly (bagpipes)


// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// START SETUP SECTION
void setup()
{
    // Set pinModes
    pinMode(breathPin,  INPUT);                             // Set analog breath sensor to INPUT

    for (int myPin = 0; myPin < NUM_PUSH_BUTTONS; myPin++)  // For all digital push buttons
    {
        pinMode(buttonPins[myPin], INPUT_PULLUP);           // Set pins to INPUT_PULLUP so that we don't need resistors
    }

    delay(3000);                                            // Delay a bit to allow computer USB detection to finish and readings to settle

    for (int myPin = 0; myPin < NUM_TOUCH_PINS; myPin++)    // For all capacitive touch pins
    {
        int myValue = touchRead(touchPins[myPin]);          // Store reading in a temporary variable
        touchMinVal[myPin] = myValue;                       // Set initial minimum value to current reading
        touchMaxVal[myPin] = myValue;                       // Set initial maximum value to current reading
        delay(40);
    }


    breathBaseVal = analogRead(breathPin);          // Set neutral breath pressure level
    breathActivationThreshold = breathBaseVal + 4;
    breathMaxVal = breathBaseVal + 64;

    midiProgram[midiChannel] = 71;                          // Default to General MIDI Program #71 (Clarinet)
    programChange(midiChannel, midiProgram[midiChannel]);   // Send an initial program change on main channel
    programChange(droneChannel, midiProgram[midiChannel]);  // Send an initial program change on drone channel

}
// END SETUP SECTION
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// START LOOP SECTION
void loop()
{
    // Print hardware diagnostic information to the serial console
    // printDiagnostics();

    // Read and store the digital button states
    readDigitalButtons();

    // Read and store the capacative pin states
    readCapacitivePins();

    // Read and store the breath sensor state
    readBreathSensor();

    // Run the octave select function
    runOctaveSelect();

    // Run the transpose select function
    runTransposeSelect();

    // Run the program select function
    runProgramSelect();

    // Run the channel select function
    runChannelSelect();

    // Run the drone function
    runDroneMode();

    // Run the velocity selection function
    runVelocitySelect();

    // Run the MIDI CC function
    runMidiCC();

    // Run the touchRead activation threshold reset function
    runResetThresholds();

    // Run the controller reset function
    runResetController();

    // Send notes to the MIDI bus
    playNotes();

    // Reset input locking variables for next loop
    for (int myButton = 0; myButton < NUM_PUSH_BUTTONS; myButton++) // For all digital push buttons
    {
        if(buttonState[myButton] == LOW)                            // If state is now LOW
        {
            previousButtonState[myButton] = LOW;                    // Set comparison variable to match
        }
    }

}

// END LOOP SECTION
// ------------------------------------------------------------------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// START FUNCTIONS SECTION


void printDiagnostics()
{
    Serial.print("breathPin:\t\t\t");
    Serial.print(breathBaseVal);Serial.print("|");Serial.print(breathActivationThreshold);Serial.print("|");Serial.println(analogRead(breathPin));

    Serial.print("touchPins:\t\t\t");
    for (int myPin = 0; myPin < NUM_TOUCH_PINS; myPin++)
    {
        Serial.print(touchMinVal[myPin]);Serial.print("|");Serial.print(touchActivationThreshold[myPin]);Serial.print("|");Serial.print(touchRead(touchPins[myPin]));Serial.print("|");Serial.print(touchMaxVal[myPin]);Serial.print("\t");
    }
    Serial.println();

    Serial.print("digiButtons:\t\t\t");
    for (int myPin = 0; myPin < NUM_PUSH_BUTTONS; myPin++)
    {
        Serial.print(!digitalRead(buttonPins[myPin]));Serial.print("\t");
    }
    Serial.println();
    Serial.println();
    delay(10);              // Delay a bit to stabalize the serial monitor stream
}


void readDigitalButtons()
{
    for (int myButton = 0; myButton < NUM_PUSH_BUTTONS; myButton++)     // For all digital push buttons
    {
        byte myReading = !digitalRead(buttonPins[myButton]);            // Invert reading due to INPUT_PULLUP
        if (myReading != pendingButtonState[myButton])                  // If a state change is detected
        {
            previousButtonTime[myButton] = millis();                    // Record the current time
            pendingButtonState[myButton] = myReading;                   // Update comparison variable
        }
        if ((millis() - previousButtonTime[myButton]) > debounceTime)   // If button passes debounce
        {
            buttonState[myButton] = pendingButtonState[myButton];       // Update final button state
        }
    }
}


void readCapacitivePins()
{
    latencyOffsetInitial = micros();                                            // Initialize latency offset tracker
    for (int myPin = 0; myPin < NUM_TOUCH_PINS; myPin++)                        // For all capacitive touch pins
    {
        int myValue = touchRead(touchPins[myPin]);                              // Store the current touchRead pin value
        if (myValue < touchMinVal[myPin] || myValue == 0) { touchMinVal[myPin] = myValue; }     // Find the minimum value for this pin
        if (myValue > touchMaxVal[myPin] || myValue == 65535) { touchMaxVal[myPin] = myValue; }     // Find the maximum value for this pin
        touchActivationThreshold[myPin] = ((touchMaxVal[myPin] - touchMinVal[myPin]) / 8) + touchMinVal[myPin];     // Determine a good activation threshold
        if (myValue > touchActivationThreshold[myPin])                          // If current touchRead value is higher than activation threshold
        {
             bitWrite(rawTouchStates, myPin, 1);                                // Write a 1 to this button's location in the bitwise variable
        }
        else
        {
            bitWrite(rawTouchStates, myPin, 0);                                 // Otherwise write a 0
        }
    }
    latencyOffset = (micros() - latencyOffsetInitial);                          // Determine total latency due to inconsistent touchRead duration
    if (pendingTouchStates != rawTouchStates)                                   // If the current button state is different from our potential candidate
    {
        pendingTouchStates = rawTouchStates;                                    // Update the comparison variable
        previousTouchTime = micros() - latencyOffset;                           // Record the current time (minus a latency offset due to inconsistent touchRead duration)
    }
    if ((micros() - previousTouchTime) >= noteSmoothingTime)                    // If the current note has stabilized past our threshold
    {
        stableTouchStates = pendingTouchStates;                                 // Update the stable stableTouchStates variable for sending to note detection
    }
}


void readBreathSensor()
{
    int pressureReading = analogRead(breathPin);                                // Take a reading from the breath sensor
    if ((pressureReading > breathActivationThreshold) || (droneNoteHold == HIGH && droneNoteActive == HIGH))
    {
        rawBreathState = HIGH;                                                  // Set raw input state to HIGH
    }
    else
    {
        rawBreathState = LOW;                                                   // Otherwise set to LOW
    }
    if (pendingBreathState != rawBreathState)                                   // If a change has been detected
    {
        pendingBreathState = rawBreathState;                                    // Capture the value in our candidate variable
        previousBreathTime = micros() - latencyOffset;                          // Record the time for input smoothing/debounce
    }
    if ((micros() - previousBreathTime) > noteSmoothingTime)                    // If the value has stabalized past our desired duration
    {
        stableBreathState = pendingBreathState;                                 // Capture the stabalized value in a variable
    }
    if (stableBreathState == HIGH)                                              // Determine expression level and map to 7-bit value within an appropriate range
    {
        expression = map(constrain(pressureReading, breathActivationThreshold, breathMaxVal), breathActivationThreshold, breathMaxVal,  95, 127); // Map expression to a usable range
    }
    if (stableBreathState == LOW)                                               // If the breath sensor is inactive
    {
        expression = 95;                                                        // Revert to a neutral expression level
    }

    if (stableBreathState == HIGH && droneNoteHold == LOW)                      // Don't send expression changes when in droneNoteHold (think bagpipes) mode
    {
        if (expression != previousExpression)                                   // Only send controlChange messages if there is a difference to minimize updates
        {
            if (pressureReading > breathActivationThreshold)                 // If pressure reading passes our threshold for normal blowing
            {
                controlChange(midiChannel, 11, expression);                     // Send MIDI CC #11 (Expression) volume changes
                controlChange(droneChannel, 11, expression);                    // Send MIDI CC #11 (Expression) volume changes
            }
            previousExpression = expression;                                    // Update the comparison variable
        }
    }

}


void runOctaveSelect()
{
    if (buttonState[8] == LOW)  // If the meta key is inactive
    {
        // Octave Down
        // ● ○
        // ○ ○
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[0] == HIGH && previousButtonState[0] == LOW && octave > -36)    // Keep note value in range of 0-127 (base range 59 - 91)
        {
            previousButtonState[0] = HIGH;                                              // Lock input until released
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + octave - 12 + transpose), velocity); // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave - 12 + transpose), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            octave = octave - 12;                                                       // Decrement octave modifier (minimum value 59 - 36 - 12 = 11)
        }

        // Octave Up
        // ○ ●
        // ○ ○
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[1] == HIGH && previousButtonState[1] == LOW && octave < 24)     // Keep note value in range of 0-127 (base range 59 - 91)
        {
            previousButtonState[1] = HIGH;                                              // Lock input until released
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + octave + 12 + transpose), velocity); // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave + 12 + transpose), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            octave = octave + 12;                                                       // Increment octave modifier (maximum value 91 + 24 + 12 = 127)
        }

        // Octave Reset
        // ● ●
        // ○ ○
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[0] == HIGH && buttonState[1] == HIGH)
        {
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + 0 + transpose), velocity);   // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + 0 + transpose), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            octave = 0;                                                                 // Reset octave modifier to 0
        }
    }
}


void runTransposeSelect()
{
    if (buttonState[8] == HIGH)  // If the meta key is active
    {
        // Transpose Down
        // ● ○
        // ○ ○
        //  -
        // ○ ○
        // ○ ○
        //  ●
        if (buttonState[0] == HIGH && previousButtonState[0] == LOW && transpose > -12) // Keep note value in range of 0-127 (base range 59 - 91)
        {
            previousButtonState[0] = HIGH;                                              // Lock input until released
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + octave + transpose - 1), velocity);  // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave + transpose - 1), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            transpose = transpose - 1;                                                  // Decrement transpose modifier (minimum value 59 - 36 - 12 = 11)
        }

        // Transpose Up
        // ○ ●
        // ○ ○
        //  -
        // ○ ○
        //  ●
        if (buttonState[1] == HIGH && previousButtonState[1] == LOW && transpose < 12)  // Keep note value in range of 0-127 (base range 59 - 91)
        {
            previousButtonState[1] = HIGH;                                              // Lock input until released
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + octave + transpose + 1), velocity);  // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave + transpose + 1), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            transpose = transpose + 1;                                                  // Increment transpose modifier (maximum value 91 + 24 + 12 = 127)
        }

        // Transpose Reset
        // ● ●
        // ○ ○
        //  -
        // ○ ○
        //  ●
        if (buttonState[0] == HIGH && buttonState[1] == HIGH )
        {
            if (stableBreathState == LOW)
            {
                noteOn(midiChannel, (previousActivePitch + octave + 0), velocity);  // Play a sample note
                delay(500);
                noteOff(midiChannel, (previousActivePitch + octave + 0), 0);
            }
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            transpose = 0;                                                              // Reset transpose modifier to 0
        }
    }
}


void runProgramSelect()
{
    if (buttonState[8] == LOW)  // If the meta key is inactive
    {
        // Program Down
        // ○ ○
        // ● ○
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[2] == HIGH && previousButtonState[2] == LOW && midiProgram[midiChannel] > 0)    // Stay in range of 0-127
        {
            previousButtonState[2] = HIGH;                                              // Lock input until released
            midiProgram[midiChannel] = midiProgram[midiChannel] - 1;                    // Decrement MIDI program
            programChange(midiChannel, midiProgram[midiChannel]);                       // Send a program change
            programChange(droneChannel, midiProgram[midiChannel]);                      // Send a program change
            stableBreathState = LOW;                                                    // Blip the breath button to trigger a new note
            playNotes();                                                                // Send note changes to MIDI
        }

        // Program Up
        // ○ ○
        // ○ ●
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[3] == HIGH && previousButtonState[3] == LOW && midiProgram[midiChannel] < 127)    // Stay in range of 0-127
        {
            previousButtonState[3] = HIGH;                                              // Lock input until released
            midiProgram[midiChannel] = midiProgram[midiChannel] + 1;                    // Increment MIDI program
            programChange(midiChannel, midiProgram[midiChannel]);                       // Send a program change
            programChange(droneChannel, midiProgram[midiChannel]);                      // Send a program change
            stableBreathState = LOW;                                                    // Blip the breath button to trigger a new note
            playNotes();                                                                // Send note changes to MIDI
        }

        // Program Reset
        // ○ ○
        // ● ●
        //  -
        // ○ ○
        // ○ ○
        //  ○
        if (buttonState[2] == HIGH && buttonState[3] == HIGH)
        {
            midiProgram[midiChannel] = 71;                                              // Reset to General MIDI program #71 Clarinet
            programChange(midiChannel, midiProgram[midiChannel]);                       // Send a program change to MIDI
            programChange(droneChannel, midiProgram[midiChannel]);                      // Send a program change to MIDI
            stableBreathState = LOW;                                                    // Blip stableBreathState to trigger a new note
            playNotes();                                                                // Send note changes to MIDI
            delay(500);                                                                 // No input locking, so delay to allow the buttons some time to be released
        }
    }
}


void runChannelSelect()
{
    if (buttonState[8] == HIGH)  // If the meta key is active
    {
        // Channel Down
        // ○ ○
        // ● ○
        //  -
        // ○ ○
        //  ●
        if (buttonState[2] == HIGH && previousButtonState[2] == LOW && midiChannel > 1) // Stay in range of 0-15 (remember droneChannel is -1)
        {
            previousButtonState[2] = HIGH;                                              // Lock input until released
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            midiChannel = midiChannel - 1;                                              // Decrement main MIDI channel
            droneChannel = midiChannel - 1;                                             // Set drone channel to 1 lower than main
        }

        // Channel Up
        // ○ ○
        // ○ ●
        //  -
        // ○ ○
        //  ●
        if (buttonState[3] == HIGH && previousButtonState[3] == LOW && midiChannel < 15) // Stay in range of 0-15 (remember droneChannel is -1)
        {
            previousButtonState[3] = HIGH;                                              // Lock input until released
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            midiChannel = midiChannel + 1;                                              // Increment main MIDI channel
            droneChannel = midiChannel - 1;                                             // Set drone channel to 1 lower than main
        }

        // Channel Reset
        // ○ ○
        // ● ●
        //  -
        // ○ ○
        //  ●
        if (buttonState[2] == HIGH && buttonState[3] == HIGH)
        {
            stableBreathState = LOW;                                                    // Blip stableBreathState to close the current new note
            playNotes();                                                                // Send note changes to MIDI
            midiChannel = 15;                                                           // Reset main channel to 15 (MIDI channel 16)
            droneChannel = midiChannel - 1;                                             // Set drone channel to 1 lower than main
            delay(500);                                                                 // No input locking, so delay to allow the buttons some time to be released
        }
    }
}


void runDroneMode()
{
    // Drone Note Toggle
    // ○ ○
    // ○ ○
    //  -
    // ● ○
    // ○ ○
    //  ○
    if (buttonState[8] == LOW)  // If the meta key is inactive
    {
        if (stableBreathState == HIGH)                                                              // Only allow enable/set if a note is active
        {
            if (buttonState[4] == HIGH && previousButtonState[4] == LOW && droneNoteActive == LOW)  // If currently off, turn on
            {
                previousButtonState[4] = HIGH;                                                      // Lock input until released
                droneNoteActive = HIGH;                                                             // Toggle drone on
                droneNoteValue = (activePitch + octave + transpose);                                // Set drone note to one octave below currently active note
                if (stableBreathState == HIGH)                                                      // If the breath sensor is currently active
                {
                    noteOn(droneChannel, droneNoteValue, velocity - 16);                            // Send a note on
                    pitchBendChange(midiChannel, 8192);                                             // Set pitchBend on main channel to neutral 8192
                    pitchBendChange(droneChannel, 8064);                                            // *Slightly* detune the drone from 8192 so it sounds less artificial
                }
            }
        }
        if (buttonState[4] == HIGH && previousButtonState[4] == LOW && droneNoteActive == HIGH)     // If currently on, turn off
        {
            previousButtonState[4] = HIGH;                                                          // Lock input until released
            droneNoteActive = LOW;                                                                  // Toggle drone off
            noteOff(droneChannel, droneNoteValue, 0);                                               // Send a note off
        }
    }
    if (buttonState[8] == HIGH) // If the meta key is active
    {
        // Drone Note Hold Toggle
        // ○ ○
        // ○ ○
        //  -
        // ●
        // ○ ○
        //  ●
        if (buttonState[4] == HIGH && previousButtonState[4] == LOW && droneNoteHold == LOW)        // If currently off, turn on
        {
            previousButtonState[4] = HIGH;                                                          // Lock input until released
            droneNoteHold = HIGH;                                                                   // Toggle drone hold on
            if (droneNoteActive == HIGH)                                                            // If drone note is enabled
            {
                noteOn(droneChannel, droneNoteValue, velocity - 16);                                // Send a note on
            }
            controlChange(midiChannel, 11, 95);                                                     // Lock expression at a set level
            controlChange(droneChannel, 11, 95);                                                    // Lock expression at a set level
        }
        if (buttonState[4] == HIGH && previousButtonState[4] == LOW && droneNoteHold == HIGH)       // If currently off, turn on
        {
            previousButtonState[4] = HIGH;                                                          // Lock input until released
            droneNoteHold = LOW;                                                                    // Toggle drone hold off
            noteOff(droneChannel, droneNoteValue, 0);                                               // Send a note off
        }
    }
}


void runVelocitySelect()
{
    if (buttonState[8] == LOW)  // If the meta key is inactive
    {
        // Velocity Down
        // ○ ○
        // ○ ○
        //  -
        // ○ ○
        // ● ○
        //  ○
        if (buttonState[6] == HIGH && previousButtonState[6] == LOW && velocity > 23)           // Remember drone note is -16 velocity, so stay above 0
        {
            previousButtonState[6] = HIGH;                                                      // Lock input until released
            if (stableBreathState == LOW)                                                       // Only play a sample note if the breath sensor is currently inactive
            {
                noteOn(midiChannel, (previousActivePitch + octave + transpose), velocity - 8);  // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave + transpose), 0);
            }
            stableBreathState = LOW;                                                            // Blip stableBreathState to close the current new note
            playNotes();                                                                        // Send note changes to MIDI
            velocity = velocity - 8;                                                            // Decrement velocity
        }

        // Velocity Up
        // ○ ○
        // ○ ○
        //  -
        // ○ ○
        // ○ ●
        //  ○
        if (buttonState[7] == HIGH && previousButtonState[7] == LOW && velocity < 127)
        {
            previousButtonState[7] = HIGH;                                                      // Lock input until released
            if (stableBreathState == LOW)                                                       // Only play a sample note if the breath sensor is currently inactive
            {
                noteOn(midiChannel, (previousActivePitch + octave + transpose), velocity + 8);  // Play a sample note
                delay(250);
                noteOff(midiChannel, (previousActivePitch + octave + transpose), 0);
            }
            stableBreathState = LOW;                                                            // Blip stableBreathState to close the current new note
            playNotes();                                                                        // Send note changes to MIDI
            velocity = velocity + 8;                                                            // Increment velocity
        }

        // Velocity Reset
        // ○ ○
        // ○ ○
        //  -
        // ○ ○
        // ● ●
        //  ○
        if (buttonState[6] == HIGH && buttonState[7] == HIGH)
        {
            if (stableBreathState == LOW)                                                       // Only play a sample note if the breath sensor is currently inactive
            {
                noteOn(midiChannel, (previousActivePitch + octave + transpose), 95);            // Play a sample note
                delay(500);
                noteOff(midiChannel, (previousActivePitch + octave + transpose), 0);
            }
            stableBreathState = LOW;                                                            // Blip stableBreathState to close the current new note
            playNotes();                                                                        // Send note changes to MIDI
            velocity = 95;                                                                      // Reset velocity to default 95
        }
    }
}


void runMidiCC()
{
    if (buttonState[8] == HIGH) // If the meta key is active
    {
        // Reverb Toggle
        // ○ ○
        // ○ ○
        //  -
        // ○ ○
        // ● ○
        //  ●
        if (buttonState[6] == HIGH && previousButtonState[6] == LOW && reverbActive == LOW)     // If off, toggle on
        {
            previousButtonState[6] = HIGH;                                                      // Lock input until released
            reverbActive = HIGH;                                                                // Toggle reverb on
            controlChange(midiChannel, 91, 127);                                                // Enable Reverb (MIDI CC #91) - 7 bit value
            controlChange(droneChannel, 91, 127);                                               // Enable Reverb (MIDI CC #91) - 7 bit value
        }
        if (buttonState[6] == HIGH && previousButtonState[6] == LOW && reverbActive == HIGH)    // If on, toggle off
        {
            previousButtonState[6] = HIGH;                                                      // Lock input until released
            reverbActive = LOW;                                                                 // Toggle reverb on
            controlChange(midiChannel, 91, 0);                                                  // Enable Reverb (MIDI CC #91) - 7 bit value
            controlChange(droneChannel, 91, 0);                                                 // Enable Reverb (MIDI CC #91) - 7 bit value
        }
        // Chorus Toggle
        // ○ ○
        // ○ ○
        //  -
        // ○ ○
        // ○ ●
        //  ●
        if (buttonState[7] == HIGH && previousButtonState[7] == LOW && chorusActive == LOW)     // If off, toggle on
        {
            previousButtonState[7] = HIGH;                                                      // Lock input until released
            chorusActive = HIGH;                                                                // Toggle chorus on
            controlChange(midiChannel, 93, 127);                                                // Enable Chorus (MIDI CC #93) - 7 bit value
            controlChange(droneChannel, 93, 127);                                               // Enable Chorus (MIDI CC #93) - 7 bit value
        }
        if (buttonState[7] == HIGH && previousButtonState[7] == LOW && chorusActive == HIGH)    // If on, toggle off
        {
            previousButtonState[7] = HIGH;                                                      // Lock input until released
            chorusActive = LOW;                                                                 // Toggle chorus on
            controlChange(midiChannel, 93, 0);                                                  // Enable Chorus (MIDI CC #93) - 7 bit value
            controlChange(droneChannel, 93, 0);                                                 // Enable Chorus (MIDI CC #93) - 7 bit value
        }


    }
}


void runResetThresholds()
{
    if (buttonState[8] == LOW) // If the meta key is inactive
    {
        // Reset Thresholds
        // ○ ○
        // ○ ○
        //  -
        // ○ ●
        // ○ ○
        //  ○
        if (buttonState[5] == HIGH && previousButtonState[5] == LOW)
        {
            previousButtonState[5] = HIGH;                                                      // Lock input until released
            breathBaseVal = analogRead(breathPin);                                      // Set neutral pressure level
            breathActivationThreshold = breathBaseVal + 4;
            breathMaxVal = breathBaseVal + 64;
            for (int myPin = 0; myPin < NUM_TOUCH_PINS; myPin++)                       // For all capacitive touch pins
            {
                int myValue = touchRead(touchPins[myPin]);
                touchMinVal[myPin] = myValue;                                    // Set initial minimum value to current reading
                touchMaxVal[myPin] = myValue;                                    // Set initial minimum value to current reading
            }
        }
    }
}


void runResetController()
{
    if (buttonState[8] == HIGH) // If the meta key is active
    {
        // Reset the instrument to default power-on settings
        // ○ ○
        // ○ ○
        //  -
        // ○ ●
        // ○ ○
        //  ●
        if (buttonState[5] == HIGH && previousButtonState[5] == LOW)
        {
            // Play a notification chime
            noteOn(0, 60, 95);
            delay(125);
            noteOn(0, 64, 95);
            delay(125);
            noteOn(0, 67, 95);
            delay(125);
            noteOn(0, 72, 95);
            delay(250);
            noteOff(0, 60, 0);
            noteOff(0, 64, 0);
            noteOff(0, 67, 0);
            noteOff(0, 72, 0);
            for (byte i = 0; i < 127; i++)                                  // For all note values 0-127
            {
                noteOff(midiChannel, i, 0);                                 // Send a noteOff for every note to close any strays
                delay(1);
            }
            WRITE_RESTART(0x5FA0004);                                       // Write value to memory location to trigger a restart
        }
    }
}


void playNotes()
{
    if (stableBreathState == LOW)                                       // If the breath controller is inactive...
    {
        activePitch = 0;                                                // Reset activePitch to initial value for this loop (MIDI note 0 won't ever be played)
        goto noteFound;
    }

    // The bitwise variable stableTouchStates maps to the following note buttons on the instrument
    // Thumb = 0, Left Hand = 1-3, Right Hand = 4-7, Bell Hole = 8
    // 00 | 1 2 3 | 4 5 66 77 | 8
    // AB | C D E | F G HI JK | L
    // 0b0000ABCDEFGHIJKL

    // None     - ○○ | ○ ○ ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000000000000000) { stableBreathState = LOW; activePitch = 0; goto noteFound; }

    // B3       - ●● | ● ● ● | ● ● ●● ●● | ●
    if (stableTouchStates == 0b0000111111111111) { activePitch = 59; goto noteFound; }

    // C4       - ●● | ● ● ● | ● ● ●● ●● | ○
    if (stableTouchStates == 0b0000111111111110) { activePitch = 60; goto noteFound; }

    // C#4/Db4  - ●● | ● ● ● | ● ● ●● ●○ | ○
    if (stableTouchStates == 0b0000111111111100) { activePitch = 61; goto noteFound; }

    // D4       - ●● | ● ● ● | ● ● ●● ○○ | ○
    if (stableTouchStates == 0b0000111111111000) { activePitch = 62; goto noteFound; }

    // D#4/Eb4  - ●● | ● ● ● | ● ● ●○ ○○ | ○
    if (stableTouchStates == 0b0000111111110000 ) { activePitch = 63; goto noteFound; }

    // E4       - ●● | ● ● ● | ● ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000111111100000 ) { activePitch = 64; goto noteFound; }

    // F4       - ●● | ● ● ● | ● ○ ●● ●● | ○
    if (stableTouchStates == 0b0000111111011110 ) { activePitch = 65; goto noteFound; }

    // F#4/Gb4  - ●● | ● ● ● | ○ ● ●● ○○ | ○
    if (stableTouchStates == 0b0000111110111000 ) { activePitch = 66; goto noteFound; }

    // G4       - ●● | ● ● ● | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000111110000000 ) { activePitch = 67; goto noteFound; }

    // G#4/Ab4  - ●● | ● ● ○ | ● ● ●○ ○○ | ○
    if (stableTouchStates == 0b0000111101110000 ) { activePitch = 68; goto noteFound; }

    // A4       - ●● | ● ● ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000111100000000 ) { activePitch = 69; goto noteFound; }

    // A#4/Bb4  - ●● | ● ○ ● | ○ ● ●● ○○ | ○
    //            ●● | ● ○ ● | ● ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000111010111000 ) { activePitch = 70; goto noteFound; }
    if (stableTouchStates == 0b0000111011000000 ) { activePitch = 70; goto noteFound; }

    // B4       - ●● | ○ ● ● | ○ ○ ○○ ○○ | ○
    //            ●● | ● ○ ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000110110000000 ) { activePitch = 71; goto noteFound; }
    if (stableTouchStates == 0b0000111000000000 ) { activePitch = 71; goto noteFound; }

    // C5       - ●● | ○ ● ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000110100000000 ) { activePitch = 72; goto noteFound; }

    // C#5/Db5  - ○○ | ● ● ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000001100000000 ) { activePitch = 73; goto noteFound; }

    // D5       - ○○ | ● ● ● | ● ●  ●● ●●| ○
    //            ○○ | ○ ● ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000001111111110 ) { activePitch = 74; goto noteFound; }
    if (stableTouchStates == 0b0000000100000000 ) { activePitch = 74; goto noteFound; }

    // D#5/Eb5  - ○○ | ● ● ● | ● ● ●● ○○ | ○
    //            ○○ | ○ ● ● | ● ● ●● ○○ | ○
    if (stableTouchStates == 0b0000001111111000 ) { activePitch = 75; goto noteFound; }
    if (stableTouchStates == 0b0000000111111000 ) { activePitch = 75; goto noteFound; }

    // E5       - ●○ | ● ● ● | ● ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000101111100000 ) { activePitch = 76; goto noteFound; }

    // F5       - ●○ | ● ● ● | ● ○ ●● ○○ | ○
    if (stableTouchStates == 0b0000101111011000 ) { activePitch = 77; goto noteFound; }

    // F#5/Gb5  - ●○ | ● ● ● | ○ ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000101110100000 ) { activePitch = 78; goto noteFound; }

    // G5       - ●○ | ● ● ● | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000101110000000 ) { activePitch = 79; goto noteFound; }

    // G#5/Ab5  - ●○ | ● ● ○ | ● ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000101101000000 ) { activePitch = 80; goto noteFound; }

    // A5       - ●○ | ● ● ○ | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000101100000000 ) { activePitch = 81; goto noteFound; }

    // A#5/Bb5  - ●○ | ● ● ○ | ○ ● ●● ●○ | ○
    //            ●○ | ● ● ○ | ○ ● ●● ○○ | ○
    if (stableTouchStates == 0b0000101100111100 ) { activePitch = 82; goto noteFound; }
    if (stableTouchStates == 0b0000101100111000 ) { activePitch = 82; goto noteFound; }

    // B5       - ●○ | ● ● ○ | ● ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000101101100000 ) { activePitch = 83; goto noteFound; }

    // C6       - ●○ | ● ○ ○ | ● ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000101001100000 ) { activePitch = 84; goto noteFound; }
    
    // C#6/Db6  - ●○ | ● ○ ● | ● ○ ●● ●○ | ●
    //            ●○ | ● ○ ● | ● ○ ●● ○○ | ●
    if (stableTouchStates == 0b0000101011011101 ) { activePitch = 85; goto noteFound; }
    if (stableTouchStates == 0b0000101011011001 ) { activePitch = 85; goto noteFound; }

    // D6       - ●○ | ● ○ ● | ● ○ ●● ●○ | ○
    //            ●○ | ● ○ ● | ● ○ ●● ○○ | ○
    if (stableTouchStates == 0b0000101011011100 ) { activePitch = 86; goto noteFound; }
    if (stableTouchStates == 0b0000101011011000 ) { activePitch = 86; goto noteFound; }

    // D#6/Eb6  - ●○ | ○ ● ● | ○ ● ●● ●○ | ○
    //            ●○ | ○ ● ● | ○ ● ●● ○○ | ○
    //            ●○ | ○ ● ● | ○ ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000100110111100 ) { activePitch = 87; goto noteFound; }
    if (stableTouchStates == 0b0000100110111000 ) { activePitch = 87; goto noteFound; }
    if (stableTouchStates == 0b0000100110000000 ) { activePitch = 87; goto noteFound; }

    // E6       - ●○ | ○ ● ● | ○ ● ●● ●○ | ●
    //          - ●○ | ○ ● ● | ○ ● ●● ○○ | ●
    if (stableTouchStates == 0b0000100110111101 ) { activePitch = 88; goto noteFound; }
    if (stableTouchStates == 0b0000100110111001 ) { activePitch = 88; goto noteFound; }

    // F6       - ●○ | ● ● ○ | ● ● ○○ ○○ | ●
    //            ●○ | ● ○ ● | ● ● ○○ ○○ | ●
    if (stableTouchStates == 0b0000101101100001 ) { activePitch = 89; goto noteFound; }
    if (stableTouchStates == 0b0000101011100001 ) { activePitch = 89; goto noteFound; }

    // F#6/Gb6  - ●○ | ● ○ ● | ● ● ○○ ○○ | ○
    if (stableTouchStates == 0b0000101011100000 ) { activePitch = 90; goto noteFound; }

    // G6       - ●○ | ● ○ ○ | ● ○ ○○ ○○ | ○
    if (stableTouchStates == 0b0000101001000000 ) { activePitch = 91; goto noteFound; }

noteFound:

    if (stableBreathState == HIGH && previousBreathState == LOW && activePitch != 0)    // If a new note from a clean slate
    {
        previousBreathState = HIGH;                                                     // Lock input until released
        noteOn(midiChannel, (activePitch + octave + transpose), velocity);              // Send a note on
        previousActivePitch = activePitch;                                              // Update the comparison variable
        if (droneNoteActive == HIGH && droneNoteHold == LOW)                            // If drone is active (and droneHold is inactive)
        {
            noteOn(droneChannel, droneNoteValue, velocity - 16);                        // Send a note on
        }

    }
    if (stableBreathState == HIGH && previousBreathState == HIGH && activePitch != previousActivePitch && activePitch != 0)   // If a new note from a fingering change 
    {
        noteOff(midiChannel, (previousActivePitch + octave + transpose), 0);            // Send a note off for the previous note
        noteOn(midiChannel, (activePitch + octave + transpose), velocity);              // Send a note on for our new note
        previousActivePitch = activePitch;                                              // Update the comparison variable
    }

    if (stableBreathState == LOW && previousBreathState == HIGH)                        // If all notes are complete
    {
        previousBreathState = LOW;                                                      // Lock input until released
        noteOff(midiChannel, (previousActivePitch + octave + transpose), 0);            // Send a note off to close the previous active pitch
        if (droneNoteActive == HIGH && droneNoteHold == LOW)                            // If drone is active (and droneHold is inactive)
        {
            noteOff(droneChannel, droneNoteValue, 0);                                   // Send a note off
        }
    }
}


// MIDI PACKET FUNCTIONS
//----------------------

// Send MIDI Note On
// 1st byte = Event type (0x09 = note on, 0x08 = note off).
// 2nd byte = Event type bitwise ORed with MIDI channel.
// 3rd byte = MIDI note number.
// 4th byte = Velocity (7-bit range 0-127)
void noteOn(byte channel, byte pitch, byte velocity)
{
    channel = 0x90 | channel;                                                   // Bitwise OR outside of the struct to prevent compiler warnings
    midiEventPacket_t noteOn = {0x09, channel, pitch, velocity};                // Build a struct containing all of our information in a single packet
    MidiUSB.sendMIDI(noteOn);                                                   // Send packet to the MIDI USB bus
    Serial1.write(0x90 | channel);                                              // Send event type/channel to the MIDI serial bus
    Serial1.write(pitch);                                                       // Send note number to the MIDI serial bus
    Serial1.write(velocity);                                                    // Send velocity value to the MIDI serial bus
}

// Send MIDI Note Off
// 1st byte = Event type (0x09 = note on, 0x08 = note off).
// 2nd byte = Event type bitwise ORed with MIDI channel.
// 3rd byte = MIDI note number.
// 4th byte = Velocity (7-bit range 0-127)
void noteOff(byte channel, byte pitch, byte velocity)
{
    channel = 0x80 | channel;                                                   // Bitwise OR outside of the struct to prevent compiler warnings
    midiEventPacket_t noteOff = {0x08, channel, pitch, velocity};               // Build a struct containing all of our information in a single packet
    MidiUSB.sendMIDI(noteOff);                                                  // Send packet to the MIDI USB bus
    Serial1.write(0x80 | channel);                                              // Send event type/channel to the MIDI serial bus
    Serial1.write(pitch);                                                       // Send note number to the MIDI serial bus
    Serial1.write(velocity);                                                    // Send velocity value to the MIDI serial bus
}

// Control Change
// 1st byte = Event type (0x0B = Control Change).
// 2nd byte = Event type bitwise ORed with MIDI channel.
// 3rd byte = MIDI CC number (7-bit range 0-127).
// 4th byte = Control value (7-bit range 0-127).
void controlChange(byte channel, byte control, byte value)
{
    channel = 0xB0 | channel;                                                   // Bitwise OR outside of the struct to prevent compiler warnings
    midiEventPacket_t event = {0x0B, channel, control, value};                  // Build a struct containing all of our information in a single packet
    MidiUSB.sendMIDI(event);                                                    // Send packet to the MIDI USB bus
    Serial1.write(0xB0 | channel);                                              // Send event type/channel to the MIDI serial bus
    Serial1.write(control);                                                     // Send control change number to the MIDI serial bus
    Serial1.write(value);                                                       // Send control chnage value to the MIDI serial bus
}

// Program Change
// 1st byte = Event type (0x0C = Program Change).
// 2nd byte = Event type bitwise ORed with MIDI channel.
// 3rd byte = Program value (7-bit range 0-127).
void programChange(byte channel, byte value)
{
    channel = 0xC0 | channel;                                                   // Bitwise OR outside of the struct to prevent compiler warnings
    midiEventPacket_t event = {0x0C, channel, value};                           // Build a struct containing all of our information in a single packet
    MidiUSB.sendMIDI(event);                                                    // Send packet to the MIDI USB bus
    Serial1.write(0xC0 | channel);                                              // Send event type/channel to the MIDI serial bus
    Serial1.write(value);                                                       // Send program change value to the MIDI serial bus
}

// Pitch Bend
// (14 bit value 0-16363, neutral position = 8192)
// 1st byte = Event type (0x0E = Pitch bend change).
// 2nd byte = Event type bitwise ORed with MIDI channel.
// 3rd byte = The 7 least significant bits of the value.
// 4th byte = The 7 most significant bits of the value.
void pitchBendChange(byte channel, int value)   //byte lowValue, byte highValue)
{
    channel = 0xE0 | channel;                                                   // Bitwise OR outside of the struct to prevent compiler warnings
    byte lowValue = value & 0x7F;                                               // Capture the 7 least significant bits of the value
    byte highValue = value >> 7;                                                // Capture the 7 most significant bits of the value
    midiEventPacket_t bendEvent = {0x0E, channel, lowValue, highValue};         // Build a struct containing all of our information in a single packet
    MidiUSB.sendMIDI(bendEvent);                                                // Send packet to the MIDI USB bus
    Serial1.write(0xE0 | channel);                                              // Send event type/channel to the MIDI serial bus
    Serial1.write(lowValue);                                                    // Send pitch bend low byte to the MIDI serial bus
    Serial1.write(highValue);                                                   // Send pitch bend high byte to the MIDI serial bus
}

// END FUNCTIONS SECTION
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

// END OF PROGRAM
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

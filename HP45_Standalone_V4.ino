/*
    HP45 standalone is software used to control the Teensy 3.5 based standalone controller for the HP45
    Copyright (C) 2021  Yvo de Haas

    HP45 standalone is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    HP45 standalone is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
   IMPORTANT: To make sure that the timing on critical functions, run this code at 96MHz

   The buffer contains all data to be printed. The position holds the reading and decoding of the encoder
   The printhead contains all functions that control the HP45 controller

   The flow of printing. The buffer gets loaded with data. This includes the inkjet data and a position
   The buffer changes when a new coordinate is reached. Once this is done, the direction required for the new coordinate is determined.
   Based on this direction, the condition for the new coordinate is determined (higher or lower than)
   Once a coordinate is reached, the burst of the current coordinate is made active, and the next line in the buffer is loaded

    todo:
    -Remove redundant enable and disable printhead calls
*/
/*
   Temporary serial command for printing 0-74, 75-149, 150-224, 225-299 and 0
   in 10000 micron intervals:

  SBR A AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
  SBR CcQ ////////////HAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
  SBR E4g AAAAAAAAAAAA4////////////AAAAAAAAAAAAAAAAAAAAAAAAA
  SBR HUw AAAAAAAAAAAAAAAAAAAAAAAAA////////////HAAAAAAAAAAAA
  SBR JxA AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA4////////////
  SBR MNQ AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

  SBR A AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
  SBR CcQ //////////////////////////////////////////////////
  SBR MNQ AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

  preheat 10000 times
  PHD CcQ
*/


#include <Arduino.h>
#include "buffer.cpp"
#include "Serialcom.cpp"
#include "DMAPrint.h"

#ifdef __AVR__
#error "Sorry, HP45 controller only works on Teensy 3.5 due to clocks and required hardware"
#endif

#if defined(__MK64FX512__)
#else
#error "HP45 standalone only works on Teensy 3.5 due to clocks and dma configurations"
#endif

#if F_CPU != 120000000
#error "This code requires 120 MHz CPU speed, use Tools > CPU Speed menu"
#endif


//Printhead HP45(29, 33, 24, 16, 28, 15, 26, 14, 27, 20, 8, 18, 7, 17, 25, 19, 3, 2, 23, 22, A11, A10); //V3.01
//Printhead HP45(4, 21, 15, 22, 23, 9, 10, 13, 11, 12, 2, 14, 7, 8, 6, 20, 5, 3, 19, 16, A11, A10); //dma prototype
//Printhead HP45(21, 21, 7, 14, 11, 13, 9, 12, 10, 23, 6, 20, 2, 15, 8, 22, 5, 4, 18, 16, A11, A10); //for dma tester V3.02
Buffer BurstBuffer;
SerialCommand Ser;

//modification to circuit: clk moved from shared 21 with clr to 4

//variables
uint16_t inkjetDensity = 100; //percentage, how often the printhead should burst
int32_t inkjetMinPosition[2], inkjetMaxPosition[2]; //where the inkjet needs to start and end
uint8_t inkjetEnabled[2], inkjetEnabledHistory[2]; //whether a side a allowed to jew ink or not (+ history)
int32_t CurrentPosition[2]; //odd and even position of the printhead
int32_t CurrentVelocity; //the current velocity of the printhead
int8_t CurrentDirection; //which direction the printhead is moving
int8_t requiredPrintingDirection[2]; //the direction between the current point and the next point
int32_t targetPosition[2]; //the position where the buffer needs to go next
uint8_t nextState[2]; //handles how the next line in the buffer is used
uint16_t serialBufferCounter = 0; //counter for lines received, when target reached, return a WL over serial
uint8_t serialBufferWLToggle = 1; //whether to respond with a WL when a threshold is reached
uint8_t serialBufferFirst[2] = {1, 1}; //the first buffer entry is treated different
uint8_t inkjetHardwareEnabled = 1; //allows the inkjet functions to be disabled for debug purposes
uint32_t cycleCounter; //counts the number of cycles the code can complete per second
uint32_t cycleTarget; //when the next cycle count is performed
uint8_t cycleCounterEnabled = 0; //whether the cycle counter posts or not

uint8_t burstOn = 0; //whether the printhead is currently printing or not
uint32_t inkjetBurstDelay; //how long to wait between each burst
uint32_t inkjetLastBurst; //when the last burst was
uint16_t DataBurst[22]; //the printing burst for decoding
uint16_t CurrentBurst[22]; //the current printing burst
uint8_t NozzleState[300];
uint8_t AddressState[22];
uint8_t PrimitiveState[14];

//trigger variables
#define TRIGGER_UPDATE_DELAY 1000
uint8_t triggerWhileActive = 0; //if the trigger is active in a while loop or not. Does not turn 1 for a normal trigger

//error variables
const uint8_t statusLed = 18; //the status led pin
uint8_t errorState = 0; //errors or warnings being active. 0 is no problems, 1 is warning and 2 is error
uint32_t errorList = 0; //what errors are present
uint32_t warningList = 0; //what warnings are present
uint32_t errorTarget; //counter for updating led animations
uint32_t errorDelay = 200; //time in milliseconds between each update
uint8_t errorAnimation = 0; //state machine for the animations
uint32_t checkTarget; //counter for performing checks
uint32_t checkDelay = 5000; //the interval at which basic checks are performed in milliseconds.

#define ERROR_LOGIC_VOLTAGE_LOW_BIT 0
#define ERROR_LOGIC_VOLTAGE_HIGH_BIT 1
#define ERROR_DRIVING_VOLTAGE_LOW_BIT 2
#define ERROR_DRIVING_VOLTAGE_HIGH_BIT 3
#define ERROR_HEAD_TEMPERATURE_NC_BIT 4
#define ERROR_HEAD_TEMPERATURE_HIGH_BIT 5
#define ERROR_ADDRESS_DEFECTIVE 6
#define ERROR_DUMMY1_NOT_RISING 7
#define ERROR_DUMMY1_NOT_FALLING 8
#define ERROR_DUMMY2_NOT_RISING 9
#define ERROR_DUMMY2_NOT_FALLING 10

#define WARNING_HEAD_TEMPERATURE_HIGH_BIT 0

#define LOGIC_LOWER_VOLTAGE 11000
#define LOGIC_UPPER_VOLTAGE 13000
#define DRIVING_LOWER_VOLTAGE 7000
#define DRIVING_UPPER_VOLTAGE 15000
#define HEAD_WARNING_TEMPERATURE 450
#define HEAD_ERROR_TEMPERATURE 600


//variables for transfering from serial to inkjet
uint16_t inkjetLineNumber;
uint32_t inkjetCommand;
int32_t inkjetSmallValue;
#define INKJET_RAW_SIZE 50
uint8_t inkjetRaw[INKJET_RAW_SIZE];
uint8_t serialSource;
uint8_t serialCommandEcho = 0; //used to return the value decoded from command


//constants
uint16_t printheadDPI = 600;
#define serialBufferTarget 50 //after how many lines of code received the printhead will return a writeleft to the controller

//upper and lower threshold are used for the wakeup WL response. When the lower threshold is reached, serialBufferWLToggle flips from 0 to 1
//when then the upper threshold is reached, the firmware will send a writeleft to the controller, notifying there is space again
//this measure is in place bacause the software may not be aware space has opened up in the meantime, relying on fairly slow updates
//An upper and lower threshold is picked so it does not send a WL every time a single threshold is passed, there is hysteresis
#define bufferLowerThreshold 200
#define bufferUpperThreshold 500

//for DMA
const uint32_t dmaBufferSize = 320; //242 max theoretical, the actual size of the DMA buffer (takes data per 2 bytes, 1 per port)
const uint32_t dmaFrequency = 1050000; //the frequency in hertz the buffer should update at

DMAMEM uint8_t portCMemory[dmaBufferSize]; //stores data for port C DMA
DMAMEM uint8_t portDMemory[dmaBufferSize]; //stores data for port D DMA
uint8_t portCWrite[dmaBufferSize]; //stores data for port C editing
uint8_t portDWrite[dmaBufferSize]; //stores data for port D editing

DMAPrint dmaHP45(dmaBufferSize, portCMemory, portDMemory, portCWrite, portDWrite, dmaFrequency); //init the dma library

//test variable (temporary)
uint32_t tempTimer, tempDelay = 1000000;


void setup() {
  BurstBuffer.ClearAll(); //reset the buffer
  dmaHP45.begin();

  //TestFill(); //fill buffer with test program
  inkjetEnabled[0] = 0;
  inkjetEnabled[1] = 0;
  inkjetEnabledHistory[0] = 0;
  inkjetEnabledHistory[1] = 0;
  nextState[0] = 0; nextState[1] = 0; //set next state to starting position

  pinMode(statusLed, OUTPUT); //set status led

  delay(2500); //delay to give serial time to start on pc side
  Ser.Begin(); //start serial connection
  //GenerateNewRawTables(); //uncommment to make a new nozzle table with the variables given in tab "NewNozzleTable"
}

void loop() {
  UpdateAll(); //update all critical functions
  //TestPrimitives(); //test loop through all primitives, 1s on, 1s off, next
}

void UpdateAll() { //update all time critical functions
  PositionUpdate(); //get new position
  //get all velocitys and positions
  CurrentPosition[0] = PositionGetRowPositionMicrons(0);
  CurrentPosition[1] = PositionGetRowPositionMicrons(1);
  CurrentVelocity = PositionGetVelocity();
  CurrentDirection = PositionGetDirection();

  //get inkjet values
  if (inkjetHardwareEnabled == 1) {
    BufferUpdateValues(); //see if new values in the buffer need to be called
    BufferUpdateLoop(); //update buffer loop state
    InkjetUpdateBurstDelay(); //calculate burst delay based on density and speed
    InkjetUpdateBurst(); //check if the printhead needs to be on based on required direction, actual direction, start pos and end pos

    //status update
    UpdateStatus();
  }

  if (Ser.Update() >= 1) { //get serial, if more than 1, add new command
    SerialExecute(); //get command and execute it
  }
  SerialWLPush(); //check push Write Left requirements

  //get SPI

  //Trigger update
  UpdateTrigger();

  //cycle counter
  UpdateCycle();
}

///----------------------------------------------------------
void InkjetUpdateBurst() { //checks if the head is within range to burst (direction and position) and bursts the head is conditions are met
  uint8_t tempState_changed = 0; //value to indicate enabled states have changed
  for (uint8_t s = 0; s <= 1; s++) {
    if (requiredPrintingDirection[s] == CurrentDirection && CurrentPosition[s] > inkjetMinPosition[s] && CurrentPosition[s] < inkjetMaxPosition[s] && CurrentVelocity != 0) { //if head is within the inkjet limits and direction matches and velocity is not 0
      inkjetEnabled[s] = 1; //direction and area match, enable head
      if (inkjetEnabledHistory[s] == 0) { //if history and current are not the same, mark change buffer
        inkjetEnabledHistory[s] = 1;
        tempState_changed = 1;
        BurstBuffer.SetActive(s, 1); //set side of buffer active
      }
    }
    else { //if requirements are not met, set enabled and history to 0
      inkjetEnabled[s] = 0;
      if (inkjetEnabledHistory[s] == 1) {  //if history and current are not the same, mark change buffer
        inkjetEnabledHistory[s] = 0;
        tempState_changed = 1;
        BurstBuffer.SetActive(s, 0); //set side of buffer inactive
      }
    }
  }
  if (tempState_changed == 1) { //if any of the states changed, request new data from the buffer with correct overlays
    BurstBuffer.GetBurst(CurrentBurst); //get new burst from the buffer
  }
  //check burst time conditions
  if (micros() - inkjetLastBurst > inkjetBurstDelay) { //if burst is required again based on time (updated regradless of burst conditions)
    inkjetLastBurst = micros();
    if (inkjetEnabled[0] == 1 ||  inkjetEnabled[1] == 1) { //if the head is within bounds, burst head
      dmaHP45.SetEnable(1); //enable the head
      burstOn = 1;
      dmaHP45.SetBurst(CurrentBurst, 1);
      dmaHP45.Burst(); //burst the printhead
    }
    else {
      dmaHP45.SetEnable(0); //disable the head
      burstOn = 0;
    }
  }
}
void InkjetUpdateBurstDelay() { //recalculates burst delay
  if (CurrentVelocity != 0) { //if velocity is more than 0
    float temp_dpi = float(printheadDPI) * float(inkjetDensity); //calculate actual DPI
    temp_dpi /= 100.0; //divide by percentage
    float temp_vel = float(abs(CurrentVelocity)) / 25.4; //get absolute velocity in inches per second
    float temp_calc = 1000000.0; //get microseconds
    temp_calc /= temp_vel; //divide by velocity in inch per second and actual DPI to get burst delay
    temp_calc /= temp_dpi;

    inkjetBurstDelay = long(temp_calc); //write to the variable
  }
}
void BufferUpdateValues() { //checks if the next positions in the buffer can be called
  for (uint8_t s = 0; s <= 1; s++) { //check if the burst needs to change (for odd and even)
    uint8_t update_values = 0;
    //if there is a first line on the given side, force an update every cycle
    if (serialBufferFirst[s] == 1) { //if the pin is on read first, force a check constantly
      update_values = 1; //force the check of the values
    }
    else if (requiredPrintingDirection[s] == -1) { //if required direction is negative
      if (CurrentPosition[s] < targetPosition[s]) { //check if new position requirement is met
        update_values = 1; //set buffer to next position
      }
    }
    else { //if required direction is positive
      if (CurrentPosition[s] > targetPosition[s]) { //check if new position requirement is met
        update_values = 1; //set buffer to next position
      }
    }

    if (update_values == 1) {//if a position in the buffer needs to advance
      //first, modify the inkjet burst data to the now reached position, regardless of whether a further line is available
      //(any last all off command should never be ignored)
      if (BurstBuffer.ReadLeftSide(s) > 0 && nextState[s] == 0) { //if there is space left to read
        nextState[s] = 1; //set next state to 1, so other target functions can happen if possible
        BurstBuffer.Next(s); //go to next position in the buffer
        BurstBuffer.GetBurst(CurrentBurst); //active the burst on the now reached position (do so regardless of new line)
        //Serial.print("Buffer Next: "); Serial.print(s); Serial.print(", Pos: "); Serial.println(targetPosition[s]);
      }

      if (BurstBuffer.ReadLeftSide(s) > 0 && nextState[s] == 1) { //if there is space left to read for the look ahead function
        int32_t temp_ahead_pos; //get old position stored before it is overwritten
        temp_ahead_pos = BurstBuffer.LookAheadPosition(s);

        if (serialBufferFirst[s] == 1) { //if not first line, use last position in the buffer
          targetPosition[s] = CurrentPosition[s];
          serialBufferFirst[s] = 0; //set first to 0 to stop future use
        }

        //Serial.print("look ahead: "); Serial.println(temp_ahead_pos);
        if (temp_ahead_pos - targetPosition[s] >= 0) requiredPrintingDirection[s] = 1; //determine required direction
        else requiredPrintingDirection[s] = -1;
        //set the inkjet limits, the coordinates where the printhead is allowed to print
        if (temp_ahead_pos < targetPosition[s]) { //determine smallest value
          inkjetMinPosition[s] = temp_ahead_pos;
          inkjetMaxPosition[s] = targetPosition[s];
        }
        else {
          inkjetMinPosition[s] = targetPosition[s];
          inkjetMaxPosition[s] = temp_ahead_pos;
        }
        targetPosition[s] = temp_ahead_pos; //set new position
        nextState[s] = 0; //set next state to next line again
      }
      //if the buffer is completely empty on both sides, set buffer first to 1 to allow for a clean restart when the buffer refills
      if (BurstBuffer.ReadLeftSide(0) == 0 && BurstBuffer.ReadLeftSide(1) == 0) { //if read buffer is completely empty
        serialBufferFirst[0] = 1;
        serialBufferFirst[1] = 1;
      }

    }
  } //end of s(ide) for loop
}
void BufferUpdateLoop() { //checks if the buffer looped, forcing a position reset
  static uint8_t bufferLoopHistory = 0;
  uint8_t tempLoop = BurstBuffer.GetLoopCounter();
  if (bufferLoopHistory != tempLoop) { //if buffer loop value changed, reset position
    bufferLoopHistory = tempLoop; //set new history
    //Serial.println("Buffer looped");
    if (triggerWhileActive == 1) { //only actually reset the position if printing is in while high mode
      PositionVirtualTrigger();
    }
  }
}
void BufferReset(){ //savely resets the buffer in a way that allows for a stable restart
  BurstBuffer.Reset(); //reset the buffer
  serialBufferFirst[0] = 1; //set buffers to firstline
  serialBufferFirst[1] = 1; //set buffers to firstline
}
void BufferClear(){ //savely clears the buffer in a way that allows for a stable restart
  BurstBuffer.ClearAll(); //clear all data in the buffer
  serialBufferFirst[0] = 1; //set buffers to firstline
  serialBufferFirst[1] = 1; //set buffers to firstline
}
void SerialExecute() { //on 1 in serial update, get values and execute commands
  //inkjetLineNumber = Ser.GetLineNumber(); //get line number
  inkjetCommand = Ser.GetCommand(); //get command
  inkjetSmallValue = Ser.GetSmallValue();  //get small value
  Ser.GetRaw(inkjetRaw); //get raw
  serialSource = Ser.GetSource();
  Ser.SetResponseSource(serialSource); //set serial response to go to the given source

  if (serialCommandEcho == 1) {
    Serial.print("#COM:"); Serial.println(inkjetCommand);
    if (serialSource == 1) { //if serial command came from externally
      Serial1.print("#COM:"); Serial1.println(inkjetCommand);
    }
  }

  //check line number (lol, nope, not right now, why did I bother then with a linenumber? dunno, might actually disable it)
  //Later edit, I already disabled it.

  //Serial.print("Executing command: "); Serial.println(inkjetCommand);
  switch (inkjetCommand) { //check command, execute based on command
    case 5456466: { //SBR, send buffer raw
        //Serial.println("Send to buffer, raw");
        //Serial.println(inkjetSmallValue);
        //for (uint8_t B = 0; B < 50; B++){
        //  Serial.print(inkjetRaw[B]); Serial.print(" ");
        //}
        //Serial.println("");
        dmaHP45.ConvertB6RawToBurst(inkjetRaw, DataBurst);
        //for (uint8_t B = 0; B < 22; B++){
        //  Serial.print(CurrentBurst[B]); Serial.print(" ");
        //}
        //Serial.println("");
        BurstBuffer.Add(inkjetSmallValue, DataBurst);
      } break;
    case 5456468: { //SBT, send buffer toggle

      } break;
    case 5456210: { //SAR, send asap raw
        dmaHP45.ConvertB6RawToBurst(inkjetRaw, DataBurst);
        dmaHP45.SetBurst(DataBurst, 1);
        dmaHP45.Burst();
      } break;
    case 5456212: { //SAT, send asap toggle

      } break;
    case 5261396: { //PHT, preheat
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 25000);
        uint8_t temp_enabled = dmaHP45.GetEnabledState();
        dmaHP45.SetEnable(1); //temporarily enable head
        dmaHP45.Preheat(inkjetSmallValue); //preheat for n times
        dmaHP45.SetEnable(temp_enabled); //set enable state to previous
      } break;
    case 5263949: { //PRM, prime printhead
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 25000);
        uint8_t temp_enabled = dmaHP45.GetEnabledState();
        dmaHP45.SetEnable(1); //temporarily enable head
        dmaHP45.Prime(inkjetSmallValue); //preheat for n times
        dmaHP45.SetEnable(temp_enabled); //set enable state to previous
      } break;
    case 5523524: { //THD, Test head
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 1);
        TestPrintheadFunctions(inkjetSmallValue); //test printhead
      } break;
    case 4674640: { //GTP, Get temperature
        Ser.RespondTemperature(dmaHP45.GetTemperature());
      } break;
    case 4669776: { //GAP, get active position
        Ser.RespondEncoderPos(PositionGetBasePositionMicrons());
      } break;
    case 4670800: { //GEP, Get encoder position
        //Serial.print("sending position: "); Serial.println(PositionGetBasePositionMicrons());
        Ser.RespondEncoderPos(PositionGetBaseEncoderPositionMicrons());
      } break;
    case 5457232: { //SEP, Set encoder position
        PositionSetBaseEncoderPositionMicrons(inkjetSmallValue); //Set encoder position with small value
      } break;
    case 5456976: { //SDP, Set DPI
        dmaHP45.SetDPI(inkjetSmallValue); //set DPI with small value
      } break;
    case 1396985929: { //SDPI, Set DPI
        dmaHP45.SetDPI(inkjetSmallValue); //set DPI with small value
      } break;
    case 5456974: { //SDN, Set Density
        inkjetDensity = inkjetSmallValue; //set density with small value
      } break;
    case 1397967172: { //SSID, Set Side
        BurstBuffer.SetPrintMode(inkjetSmallValue);
      } break;
    case 1112362820: { //BMOD, buffer mode
        BurstBuffer.SetMode(inkjetSmallValue); //set mode with small value
      } break;
    case 4346444: { //BRL, Buffer, read left
        Ser.RespondBufferReadLeft(BurstBuffer.ReadLeft());
      } break;
    case 1112689747: { //BRLS, buffer read left side
      Ser.RespondBufferReadLeftSide(BurstBuffer.ReadLeftSide(inkjetSmallValue));
    } break;
    case 4347724: { //BWL, Buffer, write left
        Ser.RespondBufferWriteLeft(BurstBuffer.WriteLeft());
      } break;
    case 4342604: { //BCL, Buffer clear
        BufferClear();
      } break;
    case 1112687955: { //BRES, Buffer reset
        BufferReset();
      } break;
    case 5391698: { //RER, reset error

      } break;
    case 5393486: { //RLN, reset line number

      } break;
    case 5457230: { //SEN, software enable

      } break;
    case 4736334: { //HEN, hardware enable
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 1);
        dmaHP45.SetEnable(inkjetSmallValue);
      } break;
    case 4279896: { //ANX, address next
        dmaHP45.AddressNext(); //next address
      } break;
    case 4280915: { //ARS, address reset
        dmaHP45.AddressReset(); //reset address
      } break;
    case 4278100: { //AGT, address goto
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 22); //limit size of small
        dmaHP45.AddressReset(); //reset address
        for (uint8_t a = 0; a < inkjetSmallValue; a++) {
          dmaHP45.AddressNext(); //next address
        }
      } break;
    case 5264212: { //PST, primitive set
        dmaHP45.SetPrimitivePins(inkjetSmallValue);
      } break;
    case 5260108: { //PCL, primtiveClock
        dmaHP45.SetPrimitiveClock(inkjetSmallValue);
      } break;
    case 5263436: { //PPL, primitive pulse
        //PrimitivePulse(uint16_t tempState)
      } break;
    case 1196311379: { //GNCS, get nozzle check state
        Ser.RespondNozzleCheck(dmaHP45.GetNozzleCheck());
      } break;
    case 5457234: { //SER, set encoder resolution
        PositionSetEncoderResolution(float(inkjetSmallValue));
      } break;
    case 4670802: { //GER, Get encode resolution
        Ser.RespondEncoderResolution(PositionGetEncoderResolution());
      } break;
    case 1447382593: { //VENA, Virtual enable
        PositionVirtualEnable(inkjetSmallValue);
      } break;
    case 1448366665: { //VTRI, Virtual Trigger
        TriggerVirtual();
      } break;
    case 1448301647: { //VSTO, Virtual stop
        PositionVirtualStop();
      } break;
    case 4675152: { //GVP, Get virtual position
        Ser.RespondVirtualPosition(PositionGetVirtualPosition());
      } break;
    case 5461590: { //SVV, Set virtual velocity
        PositionSetVirtualVelocity(inkjetSmallValue);
      } break;
    case 4675158: { //GVV, Get virtual velocity
        Ser.RespondVirtualVelocity(PositionGetVirtualVelocity());
      } break;
    case 5461586: {  //SVR, Set virtual position reset
        PositionVirtualSetStart(inkjetSmallValue);
      } break;
    case 1397771589: { //SPME, Set position mode to encoder
        PositionSetModeEncoder();
      } break;
    case 1397771606: { //SPMV, Set position mode to virtual
        PositionSetModeVirtual();
      } break;
    case 1398033712: { //STM0, set trigger mode 0
        TriggerSetPinMode(0, inkjetSmallValue);
      } break;
    case 1398033713: { //STM1, set trigger mode 1
        TriggerSetPinMode(1, inkjetSmallValue);
      } break;
    case 1398033714: { //STM2, set trigger mode 2
        TriggerSetPinMode(2, inkjetSmallValue);
      } break;
    case 1398033715: { //STM3, set trigger mode 3
        TriggerSetPinMode(3, inkjetSmallValue);
      } break;
    case 1398033716: { //STM4, set trigger mode 4
        TriggerSetPinMode(4, inkjetSmallValue);
      } break;
    case 1398033717: { //STM5, set trigger mode 5
        TriggerSetPinMode(5, inkjetSmallValue);
      } break;
    case 1398033718: { //STM6, set trigger mode 6
        TriggerSetPinMode(6, inkjetSmallValue);
      } break;
    case 1398033719: { //STM7, set trigger mode 7
        TriggerSetPinMode(7, inkjetSmallValue);
      } break;
    case 1398033720: { //STM8, set trigger mode 7
        TriggerSetPinMode(8, inkjetSmallValue);
      } break;
    case 1398034992: { //STR0, set trigger resistor 0
        TriggerSetResistor(0, inkjetSmallValue);
      } break;
    case 1398034993: { //STR1, set trigger resistor 1
        TriggerSetResistor(1, inkjetSmallValue);
      } break;
    case 1398034994: { //STR2, set trigger resistor 2
        TriggerSetResistor(2, inkjetSmallValue);
      } break;
    case 1398034995: { //STR3, set trigger resistor 3
        TriggerSetResistor(3, inkjetSmallValue);
      } break;
    case 1398034996: { //STR4, set trigger resistor 4
        TriggerSetResistor(4, inkjetSmallValue);
      } break;
    case 1398034997: { //STR5, set trigger resistor 5
        TriggerSetResistor(5, inkjetSmallValue);
      } break;
    case 1398034998: { //STR6, set trigger resistor 6
        TriggerSetResistor(6, inkjetSmallValue);
      } break;
    case 1398034999: { //STR7, set trigger resistor 7
        TriggerSetResistor(7, inkjetSmallValue);
      } break;
    case 1398035000: { //STR8, set trigger resistor 8
        TriggerSetResistor(8, inkjetSmallValue);
      } break;
    case 1397903445: { //SRPU: Set trigger push
        TriggerSetTriggerPush(inkjetSmallValue);
      } break;
    case 1397773136: { //SPSP:  Set pulse split
        dmaHP45.DMASetPulseSplit(inkjetSmallValue);
      } break;
    case 1196446544: { //GPSP:  Get pulse split
        Ser.RespondPulseSplit(dmaHP45.DMAGetPulseSplit());
      } break;
    case 1196900690: { //GWAR, get warning
        Ser.RespondWarning(warningList); //respond with warning
      } break;
    case 1195725394: { //GERR, get error
        Ser.RespondError(errorList); //respond with error
      } break;
    case 591613773: { //#COM, command echo (debug function)
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 1);
        serialCommandEcho = inkjetSmallValue;
        Serial.print("command echo:"); Serial.println(inkjetSmallValue);
        if (serialSource == 1) { //if serial command came from externally
          Serial1.print("command echo:"); Serial1.println(inkjetSmallValue);
        }
      } break;
    case 592331859: { //#NDS, number small decode
        Serial.print("#NDS:"); Serial.println(inkjetSmallValue);
        if (serialSource == 1) { //if serial command came from externally
          Serial1.print("#NDS:"); Serial1.println(inkjetSmallValue);
        }
      } break;
    case 592331858: { //#NDR, number raw decode
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 1);
        Ser.RespondRaw(inkjetRaw, INKJET_RAW_SIZE, inkjetSmallValue);
      } break;
    case 592332115: { //#NES, Number encode small
        Ser.RespondEncodeSmall(inkjetSmallValue);
      } break;
    case 591676738: { //#DEB, debug set
        DebugSet(inkjetSmallValue);
      } break;
    case 591875915: { //#GOK, get ok state
        Ser.RespondOkState();
      } break;
    case 592596811: { //#ROK, reset ok state
        Ser.ResetOkState();
      } break;
    case 591616323: { //#CYC set cycle counter state
        CycleCounterEnable(inkjetSmallValue);
      } break;
    case 63: { //?, help
        Ser.PrintHelp();
      }
    case 559370322: {//!WPR, write pin raw
        dmaHP45.WritePinRaw(inkjetSmallValue);
      } break;
    case 558452301: { //!INM, Inkjet mode (print function override)
        inkjetSmallValue = constrain(inkjetSmallValue, 0, 1);
        inkjetHardwareEnabled = inkjetSmallValue;
      } break;
    case 1413829460: { //TEST, arbitrary test function, only use for immediate debugging.
        Serial.println("Calling test function");
        //Serial.println(dmaHP45.TestAddress());
        Serial.println(EepromCheckSaved());
      } break;
    default: {
        Serial.println("Unknown command");
        if (serialSource == 1) { //if serial command came from externally
          Serial1.println("Unknown command");
        }
      }
  }
  //auto Writeleft status block
  serialBufferCounter++;
  if (serialBufferCounter > serialBufferTarget) {
    Ser.RespondBufferWriteLeft(BurstBuffer.WriteLeft()); //respond with write left
    serialBufferCounter = 0; //reset value
  }
}

void SerialWLPush() { //checks if requirements for a pushed WL response are met (write left)
  int32_t temp_buffer = BurstBuffer.WriteLeft();
  if (temp_buffer < bufferLowerThreshold) { //if write left is lower than threshold
    serialBufferWLToggle = 0; //set flag to low
  }
  if (temp_buffer > bufferUpperThreshold) { //if write left becomes higher than threshold
    if (serialBufferWLToggle == 0) { //if write left was not higher than threshold
      serialBufferWLToggle = 1; //set flag to high
      Ser.RespondBufferWriteLeft(temp_buffer); //respond with write left
    }
  }
}

//Trigger functions
void UpdateTrigger() { //looks at whether a trigger event happened and what needs to be done next
  static uint32_t triggerUpdateHistory;
  if (micros() - triggerUpdateHistory > TRIGGER_UPDATE_DELAY) { //limit the numer of updates
    triggerUpdateHistory = micros(); //set the new time
    //look if a trigger happened
    uint8_t tempResponse = TriggerUpdate();
    static uint8_t responseHistory;
    if (tempResponse == 1) {
      //what type of trigger happened (for now only "trigger")
      Serial.println("Trigger");

      //pass trigger to buffer
      if (BurstBuffer.GetMode() == 1 || BurstBuffer.GetMode() == 2) { //only reset the buffer is the printing mode is static or looping
        BufferReset(); //reset the buffer
      }

      //pass trigger to position
      PositionVirtualTrigger();
    }
    if (responseHistory == 1 && tempResponse == 2) { //if the response goes from a trigger to a while trigger, set while active to 1
      triggerWhileActive = 1;
    }
    if (responseHistory == 2 && tempResponse == 0) { //if history shows an active while trigger which became low, stop the trigger
      PositionVirtualStop();
      triggerWhileActive = 0;
      //Serial.println("Trigger stop");
    }
    responseHistory = tempResponse; //write trigger history to variable
  }
}

//error functions
void UpdateStatus() { //handles regular health checks and status led
  if (burstOn == 0) { //only check when the printhead is not actively printing to prevent errors
    if (millis() > checkTarget) { //check status
      checkTarget = millis() + checkDelay;
      errorState = 0; //reset error state
      int32_t temp_response;
      //check head logic voltage
      temp_response = dmaHP45.GetVoltageLogic();
      //Serial.println(temp_response);
      if (temp_response <  LOGIC_LOWER_VOLTAGE) {
        bitWrite(errorList, ERROR_LOGIC_VOLTAGE_LOW_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_LOGIC_VOLTAGE_LOW_BIT, 0);
      }
      if (temp_response >  LOGIC_UPPER_VOLTAGE) {
        bitWrite(errorList, ERROR_LOGIC_VOLTAGE_HIGH_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_LOGIC_VOLTAGE_HIGH_BIT, 0);
      }

      //check head driving voltage
      temp_response = dmaHP45.GetVoltageHead();
      //Serial.println(temp_response);
      if (temp_response <  DRIVING_LOWER_VOLTAGE) {
        bitWrite(errorList, ERROR_DRIVING_VOLTAGE_LOW_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_DRIVING_VOLTAGE_LOW_BIT, 0);
      }
      if (temp_response >  DRIVING_UPPER_VOLTAGE) {
        bitWrite(errorList, ERROR_DRIVING_VOLTAGE_HIGH_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_DRIVING_VOLTAGE_HIGH_BIT, 0);
      }

      //check head temperature
      temp_response = dmaHP45.GetTemperature();
      //Serial.println(temp_response);
      if (temp_response == -2) {
        bitWrite(errorList, ERROR_HEAD_TEMPERATURE_NC_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_HEAD_TEMPERATURE_NC_BIT, 0);
      }
      if (temp_response >  HEAD_WARNING_TEMPERATURE) {
        bitWrite(errorList, WARNING_HEAD_TEMPERATURE_HIGH_BIT, 1);
      }
      else {
        bitWrite(warningList, WARNING_HEAD_TEMPERATURE_HIGH_BIT, 0);
      }
      if (temp_response >  HEAD_ERROR_TEMPERATURE) {
        bitWrite(errorList, ERROR_HEAD_TEMPERATURE_HIGH_BIT, 1);
      }
      else {
        bitWrite(errorList, ERROR_HEAD_TEMPERATURE_HIGH_BIT, 0);
      }

      if (warningList != 0) { //check for warnings
        errorState = 1;
      }
      if (errorList != 0) { //check for errors
        errorState = 2;
      }
    }
  }

  if (millis() > errorTarget) { //update led
    errorTarget = millis() + errorDelay;
    if (errorState == 0) { //if no errors, simply set to solid on
      digitalWrite(statusLed, 1);
    }
    else {
      uint8_t temp_state = 1;
      if (errorState == 1) { //if warning, slow blink
        temp_state = errorAnimation / 5;
      }
      if (errorState == 2) { //if error, fast blink
        temp_state = errorAnimation % 2;
      }
      digitalWrite(statusLed, temp_state);
      errorAnimation ++;//update animation counter
      if (errorAnimation >= 10) { //reset value
        errorAnimation = 0;
      }
    }
  }
}
void TestPrintheadFunctions(uint8_t tempReport) { //test all hardware functions of the printhead. 0 prints only the basics, 1 gives a full report on everything
  if (tempReport == 1) {
    Serial.println("Generating detailed report");
  }
  //check voltage
  uint8_t voltage_check = 1;
  int32_t temp_response;
  temp_response = dmaHP45.GetVoltageLogic();
  if (tempReport == 1) {
    Serial.print("Logic voltage: "); Serial.print(temp_response / 1000); Serial.print(","); Serial.print((temp_response / 10) % 100); Serial.println("V");
  }
  if (temp_response <  LOGIC_LOWER_VOLTAGE || temp_response > LOGIC_UPPER_VOLTAGE) {
    voltage_check = 0;
  }
  temp_response = dmaHP45.GetVoltageHead();
  if (tempReport == 1) {
    Serial.print("Head voltage: "); Serial.print(temp_response / 1000); Serial.print(","); Serial.print((temp_response / 10) % 100); Serial.println("V");
  }
  if (temp_response <  DRIVING_LOWER_VOLTAGE || temp_response > DRIVING_UPPER_VOLTAGE) {
    voltage_check = 0;
  }
  if (voltage_check == 0) {
    Serial.println("Voltage too low for testing");
    if (serialSource == 1) { //if serial command came from externally
      Serial1.println("Voltage too low for testing");
    }
    return; //stop, since there is nothing to test without power
  }

  //see if a printhead is present
  uint8_t headPresent = 1;
  if (dmaHP45.GetTemperature() == -2) { //if the head is not present, set to 0
    headPresent = 0;
    if (tempReport == 1) {
      Serial.println("Printhead not found");
    }
  }
  else {
    if (tempReport == 1) {
      Serial.println("Printhead found");
    }
  }

  //test address functionality
  if (dmaHP45.TestAddress() == 0) { //if address does not work
    bitWrite(errorList, ERROR_ADDRESS_DEFECTIVE, 1);
    if (tempReport == 1) {
      Serial.println("Address circuit not working");
    }
  }
  else {
    bitWrite(errorList, ERROR_ADDRESS_DEFECTIVE, 0);
    if (tempReport == 1) {
      Serial.println("Address circuit functional");
    }
  }

  //test dummy nozzles
  uint8_t tempResult;
  if (headPresent == 1) {
    //Serial.println("Head present, testing bare dummy");
    //dummy 2 is bare resistor
    if (tempReport == 1) {
      Serial.println("Testing dummy 2, bare dummy");
    }
    tempResult = dmaHP45.TestDummy(1);
    if (tempResult == 1) { //if dummy tested ok
      bitWrite(errorList, ERROR_DUMMY2_NOT_RISING, 0);
      bitWrite(errorList, ERROR_DUMMY2_NOT_FALLING, 0);
      if (tempReport == 1) {
        Serial.println("Dummy 2 functional");
      }
    }
    else if (tempResult == 2) { //if dummy rose, but did not fall
      bitWrite(errorList, ERROR_DUMMY2_NOT_FALLING, 1);
      //Serial.println("not falling");
      if (tempReport == 1) {
        Serial.println("Dummy 2 not falling");
      }
    }
    else { //if dummy did not rise
      bitWrite(errorList, ERROR_DUMMY2_NOT_RISING, 1);
      //Serial.println("not rising");
      if (tempReport == 1) {
        Serial.println("Dummy 2 not rising");
      }
    }
  }
  else {
    //Serial.println("Head not present, testing grounded dummy");
    //dummy 1 is pulled down dummy
    if (tempReport == 1) {
      Serial.println("Testing dummy 1, resistor dummy");
    }
    tempResult = dmaHP45.TestDummy(0);
    if (tempResult == 1) { //if dummy tested ok
      bitWrite(errorList, ERROR_DUMMY1_NOT_RISING, 0);
      bitWrite(errorList, ERROR_DUMMY1_NOT_FALLING, 0);
      if (tempReport == 1) {
        Serial.println("Dummy 1 functional");
      }
    }
    else if (tempResult == 2) { //if dummy rose, but did not fall
      bitWrite(errorList, ERROR_DUMMY1_NOT_FALLING, 1);
      //Serial.println("not falling");
      if (tempReport == 1) {
        Serial.println("Dummy 1 not falling");
      }
    }
    else { //if dummy did not rise
      bitWrite(errorList, ERROR_DUMMY1_NOT_RISING, 1);
      //Serial.println("not rising");
      if (tempReport == 1) {
        Serial.println("Dummy 1 not rising");
      }
    }
  }

  dmaHP45.TestHead(NozzleState, AddressState, PrimitiveState); //test nozzles second
  if (tempReport == 1) { //if a full report is requested
    //report of working nozzles
    uint16_t workingNozzles = 0;
    for (uint16_t n = 0; n < 300; n++) {
      if (NozzleState[n] == 1) {
        workingNozzles++;
      }
    }
    Serial.print("nozzles functional: "); Serial.print(workingNozzles); Serial.println(" of 300");

    //report of address array
    Serial.print("Nozzles per address: ");
    for (uint8_t a = 0; a < 22; a++) {
      Serial.print(AddressState[a]);
      Serial.print(", ");
    }
    Serial.println();

    //report of primitive array
    Serial.print("Nozzles per primitive: ");
    for (uint8_t p = 0; p < 14; p++) {
      Serial.print(PrimitiveState[p]);
      Serial.print(", ");
    }
    Serial.println();
  }
  else {
    Ser.RespondTestResults(1, NozzleState);
  }
}

//debugging
void CycleCounterEnable(uint8_t tempState) { //sets the cycle counter
  tempState = constrain(tempState, 0, 1);
  cycleCounterEnabled = tempState;
}
void UpdateCycle() { //a counter that counts the number of program cycles per second. Used for debugging
  if (cycleCounterEnabled == 1) {
    cycleCounter++; //add one to cycle
    if (millis() > cycleTarget) {
      cycleTarget = millis() + 1000; //set next target
      Serial.print("Cycles p / s: "); Serial.println(cycleCounter); //port cycles
      cycleCounter = 0; //reset counter
    }
  }
}
void DebugSet(uint8_t tempInput) {
  if (tempInput == 0) { //set all off
    Ser.DebugSet(0);
  }
  else if (tempInput == 1) { //set all on
    Ser.DebugSet(1);
  }
  else if (tempInput == 2) { //serial only
    Ser.DebugSet(1);
  }
}

void TestPrimitives() {
  uint16_t tempVal = 0;
  for (uint8_t t = 0; t < 16; t++) {
    tempVal = 0;
    bitWrite(tempVal, t, 1);
    dmaHP45.SetPrimitiveClock(0);
    dmaHP45.SetPrimitivePins(tempVal);
    dmaHP45.SetPrimitiveClock(1);
    delay(1000);
    dmaHP45.SetPrimitiveClock(0);
    delay(1000);
  }
}

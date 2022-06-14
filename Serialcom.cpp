/*
   Serial block takes care of receiving and decoding the serial connection
   The serial port used is the hardware USB to serial converter within the microcontroller itself.

   The serial, both ways sends in blocks of 64 bytes. The block buffer both ways is 64 bytes
   Each block consists of a semi fixed structure, the default is:
   CC-PPPPP-[50xR]e
   (CCC-PPPPP-RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRe)

   The actual amount can change, but a full line may not be more than 64 bytes including end characters

   - is a whitespace, to indicate an end of a line
   C is a command. Each function has a 3 letter command
   P is a small number, in inkjet reserved for position
   R Raw data, in inkjet reserved for inkjet data
   e end character ('\r' of '\n')

   Based on context, some blocks can be different, but by default all value carrying blocks will be encoded in base 64
   from 0 to 63: ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/

  The list of commands is as follows:
  //direct inkjet commands
  -SBR:  Send inkjet to buffer raw
  -SBT:  Send inkjet to buffer toggle format <------------- to do
  -SAR:  Send inkjet to print ASAP raw
  -SAT:  Send inkjet to print ASAP toggle format <------------- to do

  //text print commands
  -SBX:  Send buffer text <------------- to do
  -SFNT: Set font <------------- to do
  -STXS: Set text Size <------------- to do


  //inkjet support functions
  -PHT:  Preheat printhead
  -PRM:  Prime printhead
  -THD:  Test printhead (small value 1 returns n of 300 nozzles)
  -GTP:  Get temperature
  -SDP:  Set DPI
  -SDPI: Set DPI
  -SDN:  Set density
  -SSID: Set side
  -SPSP: Set pulse splits
  -GPSP: Get pulse splits

  -PRMD: Print mode (serial, eeprom, text) <------------ to do

  //position commands
  -SPME: Set position mode to encoder
  -SPMV: Set position mode to virtual
  
  -SEP: Set encoder position
  -GEP: Get encoder position
  -SER: Set encoder Resolution
  -GER: Get encoder Resolution

  -VENA: Virtual enable 
  -GVP: Get virtual position 
  -SVV: Set virtual velocity 
  -GVV: Get virtual velocity 
  -SVR: Set virtual position reset 

  //triggers
  -VTRI: Virtual Trigger 
  -VSTO: Virtual stop 
  -STM0: Set trigger mode trigger 0 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM1: Set trigger mode trigger 1 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM2: Set trigger mode trigger 2 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM3: Set trigger mode trigger 3 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM4: Set trigger mode trigger 4 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM5: Set trigger mode trigger 5 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM6: Set trigger mode trigger 6 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM7: Set trigger mode trigger 7 (0 off, 1 rising, 2 falling, 3 toggle)
  -STM8: Set trigger mode trigger 8 (0 off, 1 rising, 2 falling, 3 toggle)
  -STR0: Set trigger resistor 0 (0 floating, 2 pulldown, 3 pullup)
  -STR1: Set trigger resistor 1 (0 floating, 2 pulldown, 3 pullup)
  -STR2: Set trigger resistor 2 (0 floating, 2 pulldown, 3 pullup)
  -STR3: Set trigger resistor 3 (0 floating, 2 pulldown, 3 pullup)
  -STR4: Set trigger resistor 4 (0 floating, 2 pulldown, 3 pullup)
  -STR5: Set trigger resistor 5 (0 floating, 2 pulldown, 3 pullup)
  -STR6: Set trigger resistor 6 (0 floating, 2 pulldown, 3 pullup)
  -STR7: Set trigger resistor 7 (0 floating, 2 pulldown, 3 pullup)
  -STR8: Set trigger resistor 8 (0 floating, 2 pulldown, 3 pullup)
  -STPU: Set trigger push <----------- to do

  //buffer commands
  -BMOD: Buffer mode: 0 for clearing, 1 for static, 2 for looping
  -BRL:  Buffer get read left
  -BRLS: Buffer get read left per side
  -BWL:  Buffer get write left
  -BCL:  Buffer clear, remove all data from the buffer
  -BRES: Buffer reset, set to position 0

  //status calls
  -GWAR: Get warnings. All non critical issues
  -GERR: Get error. All critical issues
  
  //direct control commands
  -SEN:  Software enable printhead
  -HEN:  Hardware enable printhead

  -ANX:  Address Next
  -ARS:  Address Reset
  -AGT:  Address Goto
  -PST:  Primitive set
  -PCL:  PrimitiveClock
  -PPL:  Primitive pulse
  -GNCS: Get nozzle check state

  //debug commands
  -#COM: Command echo
  -#NDS: Numberdecode small
  -#NDR: Number decode raw
  -#NES: Number encode small
  -#TST: Test program <------------- to do
  -#DEB: Debug mode
  -#GOK: Get ok state (of external serial)
  -#ROK: Reset the ok state (of external serial)
  -#CYC: set cycle counter state (A or B)
  -!WPR: Write pin raw
  -!INM: Inkjet Mode

  -debug modes:
  0: all off
  1: all on
  2: Serial only

  Also add functions for software triggers, virtual speed
  Add flush buffer at overflow

  timing the update code for ordinary serial commands gives 24us for a simple "GTP" and 55us for "SBR A AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
  It is logical to conclude that the most time it will take to convert a line is 55us for
*/

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//add serial port read function
//add ok check and send clear
//add serial send function with block
//add serial send variable
//external serial echo to main serial

#include "Arduino.h"

#define MAX_READ_LENGTH 130

class SerialCommand {
    //class member variables
    uint8_t serialUpdateState = 0; //whether the outside needs to call functions
    int8_t errorFlag = 0; //what error if any is happening
    uint8_t serialExternalState = 2; //external serial mode, 0 is off, 1 is passthrough, 2 is input mode
    uint8_t serialResponseSource = 0; //where to respond to. 0 is serial, 1 is serial1
    uint8_t serialExternaPassthroughResponse = 0; //whether data received from the external serial needs to be passed to
#define EXT_SERIAL_BAUDRATE 115200
#define EXT_SERIAL_UPDATE_DELAY 10 //in milliseconds
    uint32_t extSerialUpdateTarget;

    //read variables
    uint16_t serialAvailable; //number of available bytes in hardware buffer
    
    char readBuffer[64]; //buffer for storing serial characters
    char decodeBuffer[MAX_READ_LENGTH]; //buffer for storing serial line
    uint16_t decodeBufferPos = 0; //how much is in the read buffer

    char intDecodeBuffer[MAX_READ_LENGTH]; //buffer for storing serial line
    uint16_t intDecodeBufferPos = 0; //how much is in the read buffer
    uint16_t intSerialAvailable; //number of lines available on the internal USB to serial
    
    char extDecodeBuffer[MAX_READ_LENGTH]; //buffer for storing serial line
    uint16_t extDecodeBufferPos = 0; //how much is in the read buffer
    uint16_t extSerialAvailable; //number of lines available on the hardware serial (Serial1)
   
    int16_t endCharacterPos; //where the end character is
    uint8_t anotherLine = 0; //whether there might be another line.

    //write variables
    char writeBuffer[64]; //buffer for handling serial output
    uint8_t writeCharacters; //how many characters are in the buffer to be sent out

#define EXTERNAL_SERIAL_BUFFER_SIZE 128
    char externalInputBuffer[EXTERNAL_SERIAL_BUFFER_SIZE]; //for handling the input from the external serial
    uint16_t externalInputPosition = 0; //where in the input buffer the cursor is
    uint8_t externalSerialClear = 1;
    char externalOutputBuffer[EXTERNAL_SERIAL_BUFFER_SIZE]; //for handling outgoing text of the external serial


    //latest decoded line values
    uint16_t serialLineNumber;
    uint32_t serialCommand;
    int32_t serialSmallValue;
    uint8_t serialRaw[50];
    uint8_t serialSource;

    //debug
    uint8_t serialDebugEnabled;

  public:
    SerialCommand() {
      Serial.begin(115200); //Speed is actually irrelephant for this serial, it is a virtual port
      writeCharacters = 0;

      Serial1.begin(EXT_SERIAL_BAUDRATE); //start external serial with the defined baudrate
    }

    void Begin() { //sends start message to indicate live connection
      Serial.println("HP45 standalone V4 Version 0.08"); //Send version number over serial
      Serial.println("Type ? for help"); //help prompt

      serialDebugEnabled = 0; //set debug to 0
    }

    int16_t Update() { //updates all serial command functions
      /*
         Read what is currently in the buffer
         if a full line is in buffer, read and decode it
         If the program needs to act on a new line, update will return a 1, else a 0
         returns a -negative on error
      */

      if (serialExternalState == 1) { //if external serial is in passthrough mode
        if (millis() > extSerialUpdateTarget) { //update limiter
          extSerialUpdateTarget = millis() + EXT_SERIAL_UPDATE_DELAY;
          uint16_t extSerialLines = Serial1.available();
          uint8_t is_ok = 0;
          if (extSerialLines > 0) { //check external serial data is received
            //Serial.print("Serial characters found: "); Serial.println(extSerialLines);
            for (uint16_t r = 0; r < extSerialLines; r++) { //pass data to input buffer
              externalInputBuffer[externalInputPosition] = Serial1.read();
              externalInputPosition++;

              if (externalInputPosition >= EXTERNAL_SERIAL_BUFFER_SIZE) { //if last space in buffer is reached
                Serial.println("External input buffer full!"); //return error
              }
            }
            //Serial.println(externalInputBuffer);
          }

          if (externalInputPosition > 0) {

            int16_t extEndCharacter = -1; //end character position for external serial port
            for (uint16_t r = 0; r < EXTERNAL_SERIAL_BUFFER_SIZE; r++) { //see if data contains end of line character
              if (IsEndCharacter(externalInputBuffer[r]) == 2) { //end character found
                extEndCharacter = r;
                //Serial.print("End character: "); Serial.println(r);
                break; //break from checking
              }
            }

            //see if data contains ok
            uint8_t tempOk = 0;
            if (externalInputBuffer[0] == 'o' || externalInputBuffer[0] == 'O') {
              tempOk ++;
            }
            if (externalInputBuffer[1] == 'k' || externalInputBuffer[1] == 'K') {
              tempOk ++;
            }
            if (tempOk == 2) { //if both characters matched, line is ok
              is_ok = 1;
              externalSerialClear = 1;
              //Serial.println("Ok found");
            }

            if (serialExternaPassthroughResponse == 1) { //if the non ok responses of the external serial need to be passed through to main serial
              if (is_ok == 0) {
                //Serial.println("Sending external serial response");
                for (uint8_t w = 0; w < extEndCharacter; w++) { //pass all data that is not ok to the serial line.
                  Serial.print(externalInputBuffer[w]);
                }
              }
            }

            //remove processed lines from the buffer
            uint16_t overwritePos = extEndCharacter + 1;
            for (uint16_t w = 0; w < EXTERNAL_SERIAL_BUFFER_SIZE; w++) { //loop through the buffer
              if (overwritePos < EXTERNAL_SERIAL_BUFFER_SIZE) { //if the overwrite pos is still within the bounds of the buffer
                externalInputBuffer[w] = externalInputBuffer[overwritePos];
                overwritePos ++;
              }
            }
            //set read cursor to correct position
            //Serial.print("end character start pos"); Serial.println(externalInputPosition);
            externalInputPosition -= (extEndCharacter + 1); //subtract end character pos from read position (plus 1 to prevent off by 1)
            //Serial.print("end character end pos"); Serial.println(externalInputPosition);
          }
        }
      }
 


      //get USB serial and HW serial
      serialSource = 0; //set source to preferred value, 0 is internal, 1 is external
      intSerialAvailable = Serial.available(); //get number available
      if (serialExternalState == 2){
        extSerialAvailable = Serial1.available(); //get number available
      }
      else {
        extSerialAvailable = 0;
      }
      //decide which serial to read (USB has preference)
      if (intSerialAvailable > 0){ //prefer external serial over internal
        serialAvailable = intSerialAvailable; //write internal to available
      }
      else {
        serialAvailable = extSerialAvailable; //write external to available
        serialSource = 1;
      }

      //parse the chosen serial
      if (serialAvailable || anotherLine == 1) { //if there are lines in hardware buffer
        anotherLine = 0; //reset another line
        static uint32_t temp_timer;
        if (serialDebugEnabled == 1) {
          Serial.println("Starting conversion");
          Serial.print("Source "); if (serialSource == 0){ Serial.println("USB serial");} if (serialSource == 1){ Serial.println("external serial");}
          Serial.print("Available: ");  Serial.println(serialAvailable);
          temp_timer = micros(); //start of timing code
        }
        
        //write the source array to the read array 
        if (serialSource == 0){ //if internal
          memcpy(decodeBuffer, intDecodeBuffer, sizeof(intDecodeBuffer[0])*MAX_READ_LENGTH);
          decodeBufferPos = intDecodeBufferPos; //set read position
        }
        else { //if external
          memcpy(decodeBuffer, extDecodeBuffer, sizeof(extDecodeBuffer[0])*MAX_READ_LENGTH);
          decodeBufferPos = extDecodeBufferPos; //set read position
        }

        serialAvailable = constrain(serialAvailable, 0, 64); //constrain lines available to hardware block size
        if (serialSource == 0){
          Serial.readBytes(readBuffer, serialAvailable); //read bytes from hardware buffer
          //send received response
          Serial.print("OK\n"); //print ok
          Serial.send_now(); //send data ASAP
        }
        else {
          Serial1.readBytes(readBuffer, serialAvailable); //read bytes from hardware buffer
          //send received response
          Serial1.print("OK\n"); //print ok
        }
        

        //calculate decode buffer space left
        int16_t tempSpaceLeft = MAX_READ_LENGTH;
        tempSpaceLeft -= decodeBufferPos; 
        tempSpaceLeft -= serialAvailable;
        uint8_t gcodeSuspected = 0;
        uint8_t gcodeFound = 0;

        if (serialDebugEnabled == 1) {
          Serial.print("received: "); Serial.write(readBuffer, serialAvailable); Serial.println("");
          Serial.print("Spaceleft: "); Serial.println(tempSpaceLeft);
        }

        uint16_t temp_read = 0;
        if (tempSpaceLeft > 0) { //add readBuffer to serial_buffer if possible 
          for (uint16_t b = decodeBufferPos; b < decodeBufferPos + serialAvailable; b++) { //add to read buffer 
            decodeBuffer[b] = readBuffer[temp_read]; 
            temp_read ++; //add one to readbuffer position
          }
          decodeBufferPos += serialAvailable; 
          if (serialDebugEnabled == 1) {
            Serial.print("Buffer: '"); for (uint16_t r = 0; r < MAX_READ_LENGTH; r++) {
              Serial.print(decodeBuffer[r]); 
            } Serial.println("'");
            Serial.print("Decode buffer pos: "); Serial.println(decodeBufferPos);
          }
          
          endCharacterPos = -1; //reset end character pos
          for (uint16_t b = 0; b < MAX_READ_LENGTH; b++) { //loop through buffer to find end character (start at front)
            if (IsEndCharacter(decodeBuffer[b]) == 2) { //if end of line character
              endCharacterPos = b; //set end character search
              break; //break from search loop
            }
          }

          uint16_t tempStart = 0, tempEnd = 0; //keeps track of the blocks to decode
          uint8_t tempCharacter = 0; //keeps track of which character is being read
          uint8_t tempReadError = 0, tempEnded = 0;
          uint32_t tempDecode;
          if (endCharacterPos != -1) { //if end character found, start decoding from front
            if (serialDebugEnabled == 1) {
              Serial.print("End of line character found: ");
              Serial.println(endCharacterPos);
            }

            //do initial character check for G and M and set a variable <
            if (serialExternalState == 1) {  //see if external serial port is in passthrough mode
              //check if the first character is a G or M
              if (decodeBuffer[tempStart] == 'G' || decodeBuffer[tempStart] == 'M') {
                gcodeSuspected = 1; //mark gcode suspected flag for processing later
                if (serialDebugEnabled == 1) {
                  Serial.println("Potential Gcode found");
                }
              }
            }

            //reset everything to 0 when reading starts
            //serialLineNumber = 0;
            serialCommand = 0;
            serialSmallValue = 0;
            for (uint8_t r = 0; r < 50; r++) {
              serialRaw[r] = 0;
            }

            //look for first break, command
            if (tempReadError == 0 && tempEnded == 0) { //check if no error and not ended
              if (serialDebugEnabled == 1) {
                Serial.println("Start looking for command");
              }
              for (uint16_t e = tempStart; e <= endCharacterPos; e++) { //look for second break, command
                if (IsEndCharacter(decodeBuffer[e])) {
                  if (IsEndCharacter(decodeBuffer[e]) == 2) {
                    tempEnded = 1;  //set ended to 1 if this was a line end, not a space
                    //if (serialDebugEnabled == 1){ Serial.println("Ended");}
                  }
                  if (serialDebugEnabled == 1) {
                    Serial.print("End char in CM: ");
                    Serial.println(e);
                  }
                  serialCommand = 0; //reset command number
                  tempEnd = e; //note the position where the line ends
                  for (int16_t d = tempEnd - 1; d >= tempStart; d--) { //decode from end (LSB) to start (MSB)
                    tempDecode = decodeBuffer[d];//find command (raw ascii to value)
                    tempDecode = tempDecode << 8 * tempCharacter; //store temp command
                    tempCharacter++;
                    serialCommand |= tempDecode;
                  }
                  if (serialDebugEnabled == 1) {
                    Serial.print("Decoded CM: ");
                    Serial.println(serialCommand);
                  }
                  tempStart = e + 1; //set new start
                  tempCharacter = 0;
                  break; //break from current for loop
                }
              }
            }

            //check for gcode commands to pass through 
            //break from decoding when ordinary code is found
            //pass through line over ordinary serial port
            if (gcodeSuspected == 1) { //if gcode was suspected in previous line
              gcodeFound = 1;
              switch (serialCommand) {
                case 18225: //G1
                  break;
                case 4665904: //G20
                  break;
                case 4665905: //G21
                  break;
                case 4665912: //G28
                  break;
                case 4667696: //G90
                  break;
                case 4667697: //G91
                  break;
                case 4667698: //G92
                  break;
                case 1295069238: //M106
                  break;
                case 1295069239: //M107
                  break;
                case 1295069490: //M112
                  break;
                default:
                  gcodeFound = 0;
                  break;
              }
              if (gcodeFound == 1) { //if a full gcode statement was found, break from reading
                tempEnded = 1;
                //externalSerialClear = 1; //comment if ok is not reliable
                if (externalSerialClear == 1) {
                  //write gcode to external serial port
                  Serial.println("Sending to motion:");
                  for (uint16_t w = 0; w <= endCharacterPos; w++) {
                    Serial1.print(decodeBuffer[w]);
                    Serial.print(decodeBuffer[w]);
                  }
                  Serial.println("");
                  externalSerialClear = 0;
                }
                else {
                  //if new data was received before old data was passed on, give error over serial port
                  Serial.println("Line LOST!");
                }
              }

              if (serialDebugEnabled == 1) {
                Serial.println(serialCommand); //give the numer of the serial command
                if (gcodeFound == 1) {
                  Serial.println("Gcode found, breaking decoding and passing line to external serial");
                }
              }
            }

            //look for second break, small value
            int8_t tempSigned = 1;
            if (tempReadError == 0 && tempEnded == 0) { //check if no error and not ended
              if (serialDebugEnabled == 1) Serial.println("Start looking for small value");
              for (uint16_t e = tempStart; e <= endCharacterPos; e++) { //look for third break, small
                if (IsEndCharacter(decodeBuffer[e])) {
                  if (IsEndCharacter(decodeBuffer[e]) == 2) {
                    tempEnded = 1;  //set ended to 1 if this was a line end, not a space
                    //if (serialDebugEnabled == 1){ Serial.println("Ended");}
                  }
                  if (serialDebugEnabled == 1) {
                    Serial.print("End char in SV: ");
                    Serial.println(e);
                  }
                  //look for first character being a sign
                  if (decodeBuffer[tempStart] == '-') { //if first character is sign
                    if (serialDebugEnabled == 1) {
                      Serial.println("Sign found");
                    }
                    tempStart ++; //ignore first character
                    tempSigned = -1; //set sign to -1
                  }
                  serialSmallValue = 0; //reset command number
                  tempEnd = e; //note the position where the line ends
                  for (int16_t d = tempEnd - 1; d >= tempStart; d--) { //decode from end (LSB) to start (MSB)
                    if (IsB64(decodeBuffer[d])) { //if B64 value
                      tempDecode = B64Lookup(decodeBuffer[d]); //decode lead number
                      tempDecode = tempDecode << (6 * tempCharacter);
                      tempCharacter++;
                      serialSmallValue |= tempDecode; //add value to lead number
                    }
                    else { //if any non B64 char found, go to error
                      tempReadError = 1;
                      if (serialDebugEnabled == 1) {
                        Serial.print("Error");
                      }
                    }
                  }
                  serialSmallValue = serialSmallValue * tempSigned; //add sign to value
                  if (serialDebugEnabled == 1) {
                    Serial.print("Decoded SV: ");
                    Serial.println(serialSmallValue);
                  }
                  tempStart = e + 1; //set new start
                  tempCharacter = 0;
                  break; //break from current for loop
                }
              }
            }

            //look for third break, raw value
            if (tempReadError == 0 && tempEnded == 0) { //check if no error and not ended
              if (serialDebugEnabled == 1) {
                Serial.println("Start looking for raw value");
              }
              for (uint16_t e = tempStart; e <= endCharacterPos; e++) { //look for last break, command
                if (IsEndCharacter(decodeBuffer[e])) {
                  if (IsEndCharacter(decodeBuffer[e]) == 2) {
                    tempEnded = 1;  //set ended to 1 if this was a line end, not a space
                    //if (serialDebugEnabled == 1){ Serial.println("Ended");}
                  }
                  if (serialDebugEnabled == 1) {
                    Serial.print("End char in Raw: ");
                    Serial.println(e);
                  }
                  tempEnd = e; //note the position where the line ends
                  for (int16_t d = tempEnd - 1; d >= tempStart; d--) { //decode from end (LSB) to start (MSB)
                    if (IsB64(decodeBuffer[d])) { //if B64 value
                      tempDecode = B64Lookup(decodeBuffer[d]); //decode lead number
                      serialRaw[tempCharacter] = tempDecode;
                      tempCharacter++;
                    }
                    else { //if any non B64 char found, go to error
                      tempReadError = 1;
                      if (serialDebugEnabled == 1) {
                        Serial.print("Error");
                      }
                    }
                  }
                  //While block is disabled because now at the start the value is reset to 0 ------
                  /*while (tempCharacter < 50){ //fill the remainder of the buffer with 0
                    serialRaw[tempCharacter] = 0;
                    tempCharacter++;
                    }*/
                  /*if (serialDebugEnabled == 1) {
                    Serial.print("Decoded Raw: ");
                    for (uint8_t r = 0; r < 50; r++) {
                      Serial.print(serialRaw[r]); Serial.print(", ");
                    }
                    Serial.println("");
                    }*/
                  break; //break from current for loop
                }
              }
            }
            
            //determine if there might be another line
            if (endCharacterPos < decodeBufferPos - 1){
              anotherLine = 1;
              //Serial.println("Another line suspected");
            }

            //remove read bytes from decode buffer
            if (serialDebugEnabled == 1) {
              Serial.println("Removing read lines");
            }
            //remove read bytes from decode buffer
            int16_t tempReset = endCharacterPos + 1;
            int16_t tempDecodeBufferPos = decodeBufferPos;  //temporary decodebuffer pos
            decodeBufferPos = decodeBufferPos - endCharacterPos - 1; //subtract end character pos from decode buffer
            for (uint16_t r = 0; r < MAX_READ_LENGTH; r++) {
              if (tempDecodeBufferPos > 0) {
                decodeBuffer[r] = decodeBuffer[tempReset];
                tempDecodeBufferPos--; //remove one pos from decode buffer
                tempReset++; //remove one to read value
              }
              else {
                decodeBuffer[r] = 0;
              }
            }

            //Write decode buffer and pos back to source buffer
            if (serialSource == 0){ //if internal
              memcpy(intDecodeBuffer, decodeBuffer, sizeof(decodeBuffer[0])*MAX_READ_LENGTH);
              intDecodeBufferPos = decodeBufferPos; //set read position
            }
            else { //if external
              memcpy(extDecodeBuffer, decodeBuffer, sizeof(decodeBuffer[0])*MAX_READ_LENGTH);
              extDecodeBufferPos = decodeBufferPos; //set read position
            }

            if (serialDebugEnabled == 1) {
              Serial.print("Decode buffer pos: "); Serial.println(decodeBufferPos);
              uint32_t total_time = micros() - temp_timer;
              Serial.print("conversion took "); Serial.println(total_time); //end of timing code
            }
            if (tempReadError == 0 && tempEnded == 1) { //if ended and no mistakes
              if (serialSource == 0){ //if source is USB
                return 1;  //return a 1
              }
              if (serialSource == 1){ //if source is external serial
                return 2;  //return a 2
              }
              return -2; //if somehow the code magics itself broken and skips both options
            }
            else {
              return 0; //mistake somewhere along the way
            }
          } //end of end character found if
          //Write decode buffer and pos back to source buffer (second time, for a different path)
          //Serial.println("no end character memcpy");
          if (serialSource == 0) { //if internal
            memcpy(intDecodeBuffer, decodeBuffer, sizeof(decodeBuffer[0])*MAX_READ_LENGTH);
            intDecodeBufferPos = decodeBufferPos; //set read position
          }
          else { //if external
            memcpy(extDecodeBuffer, decodeBuffer, sizeof(decodeBuffer[0])*MAX_READ_LENGTH);
            extDecodeBufferPos = decodeBufferPos; //set read position
          }
        } //end of spaceleft > 0
        else { //if there was no space left in buffer
          return -1;
        }
      } //end of serial available
      return 0; //no new data, return a 0
    }
    uint16_t GetBufferLeft() { //gets how many characters are left in the buffer
      uint16_t temp_return = MAX_READ_LENGTH - 1;
      temp_return -= decodeBufferPos;
      return temp_return;
    }
    uint16_t GetLineNumber() { //returns the last read line number
      return serialLineNumber;
    }
    uint32_t GetCommand() { //returns the last read command
      return serialCommand;
    }
    int32_t GetSmallValue() { //returns the last read small value
      return serialSmallValue;
    }
    uint8_t GetSource(){
      return serialSource;
    }
    void GetRaw(uint8_t* temp_raw) { //takes an uint8_t[50] array and fills it with the last read value
      for (uint8_t r = 0; r < 50; r++) {
        temp_raw[r] = serialRaw[r];
      }
    }
    uint8_t IsEndCharacter(char tempInput) { //checks if a char is an end character, 1 for block end, 2 for line end
      if (tempInput == 10) return 2; //new line
      if (tempInput == 13) return 2; //carriage return
      if (tempInput == 32) return 1; //space
      return 0;
    }
    uint8_t IsB10(char tempInput) {  //reads a character and determines if it is a valid base 10 character
      if (tempInput >= '0' && tempInput <= '9') return 1;
      if (tempInput == '-') return 1; //negative is accepted in base 10 mode
      return 0; //no match found
    }
    uint8_t IsB64(char tempInput) { //reads a character and determines if it is a valid base 64 character
      if (tempInput >= 'A' && tempInput <= 'Z') return 1;
      if (tempInput >= 'a' && tempInput <= 'z') return 1;
      if (tempInput >= '0' && tempInput <= '9') return 1;
      if (tempInput == '+') return 1;
      if (tempInput == '/') return 1;
      return 0; //no match found
    }
    int8_t B10Lookup(char c) { //returns the 0-9 from ascii
      if (c >= '0' && c <= '9') return c - 48; //0-9
      return -1; //-1
    }
    int8_t B64Lookup(char c) { //Convert a base 64 character to decimal, (returns -1 if no valid character is found)
      if (c >= 'A' && c <= 'Z') return c - 'A'; //0-25
      if (c >= 'a' && c <= 'z') return c - 71; //26-51
      if (c >= '0' && c <= '9') return c + 4; //52-61
      if (c == '+') return 62; //62
      if (c == '/') return 63; //63
      return -1; //-1
    }
    char ToB64Lookup(uint8_t tempInput) {
      if (tempInput >= 0 && tempInput <= 25) return tempInput + 'A'; //0-25
      if (tempInput >= 26 && tempInput <= 51) return tempInput + 71; //26-51
      if (tempInput >= 52 && tempInput <= 61) return tempInput - 4; //52-61
      if (tempInput == 62) return '+';
      if (tempInput == 63) return '/';
      return '&'; //return '&' if false, this will trigger code reading it to mark as mistake
    }
    void WriteValueToB64(int32_t tempInput) { //adds a value to the writebuffer in B64
      //Serial.println("Encoding to B64");
      //Serial.print("Number: "); Serial.println(tempInput);
      uint8_t tempSign = 0;
      uint8_t temp6Bit;
      char tempReverseArray[5];
      if (tempInput < 0) { //if value is negative
        tempSign = 1; //set negative flag to 1
        tempInput = abs(tempInput); //make the input positive
      }
      for (int8_t a = 0; a < 5; a++) { //loop through the input to convert it
        //Serial.print("Encoding: "); Serial.println(a);
        temp6Bit = 0; //reset 6 bit value
        temp6Bit = tempInput & 63; //read first 6 bits
        //Serial.print("Turns to: "); Serial.println(temp6Bit);
        tempReverseArray[a] = ToB64Lookup(temp6Bit);
        tempInput = tempInput >> 6; //shift bits
      }
      //add reverse array to output buffer
      if (tempSign == 1) { //add negative to value
        writeBuffer[writeCharacters] = '-';
        writeCharacters++;
      }
      uint8_t tempStarted = 0;
      for (int8_t a = 4; a >= 0; a--) {
        if (tempStarted == 0) { //until a value is seen all zero's ('A''s) are ignored
          if (tempReverseArray[a] != 'A') tempStarted = 1; //start when first non 0 is found
          if (a == 0) tempStarted = 1; //also start when the first character is reached
        }
        if (tempStarted == 1) { //when started, write to output buffer
          writeBuffer[writeCharacters] = tempReverseArray[a];
          writeCharacters++;
        }
      }
    }
    void WriteTestArrayToB64(uint8_t tempInput[]) { //adds a test array to the writebuffer
      uint16_t tempN = 0; //nozzle counter
      uint8_t temp6Bit; //6 bit decode value
      char tempOutput[50]; //50 char output array
      for (uint8_t r = 0; r < 50; r++) {
        tempOutput[r] = ' '; //reset array values
      }
      //decode 300 array to B64
      for (uint8_t B = 0; B < 50; B++) {
        temp6Bit = 0; //reset 6 bit value
        for (uint8_t b = 0; b < 6; b++) {
          //Serial.print("Decoding: "); Serial.print(tempN); Serial.print(", Value: "); Serial.println(tempInput[tempN]);
          temp6Bit |= (tempInput[tempN] & 1) << b;
          tempN++;
        }
        tempOutput[B] = ToB64Lookup(temp6Bit); //write 6 bit value to output
        //Serial.print("Encoding: "); Serial.print(temp6Bit); Serial.print(", Value: "); Serial.println(tempOutput[B]);
      }

      //write B64 to output buffer
      for (int8_t B = 49; B >= 0; B--) {
        writeBuffer[writeCharacters] = tempOutput[B];
        writeCharacters++;
      }
    }
    void DebugSet(uint8_t tempInput) {
      tempInput = constrain(tempInput, 0, 1);
      serialDebugEnabled = tempInput;
    }
    void SendResponse() { //will write the line in the buffer and mark it to be sent ASAP
      writeBuffer[writeCharacters] = '\n'; //add carriage return
      writeCharacters++;
      if (serialResponseSource == 0){ //if source is USB
        Serial.write(writeBuffer, writeCharacters);
        Serial.send_now(); //send all in the buffer ASAP
      }
      else if (serialResponseSource == 1){ //if source is external
        Serial1.write(writeBuffer, writeCharacters);
      }
      writeCharacters = 0;//reset write counter
    }
    void PrintHelp() {
      Serial.print( //I do not plan on sending this over external for now, sorry
        "\n"
        "A command looks as follows: CCCC-PPPPP-RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRe\n"
        "- is a whitespace, to indicate an end of a line\n"
        "C = is a command. Each function has an up to 4 letter command\n"
        "P = is a small number in base 64, optional, can have a sign ('-') before the number\n"
        "R = Raw data in base 64, optional, in inkjet reserved for inkjet data\n"
        "e = end character (carriage return or new line)\n"
        "\n"
        "Based on context, some blocks can be different, but by default all number carrying blocks will be encoded in base 64\n"
        "From 0 to 63: ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/\n"
        "Here, A is 0, and B is 1!\n"
        "\n"
        "List of commands:\n"
        "SBR: Send inkjet to buffer raw (needs small for pos and raw for inkjet data)\n"
        "SBT: Send inkjet to buffer toggle format (needs small for pos and raw for inkjet data)\n"
        "\n"
        "PHT: Preheat printhead (needs small for n pulses)\n"
        "PRM: Prime printhead (needs small for n pulses)\n"
        "THD: Test printhead (no extra input)\n"
        "GTP: Get temperature (no extra input)\n"
        "\n"
        "SPME: Set position mode to encoder (no extra input)\n"
        "SPMV: Set position mode to virtual (no extra input)\n"
        "\n"
        "SEP: Set encoder position (needs small for n position in microns)\n"
        "GEP: Get encoder position (no extra input)\n"
        "SDP: Set DPI (needs small for n DPI)\n"
        "SDN: Set density (needs small for n percentage)\n"
        "SSID: Set printhead side. (0 for both, 1 for odd, 2 for even)\n"
        "\n"
        "VENA: Virtual enable (needs small for state (1 or 0))\n"
        "SVV: Set virtual velocity (needs small for n velocity in mm/s)\n"
        "SVR: Set virtual position reset (needs small for n position in microns)\n"
        "VTRI: Virtual Trigger (no extra input)\n"
        "VSTO: Virtual stop (no extra input)\n"
        "STM0: Set trigger mode trigger 0 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM1: Set trigger mode trigger 1 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM2: Set trigger mode trigger 2 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM3: Set trigger mode trigger 3 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM4: Set trigger mode trigger 4 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM5: Set trigger mode trigger 5 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM6: Set trigger mode trigger 6 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM7: Set trigger mode trigger 7 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STM8: Set trigger mode trigger 8 (0 off, 1 rising, 2 falling, 3 toggle)\n"
        "STR0: Set trigger resistor 0 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR1: Set trigger resistor 1 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR2: Set trigger resistor 2 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR3: Set trigger resistor 3 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR4: Set trigger resistor 4 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR5: Set trigger resistor 5 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR6: Set trigger resistor 6 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR7: Set trigger resistor 7 (0 floating, 2 pulldown, 3 pullup)\n"
        "STR8: Set trigger resistor 8 (0 floating, 2 pulldown, 3 pullup)\n"

        "STPU: Set trigger push notification (needs small for state (1 or 0))\n"
        "\n"
        "BRL: Buffer get read left (no extra input)\n"
        "BRLS: Buffer get read left per side (0 or 1 for side)\n"
        "BWL: Buffer get write left (no extra input)\n"
        "BCL: Buffer clear, remove all data (no extra input)\n"
        "BRES: Buffer reset, only reset read position (no extra input)\n"
        "\n"
        "#COM: Command echo (needs small for state (1 or 0))\n"
        "#NDS: Numberdecode small (needs small to decode)\n"
        "#NDR: Number decode raw (needs small for mode (0 for binary, 1 for numbers) and raw to decode)\n"
        "#NES: Number encode small (needs small in decimal integer, encodes it to a base64 number)\n"
        "#DEB: Debug mode (needs small for debug mode)\n"
        "\n"
        "debug modes:\n"
        "0: all off\n"
        "1: all on\n"
        "2: Serial only\n"
        "\n"
      );
    }
    void RespondTestPrinthead() {

    }
    void RespondOkState(){
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = '#';
      writeBuffer[1] = 'G';
      writeBuffer[2] = 'O';
      writeBuffer[3] = 'K';
      writeBuffer[4] = ':';
      WriteValueToB64(externalSerialClear); //convert ok state to 64 bit
      SendResponse(); //send ok
    }
    void ResetOkState(){ //resets ok state
      externalSerialClear = 1;
    }
    void RespondTemperature(int32_t tempTemp) {
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'T';
      writeBuffer[2] = 'P';
      writeBuffer[3] = ':';
      WriteValueToB64(tempTemp); //convert temperature to 64 bit
      SendResponse(); //send temperature
    }
    void RespondEncoderPos(int32_t tempPos) {
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'E';
      writeBuffer[2] = 'P';
      writeBuffer[3] = ':';
      WriteValueToB64(tempPos); //convert temperature to 64 bit
      SendResponse(); //send temperature
    }
    void RespondEncoderResolution(float tempResolution) {
      int32_t temp_response = long(tempResolution);
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'E';
      writeBuffer[2] = 'R';
      writeBuffer[3] = ':';
      WriteValueToB64(temp_response); //convert temperature to 64 bit
      SendResponse(); //send left
    }
    void RespondBufferReadLeft(int32_t tempLeft) {
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'B';
      writeBuffer[1] = 'R';
      writeBuffer[2] = 'L';
      writeBuffer[3] = ':';
      WriteValueToB64(tempLeft); //convert temperature to 64 bit
      SendResponse(); //send left
    }
    void RespondBufferReadLeftSide(int32_t tempLeft) {
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = 'B';
      writeBuffer[1] = 'R';
      writeBuffer[2] = 'L';
      writeBuffer[3] = 'S';
      writeBuffer[4] = ':';
      WriteValueToB64(tempLeft); //convert temperature to 64 bit
      SendResponse(); //send left
    }
    void RespondBufferWriteLeft(int32_t tempLeft) {
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'B';
      writeBuffer[1] = 'W';
      writeBuffer[2] = 'L';
      writeBuffer[3] = ':';
      WriteValueToB64(tempLeft); //convert temperature to 64 bit
      SendResponse(); //send left
    }
    void RespondTestResults(uint8_t tempMode, uint8_t temp_result[]) {
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'T';
      writeBuffer[1] = 'H';
      writeBuffer[2] = 'D';
      writeBuffer[3] = ':';
      WriteTestArrayToB64(temp_result); //convert 1B array to 64 bit
      SendResponse(); //send left
    }
    void RespondVirtualPosition(int32_t tempPos){
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'V';
      writeBuffer[2] = 'P';
      writeBuffer[3] = ':';
      WriteValueToB64(tempPos); //convert 1B array to 64 bit
      SendResponse(); //send left
    }
    void RespondVirtualVelocity(int32_t tempVelocity){
      writeCharacters = 4; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'V';
      writeBuffer[2] = 'V';
      writeBuffer[3] = ':';
      WriteValueToB64(tempVelocity); //convert 1B array to 64 bit
      SendResponse(); //send left
    }
    void RespondWarning(int32_t tempInput) {
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'W';
      writeBuffer[2] = 'A';
      writeBuffer[3] = 'R';
      writeBuffer[4] = ':';
      WriteValueToB64(tempInput); //convert temperature to 64 bit
      SendResponse(); //send temperature
    }
    void RespondError(int32_t tempInput) {
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'E';
      writeBuffer[2] = 'R';
      writeBuffer[3] = 'R';
      writeBuffer[4] = ':';
      WriteValueToB64(tempInput); //convert temperature to 64 bit
      SendResponse(); //send temperature
    }
    void RespondPulseSplit(uint8_t tempSplit){ //returns the pulse split
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'P';
      writeBuffer[2] = 'S';
      writeBuffer[3] = 'P';
      writeBuffer[4] = ':';
      WriteValueToB64(tempSplit); //convert temperature to 64 bit
      SendResponse(); //send split
    }
    void RespondNozzleCheck(uint8_t tempCheck){
      writeCharacters = 5; //set characters to value after adding response header
      writeBuffer[0] = 'G';
      writeBuffer[1] = 'N';
      writeBuffer[2] = 'C';
      writeBuffer[3] = 'S';
      writeBuffer[4] = ':';
      WriteValueToB64(tempCheck); //convert temperature to 64 bit
      SendResponse(); //send split
    }
    void RespondRaw(uint8_t tempInput[50], uint8_t tempSize, uint8_t tempMode) {
      if (serialResponseSource == 1) {Serial1.print("#NDR:");} else {Serial.print("#NDR:");}
      for (int16_t n = tempSize - 1; n >= 0; n--) {
        if (tempMode == 0) { //binary mode
          for (uint8_t b = 0; b < 6; b++) {
            if (serialResponseSource == 1) {Serial1.print(bitRead(tempInput[n], b));} else {Serial.print(bitRead(tempInput[n], b));}
          }
        }
        else { //number mode
          if (serialResponseSource == 1) {Serial1.print(tempInput[n]); Serial.print(" ");} else {Serial.print(tempInput[n]); Serial.print(" ");}
        }
      }
      if (serialResponseSource == 1) {Serial1.println("");} else {Serial.println("");}
    }
    void RespondEncodeSmall(int32_t tempInput) { //takes a numeric input and encodes it back to base 64. Used for manual typing help, can only do up to 5 character numbers
      uint8_t tempEncode;
      uint32_t tempResponseValue = 0;
      uint32_t tempDecimal = 1;
      char responseArray[5];
      for (uint8_t r  = 0; r < 5; r++) { //reset all the numbers in the array
        responseArray[r] = 'A';
      }
      for (uint8_t d = 0; d < 25; d += 6) { //look at the value in 6 bit chunks
        tempEncode = (tempInput >> d) & 0B00111111;
        if (tempEncode >= 52 && tempEncode <= 62) { //only act if it is a number
          tempEncode -= 52; //change to integer numbers
        }
        else {
          tempEncode = 0; //change to 0
        }
        //Serial.print(tempEncode); Serial.print(", ");
        tempResponseValue += tempEncode * tempDecimal; //add to counting value
        tempDecimal *= 10; //move one order of magnitude
      }
      //Serial.print(": "); Serial.println(tempResponseValue);
      if (serialResponseSource == 1) {Serial1.print("#NES: ");}  else {Serial.print("#NES: ");} //Serial.print(tempResponseValue); Serial.print(", Base64: ");

      //decode value to base 64 small value
      for (uint8_t d = 0; d < 31; d += 6) { //loop through the entire number
        tempResponseValue = tempResponseValue >> d; //go to next 6 bits
        uint8_t tempBase = tempResponseValue & 63; //isolate 6 lsb
        responseArray[d / 6] = ToB64Lookup(tempBase); //convert number to base64 number
        //Serial.print(tempBase); Serial.print(", ");  Serial.println(ToB64Lookup(tempBase));
      }
      //Serial.println(responseArray);

      uint8_t tempNumberFound = 0; //the number to indicate that something else than a 0 ('A') was found
      //reverse and decode
      for (uint8_t r = 0; r < 6; r++) {
        if (responseArray[5 - r] != 'A') { //something else than 0 found (used to get rid of the useless zeroes, like 0025 becomes 25)
          tempNumberFound = 1; //now real numbers
        }
        if (tempNumberFound == 1) { //if real numbers are found
          if (serialResponseSource == 1) {Serial1.print(responseArray[5 - r]);} else {Serial.print(responseArray[5 - r]);} //print the character
        }
      }
      if (serialResponseSource == 1) {Serial1.println("");} else {Serial.println("");}
    }

    void SetResponseSource(uint8_t tempSource){
      tempSource = constrain(tempSource, 0, 1);
      serialResponseSource = tempSource;
    }
};

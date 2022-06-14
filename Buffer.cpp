/*
   The buffer is a class than handles all data regarding the inkjet. It takes burst and position information and stores it in a FiFo buffer. The buffer is what takes the most memory on the microcontroller

   Todo:
   - Do not meticulously calculate buffer read and write left, but simply hold a variable that keeps this data. It will not magically add 15 lines without the functions knowing about it.
*/

#include "Arduino.h"
#include "Buffer.h" //<*whispers: "there is actually nothing in there"

#define BUFFER_SIZE 4003 //the number of 48 byte blocks the buffer consists of (44 for inkjet, 4 for coordinate) (+3 for off by ones)
#define PRIMITIVE_OVERLAY_EVEN 7384 //B0001110011011000 
#define PRIMITIVE_OVERLAY_ODD 8999  //B0010001100100111

#define BUFFER_MODE_CLEARING 0 //where the buffer line is cleared after use
#define BUFFER_MODE_STATIC 1 //where the buffer line is retained after use
#define BUFFER_MODE_LOOPING 2 //where the buffer resets to position 0 after it reaches the end

#define BUFFER_PRINT_MODE_ALL 0 //both odd and even active
#define BUFFER_PRINT_MODE_ODD 1 //Only odd active
#define BUFFER_PRINT_MODE_EVEN 2 //Only even active

class Buffer {
    //class member variables
    uint16_t burstBuffer[BUFFER_SIZE][22]; //burst data
    int32_t positionBuffer[BUFFER_SIZE]; //position data
    int32_t readPosition[2], writePosition; //read and write positions for odd and even
    uint16_t primitiveOverlay[2]; //0 or 1 for even or odd
    //int32_t readLeft[2], writeLeft; //future additions
    uint8_t sideActive[2] = {0, 0}; //turns the overlay on or off for a side
    uint8_t bufferMode = BUFFER_MODE_CLEARING;
    uint8_t bufferPrintMode = BUFFER_PRINT_MODE_ALL;
    uint8_t bufferLoopCounter = 0; //contains how often the buffer has looped. Is used to activate other functions

  public:
    Buffer() {
      //make odd and even overlays, used for  the 2 positions in the buffer
      primitiveOverlay[0] = PRIMITIVE_OVERLAY_ODD;
      primitiveOverlay[1] = PRIMITIVE_OVERLAY_EVEN;

      ClearAll(); //reset the buffer
    }
    void SetMode(uint8_t tempMode) { //sets the mode the buffer operates at
      if (tempMode == BUFFER_MODE_CLEARING || tempMode == BUFFER_MODE_STATIC || tempMode == BUFFER_MODE_LOOPING) { //verify if the requested value is valid
        bufferMode = tempMode;
      }
    }
    uint8_t GetMode() { //get the current buffer mode
      return bufferMode;
    }
    int32_t Next(uint8_t tempSide) { //if possible, adds one to the read position and returns lines left. Data needs to be fetched using other functions
      tempSide &= 1; //constrain side
      
      if (ReadLeftSide(tempSide) > 0) { //if there is something left to read
        readPosition[tempSide] ++; //add one to the read position
        readPosition[tempSide] = readPosition[tempSide] % BUFFER_SIZE; //overflow protection
        //Serial.print("Buffer next side: "); Serial.print(tempSide);  Serial.print(", left: "); Serial.println(ReadLeftSide(tempSide));

        if (bufferMode == BUFFER_MODE_LOOPING ) { //if the mode is looping
          if (ReadLeft() == 0) { //if there is nothing left to read on either side
            Reset(); //reset the buffer
            //Serial.println("Loop: Resetting buffer");
            bufferLoopCounter++;
          }
        }
        return ReadLeftSide(tempSide);
      }
      return -1; //return back -1 if it is not possible
    }
    int32_t LookAheadPosition(uint8_t tempSide) { //takes the position in the next buffer position
      tempSide &= 1; //constrain side
      int32_t tempReadPos;
      if (ReadLeftSide(tempSide) > 0) { //if there is something left ot read
        tempReadPos = readPosition[tempSide]; //make a temporary read position and make it go to the next pos
        tempReadPos ++;
        tempReadPos = tempReadPos % BUFFER_SIZE; //overflow protection
        return positionBuffer[tempReadPos];
      }
      return -1; //return -1 for error
    }
    int32_t Add(int32_t temp_position, uint16_t tempInput[22]) { //adds a coordinate (int32_t and a burst to the buffer, returns space left if successful, -1 if failed
      int32_t temp_left = WriteLeft();
      if (temp_left > 0) { //if there is space left in the buffer
        positionBuffer[writePosition] = temp_position; //add position
        //Serial.print("Add to buffer: "); Serial.print(temp_position); Serial.print(": ");
        for (uint8_t a = 0; a < 22; a++) { //add burst
          burstBuffer[writePosition][a] = tempInput[a];
          //Serial.print(tempInput[a]); Serial.print(", ");
        }
        //Serial.println("");
        writePosition++; //add one to write position
        writePosition = writePosition % BUFFER_SIZE; //overflow protection
        temp_left--;
        //Serial.print("Buffer write left: "); Serial.println(temp_left);
        //Serial.print("Buffer read left: "); Serial.println(ReadLeft());
        return temp_left;
      }
      return -1; //return a -1 if this failed
    }
    int32_t ReadLeft() { //returns the number of filled buffer read slots. Automatically returns the largest value
      //calculate read lines left on pos 0 by subtracting read position and adding write position, and then take the modulo of the buffer size to protect from overflows
      int32_t tempCalc1, tempCalc2;
      tempCalc1 = BUFFER_SIZE;
      tempCalc1 += writePosition;
      tempCalc2 = tempCalc1;
      tempCalc1 -= readPosition[0];
      tempCalc2 -= readPosition[1];
      tempCalc1 = tempCalc1 % BUFFER_SIZE; //constrain to buffer size
      tempCalc2 = tempCalc2 % BUFFER_SIZE;
      tempCalc1 -= 1; //subtract one because read can never be equal to write
      tempCalc2 -= 1;
      if (tempCalc1 > tempCalc2) { //return smallest value
        return tempCalc1;
      }
      else {
        return tempCalc2;
      }
    }
    int32_t ReadLeftSide(uint8_t tempSide) {//returns the number of lines left to read for a given size
      tempSide &= 1; //constrain side
      int32_t tempCalc;
      tempCalc = BUFFER_SIZE;
      tempCalc += writePosition;
      tempCalc -= readPosition[tempSide];
      tempCalc = tempCalc % BUFFER_SIZE; //constrain to buffer size
      tempCalc -= 1; //subtract one because read can never be equal to write
      return tempCalc;
    }
    int32_t WriteLeft() { //returns the number of free buffer write slots. Automatically returns the smallest value
      //calculate write lines left on pos 0 by subtracting write position and adding read position, and then take the modulo of the buffer size to protect from overflows
      int32_t tempCalc1, tempCalc2;
      if (bufferMode == BUFFER_MODE_CLEARING) { //if the buffer is cleared after a line is printed, calculate rolling value
        tempCalc1 = BUFFER_SIZE;
        tempCalc1 -= writePosition;
        tempCalc2 = tempCalc1;
        tempCalc1 += readPosition[0];
        tempCalc2 += readPosition[1];
        tempCalc1 = tempCalc1 % BUFFER_SIZE; //constrain to buffer size
        tempCalc2 = tempCalc2 % BUFFER_SIZE;
        tempCalc1 -= 2; //subtract to create some protection
        tempCalc2 -= 2;
        if (tempCalc1 < tempCalc2) { //return smallest value
          return tempCalc1;
        }
        else {
          return tempCalc2;
        }
      }
      if (bufferMode == BUFFER_MODE_STATIC  || bufferMode == BUFFER_MODE_LOOPING) { //if the line is retained after it is printed
        tempCalc1 = BUFFER_SIZE; //start with the buffer size
        tempCalc1 -= writePosition; //subtract where we are currently writing
        tempCalc1 -= 2; //subtract to create some protection
        return tempCalc1;
      }
      return 0; //if you got here, something went horribly wrong
    }
    void ClearAll() { //resets the read and write positions in the buffer
      for (uint16_t b = 0; b < BUFFER_SIZE; b++) {
        for (uint8_t a = 0; a < 22; a++) {
          burstBuffer[b][a] = 0;
        }
        positionBuffer[b] = 0;
      }
      readPosition[0] = 0;
      readPosition[1] = 0;
      writePosition = 1;
    }
    void Reset() { //only resets the read position to the first array position
      readPosition[0] = 0;
      readPosition[1] = 0;
    }
    uint16_t GetPulse(uint8_t temp_address) {
      temp_address = constrain(temp_address, 0, 21);
      uint16_t tempPulse = 0; //make the return pulse
      tempPulse = tempPulse & PRIMITIVE_OVERLAY_EVEN & burstBuffer[readPosition[0]][temp_address];
      tempPulse = tempPulse & PRIMITIVE_OVERLAY_ODD & burstBuffer[readPosition[1]][temp_address];
      return tempPulse;
    }
    void SetActive(uint8_t tempSide, uint8_t tempState) { //set which side (odd or even) is on or off
      tempSide = constrain(tempSide, 0, 1);
      tempState = constrain(tempState, 0, 1);
      sideActive[tempSide] = tempState;
    }
    void SetPrintMode(uint8_t tempMode){ //set what mode the head prints at. 0 is both on, 1 is odd only, 2 is even only
      //this function looks a lot like SetActive, but is meant for more permanent settings
      if (tempMode == BUFFER_PRINT_MODE_ALL || tempMode == BUFFER_PRINT_MODE_ODD || tempMode == BUFFER_PRINT_MODE_EVEN){
        bufferPrintMode = tempMode;
      }
      
    }
    uint16_t *GetBurst(uint16_t tempBurst[22]) { //gets the complete current burst, based on the 2 positions from buffer and writes it to the input uint16_t[22] array
      uint16_t tempOdd, tempEven; //temporary burst values
      for (uint8_t a = 0; a < 22; a++) { //walk through the entire pulse
        tempBurst[a] = 0; //reset value
        if (sideActive[0] == 1 && bufferPrintMode != BUFFER_PRINT_MODE_EVEN) { //if odd side is active
          tempOdd = PRIMITIVE_OVERLAY_ODD & burstBuffer[readPosition[0]][a]; //odd side
        }
        else {
          tempOdd = 0;
        }
        if (sideActive[1] == 1 && bufferPrintMode != BUFFER_PRINT_MODE_ODD) { //if even side is active
          tempEven = PRIMITIVE_OVERLAY_EVEN & burstBuffer[readPosition[1]][a]; //even side
        }
        else {
          tempEven = 0;
        }
        tempBurst[a] = tempBurst[a] | tempEven; //add even
        tempBurst[a] = tempBurst[a] | tempOdd; //add odd
      }
      return tempBurst;
    }
    int32_t GetPosition(uint8_t tempSide) {
      tempSide &= 1; //constrain side
      return positionBuffer[readPosition[tempSide]];
    }
    uint8_t GetLoopCounter(){
      return bufferLoopCounter;
    }

  private:
};

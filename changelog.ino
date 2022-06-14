//all the changes that have happend since V4.01.05

//V4.01.06: 
//Added while high and while low trigger support
//changed behavior in looping mode to auto reset position when in while high and while low mode
//trigger in UpdateTrigger was limited to 1000 cycles per second to aid debouncing
//Made the while loop position reset only trigger while the trigger is 2 (looping). Added a variable to track active while state.
//Made the buffer also reset on trigger when a buffer is in looping mode
//changed PositionGetVelocity() behaviour to send a 0 in virtual velocity when there is an overflow (virtual movement stopped)
//made a separate buffer reset function that also resets the first line to prevent the buffer from gaining a failure to start
//added a clear buffer function to make it consistent with the addition of a reset buffer function
//added BufferReadLeftSide to serial (BRLS)
//Changed buffer ReadLeft() to always return LARGEST number, not smallest. Only when readleft is 0 on both sides should any behavior treat the buffer as empty
//Change in buffer Next() to check for looping coditions After the next has been called, looping immediately after the next line has been called

//V4.01.07:
//EEPROM functions EepromCheckSaved() and EepromSetSaved() were added to verify if HP45 standalone has saved in EEPROM or not

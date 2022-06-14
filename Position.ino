/*
   The position tab (not class, got issues) handles the coordinates of the printhead using a linear optical encoder strip.
   Virtual velocity with various triggers is also an option.

   Note that if virtual overflows all positions and velocities remain at their last value. While not ideal, it is worse than resetting all to 0

   Todo:
   -Change test conditions to also include a timebase (of say 100us or 1ms) to make 0 a posible velocity.
   Right now the only recalculation is when there was a change, and this only happens with movement
   -Velocity output seems fairly eratic (varying +/-5mm/s @100mm/s with a simple serial test)
   See if there is mathematical reasons for this eratic time behavior, else add smoothing
   (maybe eratic output is due to frequent testing combined with low step size (position is in microns, speed in mm/s))

   o Test functions
   -return values based on mode
   -specific encoder and virtual returns
   -calculate virtual mode
   -virtual mode auto stop

   o to be added functions
   -set position mode
   -While high mode
   -While low mode

   -push trigger event message
*/

#include "Arduino.h"

float encoderResolution = 600.0; //406.0; //the lines per inch (150) x4 for encoder type (quadrature) as a float
//Test encoder, 600 pulses per roatation, 150mm circ. wheel. 5.9055" per rotation. 2400/5.9055 = 406 pulses per inch.
#define ROW_GAP 4050 //the distance in microns between odd and even row in long

//#define ENCODER_DO_NOT_USE_INTERRUPTS //interrupts risk firing while printhead is triggering, off for now
#include "Encoder.h"

#define POSITION_ENCODER_PINS 2
#define POSITION_ENCODER_PIN1 33
#define POSITION_ENCODER_PIN2 34
uint8_t positionEncoderPin[2] = {POSITION_ENCODER_PIN1, POSITION_ENCODER_PIN2};

#define POSITION_STEP_COUNT 50 //how many steps are used to calculate velocity
#define POSITION_STEP_TIMEOUT 100000 //how many microseconds no step has to be seen to set the velocity to 0

Encoder positionEncoder(34, 33); //make an encoder instance
uint8_t positionMode = 0; //what mode is active, 0 is encoder mode, 1 is virtual mode

//encoder variables
int32_t positionEncoderRaw, positionEncoderHistory; //the current and history pulse position
int32_t positionBaseEncoderMicrons, positionBaseEncoderMicronsHistory; //the reference micron position and history
int32_t positionRowEncoderMicrons[2]; //the micron positions of odd and even
int32_t positionEncoderVelocity; //the current velocity of the printhead (mm/s)
int32_t positionEncoderMicronVelocity; //the current micron velocity
uint32_t positionLastStepTime, positionLastCalcTime; //the value that holds the time of the last encoder pulse in microseconds and when the last velocity measurement was done
int32_t positionEncoderVelocityLastMicrons; //where the last recalculation of velocity was
int8_t positionEncoderDirection; //1 or -1, tells the direction of the printhead
uint16_t positionEncoderVelocityUpdateCounter; //counts up every encoder pulse, when reached StepCount, converts to velocity

int32_t positionVirtualVelocity = 0; //what virtual mm/s speed the printhead is moving at
int32_t positionVirtualMicronVelocity = 0; //what virtual um/s speed the printhead is moving at
int32_t positionBaseVirtualMicrons; //the reference micron position
int32_t positionRowVirtualMicrons[2]; //the micron positions of odd and even
int32_t positionVirtualResetPosition = 0; //where the virtual movement starts upon a reset (defaults to 0)
uint8_t positionVirtualEnabled = 0; //if the virtual position is enabled
uint32_t positionVirtualStartTime; //when the virtual move started
uint8_t positionVirtualOverflow = 1; //whether the position has overflowed (and then automatically stops) (start overflowed)
int32_t positionVirtualStartPosition; //where the virtual has started
int8_t positionVirtualDirection; //1 or -1, tells the direction of the printhead
uint32_t positionVirtualMaxTime = 120000000; //maximum time in microseconds the virtual speed runs


#define ENCODER_MODE 0
#define VIRTUAL_MODE 1


void PositionUpdate() {
  //calculate encoder position
  positionEncoderRaw = positionEncoder.read();
  if (positionEncoderHistory != positionEncoderRaw) { //if history and raw do not match, recalculate positions
    uint32_t temp_time = micros(); //write down time of change

    //get micron position
    float temp_calc = float(positionEncoderRaw);
    temp_calc *= 25400.0; //by microns in an inch
    temp_calc /= float(encoderResolution); //divide by lines per inch
    positionBaseEncoderMicrons = long(temp_calc);

    //get odd and even micron position
    temp_calc = float(ROW_GAP) / 2.0; //divide by 2 to give the offset from 0
    temp_calc += float(positionBaseEncoderMicrons); //add to current position
    positionRowEncoderMicrons[1] = long(temp_calc); //odd (1) is on the positive side, add row gap
    positionRowEncoderMicrons[0] = positionRowEncoderMicrons[1] - ROW_GAP; //even (0) is on the negative side, subtract row gap

    //calculate velocity (new pos - old pos)/ time it took is microns per microsecond
    positionEncoderVelocityUpdateCounter++; //add one encoder pulse to counter
    if (positionEncoderVelocityUpdateCounter >= POSITION_STEP_COUNT) { //if number of pulses is reached
      positionEncoderVelocityUpdateCounter = 0;

      temp_calc = float(positionBaseEncoderMicrons);
      temp_calc -= float(positionBaseEncoderMicronsHistory); //get delta distance
      float temp_calc2 = float(temp_time);
      temp_calc2 -= float(positionLastCalcTime);
      temp_calc = temp_calc / temp_calc2; //get microns per microsecond (is mm/millisecond because metric)
      positionEncoderMicronVelocity = long(temp_calc); //set microns per second
      temp_calc *= 1000.0; //multiply by 1000 to get millimeters per second
      positionEncoderVelocity = long(temp_calc); //set millimeters per second
      positionEncoderDirection = constrain(positionEncoderVelocity, -1, 1); //set direction
      positionBaseEncoderMicronsHistory = positionBaseEncoderMicrons; //set new micron history
      positionLastCalcTime = micros(); //reset last velocity calculation time
    }

    positionLastStepTime = temp_time; //set new time
    positionEncoderHistory = positionEncoderRaw; //set new history
  }

  if (micros() - positionLastCalcTime > POSITION_STEP_TIMEOUT) { //velocity timeout conditions
    positionEncoderVelocity = 0;
  }

  //look for trigger requirements------------------------------------- <----------
  uint8_t temp_triggered = 0; //if triggered variable
  if (temp_triggered == 1) { //trigger if required
    PositionVirtualTrigger();
  }

  //calculate virtual position
  if (positionVirtualEnabled == 1 && positionVirtualOverflow == 0) { //if virtual mode and no overflow
    positionVirtualDirection = constrain(positionVirtualVelocity, -1, 1); //calculate direction

    //calculate position based on start time and velocity
    uint32_t temp_time_passed; //how much time has passed since start
    uint32_t temp_current_time = micros(); //write millis to variable to prevent error if it overflows between check and calculation
    //check for potential overflow conditions (micros timer overflows every 71 minutes)
    if (positionVirtualStartTime > temp_current_time) { //if the millis() has overflown
      //calculate time passed current time minus start
      temp_time_passed = 4294967295 - positionVirtualStartTime; //get max time - start time for first component of the current time (the time until the overflow)
      temp_time_passed += micros(); //add the time passed after the overflow
    }
    else { //conventional time check
      //calculate time passed current time minus start
      temp_time_passed = temp_current_time - positionVirtualStartTime;
    }

    //calculate distance moved
    float temp_fcalc = float(temp_time_passed); //take time
    temp_fcalc /= 1000000.0; //convert to whole seconds
    temp_fcalc *= float(positionVirtualMicronVelocity); //multiply by microns per second

    positionBaseVirtualMicrons = long(temp_fcalc); //make new position
    positionBaseVirtualMicrons += positionVirtualStartPosition; //add starting position

    //calculate row positions
    int32_t temp_calc = ROW_GAP / 2;
    positionRowVirtualMicrons[1] = positionBaseVirtualMicrons + temp_calc; //odd (1)is on positive side of the row gap
    positionRowVirtualMicrons[0] = positionBaseVirtualMicrons - temp_calc; //even (0)is on negative side of the row gap


    //check if the timer has overflown and virtual position has stopped
    if (temp_time_passed > positionVirtualMaxTime) {
      positionVirtualOverflow = 1;
      //Serial.println("Position overflow");
    }
  }
}

void PositionSetModeEncoder() { //set position to encoder mode
  positionMode = ENCODER_MODE;
}

void PositionSetModeVirtual() { //set position to virtual mode
  positionMode = VIRTUAL_MODE;
}

//overal mode
int32_t PositionGetBasePositionMicrons() { //returns the position of the base in microns
  if (positionMode == ENCODER_MODE) { //if the position is in encoder mode
    return positionBaseEncoderMicrons;
  }
  else if (positionMode == VIRTUAL_MODE) { //if the position is in virtual mode
    return positionBaseVirtualMicrons;
  }
  return 0;
}

int32_t PositionGetRowPositionMicrons(uint8_t temp_side) { //returns the position of the given side
  temp_side = constrain(temp_side, 0, 1);
  if (positionMode == ENCODER_MODE) { //if the position is in encoder mode
    return positionRowEncoderMicrons[temp_side];
  }
  else if (positionMode == VIRTUAL_MODE) { //if the position is in virtual mode
    return positionRowVirtualMicrons[temp_side];
  }
  return 0;
}

int32_t PositionGetVelocity() { //returns the current velocity in millimeters per second. Virtual return 0 if it is stopped
  if (positionMode == ENCODER_MODE) { //if the position is in encoder mode
    return positionEncoderVelocity;
  }
  else if (positionMode == VIRTUAL_MODE) { //if position is in virtual mode
    if (positionVirtualOverflow == 0){
      return positionVirtualVelocity;
    }
    else {
      return 0;
    }
  }
  return 0;
}

int8_t PositionGetDirection() { //returns the current direction
  if (positionMode == ENCODER_MODE) { //if the position is in encoder mode
    return positionEncoderDirection;
  }
  else if (positionMode == VIRTUAL_MODE) { //if position is in virtual mode
      return positionVirtualDirection;
  }
  return 0;
}

//encoder mode
int32_t PositionGetBasepositionEncoderRaw() { //returns the position of the base in pulses
  PositionUpdate();
  return positionEncoderRaw;
}

void PositionSetBaseEncoderPositionMicrons(int32_t temp_position) { //sets the new encoder pulse position
  int32_t temp_raw = map(temp_position, 0, 25400, 0, long(encoderResolution)); //recalculate encoder position from microns to pulses
  positionEncoder.write(temp_raw); //set new position
}

int32_t PositionGetBaseEncoderPositionMicrons() {
  return positionBaseEncoderMicrons;
}

int32_t PositionGetRowEncoderPositionMicrons(uint8_t temp_side) { //returns the position of the given side
  temp_side = constrain(temp_side, 0, 1);
  return positionRowEncoderMicrons[temp_side];
}


int32_t PositionGetEncoderVelocity() { //returns the current velocity in millimeters per second
  return positionEncoderVelocity;
}

int8_t PositionGetEncoderDirection() { //returns the current direction
  return positionEncoderDirection;
}

void PositionSetEncoderResolution(float temp_resolution) {
  encoderResolution = temp_resolution;
}

float PositionGetEncoderResolution() {
  return encoderResolution;
}

//virtual mode
int32_t PositionGetVirtualVelocity() { //returns the current velocity in millimeters per second
  return positionVirtualVelocity;
}

int8_t PositionGetVirtualDirection() { //returns the current direction
  return positionVirtualDirection;
}

int32_t PositionGetVirtualPosition() { //returns the virtual position
  return positionBaseVirtualMicrons;
}

void PositionSetVirtualVelocity (int32_t temp_velocity) { //set the velocity in millimeters per second
  //set velocity in mm/s
  positionVirtualVelocity = temp_velocity;
  positionVirtualMicronVelocity = positionVirtualVelocity * 1000;
  PositionResetVirtualBasevariables();
}

void PositionSetVirtualVelocityMicrons (int32_t temp_velocity) { //set the velocity in microns per second
  //set velocity in um/s
  positionVirtualMicronVelocity = temp_velocity;
  positionVirtualVelocity = positionVirtualMicronVelocity / 1000;
  PositionResetVirtualBasevariables();
}

void PositionVirtualEnable(uint8_t temp_mode) { //enable or disable virtual mode
  //set the virtual mode to temp_mode
  temp_mode = constrain(temp_mode, 0, 1);
  positionVirtualEnabled = temp_mode;
  PositionResetVirtualBasevariables();
}

void PositionResetVirtualBasevariables() { //internal function used to reset start time and position upon update
  positionVirtualStartTime = micros(); //set start time to current time
  positionVirtualStartPosition = positionBaseVirtualMicrons; //reset start to current position
}

void PositionVirtualSetStart(int32_t temp_input) { //sets where virtual moves upon a reset
  positionVirtualResetPosition = temp_input;
}

void PositionVirtualTrigger() { //resets the position to reset position and restarts all variables
  if (positionMode == 1) { //if in virtual mode
    positionVirtualOverflow = 0; //reset any overflow
    positionVirtualStartTime = micros(); //set start time to current time
    positionVirtualStartPosition = positionVirtualResetPosition; //set start position
  }
  else {
    PositionSetBaseEncoderPositionMicrons(positionVirtualResetPosition);
  }
}

void PositionVirtualStop() { //stop the movement on the virtual position
  positionVirtualOverflow = 1; //set overflow
}

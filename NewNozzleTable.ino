
//the generate tables function is used to generate, well, new tables. It takes the raw printhead tables, a conversion table that takes the raw values and alters them
//and the prints the new table over serial. All you need to do is correct the conversion table, run the function, and copy, paste the new table in the place of the old.
void GenerateNewRawTables() {
  //raw printhead locations
  const uint8_t tempRawNozzleTableAddress[300] = //the table that returns the address for a given nozzle (do not alter)
  {
    0, 8, 15, 1, 8, 16, 1, 9, 16, 2,
    9, 17, 2, 10, 17, 3, 10, 18, 3, 11,
    18, 4, 11, 19, 4, 12, 19, 5, 12, 20,
    5, 13, 20, 6, 13, 21, 16, 14, 21, 7,
    14, 0, 7, 12, 0, 8, 15, 1, 8, 16,
    1, 9, 16, 2, 9, 17, 2, 10, 17, 3,
    10, 18, 3, 11, 18, 4, 11, 19, 4, 12,
    19, 5, 12, 20, 5, 13, 20, 6, 13, 21,
    6, 14, 21, 7, 14, 0, 7, 15, 0, 8,
    15, 1, 8, 16, 1, 9, 16, 2, 9, 17,
    2, 10, 17, 3, 10, 18, 3, 11, 18, 4,
    11, 19, 4, 12, 19, 5, 12, 20, 5, 13,
    20, 6, 13, 21, 6, 14, 21, 7, 14, 0,
    7, 15, 0, 8, 15, 1, 8, 16, 1, 9,
    16, 2, 9, 17, 2, 10, 17, 3, 10, 18,
    3, 11, 18, 4, 11, 19, 4, 12, 19, 5,
    12, 20, 5, 13, 20, 6, 13, 21, 6, 14,
    0, 7, 15, 0, 7, 15, 0, 8, 15, 1,
    8, 16, 1, 9, 18, 2, 9, 17, 2, 10,
    17, 3, 10, 18, 3, 11, 18, 4, 11, 19,
    4, 12, 19, 5, 12, 20, 5, 13, 20, 6,
    13, 21, 6, 14, 21, 7, 14, 0, 7, 15,
    0, 8, 15, 1, 8, 16, 1, 9, 16, 2,
    9, 17, 2, 10, 17, 3, 10, 18, 3, 11,
    18, 4, 11, 21, 4, 12, 19, 5, 12, 20,
    5, 13, 20, 6, 13, 21, 6, 14, 21, 7,
    14, 0, 7, 15, 0, 8, 15, 1, 8, 16,
    1, 9, 16, 2, 9, 17, 2, 10, 17, 3,
    10, 18, 3, 11, 18, 4, 11, 19, 4, 12,
    19, 5, 12, 20, 5, 13, 20, 6, 13, 20
  };

  const uint8_t tempRawNozzleTablePrimitive[300] = //the table that returns the primitive for a given nozzle (do not alter)
  {
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
    2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
    2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
    2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
    2, 3, 2, 3, 4, 5, 4, 5, 4, 5,
    4, 5, 4, 5, 4, 5, 4, 5, 4, 5,
    4, 5, 4, 5, 4, 5, 4, 5, 4, 5,
    4, 5, 4, 5, 4, 5, 4, 5, 4, 5,
    4, 5, 4, 5, 4, 5, 4, 5, 6, 7,
    6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
    6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
    6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
    6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
    6, 7, 8, 9, 8, 9, 8, 9, 8, 9,
    8, 9, 8, 9, 8, 9, 8, 9, 8, 9,
    8, 9, 8, 9, 8, 9, 8, 9, 8, 9,
    8, 9, 8, 9, 8, 9, 8, 9, 8, 9,
    8, 9, 8, 9, 8, 9, 10, 11, 10, 11,
    10, 11, 10, 11, 10, 11, 10, 11, 10, 11,
    10, 11, 10, 11, 10, 11, 10, 11, 10, 11,
    10, 11, 10, 11, 10, 11, 10, 11, 10, 11,
    10, 11, 10, 11, 10, 11, 10, 11, 10, 11,
    12, 13, 12, 13, 12, 13, 12, 13, 12, 13,
    12, 13, 12, 13, 12, 13, 12, 13, 12, 13,
    12, 13, 12, 13, 12, 13, 12, 13, 12, 13,
    12, 13, 12, 13, 12, 13, 12, 13, 12, 13
  };

  /*
    0 = 15
    1 = 22
    2 = 23
    3 = 9
    4 = 10
    5 = 13
    6 = 11
    7 = 12
    8 = 2
    9 = 14
    10= 7
    11= 8
    12= 6
    13= 20

  */

  //conversion values --------------------------------------------------------------------------------------------------------------------------------
  //these are alteres. The given number is the controller pin the given printhead pin is on, in order from P1 to P14, and A1 to A22
  //static uint8_t primtivePins[14] = {15, 22, 23, 9, 10, 13, 11, 12, 2, 14, 7, 8, 6, 20}; //(C0-C7, D0-D5) the array for the primitive pins
  //                                   0,  1,  2,  3, 4,  5,  6,  7,  8, 9,  10,11,12,13

  //odd or even                         o  e  o   e  o   e  o   e  o  e  o  e  o  e
  int8_t tempPrimitiveConversion[14] = {3, 1, 11, 2, 10, 8, 12, 5, 7, 9, 6, 0, 4, 13};
  uint8_t tempAddressConversion[22] = {6, 1, 2, 5, 11, 4, 15, 0, 12, 7, 19, 3, 17, 8, 21, 9, 16, 13, 18, 14, 20, 10};

  //primitive
  Serial.println("New Primitive Array");
  for (uint16_t n = 0; n < 300; n++) { //walk through all values
    uint8_t tempVal = tempRawNozzleTablePrimitive[n]; //take the current value
    tempVal = tempPrimitiveConversion[tempVal]; //convert it using the conversion table
    Serial.print(tempVal); //print new value
    if (n != 299) { //don't print the last comma
      Serial.print(","); //comma
    }
    if (n % 10 == 9) { //new line after 10 positions
      Serial.println(""); //<-- new line
    }
  } //<- curly bracket
  // <- Blank space :)
  //address
  Serial.println("");
  Serial.println("New Address Array");
  for (uint16_t n = 0; n < 300; n++) { //walk through all values
    uint8_t tempVal = tempRawNozzleTableAddress[n]; //take the current value
    tempVal = tempAddressConversion[tempVal]; //convert it using the conversion table
    Serial.print(tempVal); //print new value
    if (n != 299) { //don't print the last comma
      Serial.print(","); //comma
    }
    if (n % 10 == 9) { //new line after 10 positions
      Serial.println(""); //<-- new line
    }
  }

  Serial.println("");
  Serial.println("Buffer odd/even overlays");
  uint32_t tempOddArray = 0;
  uint32_t tempEvenArray = 0;
  for (uint8_t C = 0; C < 14; C++) { //loop through array for each primitive
    for (uint8_t P = 0; P < 14; P++) {
      if (tempPrimitiveConversion[P] == C) { //if the number in the array matches the currently next number
        if (P % 2 == 1) { //if odd
          bitWrite(tempOddArray, C, 1);
        }
        if (P % 2 == 0) { //if even
          bitWrite(tempEvenArray, C, 1);
        }
      }
    }
  }
  Serial.print("Even: "); Serial.print(tempEvenArray); Serial.print(" //B");
  for (int8_t B = 15; B >= 0; B--) {
    Serial.print(bitRead(tempEvenArray, B));
  }
  Serial.println("");

  Serial.print("Odd:  ");  Serial.print(tempOddArray); Serial.print(" //B");
  for (int8_t B = 15; B >= 0; B--) {
    Serial.print(bitRead(tempOddArray, B));
  }
  Serial.println("");
}

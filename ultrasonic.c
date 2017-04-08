/*******************************************************************************
    Ultrasonic Sensor Source File
  
  Company:
    Embedded System Spring 2017 Team 9
  
  File Name:
    ultrasonic.c

  Summary:
    Implementation of functions to obtain ultrasonic sensor values.

  Description:
    This file implements the prototypes of functions declared in ultrasonic.h 
 *  and obtains the ultrasonic values in millimeter and centimeter.
 *******************************************************************************/

#include "app.h"
#include "ultrasonic.h"

/*******************************************************************************
  Function:
    unsigned int getUltrasonicValue()

  Remarks:
    See prototype in ultrasonic.h.
 */

unsigned int getUltrasonicValue() {
    int value;
    if(appData.potValue >= 0) {
        value = appData.potValue * 10;
    }
    return value;
}

/*******************************************************************************
  Function:
    unsigned int getUltrasonicValueInCm(unsigned int value)

  Remarks:
    See prototype in ultrasonic.h.
 */

unsigned int getUltrasonicValueInCm(unsigned int ultrasonicValueInMm) {
    return (ultrasonicValueInMm/10);
}
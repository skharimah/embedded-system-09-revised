/*******************************************************************************
    Ultrasonic Sensor Header File
  
  Company:
    Embedded System Spring 2017 Team 9
  
  File Name:
    ultrasonic.h

  Summary:
    Prototypes of functions to obtain ultrasonic sensor values.

  Description:
    This file declare the prototypes of functions implemented in ultrasonic.c 
 *  and obtains the ultrasonic values in millimeter and centimeter.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

char ultrasonicValBuf[4]; /* The maximum ultrasonic sensor value is 1023*5 = 5115 */
unsigned int ultrasonicValue, ultrasonicValueInCm; 

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    unsigned int getUltrasonicValue()
  Summary:
    Get ultrasonic sensor value in millimeter.
  Description:
    This function multiplies the values from ADC (bytes) by 5 to get the ultrasonic value in millimeter.
  Parameters:
    None.
  Returns:
    Return the ultrasonic sensor value in millimeter.
*/

unsigned int getUltrasonicValue();

/*******************************************************************************
  Function:
    unsigned int getUltrasonicValueInCm(unsigned int ultrasonicValueInMm)
  Summary:
    Get ultrasonic sensor value in centimeter.
  Description:
    This function divides the value of ultrasonic value in mm by 10 to get the ultrasonic value in centimeter.
  Parameters:
    Ultrasonic value in millimeter.
  Returns:
    Return the ultrasonic sensor value in centimeter.
*/

unsigned int getUltrasonicValueInCm(unsigned int ultrasonicValueInMm);

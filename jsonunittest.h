/*******************************************************************************
    Unit Test for JSON Message Header File

  Company:
    Embedded System Spring 2017 Team 9

  File Name:
    jsonunittest.c

  Summary:
    Unit testing for JSON message.

  Description:
    This file contains declarations of unit tests for JSON parser.

*******************************************************************************/

#ifndef _JSONUNITTEST_H    /* Guard against multiple inclusion */
#define _JSONUNITTEST_H


#ifdef __cplusplus
extern "C" {
#endif
    
#include "json.h"


/*******************************************************************************
  Function:
    void returnTheCorrectKeyValuePairInJsonString(void)

  Summary:
    Check if the function getValueFromJsonString returns 
 * the expected value given a JSON string.
*/    
void returnTheCorrectKeyValuePairInJsonString(void);    
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _JSONUNITTEST_H */

/* *****************************************************************************
 End of File
 */

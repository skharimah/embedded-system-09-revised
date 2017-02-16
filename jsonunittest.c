/*******************************************************************************
    Unit Test for JSON Message Source File

  Company:
    Embedded System Spring 2017 Team 9

  File Name:
    jsonunittest.c

  Summary:
    Unit testing for JSON message.

  Description:
    This file contains implementations of unit tests for JSON parser.

*******************************************************************************/

#include "jsonunittest.h"
#include <assert.h>
#include <string.h>

/*******************************************************************************
  Function:
    void returnTheCorrectKeyValuePairInJsonString(void)

  Remarks:
    See prototype in jsonunittest.h.
 */
void returnTheCorrectKeyValuePairInJsonString(void) {
    int i;
    
    char *key_name = "name";
    char *key_bool = "admin";
    char *key_int = "age";
    
    const char *jsstringCorrectValue1 = "Smith";
    const char *jsstringCorrectValue2 = "true";
    const char *jsstringCorrectValue3 = "15";
    
    char *namePass = "NAME PASS";
    char *boolPass = "BOOL PASS";
    char *intPass = "INT PASS";
    
    static const char *jsstring1 = 
        "{\"name\": \"Smith\", \"admin\": true, \"age\": 15}";
    
    struct Tuple nameTupleValue = getValueFromJsonString(key_name, jsstring1);
    struct Tuple boolTupleValue = getValueFromJsonString(key_bool, jsstring1);
    struct Tuple intTupleValue = getValueFromJsonString(key_int, jsstring1);
    
    char nameValue[nameTupleValue.size], boolValue[boolTupleValue.size], intValue[intTupleValue.size];
    
    if(nameTupleValue.size > 0) {
        for(i=0; i<nameTupleValue.size; i++) {
            nameValue[i] = nameTupleValue.resultString[i];
        }
    }
    
    if(boolTupleValue.size > 0) {
        for(i=0; i<boolTupleValue.size; i++) {
            boolValue[i] = boolTupleValue.resultString[i];
        }
    }
    
    if(intTupleValue.size > 0) {
        for(i=0; i<intTupleValue.size; i++) {
            intValue[i] = intTupleValue.resultString[i];
        }
    }
    
    assert(strcmp(nameValue, "Smith") == 0);
    
    dbgUARTVal('$');

}    


/* *****************************************************************************
 End of File
 */

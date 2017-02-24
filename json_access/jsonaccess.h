#ifndef JSONUNITTEST_JSONACCESS_H
#define JSONUNITTEST_JSONACCESS_H

#include <stdio.h>
#include <string.h>
#include "parson.h"
#include "jsmn.h"
#include "jsonaccess.h"

struct Tuple {
    int size;
    char *resultString;
};

/*******************************************************************************
  Function:
    struct Tuple getValueFromJsonString(char *key)
  Summary:
    Get a Tuple struct containing a JSON value size and char  given a key.
  Description:
    This function returns a Tuple struct containing a JSON value size and char array given a key.
  Parameters:
    Char array of the JSON key and target JSON string respectively.
  Returns:
    A Tuple struct containing value size and string from a JSON string as an integer and a char array.
*/
struct Tuple getValueFromJsonString(char *key, char *jsonString);

/*******************************************************************************
  Function:
    void addStringKeyValuePairToJsonObject(JSON_Value *root_value, char *key, char *value)
  Summary:
    Append a string key value pair to an existing JSON object.
  Description:
    This function appends a string key value pair to an existing JSON object.
  Parameters:
    The exisiting JSON object, the key and the value pair in string.
  Returns:
    None.
*/
void addStringKeyValuePairToJsonObject(JSON_Value *root_value, char *key, char *value);

/*******************************************************************************
  Function:
    void addIntegerKeyValuePairToJsonObject(JSON_Value *root_value, char *key, int value)
  Summary:
    Append a string key and integer value pair to an existing JSON object.
  Description:
    This function appends a string key and integer value pair to an existing JSON object.
  Parameters:
    The exisiting JSON object, the key in string and the value pair in integer.
  Returns:
    None.
*/
void addIntegerKeyValuePairToJsonObject(JSON_Value *root_value, char *key, int value);

/*******************************************************************************
  Function:
    char * serializeJsonStringFromJsonValue(JSON_Value *root_value)
  Summary:
    Convert a JSON object into a JSON string.
  Description:
    This function converts a JSON object of type JSON_Value into a string.
  Parameters:
    An exisiting JSON object of type JSON_Value.
  Returns:
    Converted JSON string from JSON object.
*/
char *serializeJsonStringFromJsonValue(JSON_Value *root_value);

#endif //JSONUNITTEST_JSONACCESS_H

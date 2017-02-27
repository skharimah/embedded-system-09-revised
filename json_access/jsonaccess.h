#ifndef JSONUNITTEST_JSONACCESS_H
#define JSONUNITTEST_JSONACCESS_H

#include <stdio.h>
#include <string.h>
#include "json_access/jWrite.h"
#include "json_access/jsmn.h"

struct Tuple {
    int size;
    char *resultString;
};

/*******************************************************************************
  Function:
    void addStringKeyValuePairToJsonObject(char *key, char *value)
  Summary:
    Append a string key value pair to an existing JSON object.
  Description:
    This function appends a string key value pair to an existing JSON object.
  Parameters:
    The key and the value pair in string.
  Returns:
    None.
*/
void addStringKeyValuePairToJsonObject(char *key, char *value);

/*******************************************************************************
  Function:
    void addIntegerKeyValuePairToJsonObject(char *key, int value)
  Summary:
    Append a string key and integer value pair to an existing JSON object.
  Description:
    This function appends a string key and integer value pair to an existing JSON object.
  Parameters:
    The key in string and the value pair in integer.
  Returns:
    None.
*/
void addIntegerKeyValuePairToJsonObject(char *key, int value);

/*******************************************************************************
  Function:
    void addIntegerArrayToJsonObject(char *key, int values[])
  Summary:
    Add an array of integers to the JSON object.
  Description:
    This function appends an array of integers to the JSON object.
  Parameters:
    The key and array of integers.
  Returns:
    None.
*/
void addIntegerArrayToJsonObject(char *key, int values[]);

/*******************************************************************************
  Function:
    void startWritingToJsonObject(char buffer[100], unsigned int buflen)
  Summary:
    Open JSON root node as an object to write.
  Description:
    This function opens root note as an object in order to write to it.
  Parameters:
    Char buffer of size 100 and its buffer length (100).
  Returns:
    None.
*/
void startWritingToJsonObject(char buffer[100], unsigned int buflen);

/*******************************************************************************
  Function:
    void endWritingToJsonObject()
  Summary:
    Close JSON root object.
  Description:
    This function should be called when user is done writing to the JSON object.
  Parameters:
    None.
  Returns:
    None.
*/
void endWritingToJsonObject();

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

#endif //JSONUNITTEST_JSONACCESS_H

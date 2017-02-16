/*******************************************************************************
  JSON String Generation and Parsing Header File

  Company:
    Embedded System Spring 2017 Team 9

  File Name:
    json.h

  Summary:
    Prototypes for JSON creating and parsing for application.

  Description:
    Declare the function prototypes implemented in json.c

*******************************************************************************/

#ifndef _JSON_H    /* Guard against multiple inclusion */
#define _JSON_H

#include "jsmn/jsmn.h"

struct Tuple {
    int size;
    char *resultString;
};

/*******************************************************************************
  Function:
    const char * createJsonStringFromArrays(char *keys[], char *values[])

  Summary:
    Create a JSON string from keys and values arrays.

  Description:
    This function generates a JSON string from given keys and values pairs.

  Parameters:
    Char array of keys and char array of values. 
    The index of these arrays need to correspond to the keys/values pairs.
        keysArray = ["name", "age"]
        valuesArray = ["John Doe", "27"]

  Returns: 
    JSON string (char array) generated from given keys and values pairs.
    e.g. static const char *jsstring = "{\"user\": \"johndoe\", \"admin\": false, \"uid\": 1000,\n  "
	"\"groups\": [\"users\", \"wheel\", \"audio\", \"video\"]}";
*/

const char * createJsonStringFromArrays(char *keys[], char *values[]);

/*******************************************************************************
  Function:
    char * getValueFromJsonString(char *key)

  Summary:
    Get a value from a JSON string given a key.

  Description:
    This function returns a value from a JSON string given a key.

  Parameters:
    Char array of the JSON key and target JSON string respectively.

  Returns: 
    Value from a JSON string as a char array.
*/

char * getArrayValueFromJsonString(char *key, char *jsonString);

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

struct Tuple getValueFromJsonString(char *key, const char *jsonString);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _JSON_H */

/* *****************************************************************************
 End of File
 */

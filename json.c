/*******************************************************************************
  JSON String Generation and Parsing Source File

  Company:
    Embedded System Spring 2017 Team 9

  File Name:
    json.c

  Summary:
    Implementation of JSON creating and parsing for application.

  Description:
    Implement the function prototypes implemented in json.h

*******************************************************************************/

#include "json.h"
#include <string.h>

/*******************************************************************************
  Function:
    const char * createJsonStringFromArrays(char keys[], char values[])

  Remarks:
    See prototype in json.h.
 */
const char * createJsonStringFromArrays(char *keys[], char *values[]) {
    
    /* TODO: Implement createJsonStringFromArrays */

}

/*******************************************************************************
  Function:
    char * getArrayValueFromJsonString(char *key, char *jsonString)

  Remarks:
    See prototype in json.h.
 */
char * getArrayValueFromJsonString(char *key, char *jsonString) {
    int i, x;
    int result;
    jsmn_parser p;
    jsmntok_t tokens[128];
    jsmn_init(&p);
        
    char *string;
    
    int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
            if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
                strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
                    return 0;
            }
            return -1;
        }
    
    result = jsmn_parse(&p, jsonString, strlen(jsonString), tokens, 256);
    
    for(i=0; i < result; i++) {
        if(jsoneq(jsonString, &tokens[i], key) == 0) {
            int j;
            if (tokens[i+1].type != JSMN_ARRAY) {
                continue;
            }
            for (j = 0; j < tokens[i+1].size; j++) {
                jsmntok_t *g = &tokens[i+j+2];
                string = jsonString + g->start;
                //for(x=0; x<(g->end - g->start); x++) {
                    //dbgUARTVal(string[x]);
                //}
            }
            i += tokens[i+1].size + 1;
        }
    }
    
    return string;
}

/*******************************************************************************
  Function:
    char * getValueFromJsonString(char *key)

  Remarks:
    There seems to be a limit on the size of the keys and the values (limited to 9 chars).
    For example, "name": "John Doe II" seems to only be returning "John Doe "
 */
struct Tuple getValueFromJsonString(char *key, const char *jsonString) {
    
    int i;
    int result, size;
    jsmn_parser p;
    jsmntok_t tokens[128];
    jsmn_init(&p);
        
    char *string;
    
    int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
            if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
                strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
                    return 0;
            }
            return -1;
        }
    
    result = jsmn_parse(&p, jsonString, strlen(jsonString), tokens, 256);
        
    for (i = 1; i < result; i++) {
        if (jsoneq(jsonString, &tokens[i], key) == 0) {
            string = jsonString + tokens[i+1].start;
            size = tokens[i+1].end-tokens[i+1].start;
            i++;
        }
    }
    
    struct Tuple tuple = {size, string};
    
    return tuple;
}

/* *****************************************************************************
 End of File
 */

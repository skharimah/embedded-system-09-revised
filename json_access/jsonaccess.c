#include "jsonaccess.h"

/*******************************************************************************
  Function:
    int jsoneq(const char *json, jsmntok_t *tok, const char *s)
  Summary:
    Required for JSMN parsing.
*/
int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

/*******************************************************************************
  Function:
    void addStringKeyValuePairToJsonObject(JSON_Value *root_value, char *key, char *value)
  Summary:
    See jsonaccess.h
*/
void addStringKeyValuePairToJsonObject(char *key, char *value) {
    jwObj_string(key, value);
}

/*******************************************************************************
  Function:
    void addIntegerKeyValuePairToJsonObject(JSON_Value *root_value, char *key, int value)
  Summary:
    See jsonaccess.h
*/
void addIntegerKeyValuePairToJsonObject(char *key, int value) {
    jwObj_int(key, value);
}

/*******************************************************************************
  Function:
    void addIntegerArrayToJsonObject(char *key, int value[])
  Summary:
    See jsonaccess.h
*/
void addIntegerArrayToJsonObject(char *key, int values[]) {
    /* TODO: Needs to reimplement this function */
    int i = 0;
    jwObj_array(key);
    while(values[i] != NULL) {
        jwArr_int(values[i]);
        i++;
    }
    jwEnd();
}

/*******************************************************************************
  Function:
    void startWritingToJsonObject()
  Summary:
    See jsonaccess.h
*/
void startWritingToJsonObject(char buffer[100], unsigned int buflen) {
    jwOpen(buffer, buflen, JW_OBJECT, JW_COMPACT);
}


/*******************************************************************************
  Function:
    void endWritingToJsonObject()
  Summary:
    See jsonaccess.h
*/
void endWritingToJsonObject() {
    jwClose();
}

/*******************************************************************************
  Function:
    struct Tuple getValueFromJsonString(char *key, char *jsonString)
  Summary:
    See jsonaccess.h
*/
struct Tuple getValueFromJsonString(char *key, char *jsonString) {

    int i;
    int result, size;
    jsmn_parser p;
    jsmntok_t tokens[128];
    jsmn_init(&p);

    char *string;

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

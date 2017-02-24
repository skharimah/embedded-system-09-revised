#include "json_access/jsonaccess.h"
#include "unity.h"

void test_addStringKeyValuePairToJsonObject_should_correctJsonString() {

    static const char *jsstring =
            "{\"name\":\"Indiana Jones\"}";

    /* Create a new JSON object */
    JSON_Value *rootValue = json_value_init_object();

    /* Add key value pairs into the JSON object */
    addStringKeyValuePairToJsonObject(rootValue, "name", "Indiana Jones");

    TEST_ASSERT_EQUAL_STRING(jsstring, serializeJsonStringFromJsonValue(rootValue));
}

void test_addStringKeyValuePairToJsonObject_should_correctStringValue() {

    int i = 0;
    char *value = NULL;
    char buff[20] = {};

    JSON_Value *rootValue = json_value_init_object();
    addStringKeyValuePairToJsonObject(rootValue, "name", "Indiana Jones");
    value = serializeJsonStringFromJsonValue(rootValue);

    struct Tuple tuple = getValueFromJsonString("name", value);

    if(tuple.size > 0) {
        for(i=0; i<tuple.size; i++) {
            buff[i] = tuple.resultString[i];
        }
    }

    TEST_ASSERT_EQUAL_STRING("Indiana Jones", buff);
}

void test_addIntegerKeyValuePairToJsonObject_should_correctJsonString() {

    static const char *jsstring =
            "{\"name\":\"Indiana Jones\",\"awards\":93}";

    JSON_Value *rootValue = json_value_init_object();
    addStringKeyValuePairToJsonObject(rootValue, "name", "Indiana Jones");
    addIntegerKeyValuePairToJsonObject(rootValue, "awards", 93);

    TEST_ASSERT_EQUAL_STRING(jsstring, serializeJsonStringFromJsonValue(rootValue));
}

void test_addIntegerKeyValuePairToJsonObject_should_correctIntegerValue() {

    int i = 0;
    char buff[20] = {};
    static char *jsstring = "{\"name\":\"Indiana Jones\",\"awards\":93}";

    struct Tuple tuple = getValueFromJsonString("awards", jsstring);

    if(tuple.size > 0) {
        for(i=0; i<tuple.size; i++) {
            buff[i] = tuple.resultString[i];
        }
    }

    TEST_ASSERT_EQUAL_STRING("93", buff);
}

int main_unit_test(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_addStringKeyValuePairToJsonObject_should_correctJsonString);
    RUN_TEST(test_addStringKeyValuePairToJsonObject_should_correctStringValue);
    RUN_TEST(test_addIntegerKeyValuePairToJsonObject_should_correctJsonString);
    RUN_TEST(test_addIntegerKeyValuePairToJsonObject_should_correctIntegerValue);
    return UNITY_END();
}
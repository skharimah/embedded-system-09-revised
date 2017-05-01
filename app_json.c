/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_json.c
  Summary:
    This file contains the source code for the MPLAB Harmony application.
  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app_json.h"
#include "app_public.h"
#include "json_access/jsonaccess.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_JSON_DATA app_jsonData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    int msgToJSONMsgQ(Message msg)
  Remarks:
    Write to Wifly (transmit) Queue outside of ISR
    See prototype in app_public.h.
 */
int msgToJSONMsgQ(char* msg) {
    if (messageToQ(recvMsgQueue, msg) != MSG_QUEUE_IS_FULL) {
        //LATASET = 0x08;
        dbgOutputLoc(77);

        return 0;
    }

    return -1;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_JSON_Initialize ( void )
  Remarks:
    See prototype in app_json.h.
 */

void APP_JSON_Initialize(void) {
    app_jsonData.state = APP_JSON_STATE_INIT;
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void APP_JSON_Tasks ( void )
  Remarks:
    See prototype in app_json.h.
 */

void APP_JSON_Tasks(void) {
    char stupidError[200] = "{\"message_type\":\"request\",\"requested_data\":\"run\",\"sequence_id\":\"1\",\"destination\":\"192.168.1.111\",\"source\":\"192.168.1.102\"}";
    //    char *jsstring = "{\"message_type\":\"response\","
    //    "\"source\":\"SOURCE_ROVER_IP_ADDRESS\","
    //    "\"destination\":\"TARGET_ROVER_IP_ADDRESS\","
    //    "\"port\":\2000\","
    //    "\"sequence_id\":\100\"}";


    struct Tuple jsstringValuesTuples[6];

    char *message_type,
            *source,
            *destination,
            *encoder_value,
            *infrared_sensor_value,
            *port;

    char *keys[] = {"message_type", "source", "destination", "sequence_id", "port"};
    //char myMsg[ MSG_BUF_SIZE ];
    char *myMsgPtr = "";
    unsigned int buflen = MSG_BUF_SIZE;
    char buffer[200] = {};
    char buffer1[20] = {};
    char buffer2[20] = {};
    char data_requested[20] = {};

    //Send encoder data
    char encoderStr[7] = {};

    //motor command vars
    char motorCmd[20] = {};
    char motorDist[20] = {};
    int distance, rcount = 0;

    int i;

    while (1) {
        switch (app_jsonData.state) {

            case APP_JSON_STATE_INIT:
            {
                bool appInitialized = true;
                if (appInitialized) {

                    app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                    //app_jsonData.state = APP_JSON_STATE_PARSING_MESSAGE;
                }
                break;
            }

            case APP_JSON_STATE_WAITING_FOR_MESSAGE:
            {
                //dbgUARTVal('t');
                dbgOutputLoc(109);

                /* TODO: Check messageQ to see if the PIC needs a new information*/
                //app_jsonData.state = APP_JSON_STATE_WRITING_REQUEST;
                if (xQueueReceive(recvMsgQueue, &myMsgPtr, 0) == pdTRUE) {
                    //dbgUARTVal('e');
                    //received = true;
                    //dbgOutputVal(recvMsg[0]);
                    /*for(i = 0; myMsgPtr[i] != NULL; i++) {
                            dbgUARTVal(myMsgPtr[i]);
                            //dbgUARTVal('s');
                        }*/

                    if (myMsgPtr[0] == 'L' || myMsgPtr[0] == 'm' || myMsgPtr[0] == 'd') {
                        //dbgUARTVal('n');
                        app_jsonData.state = APP_JSON_STATE_PARSING_INTERNAL_MESSAGE;
                    } else if (myMsgPtr[0] == '{') {

                        app_jsonData.state = APP_JSON_STATE_PARSING_EXTERNAL_MESSAGE;
                        //dbgUARTVal('y');
                    }
                }
                /* TODO: Change state if there is a message to receive from WiFly */

                break;
            }

            case APP_JSON_STATE_PARSING_EXTERNAL_MESSAGE:
            {
                //char * msgPtr = &myMsg[0];
                dbgOutputLoc(110);

                int j;
                int tupleArraySize = sizeof (keys) / sizeof (keys[0]);
                struct Tuple msgType = getValueFromJsonString("message_type", myMsgPtr);
                struct Tuple newDest = getValueFromJsonString("source", myMsgPtr);
                struct Tuple data_req = getValueFromJsonString("requested_data", myMsgPtr);
                struct Tuple motor_command = getValueFromJsonString("motor_command", myMsgPtr);
                struct Tuple motor_distance = getValueFromJsonString("motor_dist", myMsgPtr);

                //jsstringValuesTuples[0] = getValueFromJsonString("message_type", myMsg);
                //jsstringValuesTuples[1] = getValueFromJsonString("sequence_id", myMsg);
                //                jsstringValuesTuples[2] = getValueFromJsonString("requested_data", myMsg);
                //                jsstringValuesTuples[3] = getValueFromJsonString("source", myMsg);
                //                jsstringValuesTuples[4] = getValueFromJsonString("destination", myMsg);
                //jsstringValuesTuples[5] = getValueFromJsonString("port", myMsg);

                if (msgType.size > 0) {
                    dbgOutputLoc(171);

                    for (i = 0; i < msgType.size; i++) {
                        buffer1[i] = msgType.resultString[i];
                    }
                    //dbgOutputVal(buffer1[0]);

                    app_jsonData.state = APP_JSON_STATE_WRITING_RESPONSE;
                } else {
                    dbgOutputLoc(172);
                    dbgOutputVal('b');

                    app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                }
                if (newDest.size > 0) {
                    dbgOutputLoc(173);

                    for (i = 0; i < newDest.size; i++) {
                        buffer2[i] = newDest.resultString[i];
                    }
                    dbgOutputVal(buffer2[0]);
                } else {
                    dbgOutputLoc(174);
                    dbgOutputVal('b');
                }
                if (data_req.size > 0) {
                    dbgOutputLoc(175);

                    for (i = 0; i < data_req.size; i++) {
                        data_requested[i] = data_req.resultString[i];
                    }
                    dbgOutputVal(data_requested[0]);
                } else {
                    dbgOutputLoc(176);
                    dbgOutputVal('b');
                }
                /*if (motor_command.size > 0) {
                    dbgOutputLoc(175);
                    
                    for (i = 0; i < motor_command.size; i++) {
                        motorCmd[i] = motor_command.resultString[i];
                    }
                    dbgOutputVal(motorCmd[0]);
                }
                else{
                    dbgOutputLoc(176);
                    dbgOutputVal('b');
                }
                if (motor_distance.size > 0) {
                    dbgOutputLoc(175);
                    
                    for (i = 0; i < motor_distance.size; i++) {
                        motorDist[i] = motor_distance.resultString[i];
                    }
                    dbgOutputVal(motorDist[0]);
                }
                else{
                    dbgOutputLoc(176);
                    dbgOutputVal('b');
                }*/




                break;
            }

            case APP_JSON_STATE_PARSING_INTERNAL_MESSAGE:
            {
                //myMsgPtr[i] != NULL
                //myMsgPtr[i] != NULL
                if (strcmp(myMsgPtr, "map") == 0) {
                    startWritingToJsonObject(messageptr, buflen);
                    //LATAINV = 0x8;
                    /* TODO: Get message type here */
                    addStringKeyValuePairToJsonObject("message_type", "map");
                    /* TODO: Get encoder_value here */
                    addIntegerKeyValuePairToJsonObject("rev", revision);

                    addStringKeyValuePairToJsonObject("source", "192.168.1.101");

                    /* TODO: Get encoder_value here */
                    addStringKeyValuePairToJsonObject("destination", "192.168.1.111");
                    

                    endWritingToJsonObject();
                    app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
                    memset(recvMsg1, 0, MSG_BUF_SIZE);
                    memset(data_requested, 0, 20);
                } else if (myMsgPtr[0] == 'd') {
                    startWritingToJsonObject(messageptr, buflen);

                    /* TODO: Get message type here */
                    addStringKeyValuePairToJsonObject("message_type", "debug");
                    /* TODO: Get encoder_value here */
                    addStringKeyValuePairToJsonObject("requested_data", myMsgPtr);

                    addStringKeyValuePairToJsonObject("source", "192.168.1.101");

                    /* TODO: Get encoder_value here */
                    addStringKeyValuePairToJsonObject("destination", "192.168.1.111");

                    endWritingToJsonObject();
                    app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
                    memset(recvMsg1, 0, MSG_BUF_SIZE);
                    memset(data_requested, 0, 20);
                } else {
                    for (i = 0; i < 7; i++)
                        //dbgUARTVal(myMsgPtr[i]);
                        encoderStr[i] = myMsgPtr[i];
                    app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                }
                //app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;

                break;
            }

            case APP_JSON_STATE_WRITING_RESPONSE:
            {
                char resultBuff[25];
                dbgOutputLoc(111);
                int i, sum = 0;
                int sequence_id = 0;
                if (buffer1[2] == 'q') {

                    if (strcmp(data_requested, "stop") == 0) {
                        dbgOutputLoc(113);
                        strcpy(appMsg, "stop");
                        myMsgPtr = &appMsg;
                        messageToQ(appRecvQueue, myMsgPtr);
                    } else if (strcmp(data_requested, "run") == 0) {
                        dbgOutputLoc(222);
                        strcpy(appMsg, "run");
                        myMsgPtr = &appMsg;
                        messageToQ(appRecvQueue, myMsgPtr);
                    } else if (strcmp(data_requested, "pause") == 0) {
                        dbgOutputLoc(222);
                        strcpy(appMsg, "pause");
                        myMsgPtr = &appMsg;
                        messageToQ(appRecvQueue, myMsgPtr);
                    } else if (strcmp(data_requested, "encoder") == 0) {
                        dbgOutputLoc(222);
                        strcpy(appMsg, "encoder");
                        myMsgPtr = &appMsg;
                        messageToQ(appRecvQueue, myMsgPtr);
                    }
                }
                else if (strcmp(buffer1, "map") == 0) {
                    
                    short numObs = 0;
                    struct Tuple x = getValueFromJsonString("x", myMsgPtr);
                    struct Tuple y = getValueFromJsonString("y", myMsgPtr);
                    struct Tuple rev = getValueFromJsonString("rev", myMsgPtr);
                    struct Tuple obs = getValueFromJsonString("obs", myMsgPtr);
                    struct Tuple seq_id = getValueFromJsonString("sequence_id", myMsgPtr);
                    struct Tuple total = getValueFromJsonString("total", myMsgPtr);
                    struct Tuple friendly = getValueFromJsonString("friendly", myMsgPtr);
                    char x_string[2], rev_string[3], y_string[2], total_string[3], obs_string, friendly_str;
                    
                    if (x.size > 0) {
                        dbgOutputLoc(171);

                        if (x.size == 1) {
                            x_string[0] = '0';
                            x_string[1] = x.resultString[0];
                        } else {
                            x_string[0] = x.resultString[0];
                            x_string[1] = x.resultString[1];
                        }
                    }
                    if (y.size > 0) {
                        dbgOutputLoc(171);

                        if (y.size == 1) {
                            y_string[0] = '0';
                            y_string[1] = y.resultString[0];
                        } else {
                            y_string[0] = y.resultString[0];
                            y_string[1] = y.resultString[1];
                        }
                    }
                    if (rev.size > 0) {
                        dbgOutputLoc(171);

                        if (rev.size == 1) {
                            
                            revision = rev.resultString[0] - '0';
                        } else if (rev.size == 2){
                            revision = (rev.resultString[1] -'0')+ (rev.resultString[0] - '0')*10;
                        }
                        else
                            {
                            revision = (rev.resultString[2] -'0')+ (rev.resultString[1] - '0')*10 + (rev.resultString[0] - '0')*100;
                        }
                    }
                    if (total.size > 0) {
                        dbgOutputLoc(171);

                        if (total.size == 1) {
                            
                            numObs = total.resultString[0] - '0';
                        } else if (total.size == 2){
                            numObs = (total.resultString[1] -'0')+ (total.resultString[0] - '0')*10;
                        }
                        else
                            {
                            numObs = (total.resultString[2] -'0')+ (total.resultString[1] - '0')*10 + (total.resultString[0] - '0')*100;
                        }
                    }
                    if (seq_id.size > 0) {
                        dbgOutputLoc(171);

                        if (seq_id.size == 1) {
                            
                            sequence_id = seq_id.resultString[0] - '0';
                        } else if (seq_id.size == 2){
                            sequence_id = (seq_id.resultString[1] -'0')+ (seq_id.resultString[0] - '0')*10;
                        }
                        else
                            {
                            sequence_id = (seq_id.resultString[2] -'0')+ (seq_id.resultString[1] - '0')*10 + (seq_id.resultString[0] - '0')*100;
                        }
                        if (sequence_id == numObs){
                            fullMap = true;
                        }
                        else{
                            fullMap = false;
                        }
                    }

                    if (obs.size > 0) {
                        dbgOutputLoc(171);

                        for (i = 0; i < obs.size; i++) {
                            obs_string = obs.resultString[i];
                        }
                    }
                    if (total.size > 0) {
                        dbgOutputLoc(171);

                        for (i = 0; i < total.size; i++) {
                            total_string[i] = total.resultString[i];
                        }
                    }
                    if (friendly.size > 0) {
                        dbgOutputLoc(171);

                        for (i = 0; i < friendly.size; i++) {
                            friendly_str = friendly.resultString[i];
                        }
                    }
                    dbgOutputLoc(222);
                    //if (rcount == 0){
                    snprintf(rMapMsg1[rcount], MSG_BUF_SIZE, "MX%c%cY%c%cO%cF%c", x_string[0], x_string[1], y_string[0], y_string[1], obs_string, friendly_str);
                    myMsgPtr = &(rMapMsg1[rcount]);
                    //}
                    // else if (rcount == 1){
                    // snprintf(rMapMsg1[1], MSG_BUF_SIZE, "MX%c%cY%c%cO%cF%c", x_string[0], x_string[1], y_string[0], y_string[1], obs_string, friendly_str);
                    //myMsgPtr = &(rMapMsg1[1]);
                    //}
                    messageToQ(appRecvQueue, myMsgPtr);
                    rcount = (rcount + 1) % MAX_MSGS;
                    app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                } else{
                    //app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                /*else if (strcmp(data_requested, "motor")){
                    MOTOR_MESSAGE motorMsg;
                    if (strcmp(motorCmd, "forward")){
                        distance = atoi(motorDist);
                        dbgOutputLoc(225);
                        motorMsg.motorState = MOTOR_FORWARD;
                        motorMsg.messageType = 'M';
                        motorMsg.dist = distance;
                            
                        if(xQueueSend(encoderQueue, &motorMsg, NULL) != pdTRUE) {
                            //send failed
                        }
                    }
                }*/

                startWritingToJsonObject(messageptr, buflen);

                /* TODO: Get message type here */
                addStringKeyValuePairToJsonObject("message_type", "response");
                /* TODO: Get encoder_value here */

                addIntegerKeyValuePairToJsonObject("good_messages", good_messages);

                addIntegerKeyValuePairToJsonObject("bad_messages", bad_messages);

                addStringKeyValuePairToJsonObject("encoders", encoderStr);
                int tens = 1;
                if (jsstringValuesTuples[3].size > 1)
                    tens = 10 * (jsstringValuesTuples[3].size - 1);
                for (i = 0; i < jsstringValuesTuples[3].size; i++) {
                    sum += (tens)*(jsstringValuesTuples[3].resultString[i] - '0');
                    tens = tens / 10;
                }
                sum++;
                addIntegerKeyValuePairToJsonObject("sequence_id", sum);
                /* TODO: Get IR_sensor_value here */
                //addIntegerKeyValuePairToJsonObject("infrared_sensor_value", 150);
                /* TODO: Get port number here */
                addStringKeyValuePairToJsonObject("source", "192.168.1.101");
                /* TODO: Get encoder_value here */
                addStringKeyValuePairToJsonObject("destination", buffer2);

                endWritingToJsonObject();

                app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
                }
                memset(recvMsg1, 0, MSG_BUF_SIZE);
                memset(data_requested, 0, 20);
                


                //                    for (i = 0; i < MSG_BUF_SIZE; i++) {
                //                        dbgOutputVal(recvMsg[i]);
                //                    }
                break;

            }

            case APP_JSON_STATE_WRITING_REQUEST:
            {
                app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
                break;
            }

            case APP_JSON_STATE_SENDING_MESSAGE:
            {
                msgToWiflyMsgQ(&messageptr);
                //msgToWiflyMsgQ(&stupidError);
                app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                break;
                dbgOutputVal(0);
            }

            default:
            {
                break;
            }

        }
    }
}



/*******************************************************************************
 End of File
 */
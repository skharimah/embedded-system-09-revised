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


/* TODO:  Add any necessary local functions.
*/


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

void APP_JSON_Initialize ( void )
{
    app_jsonData.state = APP_JSON_STATE_INIT;
    
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

void APP_JSON_Tasks ( void )
{
    Message msg;
    char *jsstring = "{\"message_type\":\"response\","
    "\"source\":\"SOURCE_ROVER_IP_ADDRESS\","
    "\"destination\":\"TARGET_ROVER_IP_ADDRESS\","
    "\"port\":\2000\","
    "\"sequence_id\":\100\"}";

    struct Tuple jsstringValuesTuples[5];
    
    char *message_type,
         *source,
         *destination,
         *encoder_value,
         *infrared_sensor_value,
         *port;
    
    char *keys[] = {"message_type", "source", "destination", "sequence_id", "port"};
    
    switch(app_jsonData.state) {
        
        case APP_JSON_STATE_INIT: {
            bool appInitialized = true;
            if(appInitialized) {
                
                //app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
                app_jsonData.state = APP_JSON_STATE_PARSING_MESSAGE;
            }
            break;
        }
        
        case APP_JSON_STATE_WAITING_FOR_MESSAGE: {
            /* TODO: Check messageQ to see if the PIC needs a new information*/
            //app_jsonData.state = APP_JSON_STATE_WRITING_REQUEST;
            
            /* TODO: Change state if there is a message to receive from WiFly */
            app_jsonData.state = APP_JSON_STATE_PARSING_MESSAGE;
            break;
        }
            
        case APP_JSON_STATE_PARSING_MESSAGE: {
            
            int i, j;
            int tupleArraySize = sizeof(keys)/sizeof(keys[0]);
            
            jsstringValuesTuples[0] = getValueFromJsonString("message_type", jsstring);
            jsstringValuesTuples[1] = getValueFromJsonString("source", jsstring);
            jsstringValuesTuples[2] = getValueFromJsonString("destination", jsstring);
            jsstringValuesTuples[3] = getValueFromJsonString("sequence_id", jsstring);
            jsstringValuesTuples[4] = getValueFromJsonString("port", jsstring);
            
            for(i=0; i<jsstringValuesTuples[0].size; i++) {
                message_type[i] = jsstringValuesTuples[0].resultString[i];
            }
            
            for(i=0; i<jsstringValuesTuples[1].size; i++) {
                message_type[i] = jsstringValuesTuples[1].resultString[i];
            }
           
            app_jsonData.state = APP_JSON_STATE_WRITING_RESPONSE;
            break;
        }
        
        case APP_JSON_STATE_WRITING_RESPONSE: {
            int i;
            unsigned int buflen = 100;
            char buffer[buflen];
            
            startWritingToJsonObject(buffer, buflen);
            
            /* TODO: Get message type here */
            addStringKeyValuePairToJsonObject("message_type", "response");
            /* TODO: Get encoder_value here */
            addIntegerKeyValuePairToJsonObject("encoder_value", 12);
            /* TODO: Get IR_sensor_value here */
            addIntegerKeyValuePairToJsonObject("infrared_sensor_value", 150);
            /* TODO: Get port number here */
            addIntegerKeyValuePairToJsonObject("port", 2000);
            /* TODO: Get encoder_value here */
            addStringKeyValuePairToJsonObject("destination", "TARGET_ROVER_IP_ADDRESS");
            
            endWritingToJsonObject();
            
            for (i = 0; buffer[i] != '\0'; i++) {
                msg.ucData[i] = buffer[i];
            }
            msg.ucMessageID = '\0';
            
            app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
            break;
        }
        
        case APP_JSON_STATE_WRITING_REQUEST: {
            app_jsonData.state = APP_JSON_STATE_SENDING_MESSAGE;
            break;
        }
            
        case APP_JSON_STATE_SENDING_MESSAGE: {
            app_jsonData.state = APP_JSON_STATE_WAITING_FOR_MESSAGE;
            break;
        }    
            
        default: {
            break;
        }
            
    }
}

 

/*******************************************************************************
 End of File
 */

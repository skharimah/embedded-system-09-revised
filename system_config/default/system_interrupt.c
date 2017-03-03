/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include <string.h>
#include "app_json.h"
#include "system_definitions.h"
#include "app_public.h"
#include "debug.h"
#include "json_access/jsonaccess.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/* Timer 2 Interrupt.
 * This interrupt sends a message to the queue (using app1SendTimerValToMsgQ
 * (unsigned int millisecondsElapsed)). Afterwards, this interrupt will call the
 *  debug UART and debug Output value functions using our team name (Team 9).
 * 
 * Additionally, this interrupt has been used to help debug the logic analyzer.
 * 
 * Parameters: none
 * Returns: none
 */
char name[7] = {'T', 'E', 'A', 'M', ' ', '9', ' '};
char mystring[100] = "Team 9: Hard at work!"; //{'R', 'E', 'A', 'D', 'Y', '.', ' ', '\0'};
bool received = true;
int counter = 0;

DBG_POS namepos = T;
unsigned char out;
unsigned int ch = 1;

unsigned int millisec = 0;
int itterate = 0;
int leftTicks = 0;
int rightTicks;
unsigned int count = 0;
int leftTicksPrev = 0;

void IntHandlerDrvTmrInstance0(void) {
    millisec++;
    dbgOutputLoc(TMR_START);


    //dbgOutputLoc(millisec);
    if (millisec % 500 == 0) {//Get timer values
        leftTicksPrev = leftTicks;
        leftTicks = PLIB_TMR_Counter16BitGet(TMR_ID_3);
        rightTicks = PLIB_TMR_Counter16BitGet(TMR_ID_4);
        dbgOutputLoc(TMR_START + 2);
        //dbgOutputVal(leftTicks - leftTicksPrev);
        //Send encoder data to queue
        ENCODER_DATA ticksMessage;
        ticksMessage.leftTicks = leftTicks;
        ticksMessage.rightTicks = rightTicks;
        unsigned char val;
        count++;

        LATAINV = 0x8;

        dbgOutputLoc(TMR_START + 3);



        //Clear Interrupt Flag 

        char mymsg[MSG_BUF_SIZE];
        char buffer[MSG_BUF_SIZE];
        unsigned int buflen = MSG_BUF_SIZE;
        int array[] = {1, 2, 3, 4, 5};

        
//        startWritingToJsonObject(buffer, buflen);
//        addIntegerKeyValuePairToJsonObject("sequence_id", 1);
//        addStringKeyValuePairToJsonObject("message_type", "request");
//        addStringKeyValuePairToJsonObject("source", "192.168.1.102");
//        addStringKeyValuePairToJsonObject("destination", "192.168.1.102");
//        endWritingToJsonObject();
//        
//        
//        //char buffer[] = "some json string";
//
//        int i = 0;
//        //strcpy(messageptr, buffer);
//        for (i = 0; buffer[i] != '\0'; i++) {
//            messageptr[i] = buffer[i];
//        }
//        messageptr[i] = '\0';
//        dbgOutputLoc(TMR_START + 7);
//        msgToWiflyMsgQISR(&messageptr);
        //}
    }
    if (millisec % 1000 == 0) {
        //motorsTurnDemo(itterate);
        //motorsSpeedDemo(itterate);
        if (itterate == 5)
            itterate = 0;
        else
            itterate++;


    }
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_2);
    dbgOutputLoc(TMR_STOP);
}

//Left Encoder Interrupt

void IntHandlerDrvTmrInstance1(void) {
    //dbgOutputVal(leftMotorTicks);
    dbgOutputLoc(212);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_3);
    DRV_TMR1_Tasks();
}

//Right Encoder Interrupt 

void IntHandlerDrvTmrInstance2(void) {
    //dbgOutputVal(rightMotorTicks);
    dbgOutputLoc(221);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_4);
    DRV_TMR2_Tasks();
}

void IntHandlerDrvUsartInstance0(void) {
    char mymsg[MSG_BUF_SIZE] = "";
    char * mymsgptr = "";
    char shitass[16];
    char mychar;
    dbgOutputLoc(UART_START);

    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)) {
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        
        dbgOutputLoc(99);
        ReceiveMsgFromWifly(mymsg);
        //SdbgOutputVal(mymsg[0]);
        dbgOutputLoc(100);
        wiflyToMsgQ(mymsg);

        received = true;
        counter = 0;

        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);

    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)) {
        dbgOutputLoc(WIFLY_TRANS);
        while (!xQueueIsQueueEmptyFromISR(msgQueue)) {
            dbgOutputLoc(35);
            BaseType_t xTaskWokenByReceive = pdFALSE;
            //xQueueReceiveFromISR(msgQueue, (void*) &shitass, &xTaskWokenByReceive);
            if (xQueueReceiveFromISR(msgQueue, (void*) &( mymsgptr ), &xTaskWokenByReceive)
                    == pdTRUE) {
                //(&mymsg) = (&mymsg) + 16;
                dbgOutputLoc(191);
                //TransmitMessageToWifly(qMsg.message, qMsg.message_size);
                TransmitMsgToWifly(mymsgptr);
                if (counter <= 10) {
                    counter++;
                } else if (counter <= 25) {
                    received = false;
                    counter++;
                } else {
                    counter = 0;
                }
                dbgOutputLoc(192);

            }
        }
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        dbgOutputLoc(42);
    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)) {
        //dbgOutputVal('E');
        dbgOutputLoc(WIFLY_ERROR);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    }
    dbgOutputLoc(43);

    //dbgOutputLoc(UART_STOP);
}

/*******************************************************************************
 End of File
 */
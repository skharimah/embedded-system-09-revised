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
#include "motortask.h"
#include "mapgeneratortask.h"
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

void IntHandlerDrvAdc(void) {

    int i;
    sensorValue1 = 0;
    sensorValue2 = 0;

    for (i = 0; i < ADC_NUM_SAMPLE_PER_AVERAGE; i++) {
        sensorValue1 += PLIB_ADC_ResultGetByIndex(ADC_ID_1, (2 * i));
        sensorValue2 += PLIB_ADC_ResultGetByIndex(ADC_ID_1, (2 * i + 1));
    }

    sensorValue1 = sensorValue1 / (ADC_NUM_SAMPLE_PER_AVERAGE);
    sensorValue2 = sensorValue2 / (ADC_NUM_SAMPLE_PER_AVERAGE);



    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}

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
int rightTicksPrev = 0;
int i = 0;

void IntHandlerDrvTmrInstance0(void) {
    millisec++;
    maptime++;

    if (millisec % 50 == 0) {
        if (PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, 6) == 0) {
            buttonHistory[i] = 1;
        } else
            buttonHistory[i] = 0;
        i++;
        if (i == 10)
            i = 0;

    }

    if (millisec % 500 == 0) {
        if (blink_led)
            ledBlink();
        leftTicksPrev = leftTicks;
        leftTicks = PLIB_TMR_Counter16BitGet(TMR_ID_3);
        rightTicks = PLIB_TMR_Counter16BitGet(TMR_ID_4);
        dbgOutputLoc(TMR_START + 2);
       
        unsigned char val;
        count++;

        LATAINV = 0x8;

        //Clear Interrupt Flag 

        char mymsg[MSG_BUF_SIZE];
        char buffer[MSG_BUF_SIZE];
        unsigned int buflen = MSG_BUF_SIZE;
        int array[] = {1, 2, 3, 4, 5};
    }
    if (millisec % 100 == 0) {
        leftTicksPrev = leftTicks;
        rightTicksPrev = rightTicks;
        leftTicks = PLIB_TMR_Counter16BitGet(TMR_ID_3);
        rightTicks = PLIB_TMR_Counter16BitGet(TMR_ID_4);
        
        MOTOR_MESSAGE ticksMessage;
        ticksMessage.messageType = 'E';
        ticksMessage.leftTicks = leftTicks - leftTicksPrev;
        ticksMessage.rightTicks = rightTicks - rightTicksPrev;
        
        if(ticksMessage.leftTicks > 0 && ticksMessage.leftTicks < 100 && ticksMessage.rightTicks > 0 && ticksMessage.rightTicks < 100) {
            if(xQueueSendFromISR(encoderQueue, &ticksMessage, NULL) != pdTRUE) {
                //send failed
            }
}
    }
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_2);
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
    char * msgptr;
    if (rMsgCount == 0)
        msgptr = &recvMsg1;
    else if (rMsgCount == 1)
        msgptr = &recvMsg2;

    char mychar;
    dbgOutputLoc(UART_START);

    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)) {
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        
        if (ReceiveMsgFromWifly(jsonMsg1[rMsgCount])) {
            wiflyToMsgQ(jsonMsg1[rMsgCount]);
        }

        rMsgCount = (rMsgCount + 1) % MAX_MSGS;
        received = true;
        counter = 0;
        dbgOutputLoc(42);
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);

    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)) {
        
        while (!xQueueIsQueueEmptyFromISR(msgQueue)) {
            BaseType_t xTaskWokenByReceive = pdFALSE;
            
            if (xQueueReceiveFromISR(msgQueue, (void*) &(mymsgptr), &xTaskWokenByReceive)
                    == pdTRUE) {
                
                dbgOutputLoc(191);
                
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
        //dbgOutputLoc(42);
    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)) {
        //dbgOutputVal('E');
        dbgOutputLoc(WIFLY_ERROR);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    }
}

/*******************************************************************************
 End of File
 */
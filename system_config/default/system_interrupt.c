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
#include "system_definitions.h"
#include "app_public.h"
#include "debug.h"

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


    //dbgOutputLoc(millisec);
    if (millisec % 100 == 0) {//Get timer values
        leftTicksPrev = leftTicks;
        leftTicks = PLIB_TMR_Counter16BitGet(TMR_ID_3);
        rightTicks = PLIB_TMR_Counter16BitGet(TMR_ID_4);
        
        dbgOutputVal(leftTicks - leftTicksPrev);
        //Send encoder data to queue
        ENCODER_DATA ticksMessage;
        ticksMessage.leftTicks = leftTicks;
        ticksMessage.rightTicks = rightTicks;
        unsigned char val;
        count++;

        //dbgOutputVal('.');
        //LATAINV = 0x8;
        //Inverts ChipKit LD4 to display functioning timer
        //LATAINV = 0x8;

        //assign next letter of string to unsigned character
        //out = name[namepos];

        //output to I/O pins through dbgOutputVal functions
        //dbgOutputVal(out);
        //dbgUARTVal(out);

        //Reset the string iterator
        //if(namepos == SECONDSPACE) {
        //    namepos = T;
        //}
        //else {
        //    namepos++;
        //}






        //Clear Interrupt Flag 
        switch (DRV_USART_ClientStatus(usbHandle)) {
            case DRV_USART_CLIENT_STATUS_ERROR:
                SYS_DEBUG(0, "UART ERROR");
                val = 'E';
                break;
            case DRV_USART_CLIENT_STATUS_BUSY:
                SYS_DEBUG(0, "UART BUSY");
                val = 'B';
                break;
            case DRV_USART_CLIENT_STATUS_CLOSED:
                SYS_DEBUG(0, "UART CLOSED");
                val = 'C';
                break;
            case DRV_USART_CLIENT_STATUS_READY:
                SYS_DEBUG(0, "UART READY");
                //writeStringUART(mystring);
                val = 'D';

                break;
            default:
                val = 'U';
        }
        //dbgUARTVal(val);
        //if (received) {
        //charToMsgQ(val);
        Message mymsg;
        int i, j, temp = 0;
        for (i = 0; mystring[i] != '\0'; i++) {
            mymsg.ucData[i] = mystring[i];
            temp++;
        }
        int length = count;
        /* for (j = temp; j < temp + 4; j++) {
             mymsg.ucData[temp + 3 - j] = (length % 10) + '0';

             length = (length - (length % 10)) / 10;
         }*/

        mymsg.ucData[i] = '\0';
        mymsg.ucMessageID = val;
        msgToWiflyMsgQISR(mymsg);
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
    Message mymsg;
    char mychar;
    //dbgOutputLoc(UART_START);
    //    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    //    DRV_USART_TasksReceive(sysObj.drvUsart0);
    //    DRV_USART_TasksError(sysObj.drvUsart0);

    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)) {
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        dbgOutputLoc(99);
        mymsg = ReceiveMsgFromWifly();
        mychar = mymsg.ucMessageID;
        //dbgOutputVal(mychar);
        dbgOutputLoc(100);
        wiflyToMsgQ(mymsg);

        received = true;
        counter = 0;
        
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        
    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)) {
        dbgOutputLoc(WIFLY_TRANS);
        while (!xQueueIsQueueEmptyFromISR(msgQueue)) {
            dbgOutputLoc(35);
            BaseType_t xTaskWokenByReceive = pdFALSE;
            if (xQueueReceiveFromISR(msgQueue, (void *) &mymsg, &xTaskWokenByReceive)
                    == pdTRUE) {

                //TransmitMessageToWifly(qMsg.message, qMsg.message_size);
                TransmitMsgToWifly(mymsg);
                if (counter <= 10) {
                    counter++;
                } else if (counter <= 25) {
                    received = false;
                    counter++;
                } else {
                    counter = 0;
                }

            }
        }
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        dbgOutputLoc(42);
    } else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)) {
        dbgOutputVal('E');
        dbgOutputLoc(WIFLY_ERROR);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    }
    dbgOutputLoc(43);

    //dbgOutputLoc(UART_STOP);
}

/*******************************************************************************
 End of File
 */
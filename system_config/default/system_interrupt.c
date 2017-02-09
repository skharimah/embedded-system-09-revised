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

DBG_POS namepos = T;
unsigned char out;
unsigned int ch = 1;
unsigned int millisec = 0;
void IntHandlerDrvTmrInstance0(void)

{
    dbgOutputLoc(millisec);
    millisec++;
    unsigned char val = millisec % 100;
    //dbgOutputLoc(val);
    if ( millisec % 100 == 0){
        //Inverts ChipKit LD4 to display functioning timer
        //LATAINV = 0x8;

        //assign next letter of string to unsigned character
        out = name[namepos];

        //output to I/O pins through dbgOutputVal functions
        dbgOutputVal(out);
        dbgUARTVal(out);

        //Reset the string iterator
        if(namepos == SECONDSPACE) {
            namepos = T;
        }
        else {
            namepos++;
        }

        

        if(app1SendTimerValToMsgQ(millisec) != MSG_QUEUE_IS_FULL) {
            //LATASET = 0x08;
        }
        
        //Clear Interrupt Flag 
        
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
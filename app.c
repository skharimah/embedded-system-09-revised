/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "app_public.h"
#include "debug.h"
#include <xc.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
QueueHandle_t msgQueue;
QueueHandle_t motorMsgQueue;



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

APP_DATA appData;
APP_DATA appData ={
};

// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
 */


APP_DRV_OBJECTS appDrvObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void createQueue (void)

  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createQueue(void) {
    QueueHandle_t queue;
    queue = xQueueCreate(10, sizeof (unsigned int));
    if (queue == NULL) {
        /* Queue is not created and should not be used
         * The return value will be NULL if queue is not created
         */
    }
    return queue;
}

/*******************************************************************************
  Function:
    bool receiveFromQueue (QueueHandle_t queue)

  Remarks:
    See prototype in app.h.
 */

bool receiveFromQueue(QueueHandle_t queue) {
    unsigned int buffer;
    if (queue != NULL) {
        if (xQueueReceive(queue, &buffer, portMAX_DELAY) == pdTRUE) {
            /*Unsigned char value from the queue is successfully stored in buffer*/
            return true;
        }
    }
    return false;
}

/*******************************************************************************
  Function:
    int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed)

  Remarks:
    See prototype in app_public.h.
 */
int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed) {
    if (msgQueue != NULL) {
        if (xQueueSendFromISR(msgQueue,
                (void *) &millisecondsElapsed,
                NULL) != pdTRUE) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    }
    else
        return MSG_QUEUE_DOES_NOT_EXIST;

}

/*******************************************************************************
  Function:
    int appSendMotorEncoderOutputValueToMsgQ(unsigned int motorEncoderOutputVal)

  Remarks:
    See prototype in app_public.h.
 */
int appSendMotorEncoderOutputValueToMsgQ(unsigned int motorEncoderOutputVal) {
    if (motorMsgQueue != NULL) {
        if (xQueueSendFromISR(motorMsgQueue,
                (void *) &motorEncoderOutputVal,
                NULL) != pdTRUE) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    }
    else
        return MSG_QUEUE_DOES_NOT_EXIST;

}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
   
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();

    //SYS_PORTS_Clear ( PORTS_BIT_POS_0, PORT_CHANNEL_G, 0xFF );
    //SYS_PORTS_Set( PORTS_BIT_POS_0, PORT_CHANNEL_G, 1, 0x0F0 );
    TRISGCLR = 0x0F0;
    ODCGCLR = 0x0F0;

    TRISECLR = 0xFF;
    ODCECLR = 0xFF;

    /* Set Port A bit 0x08 as output pins */
    TRISACLR = 0x8;
    ODCACLR = 0x8;
    
    usbHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
    
    msgQueue = createQueue();
    if(msgQueue == NULL){
        
        /*wait indefinitely until the queue is successfully created*/
    }
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    
    while (1) {
        
        //if(receiveFromQueue(msgQueue)){
            
            /*receiving from message queue*/
        //}
        
       if(DRV_USART_ReceiverBufferIsEmpty(usbHandle)) {
            /* UART ready to receive */
       }
        
    }
}

/*******************************************************************************
 End of File
 */
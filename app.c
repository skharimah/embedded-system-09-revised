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
#include "string.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
QueueHandle_t encoderQueue;





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
APP_DATA appData = {
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
    QueueHandle_t createQueue (void)

  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createQueue(void) {
    QueueHandle_t queue;

    int queueSize = sizeof (char *);
    queue = xQueueCreate(20, queueSize);
    if (queue == NULL) {
        /* Queue is not created and should not be used
         * The return value will be NULL if queue is not created
         */
    }
    return queue;
}

/*******************************************************************************
  Function:
    void createEncoderQueue (void)

  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createEncoderQueue(void) {
    QueueHandle_t queue;
    int queueSize = sizeof (struct ENCODER_DATA *);
    queue = xQueueCreate(10, queueSize);
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

int receiveFromEncoderQueue(ENCODER_DATA *buffer) {
    dbgOutputLoc(99);
    if (encoderQueue != NULL) {
        if (xQueueReceive(encoderQueue, buffer, 0) == pdTRUE) {
            //dbgOutputVal(buffer->leftTicks);
            return 1;
            
            /*Unsigned char value from the queue is successfully stored in buffer*/
        }
        
    }
    
    return 0;
}

/*******************************************************************************
  Function:
    int messageToQISR(QueueHandle_t queue, Message msg) 

  Remarks:
    Write to Message Queue inside of ISR
    See prototype in app.h.
 */
int messageToQISR(QueueHandle_t queue, char* msg) {
    //dbgOutputVal(msg[0]);
    //dbgOutputLoc(88);
    
    char * jsstring = "message";
    
    if (queue != NULL) {
        if ((xQueueSendFromISR(queue,
                (void *) &jsstring,
                NULL ) != pdTRUE)) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    } else
        return MSG_QUEUE_DOES_NOT_EXIST;
}

/*******************************************************************************
  Function:
    int messageToQ(QueueHandle_t queue, Message msg) 

  Remarks:
    Write to Message Queue outside of ISR
    See prototype in app.h.
 */
int messageToQ(QueueHandle_t queue, char* msg) {

    //dbgOutputLoc(89);
    if (queue != NULL) {
        if (xQueueSend(queue,
                (void *) &msg,
                NULL) != pdPASS) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    } else
        return MSG_QUEUE_DOES_NOT_EXIST;
}

int app1SendEncoderValToMsgQ(ENCODER_DATA *encoderTicks) {
    if (encoderQueue != NULL) {
        if (xQueueSendFromISR(encoderQueue,
                (void *) encoderTicks,
                NULL) != pdTRUE) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    } else
        return MSG_QUEUE_DOES_NOT_EXIST;

}

int UARTInit(USART_MODULE_ID id, int baudrate) {

    PLIB_USART_BaudSetAndEnable(id, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2), baudrate);
    //    PLIB_USART_Enable (id);
    //    PLIB_USART_ReceiverEnable (id);
    //    PLIB_USART_TransmitterEnable (id);
    return 1;
}

bool checkConnected() {

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
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    good_messages = 0;
    bad_messages = 0;


    DRV_TMR0_Initialize();
    DRV_TMR0_Start();

    DRV_TMR1_Initialize();
    DRV_TMR1_Start();

    DRV_TMR2_Initialize();
    DRV_TMR2_Start();

    //SYS_PORTS_Clear ( PORTS_BIT_POS_0, PORT_CHANNEL_G, 0xFF );
    //SYS_PORTS_Set( PORTS_BIT_POS_0, PORT_CHANNEL_G, 1, 0x0F0 );
    //TRISGCLR = 0x0F0;
    //ODCGCLR = 0x0F0;

    // TRISECLR = 0xFF;
    //ODCECLR = 0xFF;

    /* Set Port A bit 0x08 as output pins */
    TRISACLR = 0x8;
    ODCACLR = 0x8;

    msgQueue = createQueue();
    recvMsgQueue = createQueue();
    if (msgQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }
    if (recvMsgQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    UARTInit(USART_ID_1, 57600);
    char myMsg[ MSG_BUF_SIZE ];
    //    myMsg.ucData[0] = 't';
    //    myMsg.ucData[1] = 't';
    //    myMsg.ucMessageID = 't';
    //recvMsg = malloc(Message);
    char myChar;
    appState = RUN;
   
    int i = 101;
    int millisec = 0;
    int prev_ms, cur_ms = PLIB_TMR_Counter16BitGet(TMR_ID_2);
    bool connected = false;
    bool received = false;
    while (1) {
        //dbgOutputLoc(APPTASKS);
        
        prev_ms = cur_ms;
        cur_ms = PLIB_TMR_Counter16BitGet(TMR_ID_2);
        millisec += (cur_ms - prev_ms);


        if (received) {

            if (i < 100) {
                dbgOutputLoc(155 + i);
                dbgOutputVal(myMsg[i]);
                i++;
            } else {
                dbgOutputLoc(154);
                dbgOutputVal(myMsg[0]);
                i = 0;
            }

        }


        //dbgOutputLoc(APPTASKS + 1);
        switch (appState) {
            case RUN:
                break;
            case RECV:
                break;
            case TRANS:
                break;
            default:
                break;
        }
        //
        //        bool sentOnce = false;
        //
        //        if (!connected) {
        //            checkConnected();
        //        }
        //
        //        if (connected && getMsgFromRecvQ(&myMsg) == 0) {
        //            //if (!sentOnce) {
        //            dbgOutputLoc(171);
        //            dbgOutputVal(myMsg.ucMessageID);
        //            for (i = 0; myMsg.ucData[i] != '\0'; i++)
        //                dbgOutputVal(myMsg.ucData[i]);
        //        }
        //        myMsg.ucMessageID = 'M';
        //        char string[100] = "{\"test data\":\"1000\"}";
        //        int i;
        //        for (i = 0; string[i] != '\0'; i++)
        //            myMsg.ucData[i] = string[i];
        //        myMsg.ucData[i] = '\0';


    }
}

/*******************************************************************************
 End of File
 */
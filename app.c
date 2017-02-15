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
    QueueHandle_t createQueue (void)

  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createQueue(void) {
    QueueHandle_t queue;
    queue = xQueueCreate(10, sizeof (Message));
    if (queue == NULL) {
        /* Queue is not created and should not be used
         * The return value will be NULL if queue is not created
         */
    }
    return queue;
}

/*******************************************************************************
  Function:
    unsigned char receiveFromQueue (QueueHandle_t queue)

  Remarks:
    See prototype in app.h.
 */

unsigned char receiveFromQueue(QueueHandle_t queue) {
    dbgOutputLoc(33);
    unsigned char buffer = '0';
    if (queue != NULL) {
        if (xQueueReceive(queue, &buffer, portMAX_DELAY) == pdTRUE) {
            /*Unsigned char value from the queue is successfully stored in buffer*/
        }
    }
    return buffer;
}
int charToMsgQ(char val){
    if (app1SendCharToMsgQ(val) != MSG_QUEUE_IS_FULL) {
                //LATASET = 0x08;
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
                PLIB_INT_SourceFlagSet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);

                dbgOutputLoc(77);
                return 0;
            }
    
    return -1;
}

/*******************************************************************************
  Function:
    int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed)

  Remarks:
    See prototype in app_public.h.
 */
int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed) {
    dbgOutputLoc(22);
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
int app1SendCharToMsgQ(unsigned char value) {
    dbgOutputLoc(11);
    if (msgQueue != NULL) {
        if (xQueueSendFromISR(msgQueue,
                (void *) &value,
                NULL) != pdTRUE) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    }
    else
        return MSG_QUEUE_DOES_NOT_EXIST;

}
int charToMsgQFromISR(QueueHandle_t queue, unsigned char value) {
    dbgOutputLoc(88);
    Message temp;
    temp.ucMessageID = value;
    if (queue != NULL) {
        if (xQueueSendFromISR(queue,
                (void *) &temp,
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
int writeStringUART(char* string){
    dbgOutputLoc(STRING_START);
    int i;
    for (i = 0; string[i] != '\0'; i++){
        dbgUARTVal(string[i]);
    }
    dbgOutputLoc(STRING_STOP);
}

/******************************************************************************
 * Function:
 *      TransmitCharToWifly()
 * 
 * Remarks:
 *      Sends a character over UART to the wifly
 */
void TransmitCharToWifly(unsigned char value)
{
    dbgOutputLoc(39);
    while(PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) 
    {
        dbgOutputLoc(7);
    }
    dbgOutputLoc(40);
    PLIB_USART_TransmitterByteSend(USART_ID_1, value);
    dbgOutputLoc(41);
}

/******************************************************************************
 * Function:
 *      ReceiveCharFromWifly()
 * 
 * Remarks:
 *      Receives a character from the wifly via USART
 * 
 * Returns:
 *      The character received from the wifly
 */
char ReceiveCharFromWifly()
{    
    dbgOutputLoc(66);
    return PLIB_USART_ReceiverByteReceive(USART_ID_1);
}
char ReceiveCharFromWiflyBlocking()
{    
    while(!PLIB_USART_ReceiverDataIsAvailable (USART_ID_1)) 
    {
        dbgOutputLoc(8);
    }
    return PLIB_USART_ReceiverByteReceive(USART_ID_1);
}

char readCharFromQ(QueueHandle_t xQueue){
    dbgOutputLoc(77);
    Message mymessage;
    xQueueReceive(  xQueue,
                    (void *) &mymessage,
                    portMAX_DELAY 
            );
    return mymessage.ucMessageID;
}


int UARTInit(USART_MODULE_ID id, int baudrate){
    
    PLIB_USART_BaudSetAndEnable (id, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2), baudrate);
//    PLIB_USART_Enable (id);
//    PLIB_USART_ReceiverEnable (id);
//    PLIB_USART_TransmitterEnable (id);
    return 1;

        
        
        
        
        
//    DRV_USART_INIT              usartInit;
//    SYS_MODULE_OBJ              objectHandle;
//    usartInit.baud  = 57600;
//    usartInit.mode  = DRV_USART_OPERATION_MODE_NORMAL;
//    usartInit.flags = DRV_USART_INIT_FLAG_NONE;
//    usartInit.usartID   = USART_ID_1;
//    usartInit.brgClock  = 80000000;
//    usartInit.handshake = DRV_USART_HANDSHAKE_NONE;
//    usartInit.lineControl       = DRV_USART_LINE_CONTROL_8NONE1;
//    usartInit.interruptError    = INT_SOURCE_USART_1_ERROR;
//    usartInit.interruptReceive  = INT_SOURCE_USART_1_RECEIVE;
//    usartInit.queueSizeReceive  = 10;
//    usartInit.queueSizeTransmit = 10;
//    usartInit.interruptTransmit = INT_SOURCE_USART_1_TRANSMIT;
//    usartInit.moduleInit.value  = SYS_MODULE_POWER_RUN_FULL;
//        
//    
//    //define usb handle to handle the uart to read and write at the same time
//    //DRV_USART_Initialize(const SYS_MODULE_INDEX drvIndex, const SYS_MODULE_INIT * const init);
//    objectHandle = DRV_USART_Initialize(DRV_USART_INDEX_1, (SYS_MODULE_INIT*)&usartInit);
//    usbHandle = DRV_USART_Open(DRV_USART_INDEX_0, (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING));
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
    if(msgQueue == NULL){
        /* Wait indefinitely until the queue is successfully created */
    }
    if(recvMsgQueue == NULL){
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
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();
    UARTInit(USART_ID_1, 57600);
    Message myMsg;
    char myChar;
    
    //get into command mode
//    dbgUARTVal('$');
//    dbgUARTVal('$');
//    dbgUARTVal('$');
    
    //reboot every time the code starts
//    dbgUARTVal('r');
//    dbgUARTVal('e');
//    dbgUARTVal('b');
//    dbgUARTVal('o');
//    dbgUARTVal('o');
//    dbgUARTVal('t');
    
    //new line character
//    dbgUARTVal(13);
    
    
    while (1) {
        dbgOutputLoc(APPTASKS);
        myChar = readCharFromQ(recvMsgQueue);
        dbgOutputVal(myChar);
        dbgOutputLoc(APPTASKS + 1);
        //if(DRV_USART_ReceiverBufferIsEmpty(usbHandle)) {
            /* UART ready to receive */
        //}
        
    }
}

/*******************************************************************************
 End of File
 */
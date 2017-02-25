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
    void createEncoderQueue (void)

  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createEncoderQueue(void) {
    QueueHandle_t queue;
    queue = xQueueCreate(10, sizeof (ENCODER_DATA));
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

ENCODER_DATA receiveFromEncoderQueue(QueueHandle_t queue) {
    ENCODER_DATA buffer;
    if (queue != NULL) {
        if (xQueueReceive(queue, &buffer, portMAX_DELAY) == pdTRUE) {
            /*Unsigned char value from the queue is successfully stored in buffer*/
        }
    }
    return buffer;
}
/*******************************************************************************
  Function:
    int msgToWiflyMsgQ(Message msg)

  Remarks:
    Write to Wifly (transmit) Queue inside of ISR
    See prototype in app.h.
 */
int msgToWiflyMsgQISR(Message msg) {
    if (messageToQISR(msgQueue, msg) != MSG_QUEUE_IS_FULL) {
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
    int msgToWiflyMsgQ(Message msg)

  Remarks:
    Write to Wifly (transmit) Queue outside of ISR
    See prototype in app.h.
 */
int msgToWiflyMsgQ(Message msg) {
    if (messageToQ(msgQueue, msg) != MSG_QUEUE_IS_FULL) {
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
    int wiflyToMsgQ(Message msg) 

  Remarks:
    Writes message (from wifly) to the received message queue
    See prototype in app_public.h.
 */
int wiflyToMsgQ(Message msg) {
    if (messageToQISR(recvMsgQueue, msg) != MSG_QUEUE_IS_FULL) {
        //LATASET = 0x08;
        dbgOutputLoc(78);
        return 0;
    }

    return -1;
}

/*******************************************************************************
  Function:
    int messageToQISR(QueueHandle_t queue, Message msg) 

  Remarks:
    Write to Message Queue inside of ISR
    See prototype in app.h.
 */
int messageToQISR(QueueHandle_t queue, Message msg) {

    dbgOutputLoc(88);
    if (queue != NULL) {
        if (xQueueSendFromISR(queue,
                (void *) &msg,
                NULL) != pdTRUE) {
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
int messageToQ(QueueHandle_t queue, Message msg) {

    dbgOutputLoc(89);
    if (queue != NULL) {
        if (xQueueSend(queue,
                (void *) &msg,
                NULL) != pdTRUE) {
            return MSG_QUEUE_IS_FULL;
        } else
            return 0;
    } else
        return MSG_QUEUE_DOES_NOT_EXIST;
}

//int app1SendEncoderValToMsgQ(ENCODER_DATA encoderTicks) {
//    if (encoderQueue != NULL) {
//        if (xQueueSendFromISR(encoderQueue,
//                (void *) &encoderTicks,
//                NULL) != pdTRUE) {
//            return MSG_QUEUE_IS_FULL;
//        } else
//            return 0;
//    } else
//        return MSG_QUEUE_DOES_NOT_EXIST;
//
//}

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
    } else
        return MSG_QUEUE_DOES_NOT_EXIST;

}

//int writeStringUART(char* string) {
//    dbgOutputLoc(STRING_START);
//    int i;
//    for (i = 0; string[i] != '\0'; i++) {
//        dbgUARTVal(string[i]);
//    }
//    dbgOutputLoc(STRING_STOP);
//}

/******************************************************************************
 * Function:
 *      TransmitCharToWifly()
 * 
 * Remarks:
 *      Sends a character over UART to the wifly
 */
void TransmitCharToWifly(unsigned char value) {
    dbgOutputLoc(39);
    while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
        dbgOutputLoc(7);
    }
    dbgOutputLoc(40);
    PLIB_USART_TransmitterByteSend(USART_ID_1, value);
    dbgOutputLoc(41);
}
/******************************************************************************
 * Function:
 *      TransmitCharToWifly()
 * 
 * Remarks:
 *      Sends a character over UART to the wifly (non blocking)
 */
void TransmitCharToWiflyNonblocking(unsigned char value) {
    dbgOutputLoc(40);
    PLIB_USART_TransmitterByteSend(USART_ID_1, value);
    dbgOutputLoc(41);
}/*
  * Function: TransmitMsgToWifly(Message msg)
  * 
  * Remarks: Sends the message to the server via wifly according to specs:
  * LLLLMMMMMMMMM...MCCCC
  * where L are the base 10 digits of the length of the following message
  * and C is the checksum of the message, also base 10
  */

void TransmitMsgToWifly(Message msg) {
    char len[4], chksum[4];
    int i;
    intLenToChar(msg, len);
    checksum(msg, chksum);
    for (i = 0; i < 4; i++) {
        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
        TransmitCharToWiflyNonblocking(len[i]);
    }

    TransmitCharToWifly(msg.ucMessageID);

    for (i = 0; msg.ucData[i] != '\0' && msg.ucData[i] != '}' && i < MSG_BUF_SIZE; i++) {
        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
        TransmitCharToWiflyNonblocking(msg.ucData[i]);
    }
    for (i = 0; i < 4; i++) {
        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
        TransmitCharToWiflyNonblocking(chksum[i]);
    }
}
// takes in a Messaqe, calculates the length (characters) of the message

void intLenToChar(Message msg, char *len) {
    int i, j, length = 1;
    for (i = 0; msg.ucData[i] != '\0'; i++) {
        length++;
    }
    for (j = 0; j < 4; j++) {
        len[3 - j] = (length % 10) + '0';

        length = (length - (length % 10)) / 10;
    }
}
// Takes in a message, computes the sum of the bits to make sure that the 
// Data was transmitted properly

void checksum(Message msg, char *len) {
    int i, j, sum = msg.ucMessageID;
    int hex;
    for (i = 0; msg.ucData[i] != '\0'; i++) {
        sum = sum + msg.ucData[i];
    }
    for (j = 0; j < 4; j++) {
        len[3 - j] = (sum % 10) + '0';

        sum = (sum - (sum % 10)) / 10;
    }
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
char ReceiveCharFromWifly() {
    dbgOutputLoc(166);
    char retchar = PLIB_USART_ReceiverByteReceive(USART_ID_1);
    dbgOutputLoc(167);
    return retchar;
}
/******************************************************************************
 * Function:
 *      ReceiveCharFromWiflyBlocking()
 * 
 * Remarks:
 *      Receives a character from the wifly via USART (blocking)
 * 
 * Returns:
 *      The character received from the wifly
 */
char ReceiveCharFromWiflyBlocking() {
    while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
        dbgOutputLoc(8);
    }
    return PLIB_USART_ReceiverByteReceive(USART_ID_1);
}

int getMsgFromRecvQ(Message *msg) {
    dbgOutputLoc(87);
    if (xQueueReceive(recvMsgQueue,
            (void *) &msg,
            0
            ) == pdTRUE) {
        return 0;
    }
    return -1;
}

Message ReceiveMsgFromWifly() {
    Message msg;
    //msg.ucData[0]=0;
    //msg.ucMessageID=0;
    char mychar;
    char len[4], chksum[4];
    int i, length, checksum;
    i = 0;
    dbgOutputLoc(170);

    if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
        mychar = ReceiveCharFromWifly();
        if (mychar >= '0' && mychar <= '9') {
            len[3 - i] = mychar;
            dbgOutputVal(len[3 - i]);
            i++;
        }
        while (i < 4) {
            if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
                mychar = ReceiveCharFromWifly();
                if (mychar >= '0' && mychar <= '9') {
                    len[3 - i] = mychar;
                    dbgOutputVal(len[3 - i]);
                    i++;
                } else
                    dbgOutputVal(mychar);
            }
        }
        i = 0;

        length = charLenToInt(len);
        dbgOutputLoc(171);
        dbgOutputVal(length);
        char temp[MSG_BUF_SIZE];
        while (i < length) {
            if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
                dbgOutputLoc(68);
                mychar = ReceiveCharFromWifly();
                dbgOutputVal(mychar);
                if (mychar != NULL) {
                    if (i == 0) {
                        dbgOutputLoc(65);
                        msg.ucMessageID = mychar;
                    } else {
                        dbgOutputLoc(64);
                        temp[i] = mychar;
                    }
                    i++;
                }
            }
            dbgOutputLoc(67);
        }
        strcpy(msg.ucData, temp);
        i = 0;
        dbgOutputLoc(172);
        //dbgOutputVal(length)
        while (i < 4) {
            if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
                chksum[3 - i] = ReceiveCharFromWifly();
                i++;
            }

        }
        dbgOutputLoc(173);
        checksum = charLenToInt(chksum);
        dbgOutputLoc(174);
        return msg;
    }
    msg.ucMessageID = '\0';
    return msg;
}

int charLenToInt(char *len) {
    int i;
    int powTen = 1;
    int sum = 0;
    for (i = 0; i < 4; i++) {
        sum = sum + (len[i] - '0') * powTen;
        powTen = powTen * 10;
    }
    return sum;
}

char readCharFromQ(QueueHandle_t xQueue) {
    dbgOutputLoc(77);
    Message mymessage;
    xQueueReceive(xQueue,
            (void *) &mymessage,
            portMAX_DELAY
            );
    return mymessage.ucMessageID;
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

    DRV_TMR0_Initialize();
    DRV_TMR0_Start();

    DRV_TMR1_Initialize();
    DRV_TMR1_Start();

    DRV_TMR2_Initialize();
    DRV_TMR2_Start();

    motorsInitialize();

    motorsForward();
    

    encoderQueue = createEncoderQueue();
    //msgQueue = createQueue();
    if (encoderQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }




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
    UARTInit(USART_ID_1, 230400);
    Message myMsg;
    myMsg.ucData[0] = 't';
    myMsg.ucData[1] = 't';
    myMsg.ucMessageID = 't';
    //recvMsg = malloc(Message);
    char myChar;

    //Initialize encoder receive message
    ENCODER_DATA encoderReceived;
    encoderReceived.leftTicks = 0;
    encoderReceived.rightTicks = 0;

    int leftTicks = 0;
    int rightTicks = 0;
    int leftTicksPrev = 0;
    int rightTicksPrev = 0;
    int i;

    bool connected = false;

    while (1) {
        //motorsStop();
        //motorsBackward();
        //dbgOutputLoc(101);
        //dbgOutputVal('t');
        leftTicksPrev = leftTicks;
        rightTicksPrev = rightTicks;
        //motorsForwardDistance(27);

        //Receive encoder data
        encoderReceived = receiveFromEncoderQueue(encoderQueue);
        leftTicks = encoderReceived.leftTicks;
        rightTicks = encoderReceived.rightTicks;
        //dbgOutputVal(rightTicks - rightTicksPrev);


        dbgOutputLoc(APPTASKS);
        //myChar = readCharFromQ(recvMsgQueue);
        //dbgOutputVal(myChar);
        dbgOutputLoc(APPTASKS + 1);

        bool sentOnce = false;

        if (!connected) {
            checkConnected();
        }

        if (connected && getMsgFromRecvQ(&myMsg) == 0) {
            //if (!sentOnce) {
            dbgOutputLoc(171);
            dbgOutputVal(myMsg.ucMessageID);
            for (i = 0; myMsg.ucData[i] != '\0'; i++)
                dbgOutputVal(myMsg.ucData[i]);

            //myMsg.ucMessageID = 'D';
            //msgToWiflyMsgQ(myMsg);
            //sentOnce = true;
        }


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
        myMsg.ucMessageID = 'M';
        char string[100] = "{\"test data\":\"1000\"}";
        int i;
        for (i = 0; string[i] != '\0'; i++)
            myMsg.ucData[i] = string[i];
        myMsg.ucData[i] = '\0';
        //msgToMsgQ(myMsg);


        //  while (1) {
        //        if (!sentOnce){
        //            //msgToMsgQ(myMsg);
        //        }
        //        
        //            else{
        //                msgToMsgQ(myMsg);
        //            }
        //        }
        //if(DRV_USART_ReceiverBufferIsEmpty(usbHandle)) {
        /* UART ready to receive */
        //}

    }
}

/*******************************************************************************
 End of File
 */
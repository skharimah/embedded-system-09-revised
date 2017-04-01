/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "wifly.h"
#include "app_public.h"
/* This section lists the other files that are included in this file.
 * 
 * 
 * 
 */

/*******************************************************************************
  Function:
    int msgToWiflyMsgQ(Message msg)
  Remarks:
    Write to Wifly (transmit) Queue inside of ISR
    See prototype in app_publich.h.
 */
int msgToWiflyMsgQISR(char* msg) {
    if (messageToQISR(msgQueue, msg) != MSG_QUEUE_IS_FULL) {
        //LATASET = 0x08;
        //char buff[100];
        //xQueuePeekFromISR(msgQueue, (void*) buff);
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
    See prototype in app_public.h.
 */
int msgToWiflyMsgQ(char* msg) {
    if (messageToQ(msgQueue, msg) != MSG_QUEUE_IS_FULL) {
        //LATASET = 0x08;
        dbgOutputLoc(77);
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        PLIB_INT_SourceFlagSet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);


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
int wiflyToMsgQ(char* msg) {
    //dbgOutputVal(msg[0]);
    if (messageToQISR(recvMsgQueue, msg) != MSG_QUEUE_IS_FULL) {
        //LATASET = 0x08;
        dbgOutputLoc(78);
        return 0;
    }

    return -1;
}

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
        //dbgOutputLoc(7);
    }
    //dbgOutputLoc(40);
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
    //dbgOutputLoc(40);
    PLIB_USART_TransmitterByteSend(USART_ID_1, value);
    dbgOutputVal(value);
    //dbgOutputLoc(41);
}/*
  * Function: TransmitMsgToWifly(Message msg)
  * 
  * Remarks: Sends the message to the server via wifly according to specs:
  * MMMMMMMMM...MFFCCCC
  * M is the message (char array, c-string)
  * and C is the checksum of the message, also base 10, F is the fletcher checksum
  * (16 byte number)
  */

void TransmitMsgToWifly(char* msg) {
    char len[4], chksum[4];
    int i;
    //    intLenToChar(msg, len);
    checksum(msg, chksum);
    //    for (i = 0; i < 4; i++) {
    //        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
    //        TransmitCharToWiflyNonblocking(len[i]);
    //    }

    //TransmitCharToWifly(msg);

    for (i = 0; msg[i] != '\0' && i < MSG_BUF_SIZE; i++) {
        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
        TransmitCharToWiflyNonblocking(msg[i]);
        //dbgOutputVal(msg[i]);
    }
    uint16_t fletcherChecksum = fletcher16(msg, i);
    char f1 = ((fletcherChecksum >> 8) & 0xff), f2 = (fletcherChecksum & 0xff);
    while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
    TransmitCharToWiflyNonblocking(f1);
    while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
    TransmitCharToWiflyNonblocking(f2);
    for (i = 0; i < 4; i++) {
        while (PLIB_USART_TransmitterBufferIsFull(USART_ID_1));
        TransmitCharToWiflyNonblocking(chksum[i]);
    }
}
// takes in a Messaqe, calculates the length (characters) of the message

void intLenToChar(char *msg, char *len) {
    int i, j, length = 0;
    for (i = 0; msg[i] != '\0'; i++) {
        length++;
    }
    for (j = 0; j < 4; j++) {
        len[3 - j] = (length % 10) + '0';

        length = (length - (length % 10)) / 10;
    }
}
// Takes in a message, computes the sum of the bits to make sure that the 
// Data was transmitted properly

void checksum(char* msg, char *len) {
    int i, j, sum = 0;
    int hex;
    for (i = 0; msg[i] != '\0'; i++) {
        sum = sum + msg[i];
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

int getMsgFromRecvQ(char *msg) {
    dbgOutputLoc(87);
    if (xQueueReceive(recvMsgQueue,
            &(msg),
            0
            ) == pdTRUE) {
        dbgOutputVal('R');
        return 0;
    }

    return -1;
}


bool ReadJSONfromWifly(char* msg, int* msglen) {
    dbgOutputLoc(175);
    bool eom = false;
    int noDataCounter = 0;
    char mychar;
    int i = 0;
    while (!eom && noDataCounter < MSGFAILSIZE) {
        if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
            noDataCounter = 0;

            dbgOutputLoc(176);
            mychar = ReceiveCharFromWifly();
            //if ((*msglen) == 0)
            //  if (mychar != '{')
            //    return false;
            dbgOutputVal(mychar);
            msg[i] = mychar;
            i++;
            //(*msglen)++;
            //if (mychar == '{')


        } else {
            //dbgOutputLoc(177);
            noDataCounter++;
        }
        if (mychar == '}')
            eom = true;
    }
    dbgOutputLoc(178);
    if (eom) {
        //dbgOutputVal(msg->ucData[0]);
        (*msglen) = i;
        return true;
    } else {
        dbgOutputLoc(9);
        //bad_messages++;
        return false;
    }

}

bool ReceiveMsgFromWifly(char* msg) {

    dbgOutputLoc(170);
    int pos = 0;
    char chksum[4];
    uint16_t fCheck = 0, rFCheck = 0, recvFCheck[2] = {0, 0};
    int checksum1, checksum2;
    int i = 0;
    //(*msg).ucMessageID = ReceiveCharFromWifly();
    //dbgOutputVal((*msg).ucMessageID);
    dbgOutputLoc(171);
    bool eom;
    unsigned int noDataCounter = 0;
    bool validJSONMessage = ReadJSONfromWifly(msg, &pos);
    dbgOutputLoc(172);
    //recvFCheck += (ReceiveCharFromWifly() << 8); // read the most significant 8 bits in, shift left 8 bits
    //recvFCheck += (ReceiveCharFromWifly()); // read the least significant 8 bits in.
    if (validJSONMessage) {
        while (i < 2 && noDataCounter < MSGFAILSIZE) {
            if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
                noDataCounter = 0;
                dbgOutputLoc(183);
                recvFCheck[i] = ReceiveCharFromWifly();

                dbgOutputVal(recvFCheck[i]);
                i++;
            } else {
                //dbgOutputLoc(137);
                noDataCounter++;
            }
        }
        noDataCounter = 0;
        rFCheck = (recvFCheck[0] << 8 & 0xFF00) | (recvFCheck[1] & 0xFF); //sum2
        //rFCheck += recvFCheck[1]; //sum1
        dbgOutputLoc(rFCheck >> 8); // sum2
        dbgOutputVal(rFCheck & 0xFF); // sum1

        i = 0;
        while (i < 4 && noDataCounter < MSGFAILSIZE) {
            if (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)) {
                noDataCounter = 0;
                dbgOutputLoc(173);
                chksum[i] = ReceiveCharFromWifly();
                dbgOutputVal(chksum[i]);
                i++;
            } else {
                //dbgOutputLoc(137);
                noDataCounter++;
            }

        }
        if (noDataCounter >= MSGFAILSIZE) {
            bad_messages++;
            dbgOutputLoc(130);
            return false;
        }
        fCheck = fletcher16(msg, pos);
        dbgOutputLoc(fCheck >> 8); // sum2
        dbgOutputVal(fCheck & 0xFF); // sum1





        //char mychar = ReceiveCharFromWifly();
        //dbgOutputVal(mychar);
        checksum2 = charLenToInt(chksum);
        checksum(msg, chksum);
        checksum1 = charLenToInt(chksum);
        dbgOutputLoc(174);
    } else {
        bad_messages++;
        return false;
    }


    // Parse message into JSON
    // check checksum, length
    // 
    if (checksum1 == checksum2) {

        if (fCheck != rFCheck) {
            dbgOutputVal('!');
            bad_messages++;
            return false;
        }

        dbgOutputVal('T');
        //dbgOutputVal(msg->ucData[0]);
        good_messages++;
        return true;

    } else {
        dbgOutputVal('F');
        bad_messages++;
        return false;
    }

}

uint16_t fletcher16(uint8_t const *data, size_t bytes) {
    uint16_t sum1 = 0xff, sum2 = 0xff;
    size_t tlen;

    while (bytes) {
        tlen = ((bytes >= 20) ? 20 : bytes);
        bytes -= tlen;
        do {
            sum2 += sum1 += *data++;
            tlen--;
        } while (tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    /* Second reduction step to reduce sums to 8 bits */
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
    return (sum2 << 8) | sum1;
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
//
//char readCharFromQ(QueueHandle_t xQueue) {
//    dbgOutputLoc(77);
//    char mymessage;
//    xQueueReceive(xQueue,
//            (void *) &mymessage,
//            portMAX_DELAY
//            );
//    return mymessage;
//}


/* *****************************************************************************
 End of File
 */
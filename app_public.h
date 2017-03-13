/* ************************************************************************** */
/** Public Functions for Application

  @Company
    Embedded System Spring 2017 Team 9

  @File Name
    app_public.h

  @Summary
    Prototypes for public functions for application.

  @Description
    Declare the public function prototypes implemented in app.c
 
 */
/* ************************************************************************** */

#ifndef _APP_PUBLIC_H    /* Guard against multiple inclusion */
#define _APP_PUBLIC_H

#include "debug.h"
#include "app.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
    
    
#define MSG_BUF_SIZE 200
    
    char messageptr[MSG_BUF_SIZE];
    
    const char* DEVNAME;// = "sensor";
    const char* IPADDRESS;// = "192.168.1.102";
    
DRV_HANDLE usbHandle;// = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
QueueHandle_t encoderQueue;
QueueHandle_t msgQueue;
QueueHandle_t recvMsgQueue;
typedef enum  {RUN, RECV, TRANS} State;
State appState;

//typedef struct AMessage
// {
//    char ucMessageID;
//    char ucData[ MSG_BUF_SIZE ];
// } Message;
 
 //char message[ MSG_BUF_SIZE ];
 int good_messages, bad_messages;

 typedef struct
{
    //Ticks of the right motor encoder
    int rightTicks;
    
    //Ticks of the left motor encoder
    int leftTicks;
} ENCODER_DATA;
/*******************************************************************************
  Function:
    int app1SendTimerValToMsgQ(unsigned int)

  Summary:
    Sends an integer value (milliseconds elapsed) to the message queue.

  Description:
    This function sends an unsigned int value to the message queue.

  Parameters:
    Unsigned int of the number of milliseconds elapsed.

  Returns:
    0: Indicates an error did not occur;
    1: Indicates MSG_QUEUE_DOES_NOT_EXIST;
    2: Indicates MSG_QUEUE_IS_FULL;
*/
    
int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed);
int charToMsgQFromISR(QueueHandle_t queue, unsigned char value);
int app1SendCharToMsgQ(unsigned char value);
int charToMsgQ(char val);

int msgToWiflyMsgQISR(char* msg);
int msgToWiflyMsgQ(char* msg);
int writeStringUART(char* string);

bool ReceiveMsgFromWifly(char* msg);

void TransmitCharToWifly(unsigned char value);

void TransmitMsgToWifly(char* msg);

char ReceiveCharFromWifly();


/*******************************************************************************
 Encoder Queue Functions
 */
int receiveFromEncoderQueue(ENCODER_DATA *buffer);
QueueHandle_t createEncoderQueue(void);
int app1SendEncoderValToMsgQ(ENCODER_DATA *encoderTicks);


/*******************************************************************************
  Function:
    int appSendMotorEncoderOutputValueToMsgQ(unsigned int motorEncoderOutputVal)

  Summary:
    Sends motor encoder output value (in unit TBD) to a message queue.

  Description:
    This function sends an unsigned int valueq (in unit TBD) to a message queue.

  Parameters:
    Unsigned int of (unit).

  Returns:
    0: Indicates an error did not occur;
    1: Indicates MSG_QUEUE_DOES_NOT_EXIST;
    2: Indicates MSG_QUEUE_IS_FULL;
*/

int appSendMotorEncoderOutputValueToMsgQ(unsigned int motorEncoderOutputVal);


/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _APP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
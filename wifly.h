/* ************************************************************************** */
/** Descriptive File Name
  @Company
    Company Name
  @File Name
    filename.h
  @Summary
    Brief description of the file.
  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _WIFLY_H    /* Guard against multiple inclusion */
#define _WIFLY_H

#include "app_public.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


 /*******************************************************************************
  Function:
    void intLenToChar(Message msg, char *len)
  Summary:
    Send a message to a previously created message queue
  Description:
    This function will send a Message struct to a message queue, but is NOT safe
    for use within an ISR.
  Parameters:
    QueueHandle_t queue: The handle for the queue you want to write to
    Message msg: The message to write to the queue
  Returns:
    An integer 0 for success, -1 for failure.
*/
void intLenToChar(char *msg, char *len);
void checksum(char *msg, char *sum);
uint16_t fletcher16( uint8_t const *data, size_t bytes );
QueueHandle_t createEncoderQueue(void);
int getMsgFromRecvQ(char *msg);

bool ReceiveMsgFromWifly(char* msg);

void TransmitCharToWifly(unsigned char value);

void TransmitMsgToWifly(char* msg);

char ReceiveCharFromWifly();

    
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
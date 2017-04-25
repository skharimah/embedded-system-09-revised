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
    
#define BLOCK_DIST 755 //one block
#define HYPT_DIST 1080
//#define BLOCK_DIST 2000
    
#define MSG_BUF_SIZE 200
#define MAP_BUF_SIZE 500
    
#define LED_PIN 0 // D6 - silkscreen 47 - LED
#define LED_PORT PORT_CHANNEL_F
    
#define MAX_MSGS 15
    bool fullMap;
    int revision;
    char messageptr[200];
    char recvMsg1[200];
    char recvMsg2[200];
    int rMsgCount;
    char appMsg[200];
    char encoderValMsg[20];
    char jsonMsg[200];
    
    //map
    char mapMsg[MAP_BUF_SIZE];
    char rMapMsg1[MAX_MSGS][200];

    unsigned int maptime;
    unsigned int dbgCount;
    int rMsgCount;
    
    char * globalCharPtr;
    const char* DEVNAME;// = "sensor";
    const char* IPADDRESS;// = "192.168.1.102";
    
DRV_HANDLE usbHandle;// = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
QueueHandle_t encoderQueue;
QueueHandle_t msgQueue;
QueueHandle_t recvMsgQueue;
QueueHandle_t appRecvQueue;
typedef enum  {INIT, RUN, RECV, TRANS, PAUSE, STOP, RESET, WAIT, DRIVE} State;
State appState;


//typedef struct AMessage
// {
//    char ucMessageID;
//    char ucData[ MSG_BUF_SIZE ];
// } Message;
 
 //char message[ MSG_BUF_SIZE ];
 int good_messages, bad_messages;
 
 //sensor values
 int sensorValue1;
 int sensorValue2;
 int sensorValue3;

 //button debouncing
 int buttonHistory[10];
 
 char mapValMsg[200];
 char mapRecvMsg[200];
 
 typedef struct
{
    //Tracks the type of message
    char messageType;
    
    //Motor control
    int motorState;
    
    //Ticks of the right motor encoder
    int rightTicks;
    
    //Ticks of the left motor encoder
    int leftTicks;
    
    //Distance to move
    int dist;
    
    //Direction to turn before moving
    int dir;
} MOTOR_MESSAGE;

typedef struct
{   
    //X coordinate 
    int xCoordinate;
    
    //Y coordinate
    int yCoordinate;
    
    //1 is obstacle, 0 is flag rover, 2 is cm, 3 is tag, 
    int isObstacle;
    
    //1 is our team, 0 is other team
    bool isOurs;
    
    //if more than one obstacle
    int sequenceID;
    
} MAP_MESSAGE;

//Define directions
typedef enum
{
	/* Application's state machine's initial state. */
	NORTH=0,
	NORTHEAST,
    EAST,
    SOUTHEAST,
    SOUTH,
    SOUTHWEST,
    WEST,
    NORTHWEST

	/* TODO: Define states used by the application state machine. */

} DIRECTIONS;
//motorTask states

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
int taskToMsgQ(char* msg);
int writeStringUART(char* string);

bool ReceiveMsgFromWifly(char* msg);

void TransmitCharToWifly(unsigned char value);

void TransmitMsgToWifly(char* msg);

char ReceiveCharFromWifly();

int msgToJSONMsgQ(char* msg);

/*******************************************************************************
 Encoder Queue Functions
 */
int receiveFromEncoderQueue(MOTOR_MESSAGE *buffer);
QueueHandle_t createEncoderQueue(void);
int app1SendEncoderValToMsgQ(MOTOR_MESSAGE *encoderTicks);


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
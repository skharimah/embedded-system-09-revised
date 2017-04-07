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
#include <queue.h>
#include "debug.h"
#include "json_access/jsonaccess.h"
#include <xc.h>
#include "string.h"
#include "aStarLibrary.h"

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

int receiveFromEncoderQueue(MOTOR_MESSAGE *buffer) {
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
    
    //char * jsstring = "message";
    
    if (queue != NULL) {
        if ((xQueueSendFromISR(queue,
                (void *) &msg,
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

int app1SendEncoderValToMsgQ(MOTOR_MESSAGE *encoderTicks) {
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

void MoveSprite(int ID) {
    
    int northSouth;
    int eastWest;
    MOTOR_MESSAGE msg;
    msg.messageType = 'M';
    msg.motorState = MOTOR_PATH_FIND;
    msg.dist = 0;
            
    //1.Read path information
    ReadPath(ID, xLoc[ID], yLoc[ID], 1);

    //2.Move sprite. xLoc/yLoc = current location of sprite. xPath and
    //	yPath = coordinates of next step on the path that were/are
    //	read using the readPath function.
    if (yLoc[ID] > yPath[ID]) { //yLoc[ID] - speed[ID];	
        dbgUARTVal('s');
        dbgUARTVal('o');
        dbgUARTVal('u');
        dbgUARTVal('t');
        dbgUARTVal('h');
        northSouth = -1; // SOUTH
    }
    else if (yLoc[ID] < yPath[ID]) { //yLoc[ID] + speed[ID];
        dbgUARTVal('n');
        dbgUARTVal('o');
        dbgUARTVal('r');
        dbgUARTVal('t');
        dbgUARTVal('h');
        northSouth = 1; // NORTH
    }
    else
        northSouth = 0;
    if (xLoc[ID] > xPath[ID]) { //xLoc[ID] - speed[ID];
        dbgUARTVal('w');
        dbgUARTVal('e');
        dbgUARTVal('s');
        dbgUARTVal('t');
        eastWest = -1; //WEST
    }
    else if (xLoc[ID] < xPath[ID]) { //xLoc[ID] + speed[ID];
        dbgUARTVal('e');
        dbgUARTVal('a');
        dbgUARTVal('s');
        dbgUARTVal('t');
        eastWest = 1; //EAST
    }
    else
        eastWest = 0;
    dbgUARTVal('\n');

    if(northSouth == 1 && eastWest == 0) {
        msg.dir = NORTH;
        msg.dist = BLOCK_DIST;
    }
    else if(northSouth == -1 && eastWest == 0) {
        msg.dir = SOUTH;
        msg.dist = BLOCK_DIST;
    }
    else if(northSouth == 0 && eastWest == 1) {
        msg.dir = EAST;
        msg.dist = BLOCK_DIST;
    }
    else if(northSouth == 0 && eastWest == -1) {
        msg.dir = WEST;
        msg.dist = BLOCK_DIST;   
    }
    else if(northSouth == 1 && eastWest == 1) {
        msg.dir = NORTHEAST;
        msg.dist = HYPT_DIST;
    }
    else if(northSouth == 1 && eastWest == -1) {
        msg.dir = NORTHWEST;
        msg.dist = HYPT_DIST;
    }
    else if(northSouth == -1 && eastWest == 1) {
        msg.dir = SOUTHEAST;
        msg.dist = HYPT_DIST;
    }
    else if(northSouth == -1 && eastWest == -1) {
        msg.dir = SOUTHWEST;
        msg.dist = HYPT_DIST;
    }
    else {
        //Do something
       
    }


    if(msg.dist != 0) {
        LATAINV = 0x8;
        if(xQueueSend(encoderQueue, &msg, NULL) != pdTRUE) {
                //send failed
        }
    }

    //	
    ////3.When sprite reaches the end location square	(end of its current
    ////	path) ...		
    //	if (pathLocation[ID] == pathLength[ID]) 
    //	{
    ////		Center the chaser in the square (not really necessary, but 
    ////		it looks a little better for the chaser, which moves in 3 pixel
    ////		increments and thus isn't always centered when it reaches its
    ////		target).
    //		if (abs(xLoc[ID] - xPath[ID]) < speed[ID]) xLoc[ID] = xPath[ID];
    //		if (abs(yLoc[ID] - yPath[ID]) < speed[ID]) yLoc[ID] = yPath[ID];
    //	}
}

//function for parsing msg from app_json
int subStrToInt(char *len, int from, int to) {
    int i;
    int powTen = 1;
    int sum = 0;
    for (i = from; i <= to; i++) {
        sum = sum + (len[i] - '0') * powTen;
        powTen = powTen * 10;
    }
    return sum;
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
    int i, j;

    for (i = 0; i < mapWidth; i++)
        for (j = 0; j < mapHeight; j++)
            walkability [i][j].walkability = walkable;

    //wall from (1, 5) to right side
    for (i = 1; i < mapWidth; i++)
        //for (j = 0; j < mapHeight; j++)
        walkability [i][5].walkability = unwalkable;
    
    //wall from (3,5) to (3,18)
    for (i = 5; i < 18; i++)
        walkability [3][i].walkability = unwalkable;
    
    // wall from (7,10) to top
    for (i = 10; i < mapHeight; i++)
        walkability [7][i].walkability = unwalkable;
    
    //wall across map
    /*for (i = 0; i < mapHeight; i++)
        walkability [7][i].walkability = unwalkable;*/
    
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
    motorsInitialize();

    //    motorsForward();


    //encoderQueue = createEncoderQueue();
    //msgQueue = createQueue();
    //if (encoderQueue == NULL) {
    /* Wait indefinitely until the queue is successfully created */
    //}
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

    appRecvQueue = createQueue();
    if (appRecvQueue == NULL) {
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
    
    DRV_ADC_Open();    //start ADC
   
   
    
    int ID = 1;
    int goalX = 10;
    int goalY = 7;

    char myMsg[ MSG_BUF_SIZE ];
    //    myMsg.ucData[0] = 't';
    //    myMsg.ucData[1] = 't';
    //    myMsg.ucMessageID = 't';
    //recvMsg = malloc(Message);
    char myChar;
    appState = INIT;
    //    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_2);
    //    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_3);
    //    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_4);
    //Initialize encoder receive message

    //    int leftTicks = 0;
    //    int rightTicks = 0;
    //    int leftTicksPrev = 0;
    //    int rightTicksPrev = 0;
    //int blinkms = 0;
    char * myMsgPtr = "";
    int i = 101;
    int millisec = 0;
    int prev_ms = 0, cur_ms = PLIB_TMR_Counter16BitGet(TMR_ID_2);
    bool connected = false;
    bool received = false;
    
    //app_JSON parsing vars
    int xCoord;
    int yCoord;
    int obs;
    int rType;
    while (1) {
        if (xQueueReceive(appRecvQueue, (void*) &(myMsgPtr), 0) == pdTRUE) {
            //if (tempGetMsgFromQ(appRecvQueue, myMsgPtr) == 0) {
            received = true;
            if (myMsgPtr > 0)
                dbgOutputVal(myMsgPtr[0]);
            dbgOutputLoc(APPRECVMSG);
            if (strcmp(myMsgPtr, "stop") == 0) {
                dbgOutputLoc(APPSTOP);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_4);
                appState = STOP;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "run") == 0) {
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_4);
                dbgOutputLoc(APPRUN);
                appState = RUN;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "pause") == 0) {
                dbgOutputLoc(APPPAUSE);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_4);

                appState = PAUSE;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "done") == 0) {
                appState = RUN;
                xLoc[ID] = xPath[ID]; //xLoc[ID] + speed[ID];
                yLoc[ID] = yPath[ID]; //yLoc[ID] - speed[ID];
                memset(myMsgPtr, 0, MSG_BUF_SIZE);	
            }
            if (myMsgPtr[0] == 'M') {
                xCoord = subStrToInt(myMsgPtr, 2, 3);
                yCoord = subStrToInt(myMsgPtr, 5, 6);
                rType = subStrToInt(myMsgPtr, 8, 8);
                if(rType == 1) {
                    walkability [xCoord][yCoord].walkability = unwalkable;
                    walkability [xCoord][yCoord].rover = OBSTACLE;
                }
                else
                {
                    walkability [xCoord][yCoord].rover = rType;
                }
                //walkability [xCoord][yCoord].walkability = unwalkable;
                //walkability [xCoord][yCoord].rover = obs;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
        }


        //dbgOutputLoc(APPTASKS + 1);
        switch (appState) {
            case INIT:
                /*dbgUARTVal('S');
                dbgUARTVal('T');
                dbgUARTVal('A');
                dbgUARTVal('R');
                dbgUARTVal('T');*/
                if(goalX != -1 && goalY != -1)
                {
                    pathStatus[ID] = FindPath(ID, xLoc[ID], yLoc[ID], goalX, goalY);
                    appState = RUN;
                }
                break;
            case RUN:
                //int FindPath (int pathfinderID,int startingX, int startingY, int targetX, int targetY)





                //2.Move smiley.
                if (pathStatus[ID] == found) {
                    MoveSprite(ID);
                    appState = WAIT;
                }
                else {
                    //dbgUARTVal('X');
                    appState = INIT;
                }

                if (xLoc[ID] == goalX && yLoc[ID] == goalY) {
                    appState = WAIT;

                    /*dbgUARTVal('G');
                    dbgUARTVal('O');
                    dbgUARTVal('A');
                    dbgUARTVal('L');
                    dbgUARTVal('\n');*/
                }



                if (received) {
                    received = false;
                }
                break;
            case RESET:
                appState = INIT;
                break;
            case RECV:
                break;
            case TRANS:
                break;
            case PAUSE:
                if (received) {
                    received = false;
                }
                break;
            case STOP:
                if (received) {
                    received = false;
                }
                break;
            case WAIT:
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
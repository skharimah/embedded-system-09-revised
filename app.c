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

//QueueHandle_t createQueue(void) {
//    QueueHandle_t queue;
//
//    int queueSize = sizeof (char *);
//    queue = xQueueCreate(20, queueSize);
//    if (queue == NULL) {
//        /* Queue is not created and should not be used
//         * The return value will be NULL if queue is not created
//         */
//    }
//    return queue;
//}

QueueHandle_t createQueue(int size) {
    QueueHandle_t queue;

    int queueSize = sizeof (char *);
    queue = xQueueCreate(MAX_MSGS, queueSize);
    if (queue == NULL) {
        /* Queue is not created and should not be used
         * The return value will be NULL if queue is not created
         */
    }
    return queue;
}

/*******************************************************************************
  Function:
    QueueHandle_t createQueue (void)
  Remarks:
    See prototype in app.h.
 */

QueueHandle_t createAppQueue(void) {
    QueueHandle_t queue;

    int queueSize = 6;
    queue = xQueueCreate(5, queueSize);
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

int getMsgFromQ(QueueHandle_t queue, char *msg) {
    dbgOutputLoc(87);
    if (xQueueReceive(queue,
            &(msg),
            0
            ) == pdTRUE) {
        dbgOutputVal('R');
        return 0;
    }

    return -1;
}

int tempGetMsgFromQ(QueueHandle_t queue, char *msg) {
    dbgOutputLoc(87);
    if (xQueueReceive(queue,
            (void*) &(msg),
            0
            ) == pdTRUE) {
        dbgOutputVal('R');
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
int messageToQISR(QueueHandle_t queue, char* msg) {
    //dbgOutputVal(msg[0]);
    //dbgOutputLoc(88);

    //char * jsstring = "message";
    BaseType_t xHigherPriorityTaskWoken;
    if (queue != NULL) {
        if ((xQueueSendFromISR(queue,
                (void *) &msg,
                &xHigherPriorityTaskWoken) != pdTRUE)) {
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
//
//int app1SendEncoderValToMsgQ(ENCODER_DATA *encoderTicks) {
//    if (encoderQueue != NULL) {
//        if (xQueueSendFromISR(encoderQueue,
//                (void *) encoderTicks,
//                NULL) != pdTRUE) {
//            return MSG_QUEUE_IS_FULL;
//        } else
//            return 0;
//    } else
//        return MSG_QUEUE_DOES_NOT_EXIST;
//
//}

int UARTInit(USART_MODULE_ID id, int baudrate) {

    PLIB_USART_BaudSetAndEnable(id, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2), baudrate);
    //    PLIB_USART_Enable (id);
    //    PLIB_USART_ReceiverEnable (id);
    //    PLIB_USART_TransmitterEnable (id);
    return 1;
}

void requestMap() {
    dbgOutputLoc(113);
    strcpy(recvMsg1, "map");
    globalCharPtr = &recvMsg1;
    messageToQ(recvMsgQueue, globalCharPtr);
}

void dbgServer(char * msg) {
    dbgOutputLoc(113);
    strcpy(recvMsg1, msg);
    globalCharPtr = &recvMsg1;
    messageToQ(recvMsgQueue, globalCharPtr);
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

    dbgUARTVal(' ');
    dbgUARTVal(':');
    dbgUARTVal(' ');
    dbgUARTVal(walkability [5][5].walkability + '0');
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
    } else if (yLoc[ID] < yPath[ID]) { //yLoc[ID] + speed[ID];
        if (xLoc[ID] - xPath[ID] == 0) {

            dbgUARTVal('N');
            dbgUARTVal('O');
            dbgUARTVal('R');
            dbgUARTVal('T');
            dbgUARTVal('H');
        } else {
            dbgUARTVal('n');
            dbgUARTVal('o');
            dbgUARTVal('r');
            dbgUARTVal('t');
            dbgUARTVal('h');
        }

        northSouth = 1; // NORTH
    } else
        northSouth = 0;
    if (xLoc[ID] > xPath[ID]) { //xLoc[ID] - speed[ID];
        dbgUARTVal('w');
        dbgUARTVal('e');
        dbgUARTVal('s');
        dbgUARTVal('t');
        eastWest = -1; //WEST
    } else if (xLoc[ID] < xPath[ID]) { //xLoc[ID] + speed[ID];
        dbgUARTVal('e');
        dbgUARTVal('a');
        dbgUARTVal('s');
        dbgUARTVal('t');
        eastWest = 1; //EAST
    } else
        eastWest = 0;
    dbgUARTVal('\n');

    if (northSouth == 1 && eastWest == 0) {
        msg.dir = NORTH;
        msg.dist = BLOCK_DIST;
    } else if (northSouth == -1 && eastWest == 0) {
        msg.dir = SOUTH;
        msg.dist = BLOCK_DIST;
    } else if (northSouth == 0 && eastWest == 1) {
        msg.dir = EAST;
        msg.dist = BLOCK_DIST;
    } else if (northSouth == 0 && eastWest == -1) {
        msg.dir = WEST;
        msg.dist = BLOCK_DIST;
    } else if (northSouth == 1 && eastWest == 1) {
        msg.dir = NORTHEAST;
        msg.dist = HYPT_DIST;
    } else if (northSouth == 1 && eastWest == -1) {
        msg.dir = NORTHWEST;
        msg.dist = HYPT_DIST;
    } else if (northSouth == -1 && eastWest == 1) {
        msg.dir = SOUTHEAST;
        msg.dist = HYPT_DIST;
    } else if (northSouth == -1 && eastWest == -1) {
        msg.dir = SOUTHWEST;
        msg.dist = HYPT_DIST;
    } else {
        //Do something

    }

    //xLoc[ID] = xPath[ID];
    //yLoc[ID] = yPath[ID];
    if (msg.dist != 0) {
        //LATAINV = 0x8;
        if (xQueueSend(encoderQueue, &msg, NULL) != pdTRUE) {
            //send failed
        }
    }
}

DIRECTIONS getOrientation(int ID) {

    int northSouth;
    int eastWest;
    DIRECTIONS dir;

    //1.Read path information
    pathLocation[ID]--;
    ReadPath(ID, xLoc[ID], yLoc[ID], 1);
    pathLocation[ID]--;
    

    //2.Move sprite. xLoc/yLoc = current location of sprite. xPath and
    //	yPath = coordinates of next step on the path that were/are
    //	read using the readPath function.
    if (yLoc[ID] > yPath[ID]) { //yLoc[ID] - speed[ID];	
        northSouth = -1; // SOUTH
    } else if (yLoc[ID] < yPath[ID]) { //yLoc[ID] + speed[ID];
        northSouth = 1; // NORTH
    } else
        northSouth = 0;
    if (xLoc[ID] > xPath[ID]) { //xLoc[ID] - speed[ID];
        eastWest = -1; //WEST
    } else if (xLoc[ID] < xPath[ID]) { //xLoc[ID] + speed[ID];
        eastWest = 1; //EAST
    } else
        eastWest = 0;

    if (northSouth == 1 && eastWest == 0) {
        dir = NORTH;
    } else if (northSouth == -1 && eastWest == 0) {
        dir = SOUTH;
    } else if (northSouth == 0 && eastWest == 1) {
        dir = EAST;
    } else if (northSouth == 0 && eastWest == -1) {
        dir = WEST;
    } else if (northSouth == 1 && eastWest == 1) {
        dir = NORTHEAST;
    } else if (northSouth == 1 && eastWest == -1) {
        dir = NORTHWEST;
    } else if (northSouth == -1 && eastWest == 1) {
        dir = SOUTHEAST;
    } else if (northSouth == -1 && eastWest == -1) {
        dir = SOUTHWEST;
    }
    return dir;
}

//function for parsing msg from app_json

int subStrToInt(char *len, int from, int to) {
    int i;
    int powTen = 1;
    int sum = 0;
    for (i = from; i < to; i++) {
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
    int ID = 1;

    for (i = 0; i < mapWidth; i++)
        for (j = 0; j < mapHeight; j++)
            walkability [i][j].walkability = walkable;

    //wall from (1, 5) to right side
    //    for (i = 1; i < mapWidth; i++)
    //        //for (j = 0; j < mapHeight; j++)
    //        walkability [i][5].walkability = unwalkable;



    //
    //    //wall from (3,5) to (3,18)
    //    for (i = 5; i < 18; i++)
    //        walkability [3][i].walkability = unwalkable;
    //
    //    // wall from (7,10) to top
    //    for (i = 10; i < mapHeight; i++)
    //        walkability [7][i].walkability = unwalkable;

    //wall across map
    /*for (i = 0; i < mapHeight; i++)
        walkability [7][i].walkability = unwalkable;*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    good_messages = 0;
    bad_messages = 0;
    blink_led = false;


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

    msgQueue = createQueue(50);
    recvMsgQueue = createQueue(50);
    if (msgQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }
    if (recvMsgQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }

    appRecvQueue = createQueue(20);
    if (appRecvQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }
    maptime = 0;
    dbgCount = 0;
    rMsgCount = 0;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    revision = -1;
    fullMap = true;
    UARTInit(USART_ID_1, 57600);
    DRV_ADC_Open(); //start ADC
    bool newMap = false;
    steps = 0;
    int ID = 1;
    int goalX = -1;
    int goalY = -1;
    xLoc[ID] = -1;
    yLoc[ID] = -1;
    oldX = xLoc[ID];
    oldY = yLoc[ID];


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


    int sum = 0;
    int powTen = 1;
    while (1) {

        if (xQueueReceive(appRecvQueue, (void*) &(myMsgPtr), 0) == pdTRUE) {
            //if (tempGetMsgFromQ(appRecvQueue, myMsgPtr) == 0) {
            received = true;
            if (myMsgPtr > 0)
                dbgOutputVal(myMsgPtr[0]);
            dbgOutputLoc(APPRECVMSG);
            if (strcmp(myMsgPtr, "stop") == 0) {
                blink_led = false;
                ledOff();
                snprintf(recvMsg1, MSG_BUF_SIZE, "stopped!");
                //dbgServer(recvMsg1);
                dbgOutputLoc(APPSTOP);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_4);
                appState = STOP;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "run") == 0) {
                blink_led = false;
                snprintf(recvMsg1, MSG_BUF_SIZE, "started!");
                //dbgServer(recvMsg1);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_4);
                dbgOutputLoc(APPRUN);
                ledOn();
                appState = RUN;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "pause") == 0) {
                blink_led = true;
                snprintf(recvMsg1, MSG_BUF_SIZE, "paused!");
                //dbgServer(recvMsg1);
                dbgOutputLoc(APPPAUSE);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_3);
                PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_TIMER_4);

                appState = PAUSE;
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (strcmp(myMsgPtr, "done") == 0) {
                xLoc[ID] = xPath[ID]; //xLoc[ID] + speed[ID];
                yLoc[ID] = yPath[ID]; //yLoc[ID] - speed[ID];
                steps++;
                //                dbgUARTVal('S');
                //                dbgUARTVal('T');
                //                dbgUARTVal('E');
                //                dbgUARTVal('P');
                //                dbgUARTVal(steps + '0');
                appState = RUN;
                //snprintf(recvMsg1, MSG_BUF_SIZE, "d I think I am at {%d, %d}", xLoc[ID], yLoc[ID]);
                //dbgServer(recvMsg1);
                memset(myMsgPtr, 0, MSG_BUF_SIZE);
            }
            if (myMsgPtr[0] == 'M') {
                newMap = true;
                xCoord = (myMsgPtr[3] - '0') + (myMsgPtr[2] - '0')*10;
                yCoord = (myMsgPtr[6] - '0') + (myMsgPtr[5] - '0')*10;
                rType = (myMsgPtr[8] - '0');
                friendly = (myMsgPtr[10] - '0');
                if (rType == OBSTACLE) {
                    walkability [xCoord][yCoord].walkability = unwalkable;
                    walkability [xCoord][yCoord].rover = OBSTACLE;
                } else {
                    // dbgOutputLoc('!');
                    walkability [xCoord][yCoord].rover = rType;
                    if (rType == TAGGER && friendly == 0) {
                        if (oldGoalX != xCoord || oldGoalY != yCoord) {
                            walkability [xCoord][yCoord].walkability = walkable;
                            oldGoalX = xCoord;
                            oldGoalY = yCoord;
                            goalX = xCoord;
                            goalY = yCoord;
                            appState = INIT;
                        }
                    } else if (rType == CM && friendly == 1) {
                        if (oldX != xCoord || oldY != yCoord) {
                            oldX = xCoord;
                            oldY = yCoord;
                            xLoc[ID] = xCoord;
                            yLoc[ID] = yCoord;
                            appState = INIT;
                        }
                    } else
                        walkability [xCoord][yCoord].walkability = unwalkable;
                }
            }
            if (strcmp(myMsgPtr, "tapeb") == 0) {
                LATAINV = 0x8;
                DIRECTIONS dir = getOrientation(ID);
                int i, j;
                dbgUARTVal(xPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(xLoc[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yLoc[ID] + '0');
                dbgUARTVal(' ');
                walkability [xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                if (dir == SOUTH) {
                //                    
                //                    for (i = 0; i < mapWidth; i++)
                //                        for (j = yPath[ID]; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                        
                //                }
                //                else if (dir == WEST && xLoc[ID] <= 2){
                //                    for (i = 0; i <= xPath[ID]; i++)
                //                        for (j = mapHeight-1; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                       
                //                }
                //                else if (dir == EAST){
                //                    for (j = 0; j < yPath[ID]; j++){
                //                        walkability [xPath[ID]][j].walkability = unwalkable;
                //                    }
                //                }
                //                    
                //                else{
                //                    walkability[xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                }

                fullMap = true;
                appState = INIT;
            }
            if (strcmp(myMsgPtr, "tapel") == 0) {
                LATAINV = 0x8;
                DIRECTIONS dir = getOrientation(ID);
                int i, j;
                dbgUARTVal(xPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(xLoc[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yLoc[ID] + '0');
                dbgUARTVal(' ');
                walkability [xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                if (dir == SOUTHWEST) {
                //                      
                //                    for (i = 0; i < mapWidth; i++)
                //                        for (j = yPath[ID]; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                     
                //                }
                //                else if (dir == NORTHWEST && xLoc[ID] <= 2){
                //                    for (i = 0; i <= xPath[ID]; i++)
                //                        for (j = mapHeight-1; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                      
                //                }
                //                else if (dir == SOUTHEAST || dir == EAST){
                //                    for (j = 0; j < yPath[ID]; j++){
                //                        walkability [xPath[ID]][j].walkability = unwalkable;
                //                    }
                //                }
                //                else{
                //                    walkability[xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                }
                fullMap = true;
                appState = INIT;
            }
            if (strcmp(myMsgPtr, "taper") == 0) {
                LATAINV = 0x8;
                DIRECTIONS dir = getOrientation(ID);
                int i, j;
                dbgUARTVal(xPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yPath[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(xLoc[ID] + '0');
                dbgUARTVal(' ');
                dbgUARTVal(yLoc[ID] + '0');
                dbgUARTVal(' ');
                walkability [xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                if (dir == SOUTHEAST) {
                //                    
                //                    for (i = 0; i < mapWidth; i++)
                //                        for (j = yPath[ID]; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                      
                //                }
                //                else if (dir == SOUTHWEST && xLoc[ID] <= 2){
                //                    for (i = 0; i <= xPath[ID]; i++)
                //                        for (j = mapHeight-1; j >= 0; j--)
                //                            walkability [i][j].walkability = unwalkable;
                //                    
                //                }
                //                else if (dir == NORTHEAST || dir == EAST){
                //                    for (j = 0; j < yPath[ID]; j++){
                //                        walkability [xPath[ID]][j].walkability = unwalkable;
                //                    }
                //                }
                //                else{
                //                    walkability[xPath[ID]][yPath[ID]].walkability = unwalkable;
                //                }
                fullMap = true;
                appState = INIT;
            }


            memset(myMsgPtr, 0, MSG_BUF_SIZE);
        }


        //dbgOutputLoc(APPTASKS + 1);
        switch (appState) {
            case INIT:
                //LATAINV = 0x8;


                //pathStatus[ID] = FindPath(ID, xLoc[ID], yLoc[ID], goalX, goalY);
                //appState = RUN;
                //snprintf(recvMsg1, MSG_BUF_SIZE, "d Init!");
                //dbgServer(recvMsg1);
                /*dbgUARTVal('S');
                dbgUARTVal('T');
                dbgUARTVal('A');
                dbgUARTVal('R');
                dbgUARTVal('T');*/
                if (fullMap) {

                    if (goalX != -1 && goalY != -1) {
                        if (xLoc[ID] != -1 && yLoc[ID] != -1) {
                            EndPathfinder();
                            //dbgUARTVal('S');
                            dbgUARTVal('T');
                            dbgUARTVal('A');
                            dbgUARTVal('R');
                            dbgUARTVal('T');
                            //snprintf(recvMsg1, MSG_BUF_SIZE, "d Pathfinding!");
                            //dbgServer(recvMsg1);
                            pathStatus[ID] = FindPath(ID, xLoc[ID], yLoc[ID], goalX, goalY);
                            steps = 0;
                            //LATAINV = 0x8;
                            appState = RUN;
                            //snprintf(recvMsg1, MSG_BUF_SIZE, "d Finished Pathfinding!");
                            //dbgServer(recvMsg1);
                        } else {

                            //requestMap();
                            appState = WAIT;
                        }
                    } else {
                        //requestMap();
                        appState = WAIT;
                    }
                } else
                    appState = WAIT;

                break;
            case RUN:
                //int FindPath (int pathfinderID,int startingX, int startingY, int targetX, int targetY)
                //                for (i = steps; i == 0; i--){
                //                    int temp = steps % 10 * (steps - i);
                //                    if (temp < 10)
                //dbgUARTVal('-');
                //dbgUARTVal(steps);

                //}

                if (steps >= 10) {
                    appState = WAIT;
                    //requestMap();
                }//2.Move smiley.
                else if (pathStatus[ID] == found) {
                    dbgUARTVal('|');
                    dbgUARTVal(' ');
                    dbgUARTVal(xLoc[ID] + '0');
                    dbgUARTVal(' ');
                    dbgUARTVal(yLoc[ID] + '0');
                    dbgUARTVal(' ');
                    MoveSprite(ID);
                    appState = DRIVE;
                } else {
                    //dbgUARTVal('X');
                    //snprintf(recvMsg1, MSG_BUF_SIZE, "Can't find a path!");
                    //dbgServer(recvMsg1);
                    appState = WAIT;
                }

                if (xLoc[ID] == goalX && yLoc[ID] == goalY) {
                    appState = WAIT;
                }




                if (received) {
                    received = false;
                }
                break;
            case RESET:
                xLoc[ID] = 0;
                yLoc[ID] = 0;
                appState = INIT;
                break;
            case DRIVE:
                if (dbgCount > 500) {
                    //LATAINV = 0x8;
                    dbgCount = 0;
                }

                // Don't do anything until it's done moving
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
                if (maptime > 5000) {
                    //LATAINV = 0x8;
                    //                    dbgUARTVal('N');
                    //                    dbgUARTVal('E');
                    //                    dbgUARTVal('W');
                    //                    dbgUARTVal(' ');
                    //                    dbgUARTVal('M');
                    //                    dbgUARTVal('A');
                    //                    dbgUARTVal('P');
                    requestMap();
                    maptime = 0;
                }
                break;

            default:
                break;
        }
    }
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



/*******************************************************************************
 End of File
 */
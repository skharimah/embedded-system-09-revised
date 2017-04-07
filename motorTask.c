/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motortask.c

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

#include "motortask.h"

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

MOTORTASK_DATA motortaskData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTORTASK_Initialize ( void )

  Remarks:
    See prototype in motortask.h.
 */

void MOTORTASK_Initialize ( void )
{
    int i;
    /* Place the App state machine in its initial state. */
    motortaskData.state = MOTORTASK_STATE_INIT;
    dbgOutputLoc(10);
    encoderQueue = xQueueCreate(10, sizeof(MOTOR_MESSAGE));
    
    if (encoderQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }
    
    //reset button history
    for(i = 0; i < 10; i++)
        buttonHistory[i] = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

//Find turn direction function
int turnDirection(int current, int target) {
    int difference = current - target;
    if(difference < 0) {
        difference += 8;
    }
    if(difference > 3)
        return -1; //left turn
    else
        return 1; //right turn
}
/******************************************************************************
  Function:
    void MOTORTASK_Tasks ( void )

  Remarks:
    See prototype in motortask.h.
 */

void MOTORTASK_Tasks ( void )
{
    dbgOutputLoc(15);
    MOTOR_MESSAGE msgReceived;
    msgReceived.leftTicks = 0;
    msgReceived.rightTicks = 0;

    int leftTicks = 0;
    int rightTicks = 0;
    int leftTicksPrev = 0;
    int rightTicksPrev = 0;
    
    //PID Variables
    float kpl = .3;
    float kil = .2;
    float kdl = .01;
    
    float kpr = .25;
    float kir = .2;
    float kdr = .01;
    
    int targetSpeed = 600; 
    int leftMotorSpeed = 400;
    int rightMotorSpeed = 400;
    
    //UART output variables
    char tickChar[5];
    int i;
    
    //Motor turn variables
    int moving = 0;
    int leftCounter;
    int rightCounter;
    int dest = FORTYFIVE_DEG;
    int orientation = NORTH;
    int diff;
    
    //destination
    int turningPath = 0;
    int forwardPath = 0;
    int targetDist;
    int targetDir;
    
    //send to pathfinding variables
    char *pathMsgPtr = "";
    int goal = 0;
    
    //button history variables
    int buttonPressed;
    
    motorsInitialize();
    
    while(1) {
        //dbgOutputLoc(16);
        
        //Set previous encoder values
        leftTicksPrev = leftTicks;
        rightTicksPrev = rightTicks;
        
        //Check Queue for new message
        if(xQueueReceive(encoderQueue, &msgReceived, 0) == pdTRUE) {
            //change location of encoder value checks
            if(msgReceived.messageType == 'M' && moving == 0) {
                motortaskData.state = msgReceived.motorState;
                dest = msgReceived.dist;
            } 
            if(msgReceived.messageType == 'E') {
                if( (msgReceived.leftTicks != 0) && (msgReceived.leftTicks != 0))   {
                    leftTicks = msgReceived.leftTicks;
                    rightTicks = msgReceived.rightTicks;
                    dbgOutputLoc(19);

                    dbgOutputVal(rightTicks);

                    //Output ticks to UART
                    /*sprintf(tickChar, "L%d", leftTicks);
                    for(i = 0; tickChar[i] != NULL; i++)
                        dbgOutputVal(tickChar[i]);
                    dbgOutputVal(' ');
                    sprintf(tickChar, "R%d", rightTicks);
                    for(i = 0; tickChar[i] != NULL; i++)
                        dbgOutputVal(tickChar[i]);
                    dbgOutputVal('\n');*/      
                
                    //adjust speed
                    leftMotorSpeed = pidControl(leftMotorSpeed, rightTicks, targetSpeed, kpl, kil, kdl);
                    rightMotorSpeed = pidControl(rightMotorSpeed, leftTicks, targetSpeed, kpr, kir, kdr);
                    
                    
                    motortaskData.state = SEND_ENCODER_VALUES;
                }
            }
        }
        //Button Pressed
        buttonPressed = 1;
        for(i = 0; i < 10; i++) {
            if(buttonHistory[i] != 1)
                buttonPressed = 0;
        }
        if(buttonPressed == 1) {
            //LED ON (to do)
            PLIB_PORTS_PinWrite ( PORTS_ID_0, PORT_CHANNEL_F, 0, 1);
            
            //stop motors
            motorsStop();
            
            //stop moving
            moving = 0;
            
            //Wait for reset
            while(msgReceived.messageType != 'R') {
                if(xQueueReceive(encoderQueue, &msgReceived, 0) == pdTRUE) {
                    
                }
            }
            PLIB_PORTS_PinWrite ( PORTS_ID_0, PORT_CHANNEL_F, 0, 0);
        }
        
        
        //Handle move distance

        if(moving == 1) {
            motorsSetSpeed(leftMotorSpeed, rightMotorSpeed);
            leftCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            rightCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            //LATAINV = 0x8;
            if(leftCounter > dest && rightCounter > dest) {
                moving = 0;
                motortaskData.state = MOTOR_DO_NOTHING;
                }
        }
        
        if(turningPath == 1) {
            motorsSetSpeed(leftMotorSpeed, rightMotorSpeed);
            leftCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            rightCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            if(leftCounter > targetDir && rightCounter > targetDir) {
                forwardPath = 1;
                turningPath = 0;
                motorsForward(leftMotorSpeed, rightMotorSpeed);
                PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
            }
        }
        if(forwardPath == 1) {
            //motorsForward(leftMotorSpeed, rightMotorSpeed);
            motorsSetSpeed(leftMotorSpeed, rightMotorSpeed);
            leftCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            rightCounter = PLIB_TMR_Counter16BitGet(TMR_ID_3);
            if(leftCounter > targetDist && rightCounter > targetDist) {
                forwardPath = 0;
                motortaskData.state = MOTOR_SEND_TASK_COMPLETE;
            }
        }
        
        /* Check the application's current state. */
        switch ( motortaskData.state )
        {
            /* Application's initial state. */
            case MOTORTASK_STATE_INIT:
            {
                bool appInitialized = true;
                //Initialize encoder receive message
                
                if (appInitialized)
                {

                    motortaskData.state = MOTOR_STATE_IDLE;
                }
                break;
            }

            case MOTORTASK_STATE_SERVICE_TASKS:
            {
                dbgOutputLoc(17);
                break;
            }
            case MOTOR_DO_NOTHING:
            {
                dbgOutputLoc(130);

                //Stop motors
                motorsStop();
                
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
            
            case MOTOR_FORWARD:
            {
                PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
                moving = 1;
                dbgOutputLoc(131);
                motorsForward(leftMotorSpeed, rightMotorSpeed);
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
            
            case MOTOR_BACKWARD:
            {
                dbgOutputLoc(131);
                motorsBackward(leftMotorSpeed, rightMotorSpeed);
      
                break;
            }
            
            case MOTOR_PATH_FIND:
            {
                if(orientation != msgReceived.dir) {
                    //update turn distance
                    diff = abs(orientation - msgReceived.dir);
                    if(diff > 3)
                        diff = 8 - diff;
                    targetDir = diff * FORTYFIVE_DEG;
                    //update forward distance
                    targetDist = msgReceived.dist;
                    //reset counters
                    PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                    PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
                    //enable turn path
                    turningPath = 1;  
                    //Choose turn direction
                    if(turnDirection(orientation, msgReceived.dir) == 1)
                        motorsTurnRight(leftMotorSpeed, rightMotorSpeed);
                    else
                        motorsTurnLeft(leftMotorSpeed, rightMotorSpeed);

                    //Update current orientation
                    orientation = msgReceived.dir;
                    dbgOutputLoc(17);
                    
                    motortaskData.state = MOTOR_STATE_IDLE;
                }
                else {
                    //update target distance
                    targetDist = msgReceived.dist;
                    //reset counters
                    PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                    PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
                    //enable path forward
                    forwardPath = 1;
                    
                    dbgOutputLoc(131);
                    //Go to idle state
                    motorsForward(leftMotorSpeed, rightMotorSpeed);
                    motortaskData.state = MOTOR_STATE_IDLE;
                }
                break;
            }
            
            case MOTOR_TURN_LEFT:
            {
                PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
                dbgOutputLoc(132);
                moving = 1;
                motorsTurnLeft(leftMotorSpeed, rightMotorSpeed);
                
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
            
            case MOTOR_TURN_RIGHT:
            {
                PLIB_TMR_Counter16BitSet(TMR_ID_3, 0);
                PLIB_TMR_Counter16BitSet(TMR_ID_4, 0);
                dbgOutputLoc(132);
                moving = 1;
                motorsTurnRight(leftMotorSpeed, rightMotorSpeed);
                
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
                        
            case SEND_ENCODER_VALUES:
            {
                sprintf(encoderValMsg, "L%d R%d", leftTicks, rightTicks);
                msgToJSONMsgQ(&encoderValMsg);
                
                /*for (i = 0; encoderValMsg[i] != NULL; i++)
                    dbgUARTVal(encoderValMsg[i]);*/
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
            
            case MOTOR_SEND_TASK_COMPLETE:
            {
                //LATAINV = 0x8;
                strcpy(appMsg, "done");
                pathMsgPtr = &appMsg;
                messageToQ(appRecvQueue, pathMsgPtr);
                
                /*for (i = 0; encoderValMsg[i] != NULL; i++)
                    dbgUARTVal(encoderValMsg[i]);*/
                motorsStop();
                motortaskData.state = MOTOR_STATE_IDLE;
                break;
            }
            
            case MOTOR_STATE_IDLE:
            {
                
                break;
            }

            /* TODO: implement your application state machine.*/

            /* The default state should never be executed. */
            default:
            {
                /* TODO: Handle error in application's state machine. */
                break;
            }
        }
    }
}

 

/*******************************************************************************
 End of File
 */

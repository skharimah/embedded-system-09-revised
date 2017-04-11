/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mapgeneratortask.c

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

#include "mapgeneratortask.h"

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

MAPGENERATORTASK_DATA mapgeneratortaskData;

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
    void MAPGENERATORTASK_Initialize ( void )

  Remarks:
    See prototype in mapgeneratortask.h.
 */

void MAPGENERATORTASK_Initialize(void) {
    //DRV_ADC_Open(); //SEE IF THIS IS OKAY HERE
    /* Place the App state machine in its initial state. */
    mapgeneratortaskData.state = MAPGENERATORTASK_STATE_INIT;

    mapQueue = xQueueCreate(10, sizeof (char));

    if (mapQueue == NULL) {
        /* Wait indefinitely until the queue is successfully created */
    }

}

/******************************************************************************
  Function:
    void MAPGENERATORTASK_Tasks ( void )

  Remarks:
    See prototype in mapgeneratortask.h.
 */

void MAPGENERATORTASK_Tasks(void) {

    //create the message that contains map data
    //MAP_MESSAGE msg;
    MOTOR_MESSAGE msgtoMotors;
    char msgType;

    int cmMoved = 0;
    int sumOfReadings;


    //initialize the X and Y coordinate values to 0
    //msg.xCoordinate = 0;
    //msg.yCoordinate = 0;

    MAP_MESSAGE mapMessages[250];
    int readings[10];
    char buff[200];
    char bu[200];
    unsigned char out;

    int averageValue=0;
    int iterator=0;
    int j=0;
    int i=0;
    int k=0;
    bool forward= true;



    //how far I want to go
    //msgtoMotors.dist = 0;
   
    //if I receive a request.... send whatever I have in the char array
    while (1) {
        
        if(sensorData.lineArray1<100){
            //dbgUARTVal('A');
            
        }
        if(sensorData.lineArray2<100){
            
            //do something
        }
        if (xQueueReceive(mapQueue, &msgType, 0) == pdTRUE) {

            if (msgType == 'D') {

                //motors are done
                //dbgUARTVal('L');
                

                cmMoved++;
                mapgeneratortaskData.state = READ_STATE;

            }
            if (msgType == 'R') {
                //if it is a request for map data, then we want to send the obstacles back
                //dbgUARTVal('e');
                //LOOP THRU AND SEND EACH OBSTACLE
                int s;
                //this should send the number of obstacles that we have seen
                for (s = 0; s < totalObstacles; s++) {

                    //make sure I can actually do 
                    sprintf(mapValMsg, " X%3d Y%3d ", mapMessages[s].xCoordinate, mapMessages[s].yCoordinate);
                    /*for(i = 0; mapValMsg[i] != '\0'; i++){
                        dbgUARTVal(mapValMsg[i]);
                    }*/

                    msgToJSONMsgQ(&mapValMsg);

                }
                
                //WHAT STATE DO I GO TO NEXT?
                mapgeneratortaskData.state = SOME_STATE;
            }
            
        }


        /* Check the application's current state. */
        switch (mapgeneratortaskData.state) {
                /* Application's initial state. */
            case MAPGENERATORTASK_STATE_INIT:
            {
                //dbgUARTVal('i');
                bool appInitialized = true;
                //forward = true;
                yCoordinate = 0;

                //find tape/starting position??
                //get tape data if tape data < 100 stop

                if (appInitialized == true) {
                    if (sensorData.centimeterDistanceLower < 30 || sensorData.centimeterDistanceLower > 150) {

                        //invalid cannot see-- either too close or too far
                        //go to another state
                        //idle state
                        mapgeneratortaskData.state = SOME_STATE;

                    }

                    //sensorData.InterruptFlag = false;
                    mapgeneratortaskData.state = READ_STATE;

                }

                break;
            }

            case READ_STATE:
                //motorsStop();
                //dbgUARTVal('x');

                //this means that the upper sensor doesn't see anything and the lower sensor does see something
                if (((sensorData.centimeterDistanceUpper > 150 && sensorData.centimeterDistanceUpper>30) && (sensorData.centimeterDistanceLower < 150 &&
                        sensorData.centimeterDistanceLower > 30))) {
                    //this is JUST an obstacle
                    mapMessages[i].isObstacle = 1;



                }
                
                //this means that it is just a rover
                else if ((sensorData.centimeterDistanceUpper < 150 && sensorData.centimeterDistanceUpper > 30) && (sensorData.centimeterDistanceLower >150 || sensorData.centimeterDistanceLower <30)) {

                    mapMessages[i].isObstacle = 2;
                }                    
                else {
                    //this is empty space
                    mapMessages[i].isObstacle = 4;
                }
                
                
                //how to iterate through this to go through ten times???
                i++;
                readings[iterator] = sensorData.centimeterDistanceLower;
                iterator++;

                if (cmMoved >= 10) {

                    //dbgUARTVal('*');
                    cmMoved = 0;
                    mapgeneratortaskData.state = PROCESS_DATA_STATE;
                } else {
                    //dbgUARTVal('M');
                    //go back to read state to read more data
                    //CHANGE THIS BACK TO READ DATA STATE--FIGURE UP INTERRUPT LIKE THINGS THAT I SHOULD DO TO
                    //EVENTUALLY SEND A MESSAGE
                    mapgeneratortaskData.state = MOVE_STATE;

                }


                break;

                //this state does not do anything
            case SOME_STATE:
            {
                
                break;
            }

            case MOVE_STATE:
            {

                //dbgUARTVal('c');
                //1. Move forward 10 centimeters
                //2. Make the message type M for motor control
                //3. Make sure the motors know to move forward
                
                msgtoMotors.dist = 75;

                //we want motor values for motor control
                msgtoMotors.messageType = 'M';
                //snprintf(buff, BUFF_SIZE, "%d", mapMessages[i].xCoordinate);

                if (yCoordinate >= 20) {

                    forward = false;
                } else if (yCoordinate == 0) {
                    forward = true;
                }

                if (forward) {
                    //MOVE FORWARD
                    //change this BACK AFTER TEST BACKWARD
                    msgtoMotors.motorState = MOTOR_FORWARD;
                } else {

                    msgtoMotors.motorState = MOTOR_BACKWARD;

                }

                //send to the motor queue-- which controls motors
                if (xQueueSend(encoderQueue, &msgtoMotors, NULL) != pdTRUE) {
                    //send failed
                }

                //some state is good, but it is getting stuck?****
                mapgeneratortaskData.state = SOME_STATE;

                

                //if request-- go to send Message
                break;
            }

            case PROCESS_DATA_STATE:
            {

                //dbgUARTVal('d');
                //reset variables so it goes to 10
                //cmMoved = 0;
                int n;
                sumOfReadings = 0;
                averageValue = 0;
                iterator = 0;
                
                

                //take an average of the ten readings of the xCoordinate
                for (n = 0; n < 10; n++) {

                    sumOfReadings = sumOfReadings + readings[n];
                }

               
                //iterate through as you get to states?
                averageValue = sumOfReadings / 10;

                //decide if obstacle
                if(averageValue < 12){
                    
                    mapMessages[j].xCoordinate = averageValue;
                    
                }
                

                //increase the total number of obstacles
                totalObstacles++;


                mapMessages[j].yCoordinate = yCoordinate;


                //increase number of obstacles


                //we need to divide by 10 again to get the actual XCoordinate which is # of blocks of 10cm= 4inch


                //mapMessages[j].xCoordinate = averageValue;


                //increase Y coordinate by 1 to account for moving 1 block = 10cm = 4inches
                if (forward) {
                    
                    yCoordinate++;
               
                } else {

                    yCoordinate--;
                }


                j++;
                
                //some state is bad
                mapgeneratortaskData.state = MOVE_STATE;

                break;
            }
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

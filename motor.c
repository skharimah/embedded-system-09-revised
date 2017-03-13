/* ************************************************************************** */
/** Debug Header File
  @Company
    Embedded System Spring 2017 Team 9
  @File
    debug.h
  @Summary
    Prototypes of debugging functions implemented in debug.c
  @Description
    Declare the prototypes of functions implemented in debug.c
    Debugging tools that outputs to GPIO pins and UART
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "motor.h"


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

//Initialize oscillators and motors
void motorsInitialize(void) {
    
    //Start Oscillators
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    DRV_OC0_Start();
    DRV_OC1_Start();

    //Left Motor Direction Control
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1); //51 (pic32 pin 12)
    
    //Right Motor Direction Control
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 0); //49 (pic32 pin 71))
   
    //Set speed to 0
    PLIB_OC_PulseWidth16BitSet(0,0);
    PLIB_OC_PulseWidth16BitSet(1,0);
}

//The motor moves forward a specified amount of ticks ENCODERS CHANGED
/*
void motorsForwardDistance(int ticks) {
    rightMotorTicks = 0;
    leftMotorTicks = 0;
    while((rightMotorTicks < ticks) && (leftMotorTicks < ticks)) {
        dbgOutputVal(rightMotorTicks);
    }
    motorsStop();
    
}*/

//Set speed to number between 0 and 2500
void motorsSetSpeed(int speed) {
    //Change PWM to set speed
    PLIB_OC_PulseWidth16BitSet(0,speed);
    PLIB_OC_PulseWidth16BitSet(1,speed);
}

//Configure the motors to move forwards
void motorsForward(int speed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 1); //49
    
    //Set Motors to speed
    motorsSetSpeed(speed);
    
   // dbgOutputVal(leftMotorTicks);
}

//Configure the motors to move backwards 
void motorsBackward(int speed) {
    
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 0); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 0);
    
    //Set Motors to speed
    motorsSetSpeed(speed);
}

//Configure the motors to turn left
void motorsTurnLeft(int speed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 0); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 1);
    
    //Set Motors to speed
    motorsSetSpeed(speed);
}

//Configure the motors to turn right
void motorsTurnRight(int speed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 0);
    
    //Set Motors to speed
    motorsSetSpeed(speed);
}

//Stop motors
void motorsStop(void) {
    //Set speed to 0
    PLIB_OC_PulseWidth16BitSet(0,0);
    PLIB_OC_PulseWidth16BitSet(1,0);
}

//Milestone 2 turning demo
void motorsTurnDemo(int itterate) {
    switch(itterate) {
            case(0):
                dbgOutputVal(itterate);
                motorsTurnLeft(0);
                break;
            case(1):
                dbgOutputVal(itterate);
                motorsForward(0);
                break;
            case(2):
                dbgOutputVal(itterate);
                motorsTurnLeft(0);
                break;
            case(3):
                dbgOutputVal(itterate);
                motorsBackward(0);
                break;
            case(4):
                dbgOutputVal(itterate);
                motorsTurnRight(0);
                break;
            case(5):
                dbgOutputVal(itterate);
                motorsForward(0);
                break;
            default:
                while(1){dbgOutputVal(0xFF);}
        }
    
    
}

//milestone 2 speed demo
void motorsSpeedDemo(int itterate) {
    switch(itterate) {
            case(0):
                dbgOutputVal(itterate);
                motorsSetSpeed(0);
                break;
            case(1):
                dbgOutputVal(itterate);
                motorsSetSpeed(500);
                break;
            case(2):
                dbgOutputVal(itterate);
                motorsSetSpeed(1000);
                break;
            case(3):
                dbgOutputVal(itterate);
                motorsSetSpeed(2000);
                break;
            case(4):
                dbgOutputVal(itterate);
                motorsSetSpeed(2500);
                break;
            case(5):
                dbgOutputVal(itterate);
                motorsSetSpeed(1000);
                break;
            default:
                while(1){dbgOutputVal(0xFF);}
        }
}
/* *****************************************************************************
 End of File
 */

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
void motorsSetSpeed(int leftSpeed, int rightSpeed) {
    //Change PWM to set speed
    PLIB_OC_PulseWidth16BitSet(0,leftSpeed);
    PLIB_OC_PulseWidth16BitSet(1,rightSpeed);
}

//Configure the motors to move forwards
void motorsBackward(int leftSpeed, int rightSpeed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 1); //49
    
    //Set Motors to speed
    motorsSetSpeed(leftSpeed, rightSpeed);
    
   // dbgOutputVal(leftMotorTicks);
}

//Configure the motors to move backwards 
void motorsForward(int leftSpeed, int rightSpeed) {
    
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 0); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 0);
    
    //Set Motors to speed
    motorsSetSpeed(leftSpeed, rightSpeed);
}

//Configure the motors to turn left
void motorsTurnRight(int leftSpeed, int rightSpeed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 0); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 1);
    
    //Set Motors to speed
    motorsSetSpeed(leftSpeed, rightSpeed);
}

//Configure the motors to turn right
void motorsTurnLeft(int leftSpeed, int rightSpeed) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1); //51
    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 0);
    
    //Set Motors to speed
    motorsSetSpeed(leftSpeed, rightSpeed);
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
                motorsTurnLeft(0, 0);
                break;
            case(1):
                dbgOutputVal(itterate);
                motorsForward(0, 0);
                break;
            case(2):
                dbgOutputVal(itterate);
                motorsTurnLeft(0, 0);
                break;
            case(3):
                dbgOutputVal(itterate);
                motorsBackward(0, 0);
                break;
            case(4):
                dbgOutputVal(itterate);
                motorsTurnRight(0, 0);
                break;
            case(5):
                dbgOutputVal(itterate);
                motorsForward(0, 0);
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
                motorsSetSpeed(0, 0);
                break;
            case(1):
                dbgOutputVal(itterate);
                motorsSetSpeed(500, 500);
                break;
            case(2):
                dbgOutputVal(itterate);
                motorsSetSpeed(1000, 1000);
                break;
            case(3):
                dbgOutputVal(itterate);
                motorsSetSpeed(2000, 2000);
                break;
            case(4):
                dbgOutputVal(itterate);
                motorsSetSpeed(2500, 2500);
                break;
            case(5):
                dbgOutputVal(itterate);
                motorsSetSpeed(1000, 1000);
                break;
            default:
                while(1){dbgOutputVal(0xFF);}
        }
}

int pidControl(int motorSpeed, int ticks, int targetSpeed, float kp, float ki, float kd)    {
    int processSpeed;
    static int lastError;
    int error;
    int errorDir;
    int proportion;
    int integral;
    int derivative;
    
    if(ticks < 20)
        processSpeed = ticks * 15;
    else if((ticks >= 20) && (ticks < 35))
        processSpeed = ticks * 13;
    else if((ticks >= 35) && (ticks < 45))
        processSpeed = ticks * 15;
    else if(ticks >= 45)
        processSpeed = ticks * 20;
               
    if (targetSpeed > processSpeed) {
        error = targetSpeed - processSpeed;
        errorDir = 1;
    }
    else {
        error = processSpeed - targetSpeed;
        errorDir = 0;
    }
                
    proportion = kp * error;
    integral = integral + error;
    derivative = error - lastError;
    
    if(errorDir) {
        //dbgOutputVal(77);
        motorSpeed = motorSpeed + (proportion + (ki * integral) + (kd * derivative));
    }
    else
        motorSpeed = motorSpeed - (proportion + (ki * integral) + (kd * derivative));
                
    if(motorSpeed > 1000) {
        motorSpeed = 1000;
    }
    if(motorSpeed < 300) {
        motorSpeed = 300;
    }
    
    lastError = error;
    
    return motorSpeed;
}

/*MOTOR_MESSAGE figureEightDemo(int itterate) {
    MOTOR_MESSAGE msg;
    msg.messageType = 'M';
    switch(itterate) {
            case(0):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(1):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_LEFT;
                break;
            case(2):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(3):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_LEFT;
                break;
            case(4):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(5):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_LEFT;
                break;
            case(6):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(7):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_LEFT;
                break;
            case(8):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(9):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_RIGHT;
                break;
            case(10):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(11):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_RIGHT;
                break;
            case(12):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(13):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_RIGHT;
                break;
            case(14):
                msg.dist = 1000;
                msg.motorState = MOTOR_FORWARD;
                break;
            case(15):
                msg.dist = NINTY_DEG;
                msg.motorState = MOTOR_TURN_RIGHT;
                break;
            default:
                //while(1){dbgOutputVal(0xFF);}
                break;
        }
    return msg;
    
    
}
/* *****************************************************************************
 End of File
 */
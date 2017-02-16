/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "app.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************
    
    //Initialize oscillators and motors
    void motorsInitialize();

    //Move motor a set distance
    void motorsForwardDirection(uint16_t ticks);
    
    void testFunc(int ticks);
    
    //Configure the motors to move forwards
    void motorsForward();
    
    //Configure the motors to move backwards
    void motorsBackward();
    
    //Configure the motors to turn left
    void motorsTurnRight();
    
    //Configure the motors to turn right
    void motorsTurnLeft();
    
    //Configure the motors to stop
    void motorsStop();
    
    //Set motors to specific speed
    void motorsSpeed(int speed);
    
    //Milestone 2 motor control demo
    void motorsTurnDemo(int itterate);
    
    //Milestone 2 motor speed control demo
    void motorsSpeedDemo(int itterate);
/* *****************************************************************************
 End of File
 */

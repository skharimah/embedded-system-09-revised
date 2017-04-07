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

#ifndef _DEBUG_H    /* Guard against multiple inclusion */
#define _DEBUG_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif  
    
#include <stdbool.h>
/* Pin Definitions */
#define PIN0_PORT PORT_CHANNEL_E
#define PIN1_PORT PORT_CHANNEL_E
#define PIN2_PORT PORT_CHANNEL_E
#define PIN3_PORT PORT_CHANNEL_E
#define PIN4_PORT PORT_CHANNEL_E
#define PIN5_PORT PORT_CHANNEL_E
#define PIN6_PORT PORT_CHANNEL_E
#define PIN7_PORT PORT_CHANNEL_E
#define PIN9_PORT PORT_CHANNEL_A
#define PIN10_PORT PORT_CHANNEL_F
#define PIN11_PORT PORT_CHANNEL_D
#define PIN12_PORT PORT_CHANNEL_F
#define PIN13_PORT PORT_CHANNEL_D
#define PIN14_PORT PORT_CHANNEL_B
#define PIN15_PORT PORT_CHANNEL_G
#define PIN16_PORT PORT_CHANNEL_G

#define PIN0 0      //RE0  ----  93
#define PIN1 1      //RE1  ----  94
#define PIN2 2      //RE2  ----  98
#define PIN3 3      //RE3  ----  99
#define PIN4 4      //RE4  ----  100
#define PIN5 5      //RE5  ----  3
#define PIN6 6      //RE6  ----  4
#define PIN7 7      //RE7  ----  5
#define PIN9 10     //RA10 ----  29
#define PIN10 1     //RF1  ----  88
#define PIN11 10    //RD10 ----  70
#define PIN12 3     //RF3  ----  51
#define PIN13 5     //RD5  ----  39
#define PIN14 11    //RB11 ----  35
#define PIN15 15    //RG15 ----  1
#define PIN16 7     //RG7  ----  29
    
//DEBUG LOCATIONS
#define TMR_START 13
#define TMR_STOP 14
#define UART_START 145
#define UART_STOP 146
#define WIFLY_RECV 111
#define WIFLY_TRANS 121
#define WIFLY_ERROR 131
#define STRING_START 20
#define STRING_STOP 21
#define APPTASKS 1 
#define APPPAUSE 9
#define APPSTOP 8
#define APPRUN 7
#define APPRECVMSG 6
#define LEDBLINK 211
#define LEDON 210
#define LEDOFF 209    
    
    
    
    bool blink_led;

typedef enum {
    /* Describe structure member. */
    MSG_QUEUE_DOES_NOT_EXIST = 1,
            MSG_QUEUE_IS_FULL
} Error;

/*******************************************************************************
  Function:
    int dbgOutputVal(unsigned char)
  Summary:
    Write a char to the PICMax32 GPIO pins.
  Description:
    This function writes an unsigned char value to the GPIO pins.
  Parameters:
    Unsigned char to be outputted to GPIO.
  Returns:
    Returns a 1 if successfully outputs an unsigned char to GPIO. 0 otherwise.
*/

int dbgOutputVal(unsigned char val);

/*******************************************************************************
  Function:
    int dbgUARTVal(unsigned char)
  Summary:
    Write a char to the UART via USB.
  Description:
    This function writes an unsigned char value to the UART.
  Parameters:
    Unsigned char to be outputted to UART.
  Returns:
    Returns a 1 if successfully outputs an unsigned char to UART. 0 otherwise.
*/

int dbgUARTVal(unsigned char val);

/*******************************************************************************
  Function:
    int dbgOutputLoc(unsigned char)
  Summary:
    Write a location number to a specified set of pins
  Description:
    This function writes a location number to a specified set of pins
  Parameters:
    Unsigned char to be outputted to specified pins
  Returns:
    Returns a 1 if successfully outputs an unsigned char to UART. 0 otherwise.
*/

int dbgOutputLoc(unsigned char val);

void ledOn();
void ledOff();


/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
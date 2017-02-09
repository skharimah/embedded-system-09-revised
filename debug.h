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
/* Pin Definitions */
#define PIN0_PORT PORT_CHANNEL_E
#define PIN1_PORT PORT_CHANNEL_E
#define PIN2_PORT PORT_CHANNEL_E
#define PIN3_PORT PORT_CHANNEL_E
#define PIN4_PORT PORT_CHANNEL_E
#define PIN5_PORT PORT_CHANNEL_E
#define PIN6_PORT PORT_CHANNEL_E
#define PIN7_PORT PORT_CHANNEL_E
#define PIN9_PORT PORT_CHANNEL_C
#define PIN10_PORT PORT_CHANNEL_C
#define PIN11_PORT PORT_CHANNEL_D
#define PIN12_PORT PORT_CHANNEL_F
#define PIN13_PORT PORT_CHANNEL_D
#define PIN14_PORT PORT_CHANNEL_B
#define PIN15_PORT PORT_CHANNEL_G
#define PIN16_PORT PORT_CHANNEL_G

#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7
#define PIN9 2
#define PIN10 3
#define PIN11 10
#define PIN12 3
#define PIN13 5
#define PIN14 11
#define PIN15 15
#define PIN16 7
    
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


/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
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

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"
#include "app_public.h"
#include "motor.h"

#include "peripheral/usart/plib_usart.h"
#include "peripheral/devcon/plib_devcon.h"
#include "system/debug/sys_debug.h"
//#include "driver/usart/src/drv_usart_static_local.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
    
    typedef enum{
        T = 0,
        E,
                A,
                M,
                FIRSTSPACE,
                NINE,
                SECONDSPACE
    }DBG_POS;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Flag to indicate an interrupt has occured */
	bool InterruptFlag;

    /* Pointer to hold the present character of string to be transmitted */
    const char *stringPointer;

    /* Data received from UART */
    char data;

} APP_DATA;



/* Driver objects.

  Summary:
    Holds driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.

  Remarks:
    None.
*/

typedef struct
{
	//SYS_MODULE_OBJ   drvObject;
	 
} APP_DRV_OBJECTS;

//Task Stack Sizes
#define MOTOR_STACK_SIZE 100
// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    QueueHandle_t createQueue(void)

  Summary:
    Create a message queue at a fixed size and type.

  Description:
    This function will create a message queue capable of containing 10 unsigned 
    integer values.

  Parameters:
    None.

  Returns:
    A message queue handler of type QueueHandle_t.

*/

QueueHandle_t createQueue(void);
/*******************************************************************************
  Function:
    int messageToQISR(QueueHandle_t queue, Message msg)

  Summary:
    Send a message to a previously created message queue

  Description:
    This function will send a Message struct to a message queue, and is safe for
    use within an ISR

  Parameters:
    QueueHandle_t queue: The handle for the queue you want to write to
    Message msg: The message to write to the queue

  Returns:
    An integer 0 for success, -1 for failure.

*/
int messageToQISR(QueueHandle_t queue, Message msg);
/*******************************************************************************
  Function:
    int messageToQ(QueueHandle_t queue, Message msg)

  Summary:
    Send a message to a previously created message queue

  Description:
    This function will send a Message struct to a message queue, but is NOT safe
    for use within an ISR.

  Parameters:
    QueueHandle_t queue: The handle for the queue you want to write to
    Message msg: The message to write to the queue

  Returns:
    An integer 0 for success, -1 for failure.

*/
int messageToQ(QueueHandle_t queue, Message msg);
/*******************************************************************************
  Function:
    void intLenToChar(Message msg, char *len)

  Summary:
    Send a message to a previously created message queue

  Description:
    This function will send a Message struct to a message queue, but is NOT safe
    for use within an ISR.

  Parameters:
    QueueHandle_t queue: The handle for the queue you want to write to
    Message msg: The message to write to the queue

  Returns:
    An integer 0 for success, -1 for failure.

*/
void intLenToChar(Message msg, char *len);
void checksum(Message msg, char *sum);
QueueHandle_t createEncoderQueue(void);
int getMsgFromRecvQ(Message *msg);
/*******************************************************************************
  Function:
    unsigned char receiveFromQueue(QueueHandle_t queue);

  Summary:
    Receive from the specified queue.

  Description:
    This function receives from the queue.
    Additional functions to carry out specific instructions may be needed.

  Parameters:
    Queue handle for the queue this function is to read from.

  Returns:
    Returns the char item received from the queue
    If no item is successfully retrieved it will return unsigned char '0'.
*/

unsigned char receiveFromQueue(QueueHandle_t queue);

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern APP_DRV_OBJECTS appDrvObject;

extern APP_DATA appData;


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

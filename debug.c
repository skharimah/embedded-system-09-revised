/* ************************************************************************** */
/** Debug Source File
 
  @Company
    Embedded System Spring 2017 Team 9
 
  @File
    debug.c
 
  @Summary
    Implementation of function prototypes in debug.h
 
  @Description
    Implement the functions declared in debug.h
    Debugging tools that outputs to GPIO pins and UART
 
 */
/* ************************************************************************** */

#include "app.h"
#include "debug.h"



/*******************************************************************************
  Function:
    int dbgOutputVal(unsigned char)

  Remarks:
    See prototype in debug.h.
 */
/*
int dbgOutputVal(unsigned char outputVal){
    
    //Set I/O pins 30-37
    PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN9_PORT, PIN9, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN10_PORT, PIN10, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN11_PORT, PIN11, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN12_PORT, PIN12, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN13_PORT, PIN13, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN14_PORT, PIN14, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN15_PORT, PIN15, 0);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN16_PORT, PIN16, 0);
        

        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN9_PORT, PIN9, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN10_PORT, PIN10, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN11_PORT, PIN11, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN12_PORT, PIN12, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN13_PORT, PIN13, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN14_PORT, PIN14, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN15_PORT, PIN15, 1);
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN16_PORT, PIN16, 1);

    
    
    
    
    if (outputVal & 1<<0)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN9_PORT, PIN9, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN9_PORT, PIN9, 0);
    
    if (outputVal & 1<<1)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN10_PORT, PIN10, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN10_PORT, PIN10, 0);
        
    if (outputVal & 1<<2)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN11_PORT, PIN11, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN11_PORT, PIN11, 0);
    
    if (outputVal & 1<<3)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN12_PORT, PIN12, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN12_PORT, PIN12, 0);
    
    if (outputVal & 1<<4)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN13_PORT, PIN13, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN13_PORT, PIN13, 0);
    
    if (outputVal & 1<<5)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN14_PORT, PIN14, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN14_PORT, PIN14, 0);
    
    if (outputVal & 1<<6)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN15_PORT, PIN15, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN15_PORT, PIN15, 0);
    
    if (outputVal & 1<<7)
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN16_PORT, PIN16, 1);
    else
        PLIB_PORTS_PinWrite ( PORTS_ID_0, PIN16_PORT, PIN16, 0);  
    return 0;
}
*/

/*******************************************************************************
  Function:
    int dbgUARTVal(unsigned char)
 
  Remarks:
    See prototype in debug.h.
 */
int dbgUARTVal(unsigned char val){
    
    if (!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(usbHandle)) ){
        
        DRV_USART_WriteByte(usbHandle, val);
        return 0;
    }
    return 1;
}



/*******************************************************************************
  Function:
    int dbgOutputLoc(unsigned char)

  Remarks:
    See prototype in debug.h.
 */
/*
int dbgOutputLoc(unsigned char outputVal){
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  0);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  1);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  outputVal);
    
    
    
        return 0;

        
    
    
    
}
*/

/* *****************************************************************************
 End of File
 */
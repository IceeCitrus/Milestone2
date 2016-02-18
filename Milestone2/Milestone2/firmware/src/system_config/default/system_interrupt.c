/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "debugging_task.h"
#include "messaging_task.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void IntHandlerDrvAdc(void)
{
    DRV_ADC_Stop();
    unsigned char sensorRead;
    unsigned char temp;

//debugCharFromISR(start_ADC_ISR);

    //Check to see if samples are available
    if (DRV_ADC_SamplesAvailable()) {   

        //Read from the sensor
        sensorRead = DRV_ADC_SamplesRead(0);      
        temp = sensorRead/25.6;
        //debugCharFromISR(0xEE);
        //debugCharFromISR(temp);
        //Send the data to the Sensor Queue
        sensor1SendSensorValToSensorQ(temp);
    }
    /* Clear ADC Interrupt Flag */

//debugCharFromISR(end_ADC_ISR);  
//stopEverything();
    //PLIB_ADC_SampleAutoStartDisable(DRV_ADC_ID_1);
    DRV_ADC_Stop();
    PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_ADC_1);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}




void IntHandlerDrvUsartInstance0(void)
{
    //Check for any USART data
    /* Clear pending interrupt */
    if(!isQueueEmpty())
    {
        unsigned char data = 0x00;
        data = messageQ();
debugCharFromISR(before_transmitting_USART);
        PLIB_USART_TransmitterByteSend(USART_ID_1, data);
debugCharFromISR(after_transmitting_USART);
    }    
    else//(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1));
    {
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
    {
        //stopEverything();
        unsigned char byte = 0x00; 
        debugCharFromISR(before_recv_USART);
        byte= PLIB_USART_ReceiverByteReceive(USART_ID_1);
//        sendByteToWIFLY(byte);
        ReceiveUSARTMsgFromMsgQ(byte);
        debugCharFromISR(after_recv_USART);
//        PLIB_USART_TransmitterByteSend(USART_ID_1, byte);
        //unsigned char byte = 0x00; 
        //byte= PLIB_USART_ReceiverByteReceive(USART_ID_1);
    }
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
}

 
/*******************************************************************************
 End of File
*/


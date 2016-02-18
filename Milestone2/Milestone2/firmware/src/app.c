/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "app_public.h"
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

APP_DATA appData;
MSG_FORMAT msgFormat;
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
 
//Puts an unsigned int in milliseconds into the queue
//Returns 0 if successful and 1 if failed
int sensor1SendTimerValToMsgQ(unsigned int millisecondsElapsed)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( appData.local_q, &millisecondsElapsed,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
}

void sensor1SendSensorValToSensorQ(unsigned char sensorValue)
{
#ifdef MACRO_DEBUG
    debugChar(before_pushing_to_queue);
#endif 
    //debugChar(sensorValue);
    xQueueSendFromISR( appData.sensor1_q, &sensorValue,
                                   NULL );
#ifdef MACRO_DEBUG
    debugChar(after_pushing_to_queue);
#endif
}

unsigned char sensor1ReceiveVal()
{
    unsigned char sensorRead;
    BaseType_t sensorReceived;
    sensorReceived = xQueueReceive(appData.sensor1_q , &sensorRead, portMAX_DELAY);
    //stopEverything();
//    debugChar(sensorRead);
    //If not received, stop and turn on LED.
    if(sensorReceived == pdFALSE)
    {
        stopEverything();
    }
    return sensorRead;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Initialize ( void )
{
    //stopEverything();
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    appData.sensor1_q = xQueueCreate(100, sizeof(unsigned char));
    if(appData.sensor1_q == 0)
    {
        stopEverything();
    }
    
    //Setup AD Driver
    SYS_INT_SourceEnable(INT_SOURCE_ADC_1);
    DRV_ADC_Initialize();
    DRV_ADC_Open();
    DRV_ADC_ChannelScanInputsAdd(ADC_INPUT_SCAN_AN0 | ADC_INPUT_SCAN_AN1|ADC_INPUT_SCAN_AN2);
    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);
    DRV_ADC_Start();
    
    step = 0;
    step2 = 0;
    old_s1msg = 0;
    /* Initialization is done, allow the state machine to continue */
    appData.state = APP_STATE_OUTPUT;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks ( void )
{
   /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            break;
        }

        /* The running state. Get value from the queue and output character
         * if necessary */
        case APP_STATE_OUTPUT:
        {
            
            //Receive Information from the Queue
#ifdef MACRO_DEBUG
debugChar(start_of_app_task);
#endif 

//            debugChar(sensorRead);
//            stopEverything();
#ifdef MACRO_DEBUG
debugChar(before_receiving_sensordata);
#endif
            //If not received, stop and turn on LED.
            //if(sensorReceived == pdFALSE)
            //{
            //    stopEverything();
            //}
            //if(ms % 100 == 0)
            //{
            //    PLIB_ADC_SampleAutoStartEnable(DRV_ADC_ID_1);
            //}
            
            //SendUSARTMsgToMsgQ("My name is Andrew");
            unsigned char s1msg;
            
//            stopEverything();
            s1msg = sensor1ReceiveVal();
            if(old_s1msg == s1msg)
            {
                step++;
            }
            else
            {
                //debugChar(0xE1);
                old_s1msg = s1msg;
                step = 0;
            }
            /*
            if(s1msg == 0)
            {
                step++;
            }*/
            if(step >= 200)
            {
                step = 0;
                unsigned char buffer[10] = {0x81,0x10,0x00,0x00,0x00,s1msg,0xD2,0xD3,0xD4,0x88};
                debugChar(s1msg);
                //sendMsgToWIFLY(buffer);
            }/*
            if(s1msg >= 6)
            {
                step2++;
            }
            if(step2 >= 100)
            {
                step2 = 0;
                unsigned char buffer[10] = {0x81,0x10,0x00,0x00,0x00,s1msg,0xD2,0xD3,0xD4,0x88};
                debugChar(0xEA);
                //sendMsgToWIFLY(buffer);
            }*/
//            debugChar(0x00);
            //debugBuffer(buffer, 10);
            //Once the Task runs once, restart the ISR to read values
            
            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1);
            
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


/*******************************************************************************
 End of File
 */
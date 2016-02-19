/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_timer.c

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

#include "app_timer.h"

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

APP_TIMER_DATA app_timerData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/
//Puts an unsigned int in milliseconds into the queue
//Returns 0 if successful and 1 if failed
int SendTimerValToMsgQ(unsigned int millisecondsElapsed)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( app_timerData.local_q, &millisecondsElapsed,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
}

int sensor1ValToMsgQ(unsigned char data)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( app_timerData.data_q, &data,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
}
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
    void APP_TIMER_Initialize ( void )

  Remarks:
    See prototype in app_timer.h.
 */

void APP_TIMER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_timerData.state = APP_TIMER_STATE_INIT;
        
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //Create the queue
    app_timerData.local_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(app_timerData.local_q == 0)
    {
        stopEverything();
    }
    //Create the timer
    app_timerData.local_timer = xTimerCreate( "50msTimer",
                50 / portTICK_PERIOD_MS,
                pdTRUE,
                0,
                vTimerCallback );
    
    //Ensure timer was created. If not, do not continue and turn on LED
    if(app_timerData.local_timer == 0)
    {
        stopEverything();
    }
    BaseType_t started = xTimerStart(app_timerData.local_timer, 0);
    
    app_timerData.data_q = xQueueCreate(10, sizeof(unsigned char));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(app_timerData.data_q == 0)
    {
        stopEverything();
    }
    //Ensure the timer started successfully. If not, do not continue and turn
    // on LED
    if(started == pdFAIL)
    {
        stopEverything();
    }
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    app_timerData.obstacle = 0;
    app_timerData.step2 = 0;
        /* Initialization is done, allow the state machine to continue */
    app_timerData.state = APP_STATE_OUTPUT;
}


/******************************************************************************
  Function:
    void APP_TIMER_Tasks ( void )

  Remarks:
    See prototype in app_timer.h.
 */

void APP_TIMER_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( app_timerData.state )
    {
        /* Application's initial state. */
        case APP_TIMER_STATE_INIT:
        {
            break;
        }
        case APP_TIMER_STATE_OUTPUT:
        {
            //Number of elapsed ms.
            unsigned int ms;
            BaseType_t received = xQueueReceive(app_timerData.local_q , &ms, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            unsigned char data;
            BaseType_t sensorreceived = xQueueReceive(app_timerData.data_q , &data, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(sensorreceived == pdFALSE)
            {
                stopEverything();
            }
//            debugChar(data);
            //If ms is a multiple of 100
            if(data == 0) 
            {
                if(ms % 100 == 0)
                {
                    app_timerData.step2++;
                    if(app_timerData.step2 == 1) // 0x0A LR orientation
                    {
                        unsigned char buffer[10] = {0x81,'C',0x40,0x01,0x00,0x0A,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x01,0x00,0x0A,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','L','R','O','R','I','E','N',0x88};
//                        debugChar(0x0A);       
//                        sendMsgToWIFLY(buffer);
//                        sendMsgToWIFLY(bufferM,10);
                    }
                    if(app_timerData.step2 == 2) // 0x0B LR Coordinate
                    {
                        unsigned char buffer[10] = {0x81,'C',0x40,0x02,0x00,0x0B,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x02,0x00,0x0B,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','L','R','C','O','O','R','D',0x88};
//                        debugChar(0x0B);       
//                        sendMsgToWIFLY(buffer);
//                        sendMsgToWIFLY(bufferM,10);
                    }
                    if(app_timerData.step2 == 3) // 0x0C FR orientation
                    {
                        unsigned char buffer[10] = {0x81,'C',0x40,0x03,0x00,0x0C,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x03,0x00,0x0C,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','F','R','O','R','I','E','N',0x88};
//                        debugChar(0x0C);       
//                        sendMsgToWIFLY(buffer);
//                        sendMsgToWIFLY(bufferM,10);
                    }
                    if(app_timerData.step2 == 4 && app_timerData.obstacle == 1) // 0x0D FR Coordinate
                    {
                        app_timerData.step2 = 0;
                        unsigned char buffer[10] = {0x81,'C',0x40,0x04,0x00,0x0D,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x04,0x00,0x0D,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','F','R','C','O','O','R','D',0x88};
//                        debugChar(0x0D);       
//                        sendMsgToWIFLY(buffer);
//                        sendMsgToWIFLY(bufferM,10);
                    }
                    if(app_timerData.step2 == 4 && app_timerData.obstacle == 0) // 0x0E Obstacle Coordinate
                    {
                        app_timerData.obstacle = 1;
                        app_timerData.step2 = 0;
                        unsigned char buffer[10] = {0x81,'C',0x40,0x05,0x00,0x0E,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x05,0x00,0x0D,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','O','B','S','T','A','C','L',0x88};
//                        debugChar(0x0E);       
//                        sendMsgToWIFLY(buffer);
//                        sendMsgToWIFLY(bufferM,10);
                    }
                }
                if(ms % 10000 == 0)
                {
                    if(app_timerData.obstacle == 1)
                    {
                        unsigned char buffer[10] = {0x81,'C',0x40,0x04,0x00,0x0E,0xD2,0xD3,0xD4,0x88};
//                        unsigned char bufferM[10] = {0x81,'M','M',0x04,0x00,0x0E,0xD2,0xD3,0xD4,0x88};
                        unsigned char bufferM[10] = {0x81,'M','O','B','S','T','A','C','L',0x88};
//                        debugChar(0x0F);
//                        sendMsgToWIFLY(buffer);
                        sendMsgToWIFLY(bufferM,10);
                    }
                }
            }
            break;
        }
        /* TODO: implement your application state machine.*/

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

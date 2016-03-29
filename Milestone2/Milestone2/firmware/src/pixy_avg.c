/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pixy_avg.c

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

#include "pixy_avg.h"
#include "pixy_avg_public.h"
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

PIXY_AVG_DATA pixy_avgData;
PIXY_AVG pixyAvg;
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

void sendObstacleAvg(PIXY_AVG data)
{
    xQueueSendToBack(pixy_avgData.obstacle_q, &data, portMAX_DELAY);
    //debugChar(0xC6);
}
void sendLeadFrontAvg(PIXY_AVG data)
{
    debugChar(0x6C);
    xQueueSendToBack(pixy_avgData.leadFront_q, &data, portMAX_DELAY);
    debugChar(0x6D);
}
void sendFollowerFrontAvg(PIXY_AVG data)
{
    debugChar(0x6E);
    xQueueSendToBack(pixy_avgData.followerFront_q, &data, portMAX_DELAY);
    debugChar(0x6F);
}
void sendBorderAvg(PIXY_AVG data)
{
    xQueueSendToBack(pixy_avgData.border_q, &data, portMAX_DELAY);
}
PIXY_AVG readObstacleAvg()
{
    PIXY_AVG pixyRead;
    BaseType_t pixyReceived;
    pixyReceived = xQueueReceive(pixy_avgData.obstacle_q , &pixyRead, portMAX_DELAY);
    //If not received, stop and turn on LED.
    if(pixyReceived == pdFALSE)
    {
        stopEverything();
    }
    return pixyRead; 
}
PIXY_AVG readLeadFrontAvg()
{
    PIXY_AVG pixyRead;
    BaseType_t pixyReceived;
    debugChar(0x6E);
    pixyReceived = xQueueReceive(pixy_avgData.leadFront_q , &pixyRead, portMAX_DELAY);
    debugChar(0x6F);
    //stopEverything();
    //If not received, stop and turn on LED.
    if(pixyReceived == pdFALSE)
    {
        stopEverything();
    }
    return pixyRead;
}
PIXY_AVG readFollowerFrontAvg()
{
    PIXY_AVG pixyRead;
    BaseType_t pixyReceived;
    debugChar(0x7E);
    pixyReceived = xQueueReceive(pixy_avgData.followerFront_q , &pixyRead, portMAX_DELAY);
    debugChar(0x7F);
    //stopEverything();
    //If not received, stop and turn on LED.
    if(pixyReceived == pdFALSE)
    {
        stopEverything();
    }
    return pixyRead;
}
PIXY_AVG readBorderAvg()
{
    PIXY_AVG pixyRead;
    BaseType_t pixyReceived;
    pixyReceived = xQueueReceive(pixy_avgData.border_q , &pixyRead, portMAX_DELAY);
    //stopEverything();
    //If not received, stop and turn on LED.
    if(pixyReceived == pdFALSE)
    {
        stopEverything();
    }
    return pixyRead;
}
/*
void borderAvg()
{
        PIXY_DATA pixyData;
        debugChar(0xE1);
        while(objects != 5) 
        {
            pixyData = readBorderData();
            //debugChar(0xE2);
            unsigned short xtemp;
            unsigned short ytemp;
            unsigned short widthtemp;
            unsigned short heighttemp;
            xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
            ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
            widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
            heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
            if(objects == 0)
            {
                xcoord1 = xtemp;   
                ycoord1 = ytemp;
                width1 = widthtemp;
                height1 = heighttemp;
                objects++;                          
            }
            else if(objects == 1 )
            {
                xcoord2 = xtemp;   
                ycoord2 = ytemp;
                width2 = widthtemp;
                height2 = heighttemp;
                objects++;
            }
            else if(objects == 2 )
            {
                xcoord3 = xtemp;   
                ycoord3 = ytemp;
                width3 = widthtemp;
                height3 = heighttemp;
                objects++;
            }
            else if(objects == 3 )
            {
                xcoord4 = xtemp;   
                ycoord4 = ytemp;
                width4 = widthtemp;
                height4 = heighttemp;
                objects++;
            }
            else if(objects == 4 )
            {   
                unsigned short xcoordAvg;
                unsigned short ycoordAvg;
                unsigned short heightAvg;
                unsigned short widthAvg;
                
                xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4)/4;
                ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4)/4;
                heightAvg = (height1+height2+height3+height4)/4;
                widthAvg  = (width1+width2+width3+width4)/4;
                pixyAvg.checksum1 = pixyData.checksum1;
                pixyAvg.checksum2 = pixyData.checksum2;
                pixyAvg.sigNum1 = pixyData.sigNum1;
                pixyAvg.sigNum2 = pixyData.sigNum2;
                pixyAvg.xcenter1 = (xcoordAvg >> 8);
                pixyAvg.xcenter2 = xcoordAvg;
                pixyAvg.ycenter1 = (ycoordAvg >> 8);
                pixyAvg.ycenter2 = ycoordAvg;
                pixyAvg.width1 = (widthAvg >> 8);
                pixyAvg.width2 = widthAvg;
                pixyAvg.height1 = (heightAvg >> 8);
                pixyAvg.height2 = heightAvg;
                sendBorderAvg(pixyAvg);
                objects++;
            }
            else {}
        }
        objects = 0;
}
 * */
/*
void obstacleAvg()
{
        PIXY_DATA pixyData;
        debugChar(0xE4);
        while(objects != 5) 
        {
            pixyData = readObstacleData();
            //debugChar(0xE2);
            unsigned short xtemp;
            unsigned short ytemp;
            unsigned short widthtemp;
            unsigned short heighttemp;
            xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
            ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
            widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
            heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
            if(objects == 0)
            {
                xcoord1 = xtemp;   
                ycoord1 = ytemp;
                width1 = widthtemp;
                height1 = heighttemp;
                objects++;                          
            }
            else if(objects == 1 )
            {
                xcoord2 = xtemp;   
                ycoord2 = ytemp;
                width2 = widthtemp;
                height2 = heighttemp;
                objects++;
            }
            else if(objects == 2 )
            {
                xcoord3 = xtemp;   
                ycoord3 = ytemp;
                width3 = widthtemp;
                height3 = heighttemp;
                objects++;
            }
            else if(objects == 3 )
            {
                xcoord4 = xtemp;   
                ycoord4 = ytemp;
                width4 = widthtemp;
                height4 = heighttemp;
                objects++;
            }
            else if(objects == 4 )
            {   
                unsigned short xcoordAvg;
                unsigned short ycoordAvg;
                unsigned short heightAvg;
                unsigned short widthAvg;
                
                xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4)/4;
                ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4)/4;
                heightAvg = (height1+height2+height3+height4)/4;
                widthAvg  = (width1+width2+width3+width4)/4;
                pixyAvg.checksum1 = pixyData.checksum1;
                pixyAvg.checksum2 = pixyData.checksum2;
                pixyAvg.sigNum1 = pixyData.sigNum1;
                pixyAvg.sigNum2 = pixyData.sigNum2;
                pixyAvg.xcenter1 = (xcoordAvg >> 8);
                pixyAvg.xcenter2 = xcoordAvg;
                pixyAvg.ycenter1 = (ycoordAvg >> 8);
                pixyAvg.ycenter2 = ycoordAvg;
                pixyAvg.width1 = (widthAvg >> 8);
                pixyAvg.width2 = widthAvg;
                pixyAvg.height1 = (heightAvg >> 8);
                pixyAvg.height2 = heightAvg;
                sendObstacleAvg(pixyAvg);
                objects++;
            }
            else {}
        }
        refreshAvg();
        objects = 0;
}
 * */
void leadFrontAvg()
{ 
    PIXY_DATA pixyData;
    while(objects != 5) 
    {
        //debugChar(0xF0);
        //debugChar(objects);
        pixyData = readLeadFrontData();
        //debugChar(0xF1);
        unsigned short xtemp;
        unsigned short ytemp;
        unsigned short widthtemp;
        unsigned short heighttemp;
        short orienttemp;
        xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
        ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
        widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
        heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
        orienttemp = (pixyData.orient1 << 8) | pixyData.orient2;
        debugChar(0xEE);
        debugChar(pixyAvg.orient1);
        debugChar(pixyAvg.orient2);
        debugChar(0xEE);
        if(objects == 0)
        {
            //debugChar(0xF2);
            xcoord1 = xtemp;   
            ycoord1 = ytemp;
            width1 = widthtemp;
            height1 = heighttemp;
            orient1 = orienttemp;                         
        }
        else if(objects == 1 )
        {
            //debugChar(0xF3);
            xcoord2 = xtemp;   
            ycoord2 = ytemp;
            width2 = widthtemp;
            height2 = heighttemp;
            orient2 = orienttemp;
        }
        else if(objects == 2 )
        {
            //debugChar(0xF4);
            xcoord3 = xtemp;   
            ycoord3 = ytemp;
            width3 = widthtemp;
            height3 = heighttemp;
            orient3 = orienttemp;
        }
        else if(objects == 3 )
        {
            //debugChar(0xF5);
            xcoord4 = xtemp;   
            ycoord4 = ytemp;
            width4 = widthtemp;
            height4 = heighttemp;
            orient4 = orienttemp;
        }
        else if(objects == 4 )
        {   
            unsigned short xcoordAvg;
            unsigned short ycoordAvg;
            unsigned short heightAvg;
            unsigned short widthAvg;
            short orientAvg;
            signed int negorient1,negorient2,negorient3,negorient4;
            if(orient1<0)
            {
                negorient1 = 0x10000 - orient1;
                negorient1 = 360 - negorient1;
            }
            else    
                negorient1 = orient1;
            if(orient2<0)
            {
                negorient2 = 0x10000 - orient2;
                negorient2 = 360 - negorient2;
            }
            else    
                negorient2 = orient2;
            if(orient3<0)
            {
                negorient3 = 0x10000 - orient3;
                negorient3 = 360 - negorient3;
            }
            else    
                negorient3 = orient3;
            if(orient4<0)
            {
                negorient4 = 0x10000 - orient4;
                negorient4 = 360 - negorient4;             
            }
            else    
                negorient4 = orient4;
            xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4)/4;
            ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4)/4;
            heightAvg = (height1+height2+height3+height4)/4;
            widthAvg  = (width1+width2+width3+width4)/4;
            if(orientAvg > 360 && negorient1 < 360) 
                orientAvg = negorient1;
            else if(orientAvg > 360 && negorient2 < 360) 
                orientAvg = negorient2;            
            else if(orientAvg > 360 && negorient3 < 360) 
                orientAvg = negorient3;
            else if(orientAvg > 360 && negorient4 < 360) 
                orientAvg = negorient4;
            else if(orientAvg < 0 && negorient1 > 0) 
                orientAvg = negorient2;            
            else if(orientAvg < 0 && negorient2 > 0) 
                orientAvg = negorient3;
            else if(orientAvg < 0 && negorient3 > 0) 
                orientAvg = negorient4;
            else if(orientAvg < 0 && negorient4 > 0) 
                orientAvg = negorient4;
            else 
                orientAvg  = (negorient1+negorient2+negorient3+negorient4)/4;
            pixyAvg.checksum1 = pixyData.checksum1;
            pixyAvg.checksum2 = pixyData.checksum2;
            pixyAvg.sigNum1 = pixyData.sigNum1;
            pixyAvg.sigNum2 = pixyData.sigNum2;
            pixyAvg.xcenter1 = (xcoordAvg >> 8);
            pixyAvg.xcenter2 = xcoordAvg;
            pixyAvg.ycenter1 = (ycoordAvg >> 8);
            pixyAvg.ycenter2 = ycoordAvg;
            pixyAvg.width1 = (widthAvg >> 8);
            pixyAvg.width2 = widthAvg;
            pixyAvg.height1 = (heightAvg >> 8);
            pixyAvg.height2 = heightAvg;
            pixyAvg.orient1 = (orientAvg >> 8);
            pixyAvg.orient2 = orientAvg;
            debugChar(0xDD);
            debugChar(pixyAvg.orient1);
            debugChar(pixyAvg.orient2);
            debugChar(0xFF);
            sendLeadFrontAvg(pixyAvg);
        }
        else {}
        objects++;
    }
    if(objects == 5)
    {
        refreshAvg();
        objects = 0;
    }
}
void followerFrontAvg()
{
    PIXY_DATA pixyData;
    while(objects != 5) 
    {
        //debugChar(0xF0);
        //debugChar(objects);
        pixyData = readFollowerFrontData();
        //debugChar(0xF1);
        unsigned short xtemp;
        unsigned short ytemp;
        unsigned short widthtemp;
        unsigned short heighttemp;
        short orienttemp;
        xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
        ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
        widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
        heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
        orienttemp = (pixyData.orient1 << 8) | pixyData.orient2;
        debugChar(0xEE);
        debugChar(pixyAvg.orient1);
        debugChar(pixyAvg.orient2);
        debugChar(0xEE);
        if(objects == 0)
        {
            //debugChar(0xF2);
            xcoord1 = xtemp;   
            ycoord1 = ytemp;
            width1 = widthtemp;
            height1 = heighttemp;
            orient1 = orienttemp;                         
        }
        else if(objects == 1 )
        {
            //debugChar(0xF3);
            xcoord2 = xtemp;   
            ycoord2 = ytemp;
            width2 = widthtemp;
            height2 = heighttemp;
            orient2 = orienttemp;
        }
        else if(objects == 2 )
        {
            //debugChar(0xF4);
            xcoord3 = xtemp;   
            ycoord3 = ytemp;
            width3 = widthtemp;
            height3 = heighttemp;
            orient3 = orienttemp;
        }
        else if(objects == 3 )
        {
            //debugChar(0xF5);
            xcoord4 = xtemp;   
            ycoord4 = ytemp;
            width4 = widthtemp;
            height4 = heighttemp;
            orient4 = orienttemp;
        }
        else if(objects == 4 )
        {   
            unsigned short xcoordAvg;
            unsigned short ycoordAvg;
            unsigned short heightAvg;
            unsigned short widthAvg;
            short orientAvg;
            signed int negorient1,negorient2,negorient3,negorient4;
            if(orient1<0)
            {
                negorient1 = 0x10000 - orient1;
                negorient1 = 360 - negorient1;
            }
            else    
                negorient1 = orient1;
            if(orient2<0)
            {
                negorient2 = 0x10000 - orient2;
                negorient2 = 360 - negorient2;
            }
            else    
                negorient2 = orient2;
            if(orient3<0)
            {
                negorient3 = 0x10000 - orient3;
                negorient3 = 360 - negorient3;
            }
            else    
                negorient3 = orient3;
            if(orient4<0)
            {
                negorient4 = 0x10000 - orient4;
                negorient4 = 360 - negorient4;             
            }
            else    
                negorient4 = orient4;
            xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4)/4;
            ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4)/4;
            heightAvg = (height1+height2+height3+height4)/4;
            widthAvg  = (width1+width2+width3+width4)/4;
            if(orientAvg > 360 && negorient1 < 360) 
                orientAvg = negorient1;
            else if(orientAvg > 360 && negorient2 < 360) 
                orientAvg = negorient2;            
            else if(orientAvg > 360 && negorient3 < 360) 
                orientAvg = negorient3;
            else if(orientAvg > 360 && negorient4 < 360) 
                orientAvg = negorient4;
            else if(orientAvg < 0 && negorient1 > 0) 
                orientAvg = negorient2;            
            else if(orientAvg < 0 && negorient2 > 0) 
                orientAvg = negorient3;
            else if(orientAvg < 0 && negorient3 > 0) 
                orientAvg = negorient4;
            else if(orientAvg < 0 && negorient4 > 0) 
                orientAvg = negorient4;
            else 
                orientAvg  = (negorient1+negorient2+negorient3+negorient4)/4;
            pixyAvg.checksum1 = pixyData.checksum1;
            pixyAvg.checksum2 = pixyData.checksum2;
            pixyAvg.sigNum1 = pixyData.sigNum1;
            pixyAvg.sigNum2 = pixyData.sigNum2;
            pixyAvg.xcenter1 = (xcoordAvg >> 8);
            pixyAvg.xcenter2 = xcoordAvg;
            pixyAvg.ycenter1 = (ycoordAvg >> 8);
            pixyAvg.ycenter2 = ycoordAvg;
            pixyAvg.width1 = (widthAvg >> 8);
            pixyAvg.width2 = widthAvg;
            pixyAvg.height1 = (heightAvg >> 8);
            pixyAvg.height2 = heightAvg;
            pixyAvg.orient1 = (orientAvg >> 8);
            pixyAvg.orient2 = orientAvg;
            debugChar(0xDD);
            debugChar(pixyAvg.orient1);
            debugChar(pixyAvg.orient2);
            debugChar(0xFF);
            sendFollowerFrontAvg(pixyAvg);
        }
        else {}
        objects++;
    }
    if(objects == 5)
    {
        refreshAvg();
        objects = 0;
    }
}

void refreshAvg()
{
    xcoord1 = 0;
    xcoord2 = 0;
    xcoord3 = 0;
    xcoord4 = 0;
    ycoord1 = 0;
    ycoord2 = 0;
    ycoord3 = 0;
    ycoord4 = 0;
    width1 = 0;
    width2 = 0;
    width3 = 0;
    width4 = 0;
    height1 = 0;
    height2 = 0;
    height3 = 0;
    height4 = 0;
    orient1 = 0;
    orient2 = 0;
    orient3 = 0;
    orient4 = 0;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIXY_AVG_Initialize ( void )

  Remarks:
    See prototype in pixy_avg.h.
 */

void PIXY_AVG_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pixy_avgData.state = PIXY_AVG_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    pixy_avgData.obstacle_q = xQueueCreate(100, sizeof(PIXY_AVG));
    if(pixy_avgData.obstacle_q == 0)
    {
        stopEverything();
    }
    pixy_avgData.leadFront_q = xQueueCreate(100, sizeof(PIXY_AVG));
    if(pixy_avgData.leadFront_q == 0)
    {
        stopEverything();
    }
    pixy_avgData.followerFront_q = xQueueCreate(100, sizeof(PIXY_AVG));
    if(pixy_avgData.followerFront_q == 0)
    {
        stopEverything();
    }
    pixy_avgData.border_q = xQueueCreate(100, sizeof(PIXY_AVG));
    if(pixy_avgData.border_q == 0)
    {
        stopEverything();
    }
    refreshCoord();
    pixyAvg.checksum1 = 0;
    pixyAvg.checksum2 = 0;
    pixyAvg.sigNum1 = 0;
    pixyAvg.sigNum2 = 0;
    pixyAvg.xcenter1 = 0;
    pixyAvg.xcenter2 = 0;
    pixyAvg.ycenter1 = 0;
    pixyAvg.ycenter2 = 0;
    pixyAvg.width1 = 0;
    pixyAvg.width2 = 0;
    pixyAvg.height1 = 0;
    pixyAvg.height2 = 0;
    pixyAvg.orient1 = 0;
    pixyAvg.orient2 = 0;
    pixy_avgData.state = PIXY_AVG_STATE_LEAD;
}


/******************************************************************************
  Function:
    void PIXY_AVG_Tasks ( void )

  Remarks:
    See prototype in pixy_avg.h.
 */

void PIXY_AVG_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( pixy_avgData.state )
    {
        /* Application's initial state. */
        case PIXY_AVG_STATE_INIT:
        {
            break;
        }
        case PIXY_AVG_STATE_LEAD:
        {
            //borderAvg();
            //obstacleAvg();
            leadFrontAvg();
            debugChar(0x2E);
            pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER;
            break;
        }
        case PIXY_AVG_STATE_FOLLOWER:
        {
            followerFrontAvg();
            debugChar(0x2);
            pixy_avgData.state = PIXY_AVG_STATE_LEAD;
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

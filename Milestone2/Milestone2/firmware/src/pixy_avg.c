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
#include "pixy_calc_public.h"
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
    //debugChar(0x6C);
    xQueueSendToBack(pixy_avgData.leadFront_q, &data, portMAX_DELAY);
    //debugChar(0x6D);
}
void sendFollowerFrontAvg(PIXY_AVG data)
{
    //debugChar(0x6E);
    xQueueSendToBack(pixy_avgData.followerFront_q, &data, portMAX_DELAY);
    //debugChar(0x6F);
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
    //debugChar(0x6E);
    pixyReceived = xQueueReceive(pixy_avgData.leadFront_q , &pixyRead, portMAX_DELAY);
    //debugChar(0x6F);
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
void leadFrontAvg()
{ 
    PIXY_DATA pixyTemp;
    PIXY_DATA pixyTemp2;
    PIXY_DATA pixyTemp3;
    PIXY_DATA pixyTemp4;
    PIXY_DATA pixyTemp5;
    PIXY_DATA pixyData;
    while(objects != 5) 
    {
//        debugChar(0xF0);
        //debugChar(objects);
        pixyData = readLeadFrontData();
//        debugChar(0xF1);
        pixyTemp = readFollowerFrontData();
//        debugChar(0xF2);
        pixyTemp2 = readObstacleData();
//        debugChar(0xF3);
        pixyTemp3 = readObstacleData();
//        debugChar(0xF4);
        pixyTemp4 = readObstacleData();
//        debugChar(0xF5);
        pixyTemp5 = readObstacleData();
//        debugChar(0xF6);
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
        //debugChar(0xEE);
        //debugChar(pixyAvg.orient1);
        //debugChar(pixyAvg.orient2);
        //debugChar(0xEE);
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
            if(negorient1*negorient2*negorient3*negorient4 == 0)
            {
                negorient1 = 0;
                negorient2 = 0;
                negorient3 = 0;
                negorient4 = 0;
            }
            
            xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4);
            xcoordAvg = xcoordAvg / 0x4;
            ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4);
            ycoordAvg = ycoordAvg / 0x4;
            heightAvg = (height1+height2+height3+height4);
            heightAvg = heightAvg / 0x4;
            widthAvg  = (width1+width2+width3+width4);
            widthAvg = widthAvg / 0x4;
            orientAvg  = (negorient1+negorient2+negorient3+negorient4);
            orientAvg = orientAvg / 0x4;
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
            //debugChar(0xDD);
            //debugChar(pixyAvg.orient1);
            //debugChar(pixyAvg.orient2);
            //debugChar(0xFF);
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
// Retrieve from the data queue and average four values obtained from the sensor
// Then place the averaged value onto another queue to be used for the coordinator
void followerFrontAvg()
{ 
    PIXY_DATA pixyTemp;
    PIXY_DATA pixyTemp2;
    PIXY_DATA pixyTemp3;
    PIXY_DATA pixyTemp4;
    PIXY_DATA pixyTemp5;
    PIXY_DATA pixyData;
    while(objects != 5) 
    {
        debugChar(0xF0);
        //debugChar(objects);
        pixyData = readFollowerFrontData();
        debugChar(0xF1);
        pixyTemp = readLeadFrontData();
        debugChar(0xF2);
        pixyTemp2 = readObstacleData();
        debugChar(0xF3);
        pixyTemp3 = readObstacleData();
        debugChar(0xF4);
        pixyTemp4 = readObstacleData();
        debugChar(0xF5);
        pixyTemp5 = readObstacleData();
        debugChar(0xF6);
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
        //debugChar(0xEE);
        //debugChar(pixyAvg.orient1);
        //debugChar(pixyAvg.orient2);
        //debugChar(0xEE);
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
            if(negorient1*negorient2*negorient3*negorient4 == 0)
            {
                negorient1 = 0;
                negorient2 = 0;
                negorient3 = 0;
                negorient4 = 0;
            }
            
            xcoordAvg = (xcoord1+xcoord2+xcoord3+xcoord4);
            xcoordAvg = xcoordAvg / 0x4;
            ycoordAvg = (ycoord1+ycoord2+ycoord3+ycoord4);
            ycoordAvg = ycoordAvg / 0x4;
            heightAvg = (height1+height2+height3+height4);
            heightAvg = heightAvg / 0x4;
            widthAvg  = (width1+width2+width3+width4);
            widthAvg = widthAvg / 0x4;
            orientAvg  = (negorient1+negorient2+negorient3+negorient4);
            orientAvg = orientAvg / 0x4;
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
            //debugChar(0xDD);
            //debugChar(pixyAvg.orient1);
            //debugChar(pixyAvg.orient2);
            //debugChar(0xFF);
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
// Checks to make sure that the orientation is either N/E/S/W
int orientation(unsigned short orient)
{
    if(orient<=0x105 && orient>=0xBE) // Between 190 and 260 
    {
        return 0;
    }
    else if(orient<=0x15E && orient>=0x118) // Between 350 and 280 
    {
        return 0;
    }
    else if(orient<=0xAA && orient>=0x64) // Between 170 and 100 
    {
        return 0;
    }
    else if(orient<=0x50 && orient>=0xA) // Between 80 and 10  
    {
        return 0;
    }
    else if(orient<=10 || orient>=350) //Facing North 357>x<3
    {
        return 1;
    }
    else if(orient<=100 && orient>=80) // Facing East 87<x<93
    {
        return 1;
    }
    else if(orient<=190 && orient>=170) // Facing South 177<x<183
    {
        return 1;
    }
    else if(orient<=280 && orient>=260) // Facing West 267<x<273
    {
        return 1;
    }
    else
        return 0;
}
// Only send message one the rover has turned a complete 90
// Checks to see if the rover has turned 90 degrees
int orientdiff(unsigned short orient, unsigned short diff)
{
    if((orient<=100&&orient>=80) && (diff>=170&&diff<=190)) // Angle 1 is 90 and Angle 2 is 180
    {
        debugChar(0xB0);
        return 1;
    }
    else if((orient<=100&&orient>=80) && (diff>=350||diff<=10)) // Angle 1 is 90 and Angle 2 is 0
    {
        debugChar(0xB3);
        return 1;
    }
    else if((orient<=10||orient>=350) && (diff>=80&&diff<=100)) // Angle 1 is 0 and Angle 2 is 90
    {
        debugChar(0xB1);
        return 1;
    }
    else if((orient<=10||orient>=350) && (diff>=260&&diff<=280)) // Angle 1 is 0 and Angle 2 is 270
    {
        debugChar(0xB2);
        return 1;
    }
    else if((orient<=280&&orient>=260) && (diff<=10||diff>=350)) // Angle 1 is 270 and Angle 2 is 0
    {
        debugChar(0xB4);
        return 1;
    }
    else if((orient<=280&&orient>=260) && (diff<=190&&diff>=160)) // Angle 1 is 270 and Angle 2 is 180
    {
        debugChar(0xB5);
        return 1;
    }
    else if((orient<=190&&orient>=160) && (diff<=280&&diff>=260)) // Angle 1 is 180 and Angle 2 is 270
    {
        debugChar(0xB6);
        return 1;
    }
    else if((orient<=190&&orient>=160) && (diff<=100&&diff>=80)) // Angle 1 is 180 and Angle 2 is 90
    {
        debugChar(0xB7);
        return 1;
    }
    else
    {
        debugChar(0xB8);
        return 0;
    }
}
void refreshLead()
{
    xlead1 = 0;
    xlead2 = 0;
    ylead1 = 0;
    ylead2 = 0;
    widthlead = 0;
    heightlead = 0;
    olead1 = 0;
    olead2 = 0;
}
void refreshFollower()
{
    xfollower1 = 0;
    xfollower2 = 0;
    yfollower1 = 0;
    yfollower2 = 0;
    widthfollower = 0;
    heightfollower = 0;
    ofollower1 = 0;
    ofollower2 = 0;
}
unsigned char distortx2(unsigned char x, unsigned char y)
{
 
    if(x <= 15)
    {
//        x = x - 1;
        x = x+1;
    }
    else if (x >= 16)
    {
//        x = x - 2;
        x = x+2;
    }
    else {}
    return x;
}
unsigned char distorty2(unsigned char x,unsigned char y)
{
    if((y>=0)&&(y<=18))
    {
//        y = y + 1;
        y = y + 2;
    }
    else if((y>=19)&&(y<=23))
    {
//        y = y -1;
        y = y;
    }
    else if((y>=24)&&(y<=30))
    {
//        y = y;
        y = y+1;
    }
    else if((y>=31)&&(y<=36))
    {
//        y = y - 1;
        y = y;
    }
    else {}

    return y;
}

short pixy_Time(unsigned short millisecondsElapsed)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( pixy_avgData.timer_q, &millisecondsElapsed,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
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
    
    //Create the queue
    pixy_avgData.timer_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(pixy_avgData.timer_q == 0)
    {
        stopEverything();
    }
     //Create the timer
    pixy_avgData.local_timer = xTimerCreate( "50msTimer",
                10,
                pdTRUE,
                0,
                vTimerCallback );
    
    //Ensure timer was created. If not, do not continue and turn on LED
    if(pixy_avgData.local_timer == 0)
    {
        stopEverything();
    }
    BaseType_t started = xTimerStart(pixy_avgData.local_timer, 0);
    
    //Ensure the timer started successfully. If not, do not continue and turn
    // on LED
    if(started == pdFAIL)
    {
        stopEverything();
    }
     
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
    moveNum = 0;
    startLead = 0x00;
    startFollower = 0x00;
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
            //leadFrontAvg();
            //debugChar(0x2E);
            pixy_avgData.state = PIXY_AVG_STATE_LEAD_READ;
            debugChar(0xA1);
            break;
        }
        case PIXY_AVG_STATE_LEAD_READ:
        {
            debugChar(0xA2);
            if(startLead == 5) {
                debugChar(0xB2);
                finishObstacle = 5; // Tell the sensor to stop storing obstacle values
                PIXY_AVG pixyData;
                refreshLead();
                debugChar(0xA3);
                debugChar(objectsFound);
                debugChar(0xA4);
                objectsFound = 0;
                while(objectsFound != 2) 
                {
                    //Number of elapsed ms.
                    unsigned short ms;
                    BaseType_t received = xQueueReceive(pixy_avgData.timer_q , &ms, portMAX_DELAY);
                    //If not received, stop and turn on LED.
                    if(received == pdFALSE)
                    {
                        stopEverything();
                    }
                    unsigned char ms1,ms2;
                    ms1 = ms >> 8;
                    ms2 = ms;
                    leadFrontAvg();
                    pixyData = readLeadFrontAvg();
                    //debugChar(0xA2);
                    unsigned short xtemp;
                    unsigned short ytemp;
                    unsigned short widthtemp;
                    unsigned short heighttemp;
                    unsigned short orienttemp;
                    unsigned char xtemp2;
                    unsigned char ytemp2;
                    xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;
                    xtemp = (xtemp - 109)/4.54;
                    xtemp2 = distortx2(xtemp,ytemp);
                    pixyData.xcenter1 = xtemp2 >> 8;
                    pixyData.xcenter2 = xtemp2;
                    ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
                    ytemp = (ytemp - 15)/4.583;
                    ytemp2 = distorty2(xtemp,ytemp);
                    pixyData.ycenter1 = ytemp2 >> 8;
                    pixyData.ycenter2 = ytemp2;
                    widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
                    widthtemp = (xtemp - 109)/4.54;
                    pixyData.width1 = widthtemp >> 8;
                    pixyData.width2 = heighttemp;
                    heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
                    heighttemp = (heighttemp - 15)/4.583;
                    pixyData.height1 = heighttemp >> 8;
                    pixyData.height2 = heighttemp;  
                    orienttemp = (pixyData.orient1 << 8) | pixyData.orient2;   
                    debugChar(0xA2);
                    debugChar(pixyData.orient1);
                    debugChar(pixyData.orient2);
                    debugChar(pixyData.xcenter1);
                    debugChar(pixyData.xcenter2);
                    debugChar(pixyData.ycenter1);
                    debugChar(pixyData.ycenter2);
                    debugChar(0xA3);
                    debugChar(objectsFound);
                    debugChar(0xA4);
                
                    //if(objectsFound == 0 && (orientation(orienttemp)==1))
                    if(objectsFound == 0)
                    {
                        xlead1 = xtemp;   
                        ylead1 = ytemp;
                        widthlead = widthtemp;
                        heightlead = heighttemp;
                        olead1 = orienttemp;
                        debugChar(0x99);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9A);
                        if(moveNum == 10) {
                            // Sending Lead Location
                            /*
                            unsigned char buffer1[10] = {0x81,'C',0x0B,0x00,
                                0x00,pixyData.xcenter1,pixyData.xcenter2,pixyData.ycenter1,
                                pixyData.xcenter2,0x88};
                            sendMsgToWIFLY(buffer1, 10);
                            // Sending Lead Orientation
                            unsigned char buffer2[10] = {0x81,'C',0x0A,0x00,
                                0x00,0x00,0x00,pixyData.orient1,
                                pixyData.orient2,0x88};
                            sendMsgToWIFLY(buffer2, 10);
                            // Sending Follower Timer
                            unsigned char buffer3[10] = {0x81,'C',0x10,0x00,
                                0x00,0x00,0x00,ms1,
                                ms2,0x88};
                            sendMsgToWIFLY(buffer3, 10);
                            */
                            unsigned char bufferM[10] = {0x81,'E',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                                ms1,ms2};
                            sendMsgToWIFLY(bufferM, 10);
                            objectsFound = 2;  
                            moveNum = 0;
                            debugChar(0xA5);
                            debugChar(objectsFound);
                            debugChar(0xA6);
                        }
                        else{
                            moveNum++;
                        }
                    }
                    /*
                    else if((objectsFound == 1)&& (orientation(orienttemp)==1)
                            &&(orientdiff(orienttemp, olead1)==1))
                    {
                        xlead2 = xtemp;   
                        ylead2 = ytemp;
                        olead2 = orienttemp;
                        debugChar(0x9B);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9C);
                        unsigned char bufferM[10] = {0x81,'F',pixyData.xcenter1,pixyData.xcenter2,
                            pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                            ms1,ms2};
                        sendMsgToWIFLY(bufferM, 10);
//                        vTaskDelay(200);
                        objectsFound = 2;
                        moveNum = 0;
                        debugChar(0xA7);
                    debugChar(objectsFound);
                    debugChar(0xA8);
                    }
                     */
                    /*
                    else if((objectsFound==1))
                    
                    //else if((objectsFound==1)&&(orientation(orienttemp)!=1)
                    //       ||(orientdiff(orienttemp, olead1)!=1))
                   
                    {
                        debugChar(0x9E);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9F);
                        if(moveNum % 10 == 0) {
                            unsigned char bufferM[10] = {0x81,'G',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                                ms1,ms2};
                            sendMsgToWIFLY(bufferM, 10);
//                            vTaskDelay(200);
                            //moveNum = 0;
                        }
                        if(moveNum == 100) {
                            unsigned char bufferL[10] = {0x81,'L',0x23,0x00,0x00,0x00,0x5A,0xAA,0xAA,0x88}; // Turn Left
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 160) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x28,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                        }
                        if(moveNum == 260) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x0F,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 310) {
                            unsigned char bufferL[10] = {0x81,'L',0x23,0x00,0x00,0x00,0x5A,0xAA,0xAA,0x88}; // Turn Left
                            sendMsgToWIFLY(bufferL, 10);
                        }
                        if(moveNum == 370) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x28,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 470) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x0F,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 530) {
                            unsigned char bufferL[10] = {0x81,'L',0x23,0x00,0x00,0x00,0x5A,0xAA,0xAA,0x88}; // Turn Left
                            sendMsgToWIFLY(bufferL, 10);
                        }
                        if(moveNum == 590) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x28,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 690) {
                            unsigned char bufferL[10] = {0x81,'L',0x21,0x00,0x00,0x00,0x0F,0xAA,0xAA,0x88}; // Forward
                            sendMsgToWIFLY(bufferL, 10);
                            //moveNum = 0;
                        }
                        if(moveNum == 750) {
                            unsigned char bufferL[10] = {0x81,'L',0x23,0x00,0x00,0x00,0x5A,0xAA,0xAA,0x88}; // Turn Left
                            sendMsgToWIFLY(bufferL, 10);
                            moveNum = 0;
                            objectsFound = 1;
                        }
                        //else {
                            
                            moveNum++;
                        //}
                        debugChar(0x9A);
                    }
                    */
                    else 
                    {
                        debugChar(0x76);
                    }
                        debugChar(0xA3);
                        debugChar(objectsFound);
                        debugChar(0xA4);
                }
                if(objectsFound == 2) 
                {   
                    debugChar(0x77);
                    objectsFound = 0;
                }
//                pixy_avgData.state = PIXY_AVG_STATE_LEAD;
                pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER;
            }
            else {
                pixy_avgData.state = PIXY_AVG_STATE_LEAD_READ;
            }
            //pixy_avgData.state = PIXY_AVG_STATE_LEAD;
            //pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER;
            break;
        }
        case PIXY_AVG_STATE_FOLLOWER:
        {
            //borderAvg();
            //obstacleAvg();
            //leadFrontAvg();
            //debugChar(0x2E);
            finishObstacle = 5; // Tell the sensor to stop storing obstacle values
            pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER_READ;
            debugChar(0xA1);
            break;
        }
        case PIXY_AVG_STATE_FOLLOWER_READ:
        {
            debugChar(0xA2);
            if(startFollower == 5) {
                debugChar(0xB2);
                PIXY_AVG pixyData;
                refreshFollower();
                debugChar(0xA3);
                debugChar(objectsFound);
                debugChar(0xA4);
                while(objectsFound != 2) 
                {
                    unsigned int ms;
                    BaseType_t received = xQueueReceive(pixy_avgData.timer_q , &ms, portMAX_DELAY);
                    //If not received, stop and turn on LED.
                    if(received == pdFALSE)
                    {
                        stopEverything();
                    }
                    unsigned char ms1,ms2;
                    ms1 = ms >> 8;
                    ms2 = ms;
                    followerFrontAvg();
                    pixyData = readFollowerFrontAvg();
                    //debugChar(0xA2);
                    unsigned short xtemp;
                    unsigned short ytemp;
                    unsigned short widthtemp;
                    unsigned short heighttemp;
                    unsigned short orienttemp;
                    unsigned char xtemp2;
                    unsigned char ytemp2;
                    xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;
                    xtemp = (xtemp - 109)/4.54;
                    xtemp2 = distortx2(xtemp,ytemp);
                    pixyData.xcenter1 = xtemp2 >> 8;
                    pixyData.xcenter2 = xtemp2;
                    ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
                    ytemp = (ytemp - 15)/4.583;
                    ytemp2 = distorty2(xtemp,ytemp);
                    pixyData.ycenter1 = ytemp2 >> 8;
                    pixyData.ycenter2 = ytemp2;
                    widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
                    widthtemp = (xtemp - 109)/4.54;
                    pixyData.width1 = widthtemp >> 8;
                    pixyData.width2 = heighttemp;
                    heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
                    heighttemp = (heighttemp - 15)/4.583;
                    pixyData.height1 = heighttemp >> 8;
                    pixyData.height2 = heighttemp;  
                    orienttemp = (pixyData.orient1 << 8) | pixyData.orient2;   
                    debugChar(0xA2);
                    debugChar(pixyData.orient1);
                    debugChar(pixyData.orient2);
                    debugChar(0xA3);
                    debugChar(objectsFound);
                    debugChar(0xA4);
                    //if(objectsFound == 0 && (orientation(orienttemp)==1))
                    if(objectsFound == 0)
                    {
                        xfollower1 = xtemp;   
                        yfollower1 = ytemp;
                        widthfollower = widthtemp;
                        heightfollower = heighttemp;
                        ofollower1 = orienttemp;
                        debugChar(0x99);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9A);
                        if(moveNum == 5) {
                            // Sending Follower Location
                            /*
                            unsigned char buffer1[10] = {0x81,'C',0x0D,0x00,
                                0x00,pixyData.xcenter1,pixyData.xcenter2,pixyData.ycenter1,
                                pixyData.xcenter2,0x88};
                            sendMsgToWIFLY(buffer1, 10);
                            // Sending Follower Orientation
                            unsigned char buffer2[10] = {0x81,'C',0x0E,0x00,
                                0x00,0x00,0x00,pixyData.orient1,
                                pixyData.orient2,0x88};
                            sendMsgToWIFLY(buffer2, 10);
                            // Sending Follower Timer
                            unsigned char buffer3[10] = {0x81,'C',0x10,0x00,
                                0x00,0x00,0x00,ms1,
                                ms2,0x88};
                            sendMsgToWIFLY(buffer3, 10);
                            */
                            unsigned char bufferM[10] = {0x81,'H',pixyData.xcenter1,pixyData.xcenter2,
                            pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                            ms1,ms2};
                            sendMsgToWIFLY(bufferM, 10);
                            objectsFound = 2;  
//                            vTaskDelay(200);
                            moveNum = 0;
                        }
                        else{
                            moveNum++;
                        }
                        debugChar(0xA5);
                        debugChar(objectsFound);
                        debugChar(0xA6);
                    }
                    /*
                    else if((objectsFound == 1)&& (orientation(orienttemp)==1)
                            &&(orientdiff(orienttemp, ofollower1)==1))
                    {
                        xfollower2 = xtemp;   
                        yfollower2 = ytemp;
                        ofollower2 = orienttemp;
                        debugChar(0x9B);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9C);
                        unsigned char bufferM[10] = {0x81,'I',pixyData.xcenter1,pixyData.xcenter2,
                            pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                            ms1,ms2};
                        sendMsgToWIFLY(bufferM, 10);
//                        vTaskDelay(200);
                        objectsFound = 2;
                        moveNum = 0;
                        debugChar(0xA7);
                    debugChar(objectsFound);
                    debugChar(0xA8);
                    }
                    else if((objectsFound==1)&&(orientation(orienttemp)!=1)
                            ||(orientdiff(orienttemp, ofollower1)!=1))
                    {
                        debugChar(0x9D);
                        debugChar(pixyData.orient1);
                        debugChar(pixyData.orient2);
                        debugChar(0x9E);
                        if(moveNum == 5) {
                            unsigned char bufferM[10] = {0x81,'J',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                                ms1,ms2};
                            sendMsgToWIFLY(bufferM, 10);
//                            vTaskDelay(200);
                            moveNum = 0;
                        }
                        else{
                            moveNum++;
                        }
                    }
                    */
                    else 
                    {
                        debugChar(0x76);
                    }
                        debugChar(0xA3);
                        debugChar(objectsFound);
                        debugChar(0xA4);
                }
                if(objectsFound == 2) 
                {   
                    debugChar(0x77);
                    objectsFound = 0;
                }
                pixy_avgData.state = PIXY_AVG_STATE_LEAD;
            }
            else {
                pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER_READ;
            }
            //pixy_avgData.state = PIXY_AVG_STATE_LEAD;
            //pixy_avgData.state = PIXY_AVG_STATE_FOLLOWER;
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

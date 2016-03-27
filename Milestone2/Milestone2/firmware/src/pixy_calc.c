/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pixy_calc.c

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

#include "pixy_calc.h"
#include "app_public.h"
#include "pixy_avg_public.h"
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

PIXY_CALC_DATA pixy_calcData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
void refreshCoord()
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
}

void refreshLead()
{
    xlead = 0;
    ylead = 0;
    widthlead = 0;
    heightlead = 0;
}
void refreshFollower()
{
    xfollower = 0;
    yfollower = 0;
    widthfollower = 0;
    heightfollower = 0;
}
int coordinates(unsigned short xtemp, unsigned short ytemp, unsigned short x, 
        unsigned short y, unsigned short width, unsigned short height,
        unsigned short widthtemp, unsigned short heighttemp)
{
    if( ((xtemp>=x-10)&&(xtemp<=x+10)) && (((ytemp+heighttemp)<=(y-height)) || ((ytemp-heighttemp)>=(y+height))))
    {
        //debugChar(0x50);
        return 1;
    }
    else if(((ytemp>=y-10)&&(ytemp<=y+10)) && (((xtemp+widthtemp)<=(x-width)) || ((xtemp-widthtemp)>=(x+width))))
    {
        //debugChar(0x51);
        return 1;
    }
    else if(((xtemp<=x-10)||(xtemp>=x+10)) || ((ytemp<=y-10)||(ytemp>=y+10)))
    {
        return 1;
    }
    else 
    {
        //debugChar(0x52);
        return 0;
    }
}
int orientation(short orient)
{
    //Facing north
    if((orient>-23)&&(orient<=23))
    {
        return 1;
    }
    //Facing northeast
    else if((orient>23)&&(orient<=67))
    {
        return 8;
    }
    //Facing east
    else if((orient>67)&&(orient<=113))
    {
        return 7;
    }
    //Facing southeast
    else if((orient>113)&&(orient<=157))
    {
        return 6;
    }
    //Facing south
    else if((orient>157)&&(orient<=-157))
    {
        return 5;
    }
    //Facing southwest
    else if((orient>-157)&&(orient<=-113))
    {
        return 4;
    }
    //Facing west
    else if((orient>-117)&&(orient<=-67))
    {
        return 3;
    }
    //Facing northwest
    else if((orient>-67)&&(orient<=-23))
    {
        return 2;
    }
    else{
        debugChar(0xEF);
    }
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIXY_CALC_Initialize ( void )

  Remarks:
    See prototype in pixy_calc.h.
 */

void PIXY_CALC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pixy_calcData.state = PIXY_CALC_STATE_INIT;
    
    counter = 1;
    xcorner = 0;
    ycorner = 0;
    widthcorner = 0;
    heightcorner = 0;
    objectsFound = 0;
    refreshCoord();
    refreshLead();
    refreshFollower();
    pixy_calcData.state = PIXY_CALC_STATE_CALC;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void PIXY_CALC_Tasks ( void )

  Remarks:
    See prototype in pixy_calc.h.
 */

void PIXY_CALC_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( pixy_calcData.state )
    {
        /* Application's initial state. */
        case PIXY_CALC_STATE_INIT:
        {
            break;
        }

        case PIXY_CALC_STATE_CALC:
        {
            // Read the obstacle values first 
            switch(counter) 
            {
                case 0: // border1
                {
                    PIXY_AVG pixyData;
                    refreshCoord();
                    debugChar(0xE0);
                    pixyData = readBorderAvg();
                    //debugChar(0xE2);
                    unsigned short xtemp;
                    unsigned short ytemp;
                    unsigned short widthtemp;
                    unsigned short heighttemp;
                    xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
                    ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
                    widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
                    heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
                    xcorner = xtemp;   
                    ycorner = ytemp;
                    widthcorner = widthtemp;
                    heightcorner = heighttemp;
                    
                    debugChar(0x91);
                    debugChar(pixyData.xcenter1);
                    debugChar(pixyData.xcenter2);
                    debugChar(pixyData.ycenter1);
                    debugChar(pixyData.ycenter2);
                    debugChar(pixyData.width1);
                    debugChar(pixyData.width2);
                    debugChar(pixyData.height1);
                    debugChar(pixyData.height2);
                    debugChar(0x92);
                    
                    unsigned char bufferM[10] = {'U',pixyData.xcenter1,pixyData.xcenter2,
                    pixyData.ycenter1,pixyData.ycenter2,pixyData.width1,pixyData.width2,
                    pixyData.height1,pixyData.height2,0x00};
                    
                    sendMsgToWIFLY(bufferM, 10);
                    vTaskDelay(50);                        
                    counter++;
                    objectsFound = 0;
                }
                case 1: // obstacles
                {
                    PIXY_AVG pixyData;
                    refreshCoord();
                    debugChar(0xE1);
                    while(objectsFound != 4) 
                    {
                        pixyData = readObstacleAvg();
                        //debugChar(0xE2);
                        unsigned short xtemp;
                        unsigned short ytemp;
                        unsigned short widthtemp;
                        unsigned short heighttemp;
                        xtemp = (pixyData.xcenter1 << 8) | pixyData.xcenter2;   
                        ytemp = (pixyData.ycenter1 << 8) | pixyData.ycenter2;   
                        widthtemp = (pixyData.width1 << 8) | pixyData.width2;   
                        heighttemp = (pixyData.height1 << 8) | pixyData.height2;   
                        if(objectsFound == 0)
                        {
                            xcoord1 = xtemp;   
                            ycoord1 = ytemp;
                            width1 = widthtemp;
                            height1 = heighttemp;
                            debugChar(0x91);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x92);
                            unsigned char bufferM[10] = {'A',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.width1,pixyData.width2,
                                pixyData.height1,pixyData.height2,0x00};
                            sendMsgToWIFLY(bufferM, 10);
                            vTaskDelay(50);
                            objectsFound++;                          
                        }
                        else if((objectsFound == 1 ) 
                                && (coordinates(xtemp,ytemp,xcoord1,ycoord1, width1, height1, widthtemp, heighttemp) == 1))
                        {
                            xcoord2 = xtemp;   
                            ycoord2 = ytemp;
                            width2 = widthtemp;
                            height2 = heighttemp;
                            debugChar(0x93);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x94);
                            unsigned char bufferM[10] = {'B',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.width1,pixyData.width2,
                                pixyData.height1,pixyData.height2,0x00};
                            sendMsgToWIFLY(bufferM, 10);
                            vTaskDelay(50);
                            objectsFound++;
                        }
                        else if((objectsFound == 2 ) 
                                && (coordinates(xtemp,ytemp,xcoord1,ycoord1, width1, height1, widthtemp, heighttemp) == 1)
                                && (coordinates(xtemp,ytemp,xcoord2,ycoord2, width2, height2, widthtemp, heighttemp) == 1))
                        {
                            xcoord3 = xtemp;   
                            ycoord3 = ytemp;
                            width3 = widthtemp;
                            height3 = heighttemp;
                            debugChar(0x95);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x96);
                            unsigned char bufferM[10] = {'C',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.width1,pixyData.width2,
                                pixyData.height1,pixyData.height2,0x00};
                            sendMsgToWIFLY(bufferM, 10);
                            vTaskDelay(50);
                            objectsFound++;
                        }
                        else if((objectsFound == 3 ) 
                            && (coordinates(xtemp,ytemp,xcoord1,ycoord1, width1, height1, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord2,ycoord2, width2, height2, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord3,ycoord3, width3, height3, widthtemp, heighttemp) == 1))
                        {
                            xcoord4 = xtemp;   
                            ycoord4 = ytemp;
                            width4 = widthtemp;
                            height4 = heighttemp;
                            debugChar(0x97);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x98);
                            unsigned char bufferM[10] = {'D',pixyData.xcenter1,pixyData.xcenter2,
                                pixyData.ycenter1,pixyData.ycenter2,pixyData.width1,pixyData.width2,
                                pixyData.height1,pixyData.height2,0x00};
                            sendMsgToWIFLY(bufferM, 10);
                            vTaskDelay(50);
                            objectsFound++;
                        }
                        else {}
                    }
                    counter++;
                    objectsFound = 0;
                    break;
                }
                case 2: // Lead Rover
                {
                    debugChar(0xA1);
                    PIXY_AVG pixyData;
                    refreshLead();
                    pixyData = readLeadFrontAvg();
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
                    debugChar(0xA2);
                    debugChar(pixyData.orient1);
                    debugChar(pixyData.orient2);
                    while(objectsFound != 1) 
                    {
                        if((coordinates(xtemp,ytemp,xcoord1,ycoord1, width1, height1, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord2,ycoord2, width2, height2, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord3,ycoord3, width3, height3, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord4,ycoord4, width4, height4, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xfollower,yfollower, widthfollower, heightfollower, widthtemp, heighttemp) == 1))
                        {
                            xlead = xtemp;   
                            ylead = ytemp;
                            widthlead = widthtemp;
                            heightlead = heighttemp;
                            debugChar(0x99);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x9A);
                            vTaskDelay(50);
                            objectsFound++;
                        }
                         else {
                            debugChar(0x9B);
                         }
                    }
                    
                    if(orienttemp<0)
                    {
                        signed int negorient;
                        negorient = 0x10000 - orienttemp;
                        negorient = 360 - negorient;
                        pixyData.orient1 = (negorient >> 8);
                        pixyData.orient2 = negorient;
                    }
                    unsigned char bufferM[10] = {'E',pixyData.xcenter1,pixyData.xcenter2,
                        pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                        pixyData.height1,pixyData.height2,0x00};
                    sendMsgToWIFLY(bufferM, 10);
                    counter = 1;
                    objectsFound = 0;
                    break;
                }
                case 3: // Follower Rover
                {
                    PIXY_AVG pixyData;
                    refreshFollower();
                    pixyData = readFollowerFrontAvg();
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
                    while(objectsFound != 1) 
                    {
                        if((coordinates(xtemp,ytemp,xcoord1,ycoord1, width1, height1, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord2,ycoord2, width2, height2, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord3,ycoord3, width3, height3, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xcoord4,ycoord4, width4, height4, widthtemp, heighttemp) == 1)
                            && (coordinates(xtemp,ytemp,xlead,ylead, widthlead, heightlead, widthtemp, heighttemp) == 1))
                        {
                            xfollower = xtemp;   
                            yfollower = ytemp;
                            widthfollower = widthtemp;
                            heightfollower = heighttemp;
                            debugChar(0x9B);
                            debugChar(pixyData.xcenter1);
                            debugChar(pixyData.xcenter2);
                            debugChar(pixyData.ycenter1);
                            debugChar(pixyData.ycenter2);
                            debugChar(pixyData.width1);
                            debugChar(pixyData.width2);
                            debugChar(pixyData.height1);
                            debugChar(pixyData.height2);
                            debugChar(0x9C);
                            vTaskDelay(200);
                            objectsFound++;
                        }
                        else{}
                    }

                    debugChar(0x11);
                    debugChar(0x11);
                    debugChar(0x11);
                    debugChar(0x11);
                    if(orienttemp<0)
                    {
                        signed int negorient;
                        negorient = 0x10000 - orienttemp;
                        negorient = 360 - negorient;
                        pixyData.orient1 = (negorient >> 8);
                        pixyData.orient2 = negorient;
                    }
                    unsigned char bufferM[10] = {'F',pixyData.xcenter1,pixyData.xcenter2,
                        pixyData.ycenter1,pixyData.ycenter2,pixyData.orient1,pixyData.orient2,
                        pixyData.height1,pixyData.height2,0x00};
                    sendMsgToWIFLY(bufferM, 10);
                    counter++;
                    objectsFound = 0;
                    break;
                }
                case 4:
                {   
                    counter = 1;
                    objectsFound = 0;
                    debugChar(0xAF);
                    break;
                }
                default:
                {
                    break;
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

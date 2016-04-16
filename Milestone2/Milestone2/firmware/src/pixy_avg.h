/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pixy_avg.h

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

#ifndef _PIXY_AVG_H
#define _PIXY_AVG_H

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

#include "app_public.h"
#include "pixy_avg_public.h"
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
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	PIXY_AVG_STATE_INIT=0,
          
	/* TODO: Define states used by the application state machine. */
    PIXY_AVG_STATE_LEAD=1,
    PIXY_AVG_STATE_LEAD_READ=2,
    PIXY_AVG_STATE_FOLLOWER=3,
    PIXY_AVG_STATE_FOLLOWER_READ=4,
} PIXY_AVG_STATES;


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
    /* The application's current state */
    PIXY_AVG_STATES state;

    TimerHandle_t lead_timer;
    /* TODO: Define any additional data used by the application. */
    QueueHandle_t pixy_q;
    QueueHandle_t obstacle_q;
    QueueHandle_t leadFront_q;
    QueueHandle_t followerFront_q;
    QueueHandle_t border_q;
    QueueHandle_t leadTimer_q;
} PIXY_AVG_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
void vTimerCallback( TimerHandle_t pxTimer );
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIXY_AVG_Initialize ( void )

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
    PIXY_AVG_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PIXY_AVG_Initialize ( void );


/*******************************************************************************
  Function:
    void PIXY_AVG_Tasks ( void )

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
    PIXY_AVG_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */
int objects;
int moveNum;
unsigned char distortx2(unsigned char x, unsigned char y);
unsigned char distorty2(unsigned char x, unsigned char y);
unsigned short xcoord1, xcoord2, xcoord3, xcoord4, width1, width2, width3, width4;
unsigned short ycoord1, ycoord2, ycoord3, ycoord4, height1, height2, height3, height4;
unsigned short xlead1, xlead2, xfollower1, xfollower2, widthlead, widthfollower;
unsigned short ylead1, ylead2, yfollower1, yfollower2, heightlead, heightfollower;
unsigned short olead1, olead2, ofollower1, ofollower2;
short orient1, orient2, orient3, orient4;
void refreshAvg();
void refreshLead();
void refreshFollower();
void PIXY_AVG_Tasks( void );
int orientation(unsigned short orient);
int orientdiff(unsigned short orient, unsigned short diff);

void obstacleAvg();
void leadFrontAvg();
void followerFrontAvg();
void borderAvg();

#endif /* _PIXY_AVG_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */


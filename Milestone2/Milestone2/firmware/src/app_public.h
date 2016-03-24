/* ************************************************************************** */
/** Public Task Header

  @Company
 Team 5

  @File Name
    app_public.h

  @Summary
 Public function related to app.

  @Description
 This declares the function of the app that are available to other files.
 */
/* ************************************************************************** */
#ifndef _APP_PUBLIC_H    /* Guard against multiple inclusion */
#define _APP_PUBLIC_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed) 

      @Summary
     Puts a value on the queue.

      @Description
     This function is used by outside methods to put a value on the queue of 
     the app. When it is called, the integer is put onto the back of the 
     queue and exits with 0 if it was successful, or 1 if the queue is full.

      @Precondition
     None.

      @Parameters
        @param millisecondsElapsed The integer to be put on the queue

      @Returns
     0 for success, 1 for failure

      @Remarks
     None.

      @Example
        @code
        app1SendTimerValToMsgQ(100);
     */
     typedef struct
    {
        unsigned char checksum1;
        unsigned char checksum2;
        unsigned char sigNum1;
        unsigned char sigNum2;
        unsigned char xcenter1;
        unsigned char xcenter2;
        unsigned char ycenter1;
        unsigned char ycenter2;
        unsigned char width1;
        unsigned char width2;        
        unsigned char height1;
        unsigned char height2;
        char orient1;
        char orient2;
    } PIXY_DATA;
    
    int sensor1SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    void sensor1SendSensorValToSensorQ(unsigned char sensorValue);
    
    unsigned char sensor1ReceiveVal();
    
    void ReceiveUSARTMsgFromPixyQ(unsigned char pixyMsg);
    unsigned char readMsgFromPixyQ();
    void sendObstacleData(PIXY_DATA);
    void sendLeadFrontData(PIXY_DATA);
    void sendFollowerFrontData(PIXY_DATA);
    void sendBorderData(PIXY_DATA);
    PIXY_DATA readObstacleData();
    PIXY_DATA readLeadFrontData();
    PIXY_DATA readFollowerFrontData();
    PIXY_DATA readBorderData();
    unsigned char receivePixyData();
    
    int receiveFromRover(unsigned char data);
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _APP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

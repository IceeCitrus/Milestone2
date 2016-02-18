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
    int sensor1SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    void sensor1SendSensorValToSensorQ(unsigned char sensorValue);
    
    unsigned char sensor1ReceiveVal();
    int receiveFromRover(unsigned char data);
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _APP_PUBLIC_H */

/* *****************************************************************************
 End of File
 */

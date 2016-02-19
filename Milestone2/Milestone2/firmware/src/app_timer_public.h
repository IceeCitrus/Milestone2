/* 
 * File:   app_timer_public.h
 * Author: Andrew Wang
 *
 * Created on February 18, 2016, 1:48 PM
 */

#ifndef APP_TIMER_PUBLIC_H
#define	APP_TIMER_PUBLIC_H

#ifdef	__cplusplus
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
    int SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    int sensor1ValToMsgQ(unsigned char data);

#ifdef	__cplusplus
}
#endif

#endif	/* APP_TIMER_PUBLIC_H */


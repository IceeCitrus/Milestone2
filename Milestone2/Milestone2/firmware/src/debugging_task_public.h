/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _DEBUGGING_TASK_PUBLIC_H    /* Guard against multiple inclusion */
#define _DEBUGGING_TASK_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void debugChar(unsigned char toSend);
    void debugCharFromISR(unsigned char toSend);
    void debugBuffer(unsigned char buffer[], unsigned int num);
    void stopEverything( void );
    
    /*DEFINES FOR DEBUGGING************************************************/
    
#define START_OF_ANDREW 0x01
    /*Add your defines here*/
    
#define before_pushing_to_queue 0x01
#define after_pushing_to_queue 0x02
#define before_recv_to_queue 0x03
#define after_recv_to_queue 0x04
#define before_recv_from_rover 0x05
#define after_recv_from_rover 0x05
#define start_of_app_task 0x06
#define before_receiving_sensordata 0x07
#define recv_sensor1 0x08
#define start_ADC_ISR 0x0C
#define end_ADC_ISR 0x0D
#define before_transmitting_USART 0x0E
#define after_transmitting_USART 0x0F
#define before_recv_USART 0x10
#define after_recv_USART 0x11
#define start_message_task 0x12
#define after_recv_msgq 0x13
#define header_good 0x14
#define header_bad 0x15
#define msg_body 0x16
#define msg_format 0x17
#define msg_dst 0x18
#define msg_type 0x19
#define msg_msgNum1 0x1A
#define msg_msgNum2 0x1B
#define msg_data1 0x1C
#define msg_data2 0x1D
#define msg_data3 0x1E
#define msg_data4 0x1F
#define sensor1ReceiveVal_Fail 0x20   
#define sensor1_q_create 0x21
#define data_q_create 0x22
#define receiveMsg_q_create 0x23
#define footer_good 0x24
#define footer_bad 0x25
    
#define END_OF_ANDREW 0x3f
    
#define START_OF_AUSTIN 0x40
    /*Add your defines here*/
#define END_OF_AUSTIN 0x7f
    
#define START_OF_MITCHELL 0x80
    /*Add your defines here*/
#define END_OF_MITCHELL 0xbf
    
#define START_OF_TOM 0xc0
    /*Add your defines here*/
#define END_OF_TOM 0xff
    
    /*END OF DEFINES*********************************************************/
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUGGING_TASK_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
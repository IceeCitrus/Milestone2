/* 
 * File:   messaging_task_public.h
 * Author: Andrew Wang
 *
 * Created on February 10, 2016, 5:20 PM
 */

#ifndef MESSAGING_TASK_PUBLIC_H
#define	MESSAGING_TASK_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    void sendMsgToWIFLY(unsigned char message[], int num);
    void sendByteToWIFLY(unsigned char byte);
    void ReceiveUSARTMsgFromMsgQ(unsigned char usartMsg);

typedef struct
{
    int count;
    int validHeader;
    int validFooter;
    unsigned char header;
    unsigned char dst;
    unsigned char type;
    unsigned char msgNum1;
    unsigned char msgNum2;
    unsigned char data1;
    unsigned char data2;
    unsigned char data3;
    unsigned char data4;
    unsigned char footer;
    unsigned char valid;
    int numInvalid;
} MESSAGE_FORMAT;
    
#ifdef	__cplusplus
}
#endif

#endif	/* MESSAGING_TASK_PUBLIC_H */


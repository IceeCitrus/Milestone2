/* 
 * File:   pixy_avg_public.h
 * Author: Andrew Wang
 *
 * Created on March 24, 2016, 2:28 PM
 */
#include "app_public.h"
#ifndef PIXY_AVG_PUBLIC_H
#define	PIXY_AVG_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

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
    } PIXY_AVG;
    unsigned char startLead;
    unsigned char startFollower;
    void sendObstacleAvg(PIXY_AVG);
    void sendLeadFrontAvg(PIXY_AVG);
    void sendFollowerFrontAvg(PIXY_AVG);
    void sendBorderAvg(PIXY_AVG);
    void sendLeadTimer(PIXY_AVG);
    PIXY_AVG readLeadTimer();
    PIXY_AVG readObstacleAvg();
    PIXY_AVG readLeadFrontAvg();
    PIXY_AVG readFollowerFrontAvg();
    PIXY_AVG readBorderAvg();
#ifdef	__cplusplus
}
#endif

#endif	/* PIXY_AVG_PUBLIC_H */


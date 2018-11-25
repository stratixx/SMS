#ifndef DMC_DATA_H
#define DMC_DATA_H

#include <inttypes.h>

//Parametry regulatora DMC
uint8_t DMC_D = 18;
uint8_t DMC_N = 5;
uint8_t DMC_Nu = 2;
float DMC_lambda = 1.000000;

// Przeliczone wartosci do sterowania DMC
float DMC_Ke = 0.377373;

float DMC_Ku[] =
{
    0.150922, 
    0.208030, 
    0.231834, 
    0.229144, 
    0.209065, 
    0.188686, 
    0.170573, 
    0.150194, 
    0.130571, 
    0.110193, 
    0.092079, 
    0.075475, 
    0.060380, 
    0.044530, 
    0.032454, 
    0.021133, 
    0.009812, 
};

#endif

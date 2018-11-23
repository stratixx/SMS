#ifndef DMC_DATA_H
#define DMC_DATA_H

#include <inttypes.h>

//Parametry regulatora DMC
uint8_t DMC_D = 18;
uint8_t DMC_N = 18;
uint8_t DMC_Nu = 2;
float DMC_lambda = 0.100000;

// Przeliczone wartosci do sterowania DMC
float DMC_Ke = 1.970701;

float DMC_Ku[] =
{
    0.651593, 
    0.689478, 
    0.708492, 
    0.695726, 
    0.668977, 
    0.640408, 
    0.615947, 
    0.571698, 
    0.522343, 
    0.460647, 
    0.402519, 
    0.345335, 
    0.288235, 
    0.220176, 
    0.165547, 
    0.110129, 
    0.051238, 
};

#endif

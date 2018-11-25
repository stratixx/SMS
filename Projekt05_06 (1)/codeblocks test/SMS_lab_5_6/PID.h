#ifndef PID_H
#define PID_H

#include <inttypes.h>

#define PID_Tp (1/20.0)

//////////////////////////////////////////////////////////////
//dane wyznaczone na podstawie modelu z Matlaba

//% parametry PID
#define PID_Kk 199.33f
#define PID_Tu 0.1f

//% ziegler niestrojony
//#define PID_K (0.6*PID_Kk)
//#define PID_Ti (0.5*PID_Tu)
//#define PID_Td (0.12*PID_Tu)

//%rêcznie po in¿yniersku
#define PID_K 1.0f
#define PID_Ti 0.45f
#define PID_Td 0.0f
///////////////////////////////////////////////////////////////

/*
////////////////////////////////////////////////////////////////
//dane wyznaczone na laborce

#define PID_Kk 27.25f
#define PID_Tu 0.3f

// Ziegler niestrojony

//#define PID_K (0.6*PID_Kk)
//#define PID_Ti (PID_Tu/2.0)
//#define PID_Td (PID_Tu/8.0)

#define PID_K 5.0f
#define PID_Ti 0.1f
#define PID_Td 0.0f
*/
/////////////////////////////////////////////////////////////////

typedef struct
{
	uint8_t D;


}PID_type;

void PID_init(PID_type*, float, float, float, uint8_t );///////////////////// TO DO
float PID_get_control(PID_type*, float, float, float);///////////////////// TO DO



#endif

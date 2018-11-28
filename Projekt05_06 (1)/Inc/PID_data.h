#ifndef PID_DATA_H
#define PID_DATA_H

#include <inttypes.h>

/*
model wyznaczony w Matlabie
u2 = 0.0159
u1 = 0.0077
y2 =-0.6605
y1 = 1.6368
z matlaba
y(k) = u2*u(k-2) + u1*u(k-1) + y2*y(k-2) + y1*y(k-1);
ze skryptu
y(k) = 0.043209*u(k-1) + 0.030415*u(k-2) + 1.309644*y(k-1) - 0.346456*y(k-2);
*/

//Parametry regulatora PID



#define PID_Tp (1/20.0)

// parametr anti-winding
#define PID_Tv 0.050f

//////////////////////////////////////////////////////////////
//dane wyznaczone na podstawie modelu z Matlaba

//% parametry PID
#define PID_Kk 199.33f
#define PID_Tu 0.1f

//%ręcznie po inżyniersku
#define PID_K 1.0f
#define PID_Ti 0.45f
#define PID_Td 0.0f

//% ziegler niestrojony
//#define PID_K (0.6*PID_Kk)
//#define PID_Ti (0.5*PID_Tu)
//#define PID_Td (0.12*PID_Tu)
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

/*
// grocha i janasa do przetestowania
no anti-windup
PID:
K = 21 Ti = 0.4 Td =0.1
K = 17.5 Ti = 2.8 Td =0.2
K = 17.5 Ti = 0.5 Td =0.15
K = 17.5 Ti = 0.4 Td =0.3
K = 17.5 Ti = 4.0 Td =0.08



*/

#endif

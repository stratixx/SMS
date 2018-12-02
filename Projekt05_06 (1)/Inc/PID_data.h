#ifndef PID_DATA_H
#define PID_DATA_H

#include <inttypes.h>

//Parametry regulatora PID

// okres próbkowania 
#define PID_Tp (1/20.0)
// parametr anti-winding
#define PID_Tv -8.0f

///////////////////////////////////////////////////////////////
//parametry regulatora PID wyznaczone metodą Zieglera-Nicholsa
// Wzmocnienie krytyczne
#define PID_Kk 30.0f
// Okres oscylacji
#define PID_Tu (8*PID_Tp)

#define PID_K (0.6*PID_Kk)
#define PID_Ti (0.5*PID_Tu)
#define PID_Td (0.12*PID_Tu)
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
//parametry regulatora PID wyznaczone metodą inżynierską
/*
#define PID_K 15.00f
#define PID_Ti 3.5f
#define PID_Td 0.040f
*/
////////////////////////////////////////////////////////////////


#endif

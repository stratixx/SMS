#ifndef DMC_H
#define DMC_H

#include <inttypes.h>
#include "mat_lib.h"

typedef struct
{
	uint8_t D;

	float Ke;							// skalar
	float * Ku;						// wektor, dlugosc D-1
	float * delta_u_past; // wektor, dlugosc D-1
	float u;

}DMC_type;

void DMC_init(DMC_type*, uint8_t, float, float*, float);
float DMC_get_control(DMC_type*, float, float, float);


#endif

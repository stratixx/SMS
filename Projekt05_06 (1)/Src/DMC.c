#include "DMC.h"
#include <stdlib.h>
#include <string.h>

void DMC_init(DMC_type* dmc, uint8_t _D, float _Ke, float* _Ku, float u_initial)
{
	uint8_t n=0;
	
	dmc->D = _D;
	dmc->Ke = _Ke;
	dmc->Ku = malloc(sizeof(float)*(dmc->D-1));
	dmc->delta_u_past = malloc(sizeof(float)*(dmc->D-1));
	dmc->u = u_initial;
	
	memcpy(dmc->Ku, _Ku, sizeof(float)*(dmc->D-1) );
	for(n=0;n<(dmc->D-1);n++)
		dmc->delta_u_past[n] = 0.0;
}

float DMC_get_control(DMC_type* dmc, float e, float u_max, float u_min)
{
	float delta_u;
	float tmp;
	float * new_delta_u_past;
	new_delta_u_past = malloc(sizeof(float)*(dmc->D-1));
	
	mat_mul(dmc->Ku, 1, dmc->D-1, dmc->delta_u_past, dmc->D-1, 1, &tmp);
	delta_u = dmc->Ke*e - tmp;
	tmp = dmc->u + delta_u;
	
	if(tmp >  u_max) 
		tmp =  u_max;
	else if(tmp < u_min) 
		tmp = u_min;
	
	delta_u = tmp - dmc->u;
	dmc->u = tmp;
	
	mat_move_down(dmc->delta_u_past, dmc->D-1, 1, delta_u, new_delta_u_past);
	free(dmc->delta_u_past);
	dmc->delta_u_past = new_delta_u_past;
	
	return dmc->u;
}

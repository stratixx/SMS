#ifndef PID_H
#define PID_H

#include <inttypes.h>

typedef struct
{
	float Tp;
	float K;
	float Ti;
	float Td;
	float Tv;
	
	float u_i_past;
	float u_w_past;
	float u_past;
	float e_past;


}PID_type;

void PID_init(PID_type*, float, float, float, float, float );
float PID_get_control(PID_type*, float, float, float);



#endif

#include "PID.h"

void PID_init(PID_type* pid, float _PID_Tp, float _PID_K, float _PID_Ti, float _PID_Td, float _PID_Tv)
{
	pid->Tp = _PID_Tp;
	pid->K = _PID_K;
	pid->Ti = _PID_Ti;
	pid->Td = _PID_Td;
	pid->Tv = _PID_Tv;

	pid->u_i_past = 0.0;
	pid->u_w_past = 0.0;
	pid->u_past = 0.0;
	pid->e_past = 0.0;
}

float PID_get_control(PID_type* pid, float e, float u_max, float u_min)
{
	float u_p = 0; // skladowa sterowania od P
	float u_i = 0; // skladowa sterowania od I
	float u_d = 0; // skladowa sterowania od D

	u_p = pid->K * e;

	u_i = pid->u_i_past + pid->K*pid->Tp*(pid->e_past+e)/2/pid->Ti;
	if( pid->Tv > 0.0 )
		u_i += pid->Tp*(pid->u_w_past - pid->u_past)/pid->Tv;

	u_d = pid->K*pid->Td*(e-pid->e_past)/pid->Tp;

	pid->u_past = u_p + u_i + u_d;
	pid->u_w_past = pid->u_past;

	if( pid->u_w_past > u_max )
		pid->u_w_past = u_max;
	if( pid->u_w_past < u_min )
		pid->u_w_past= u_min;

	pid->u_i_past = u_i;
	pid->e_past = e;

	return ( pid->u_w_past );
}

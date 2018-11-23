#include "PID.h"

///////////////////// TO DO
void PID_init(PID_type* pid, float p, float i, float d, uint8_t antiwindup)
{
	
}

///////////////////// TO DO
float PID_get_control(PID_type* pid, float e, float u_max, float u_min)
{
		static float u_i_past = 0; // przeszla skladowa sterowania od I
		float u_p = 0; // skladowa sterowania od P
		float u_i = 0; // skladowa sterowania od I
		float u_d = 0; // skladowa sterowania od D
		
		static float e_past = 0;
	
	u_p = PID_K * e;
		u_i = u_i_past + PID_K*PID_Tp*(e_past+e)/2/PID_Ti;
		u_d = PID_K*PID_Td*(e-e_past)/PID_Tp;
		
		u_i_past = u_i;
		e_past = e;
		return ( u_p + u_i + u_d);
	
}

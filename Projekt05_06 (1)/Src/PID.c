#include "PID.h"

/*
* Plik:			PID.c
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuąca regulator PID z funkcjonalnością anti wind-up
*/


/*
*	Inicjacja struktury regulatora PID funkcjonalnością z anti wind-up
*	pid			-	 wskaźnik na strukturę PID
*	_PID_Tp		-	 okres próbkowania
*	_PID_K		-	 wzmocnienie członu proporcjonalnego P
*	_PID_Ti		-	 parametr członu całkującego I
*	_PID_Td		-	 parametr członu róźniczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny jeśli mniejszy od zera
*/
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

/*
*	Wyznaczenie nowej wartości sterowania regulatora PID
*	pid		-	wskaźnik na strukturę regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna wartość sterowania
*	u_min	-	minimalna wartość sterowania
*/
float PID_get_control(PID_type* pid, float e, float u_max, float u_min)
{
	float u_p = 0; // skladowa sterowania od P
	float u_i = 0; // skladowa sterowania od I
	float u_d = 0; // skladowa sterowania od D

	// Źródło poniższych wzorów: wzory (2) ze skryptu
	// składowa P równa iloczynowi wzmocnienia K i uchybu sterowania
	u_p = pid->K * e;
	// składowa I powiększana co krok o  K*Tp*(e_past+e)/2/Ti
	u_i = pid->u_i_past + pid->K*pid->Tp*(pid->e_past + e)/2/pid->Ti;
	// anti wind-up, aktywny jeśli Tv>0; składowa I powiększana co krok o Tp*(u_w_past-u_past)/Tv
	// Źródło: wzór ze skryptu, str. 87
	if( pid->Tv > 0.0 )
		u_i += pid->Tp*(pid->u_w_past - pid->u_past)/pid->Tv;
	// składowa D równa K*Td*(e-e_past)/Tp
	u_d = pid->K*pid->Td*(e - pid->e_past)/pid->Tp;

	// wartość sterowania równa sumie składowych;
	// Źródło: wzór (1) ze skryptu
	pid->u_past = u_p + u_i + u_d;

	// u_w_past jest ograniczonym u_past - sterowanie przekazane do obiektu
	pid->u_w_past = pid->u_past;
	// nałożenie ograniczeń sterowanie
	if( pid->u_w_past > u_max )
		pid->u_w_past = u_max;
	if( pid->u_w_past < u_min )
		pid->u_w_past= u_min;

	pid->u_i_past = u_i;
	pid->e_past = e;

	return ( pid->u_w_past );
}

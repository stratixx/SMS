#include "PID.h"

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj¹ca regulator PID z funkcjonalnoœci¹ anti wind-up
*/


/*
*	Inicjacja struktury regulatora PID funkcjonalnoœci¹ z anti wind-up
*	pid			-	 wskaŸnik na strukturê PID
*	_PID_Tp		-	 okres próbkowania
*	_PID_K		-	 wzmocnienie cz³onu proporcjonalnego P
*	_PID_Ti		-	 parametr cz³onu ca³kuj¹cego I
*	_PID_Td		-	 parametr cz³onu ró¿niczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny jeœli mniejszy od zera
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
*	Wyznaczenie nowej wartoœci sterowania regulatora PID
*	pid		-	wskaŸnik na strukturê regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna wartoœæ sterowania
*	u_min	-	minimalna wartoœæ sterowania
*/
float PID_get_control(PID_type* pid, float e, float u_max, float u_min)
{
	float u_p = 0; // skladowa sterowania od P
	float u_i = 0; // skladowa sterowania od I
	float u_d = 0; // skladowa sterowania od D

	// Ÿród³o poni¿szych wzorów: wzory (2) ze skryptu
	// sk³adowa P równa iloczynowi wzmocnienia K i uchybu sterowania
	u_p = pid->K * e;
	// sk³adowa I powiêkszana co krok o  K*Tp*(e_past+e)/2/Ti
	u_i = pid->u_i_past + pid->K*pid->Tp*(pid->e_past + e)/2/pid->Ti;
	// anti wind-up, aktywny jeœli Tv>0; sk³adowa I powiêkszana co krok o Tp*(u_w_past-u_past)/Tv
	// Ÿród³o: wzór ze skryptu, str. 87
	if( pid->Tv > 0.0 )
		u_i += pid->Tp*(pid->u_w_past - pid->u_past)/pid->Tv;
	// sk³adowa D równa K*Td*(e-e_past)/Tp
	u_d = pid->K*pid->Td*(e - pid->e_past)/pid->Tp;

	// wartoœæ sterowania równa sumie sk³adowych;
	// Ÿród³o: wzór (1) ze skryptu
	pid->u_past = u_p + u_i + u_d;

	// u_w_past jest ograniczonym u_past - sterowanie przekazane do obiektu
	pid->u_w_past = pid->u_past;
	// na³o¿enie ograniczeñ na sterowanie
	if( pid->u_w_past > u_max )
		pid->u_w_past = u_max;
	if( pid->u_w_past < u_min )
		pid->u_w_past= u_min;

	pid->u_i_past = u_i;
	pid->e_past = e;

	return ( pid->u_w_past );
}

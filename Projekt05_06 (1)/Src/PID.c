#include "PID.h"

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj�ca regulator PID z funkcjonalno�ci� anti wind-up
*/


/*
*	Inicjacja struktury regulatora PID funkcjonalno�ci� z anti wind-up
*	pid			-	 wska�nik na struktur� PID
*	_PID_Tp		-	 okres pr�bkowania
*	_PID_K		-	 wzmocnienie cz�onu proporcjonalnego P
*	_PID_Ti		-	 parametr cz�onu ca�kuj�cego I
*	_PID_Td		-	 parametr cz�onu r�niczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny je�li mniejszy od zera
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
*	Wyznaczenie nowej warto�ci sterowania regulatora PID
*	pid		-	wska�nik na struktur� regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna warto�� sterowania
*	u_min	-	minimalna warto�� sterowania
*/
float PID_get_control(PID_type* pid, float e, float u_max, float u_min)
{
	float u_p = 0; // skladowa sterowania od P
	float u_i = 0; // skladowa sterowania od I
	float u_d = 0; // skladowa sterowania od D

	// �r�d�o poni�szych wzor�w: wzory (2) ze skryptu
	// sk�adowa P r�wna iloczynowi wzmocnienia K i uchybu sterowania
	u_p = pid->K * e;
	// sk�adowa I powi�kszana co krok o  K*Tp*(e_past+e)/2/Ti
	u_i = pid->u_i_past + pid->K*pid->Tp*(pid->e_past + e)/2/pid->Ti;
	// anti wind-up, aktywny je�li Tv>0; sk�adowa I powi�kszana co krok o Tp*(u_w_past-u_past)/Tv
	// �r�d�o: wz�r ze skryptu, str. 87
	if( pid->Tv > 0.0 )
		u_i += pid->Tp*(pid->u_w_past - pid->u_past)/pid->Tv;
	// sk�adowa D r�wna K*Td*(e-e_past)/Tp
	u_d = pid->K*pid->Td*(e - pid->e_past)/pid->Tp;

	// warto�� sterowania r�wna sumie sk�adowych;
	// �r�d�o: wz�r (1) ze skryptu
	pid->u_past = u_p + u_i + u_d;

	// u_w_past jest ograniczonym u_past - sterowanie przekazane do obiektu
	pid->u_w_past = pid->u_past;
	// na�o�enie ogranicze� na sterowanie
	if( pid->u_w_past > u_max )
		pid->u_w_past = u_max;
	if( pid->u_w_past < u_min )
		pid->u_w_past= u_min;

	pid->u_i_past = u_i;
	pid->e_past = e;

	return ( pid->u_w_past );
}

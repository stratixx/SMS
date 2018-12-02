#ifndef PID_H
#define PID_H

/*
* Plik:			PID.h
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuąca regulator PID z funkcjonalnością anti wind-up
*/

#include <inttypes.h>

typedef struct
{
	float Tp;			// okres próbkowania
	float K;			// wzmocnienie członu P
	float Ti;			// parametr członu I
	float Td;			// parametr członu D
	float Tv;			// parametr anti wind-up; nieaktywny jeśli mniejszy od zera
	float u_i_past;		// poprzednia wartość składowej całkowania I
	float u_w_past;		// poprzednia wartość sterowania przekazanego do obiektu
	float u_past;		// poprzednia wartość sterowania regulatora PID
	float e_past;		// poprzedni uchyb sterowania
}PID_type;


/*
*	Inicjacja struktury regulatora PID funkcjonalnością z anti wind-up
*	pid			-	 wskaźnik na strukturę PID
*	_PID_Tp		-	 okres próbkowania
*	_PID_K		-	 wzmocnienie członu proporcjonalnego P
*	_PID_Ti		-	 parametr członu całkującego I
*	_PID_Td		-	 parametr członu róźniczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny jeśli mniejszy od zera
*/
void PID_init(PID_type*, float, float, float, float, float );

/*
*	Wyznaczenie nowej wartości sterowania regulatora PID
*	pid		-	wskaźnik na strukturę regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna wartość sterowania
*	u_min	-	minimalna wartość sterowania
*/
float PID_get_control(PID_type*, float, float, float);



#endif

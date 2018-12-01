#ifndef PID_H
#define PID_H

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj�ca regulator PID z funkcjonalno�ci� anti wind-up
*/

#include <inttypes.h>

typedef struct
{
	float Tp;			// okres pr�bkowania
	float K;			// wzmocnienie cz�onu P
	float Ti;			// parametr cz�onu I
	float Td;			// parametr cz�onu D
	float Tv;			// parametr anti wind-up; nieaktywny je�li mniejszy od zera
	float u_i_past;		// poprzednia warto�� sk�adowej ca�kowania I
	float u_w_past;		// poprzednia warto�� sterowania przekazanego do obiektu
	float u_past;		// poprzednia warto�� sterowania regulatora PID
	float e_past;		// poprzedni uchyb sterowania
}PID_type;


/*
*	Inicjacja struktury regulatora PID funkcjonalno�ci� z anti wind-up
*	pid			-	 wska�nik na struktur� PID
*	_PID_Tp		-	 okres pr�bkowania
*	_PID_K		-	 wzmocnienie cz�onu proporcjonalnego P
*	_PID_Ti		-	 parametr cz�onu ca�kuj�cego I
*	_PID_Td		-	 parametr cz�onu r�niczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny je�li mniejszy od zera
*/
void PID_init(PID_type*, float, float, float, float, float );

/*
*	Wyznaczenie nowej warto�ci sterowania regulatora PID
*	pid		-	wska�nik na struktur� regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna warto�� sterowania
*	u_min	-	minimalna warto�� sterowania
*/
float PID_get_control(PID_type*, float, float, float);



#endif

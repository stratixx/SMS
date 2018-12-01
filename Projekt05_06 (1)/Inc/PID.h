#ifndef PID_H
#define PID_H

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj¹ca regulator PID z funkcjonalnoœci¹ anti wind-up
*/

#include <inttypes.h>

typedef struct
{
	float Tp;			// okres próbkowania
	float K;			// wzmocnienie cz³onu P
	float Ti;			// parametr cz³onu I
	float Td;			// parametr cz³onu D
	float Tv;			// parametr anti wind-up; nieaktywny jeœli mniejszy od zera
	float u_i_past;		// poprzednia wartoœæ sk³adowej ca³kowania I
	float u_w_past;		// poprzednia wartoœæ sterowania przekazanego do obiektu
	float u_past;		// poprzednia wartoœæ sterowania regulatora PID
	float e_past;		// poprzedni uchyb sterowania
}PID_type;


/*
*	Inicjacja struktury regulatora PID funkcjonalnoœci¹ z anti wind-up
*	pid			-	 wskaŸnik na strukturê PID
*	_PID_Tp		-	 okres próbkowania
*	_PID_K		-	 wzmocnienie cz³onu proporcjonalnego P
*	_PID_Ti		-	 parametr cz³onu ca³kuj¹cego I
*	_PID_Td		-	 parametr cz³onu ró¿niczkowego D
*	_PID_Tv		-	 parametr anti wind-up; nieaktywny jeœli mniejszy od zera
*/
void PID_init(PID_type*, float, float, float, float, float );

/*
*	Wyznaczenie nowej wartoœci sterowania regulatora PID
*	pid		-	wskaŸnik na strukturê regulatora
*	e		-	uchyb regulacji
*	u_max	-	maksymalna wartoœæ sterowania
*	u_min	-	minimalna wartoœæ sterowania
*/
float PID_get_control(PID_type*, float, float, float);



#endif

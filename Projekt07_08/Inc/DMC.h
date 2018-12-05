#ifndef DMC_H
#define DMC_H

/*
* Plik:			DMC.h
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementująca regulator DMC.
*				Parametry regulatora wyznaczane przy pomocy dedykowanych skryptów Matlaba.
*				Do wyznaczenia parametrów wymagana jest znana odpowiedź skokowa.
*				Skrypty: "DMC_init.m", "DMC_script.m", "exporter.m"
*/

#include <inttypes.h>

typedef struct
{
	uint8_t D;				// długość horyzontu dynamiki
	float Ke;				// suma pierwszego wiersza macierzy K
	float * Ku;				// iloczyn macierzowy pierwszego wiersza K i Mp
	float * delta_u_past;	// wektor przeszłych zmian sterowania
	float u;				// wartość sterowania
}DMC_type;


/*
*	Inicjacja struktury regulatora DMC
*	dmc 		- wskaźnik na strukturę regulatora
*	_D 			- długość horyzontu dynamiki
*	_Ke 		- suma pierwszego wiersza macierzy K
*	_Ku 		- iloczyn macierzowy pierwszego wiersza K i Mp
*	u_initial 	- początkowa wartość wyjścia regulatora DMC
*/
void DMC_init(DMC_type*, uint8_t, float, float*, float);

/*
*	Wyznaczenie nowej wartości sterowania
*	dmc 		- wskaźnik na strukturę regulatora
*	e			- bieżący uchyb sterowania
*	u_max		- maksymalna wartość sterowania
*	u_min		- minimalna wartość sterowania
*/
float DMC_get_control(DMC_type*, float, float, float);


#endif

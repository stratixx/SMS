#ifndef DMC_H
#define DMC_H

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj�ca regulator DMC.
*				Parametry regulatora wyznaczane przy pomocy dedykowanych skrypt�w Matlaba.
*				Do wyznaczenia parametr�w wymagana jest znana odpowied� skokowa.
*				Skrypty: "DMC_init.m", "DMC_script.m", "exporter.m"
*/

#include <inttypes.h>

typedef struct
{
	uint8_t D;				// d�ugo�� horyzontu dynamiki
	float Ke;				// suma pierwszego wiersza macierzy K
	float * Ku;				// iloczyn macierzowy pierwszego wiersza K i Mp
	float * delta_u_past;	// wektor przesz�ych zmian sterowania
	float u;				// warto�� sterowania
}DMC_type;


/*
*	Inicjacja struktury regulatora DMC
*	dmc 		- wska�nik na struktur� regulatora
*	_D 			- d�ugo�� horyzontu dynamiki
*	_Ke 		- suma pierwszego wiersza macierzy K
*	_Ku 		- iloczyn macierzowy pierwszego wiersza K i Mp
*	u_initial 	- pocz�tkowa warto�� wyj�cia regulatora DMC
*/
void DMC_init(DMC_type*, uint8_t, float, float*, float);

/*
*	Wyznaczenie nowej warto�ci sterowania
*	dmc 		- wska�nik na struktur� regulatora
*	e			- bie��cy uchyb sterowania
*	u_max		- maksymalna warto�� sterowania
*	u_min		- minimalna warto�� sterowania
*/
float DMC_get_control(DMC_type*, float, float, float);


#endif

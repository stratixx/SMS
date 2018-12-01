#ifndef DMC_H
#define DMC_H

/*
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementuj¹ca regulator DMC.
*				Parametry regulatora wyznaczane przy pomocy dedykowanych skryptów Matlaba.
*				Do wyznaczenia parametrów wymagana jest znana odpowiedŸ skokowa.
*				Skrypty: "DMC_init.m", "DMC_script.m", "exporter.m"
*/

#include <inttypes.h>

typedef struct
{
	uint8_t D;				// d³ugoœæ horyzontu dynamiki
	float Ke;				// suma pierwszego wiersza macierzy K
	float * Ku;				// iloczyn macierzowy pierwszego wiersza K i Mp
	float * delta_u_past;	// wektor przesz³ych zmian sterowania
	float u;				// wartoœæ sterowania
}DMC_type;


/*
*	Inicjacja struktury regulatora DMC
*	dmc 		- wskaŸnik na strukturê regulatora
*	_D 			- d³ugoœæ horyzontu dynamiki
*	_Ke 		- suma pierwszego wiersza macierzy K
*	_Ku 		- iloczyn macierzowy pierwszego wiersza K i Mp
*	u_initial 	- pocz¹tkowa wartoœæ wyjœcia regulatora DMC
*/
void DMC_init(DMC_type*, uint8_t, float, float*, float);

/*
*	Wyznaczenie nowej wartoœci sterowania
*	dmc 		- wskaŸnik na strukturê regulatora
*	e			- bie¿¹cy uchyb sterowania
*	u_max		- maksymalna wartoœæ sterowania
*	u_min		- minimalna wartoœæ sterowania
*/
float DMC_get_control(DMC_type*, float, float, float);


#endif

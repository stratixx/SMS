#include "mat_lib.h"
#include "DMC.h"
#include <stdlib.h>
#include <string.h>

/*
* Plik:			DMC.c
* Autor:		Konrad Winnicki
* E-mail:		konrad_winnicki@wp.pl
* Przedmiot:	SMS
* Semestr:		18Z
* Opis:			Biblioteka implementująca regulator DMC.
*               Wersja druga - bez uzycia malloc w DMC_get_control()
*				Parametry regulatora wyznaczane przy pomocy dedykowanych skryptów Matlaba.
*				Do wyznaczenia parametrów wymagana jest znana odpowiedź skokowa.
*				Skrypty: "DMC_init.m", "DMC_script.m", "exporter.m"
*/


/*
*	Inicjacja struktury regulatora DMC
*	dmc 		- wskaźnik na strukturę regulatora
*	_D 			- długość horyzontu dynamiki
*	_Ke 		- suma pierwszego wiersza macierzy K
*	_Ku 		- iloczyn macierzowy pierwszego wiersza K i Mp
*	u_initial 	- początkowa wartość wyjścia regulatora DMC
*/
void DMC_init(DMC_type* dmc, uint8_t _D, float _Ke, float* _Ku, float u_initial)
{
	uint8_t n=0;

	dmc->D = _D;
	dmc->Ke = _Ke;
	// alokacja pamięci dla wektora współczynników regulatora DMC
	dmc->Ku = malloc(sizeof(float)*(dmc->D-1));
	// alokacja pamięci dla wektora przeszłych zmian sterowania
	dmc->delta_u_past = malloc(sizeof(float)*(dmc->D-1));
	dmc->u = u_initial;
	// przekopiowanie wektora do zaalokowanego obszaru pamięci
	memcpy(dmc->Ku, _Ku, sizeof(float)*(dmc->D-1) );
	// wyzerowanie poprzednich wartosci zmian sterowawnia
	for(n=0;n<(dmc->D-1);n++)
		dmc->delta_u_past[n] = 0.0;
}

/*
*	Wyznaczenie nowej wartości sterowania
*	dmc 		- wskaźnik na strukturę regulatora
*	e			- bieżący uchyb sterowania
*	u_max		- maksymalna wartość sterowania
*	u_min		- minimalna wartość sterowania
*/
float DMC_get_control(DMC_type* dmc, float e, float u_max, float u_min)
{
	float delta_u;
	float tmp;
	float * new_delta_u_past;
	//new_delta_u_past = malloc(sizeof(float)*(dmc->D-1));

	// iloczyn wektorów współczynników Ku i przeszłych zmian sterowania delta_u_past
	// u(k|k) = u(k-1) + Ke*e(k) - Ku*deltaUp(k)
	// Źródło: wzór u(k|k), str. 90 skryptu do projektu 1 SMS
	mat_mul(dmc->Ku, 1, dmc->D-1, dmc->delta_u_past, dmc->D-1, 1, &tmp);
	// wyznaczenie nowej zmiany sterowania
	delta_u = dmc->Ke*e - tmp;
	// wyznaczenie nowej wartości sterowania
	tmp = dmc->u + delta_u;
	// nałożenie ograniczeń na sterowanie
	if(tmp >  u_max)
		tmp =  u_max;
	else if(tmp < u_min)
		tmp = u_min;
	// przekazanie do regulatora osiągniętej zmiany sterowania
	delta_u = tmp - dmc->u;
	dmc->u = tmp;
	// przesunięcie wektora przeszłych zmian sterowania o jeden krok w tył i
	// wstawienie bieżącej zmiany sterowania na początek
	mat_move_down(dmc->delta_u_past, dmc->D-1, 1, delta_u, dmc->delta_u_past);
	// zastąpienie przeszłego wektora nowym
	//free(dmc->delta_u_past);
	//dmc->delta_u_past = new_delta_u_past;

	return dmc->u;
}

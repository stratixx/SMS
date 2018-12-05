% Plik:			DMC_script.m
% Autor:		Konrad Winnicki
% E-mail:		konrad_winnicki@wp.pl
% Przedmiot:	SMS
% Semestr:		18Z
% Opis:			Skrypt wyliczający parametry regulatora DMC
%				przeznaczonego do uruchomienia w systemie wbudowanym

% Załadowanie odpowiedzi skokowej obiektu
load('s_D164_druga_proba.mat')

% Założone parametry regulatora
D = length(s);		% horyzont dynamiki
N=100;				% horyzont predykcji
Nu=50;				% horyzont sterowania
lambda = 10		% kara za zmienność sterowania
run('DMC_init.m');

Ke = sum(K(1,:));
Ku = K(1,:)*Mp;

% wyeksportowanie wyznaczonych parametrów do pliku nagłówkowego zgodnego ze standardem języka C
run('exporter.m');

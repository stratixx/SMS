% Autor:		Konrad Winnicki
% E-mail:		konrad_winnicki@wp.pl
% Przedmiot:	SMS
% Semestr:		18Z
% Opis:			Skrypt wyliczaj¹cy parametry regulatora DMC
%				przeznaczonego do uruchomienia w systemie wbudowanym

% Za³adowanie odpowiedzi skokowej obiektu
load('s_D44.mat')

% Za³o¿one parametry regulatora
D = length(s);		% horyzont dynamiki
N=5;				% horyzont predykcji
Nu=1;				% horyzont sterowania
lambda = 0.1		% kara za zmiennoœæ sterowania
run('DMC_init.m');

Ke = sum(K(1,:));
Ku = K(1,:)*Mp;

% wyeksportowanie wyznaczonych parametrów do pliku nag³ówkowego zgodnego ze standardem jêzyka C
run('exporter.m');

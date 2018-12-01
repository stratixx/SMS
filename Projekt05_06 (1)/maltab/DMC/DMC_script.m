% Autor:		Konrad Winnicki
% E-mail:		konrad_winnicki@wp.pl
% Przedmiot:	SMS
% Semestr:		18Z
% Opis:			Skrypt wyliczaj�cy parametry regulatora DMC
%				przeznaczonego do uruchomienia w systemie wbudowanym

% Za�adowanie odpowiedzi skokowej obiektu
load('s_D44.mat')

% Za�o�one parametry regulatora
D = length(s);		% horyzont dynamiki
N=5;				% horyzont predykcji
Nu=1;				% horyzont sterowania
lambda = 0.1		% kara za zmienno�� sterowania
run('DMC_init.m');

Ke = sum(K(1,:));
Ku = K(1,:)*Mp;

% wyeksportowanie wyznaczonych parametr�w do pliku nag��wkowego zgodnego ze standardem j�zyka C
run('exporter.m');

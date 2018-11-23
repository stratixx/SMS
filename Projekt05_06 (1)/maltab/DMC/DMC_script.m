load('s.mat')

D = length(s); % horyzont dynamiki
N=D;
Nu=2;
lambda = 0.1
run('DMC_init.m');

Ke = sum(K(1,:));
Ku = K(1,:)*Mp;

run('exporter.m');
%{
TRASH
       %trajektoria swobodna
        yo = y(k)*ones(D,1)+Mp*flip(deltaUp((k-D+1):(k-1)));
        %przysz³e sterowania
        deltau = K*(yzad(k:(k+D-1))'-yo);
        %bie¿¹ca zmiana sterowania
        deltaUp(k) = deltau(1);
        %sygna³ sterujcy regulatora DMC       
        u(k) = u(k-1)+deltau(1);
  
Ke - suma pierwszego wiersza K
Ku - wektor D-1, pierwszy wiersz K razy Mp
delta_u = Ke*e - Ku*delta_u_past;
u = u_past + delta_u;
shift_vector(&delta_u_past);
insert_element_at_end(&delta_u_past, delta_u);


y_zad = 500;
y = 502;
u_past = 500;

delta_u_past = zeros(D-1,1);
e = y_zad - y;
delta_u = Ke*e - Ku*delta_u_past; % Ke jest skalarem, Ku wektorem o d³ugoœci D-1
u = u_past + delta_u;
% ogranicznik u
delta_u = u-u_past;
u_past = u;
%shift_vector(&delta_u_past);
%insert_element_at_end(&delta_u_past, delta_u);
%}

delete(instrfindall); % zamkniecie wszystkich polaczen szeregowych
clear all;
close all;
s = serial('COM10'); % COM19 to jest port utworzony przez mikrokontroler
set(s,'BaudRate',115200);
set(s,'StopBits',1);
set(s,'Parity','none');
set(s,'DataBits',8);
set(s,'Timeout',3);
set(s,'InputBufferSize',1000);
set(s,'Terminator',13);
fopen(s); % otwarcie kanalu komunikacyjnego
Tp = 1; % czas z jakim probkuje regulator
y = zeros(1,500); % wektor wyjsc obiektu
y_zad = zeros(1,500); % wektor wyjsc zadanych obiektu
u = zeros(1,500); % wektor wejsc (sterowan) obiektu
Y = 0;
Yzad=0;
U=0;
while true % zbieramy 1000 pomiarow
    txt = fread(s,40); % odczytanie z portu szeregowego
    % txt powinien zawiera´c Y=%4d;U=%4d;
    % czyli np. Y=1234;U=3232;
    eval(char(txt')) % wykonajmy to co otrzymalismy
    
    y(end)=Y; % powiekszamy wektor y o element Y
    y_zad(end)=Yzad; % powiekszamy wektor y o element Y
    u(end)=U; % powiekszamy wektor u o element U
    
    figure(1);
    clf(1);
    hold on;
    title('y');
    grid on;
    xlabel('time');
    ylabel('value');
    plot(y); % wyswietlamy y w czasie
    plot(y_zad); % wyswietlamy y w czasie
    %legend('y','y_z_a_d')
    
    figure(2); 
    clf(2);
    hold on;
    title('u');
    grid on;
    xlabel('time');
    ylabel('value');
    plot(u); % wyswietlamy u w czasie
    %legend('u')
    
    drawnow;
    
    y = circshift(y,[0,-1]);
    y_zad = circshift(y_zad,[0,-1]);
    u = circshift(u,[0,-1]);
end

figure; plot((0:(length(y)-1))*Tp,y); % wyswietlamy y w czasie
figure; plot((0:(length(u)-1))*Tp,u); % wyswietlamy u w czasie

%daneDynUcz = dlmread('../danedynucz43.txt');
%daneDynWer = dlmread('../danedynwer43.txt');
daneDynUczU = u';
daneDynUczY = y';
daneDynWerU = u';
daneDynWerY = y';
dataLength = length(daneDynUczU);
%clear daneDynUcz daneDynWer

Nmax = 2;
Na=2;
Nb=Na;
N=1;
minErrVerify = 100000.0;
minErrVerifyN = 1000;
minErrDelta = 0.2;
errArray = zeros(Nmax,3);

for Na=1:1:Nmax
    Nb=2;
    errArray(Na, 1) = Na;
    
    Mlearn = ones(dataLength-max([Na,Nb]), Na+Nb);
    Mlearn1 = Mlearn;
    for n=0:1:(Na-1)
        Mlearn(:,n+1) = daneDynUczU( (max([Na,Nb])-n):(end-n-1) );
    end
    
    Mlearn2 = Mlearn;
    for n=(0):1:(Nb-1)
        Mlearn(:,Na+n+1) = daneDynUczY( (max([Na,Nb])-n):(end-n-1) );
    end

    Mlearn3 = Mlearn;
    Wlearn = Mlearn\daneDynUczY(max([Na,Nb]+1):end);
    YlearnCalc = Mlearn*Wlearn;
    errLearn = (sum(power( daneDynUczY(max([Na,Nb]+1):end)-YlearnCalc, 2 )));
    errArray(Na, 2) = errLearn;
    
    Mverif = ones(dataLength-max([Na,Nb]), Na+Nb);
    for n=0:1:(Na-1)
        Mverif(:,n+1) = daneDynWerU( (max([Na,Nb])-n):(end-n-1) );
    end
    
    for n=(0):1:(Nb-1)
        Mverif(:,Na+n+1) = daneDynWerY( (max([Na,Nb])-n):(end-n-1) );
    end
    
    %Wverif = Mverif\daneDynWerY(max([Na,Nb]+1):end);
    YverifCalc = Mverif*Wlearn;
    errVerif = (sum(power( daneDynWerY(max([Na,Nb]+1):end)-YverifCalc, 2 )));
    errArray(Na, 3) = errVerif;
    
    if errVerif<(minErrVerify-minErrDelta)
        minErrVerify = errVerif;
        minErrVerifyN = Na;
    end
    %continue
    figure(1)
    
    subplot(2,1,1);
    hold on; grid on; box on;
    plot( daneDynUczY, '.');
    plot( YlearnCalc);
    title(strcat('Model dynamiczny na tle zbioru danych ucz¹cych, Nb=Na=',num2str(Na),', err=',num2str(errLearn)));
    %xlabel('Próbki');
    ylabel('Sygna³ wyjœciowy y');
    legend('Dane ucz¹ce','Wyjœcie modelu','location','southeast')
    %print(strcat('img/bc/noRecur/learn/learn_Nb_',num2str(Nb),'_Na_',num2str(Na)), '-dpng');
    %close 1;
    
    %figure(2)
    subplot(2,1,2);
    hold on; grid on; box on;
    plot( daneDynWerY, '.');
    plot( YverifCalc);
    title(strcat('Model dynamiczny na tle zbioru danych weryfikuj¹cych, Nb=Na=',num2str(Na),', err=',num2str(errVerif)));
    xlabel('Próbki');
    ylabel('Sygna³ wyjœciowy y');
    legend('Dane weryfikuj¹ce','Wyjœcie modelu','location','southeast')
    print(strcat('img/Nb_',num2str(Nb),'_Na_',num2str(Na)), '-dpng');
    close 1;
end

errArray
display( strcat( 'Wybrany model: N= ', num2str(minErrVerifyN), '; errVerif= ', num2str(minErrVerify) ) );

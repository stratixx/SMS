% Plik:			exporter.m
% Autor:		Konrad Winnicki
% E-mail:		konrad_winnicki@wp.pl
% Przedmiot:	SMS
% Semestr:		18Z
% Opis:			Skrypt eksportujący wyliczone parametry regulatora DMC do postaci
%				zgodnej ze standardem języka C

% powstanie plik "DMC_data.h" w folderze Inc
fileID = fopen('../../Inc/DMC_data.h','w');

fprintf(fileID,'#ifndef DMC_DATA_H\n#define DMC_DATA_H\n\n');
fprintf(fileID,'#include <inttypes.h>\n\n', D);

fprintf(fileID,'//Parametry regulatora DMC\n', D);
fprintf(fileID,'uint8_t DMC_D = %d;\n', D);
fprintf(fileID,'uint8_t DMC_N = %d;\n', N);
fprintf(fileID,'uint8_t DMC_Nu = %d;\n', Nu);
fprintf(fileID,'float DMC_lambda = %f;\n\n', lambda);

fprintf(fileID,'// Przeliczone wartosci do sterowania DMC\n', D);
fprintf(fileID,'float DMC_Ke = %f;\n\n', Ke);

fprintf(fileID,'float DMC_Ku[] =\n{\n');
fprintf(fileID,'    %f, \n',Ku)
fprintf(fileID,'};\n\n');


fprintf(fileID,'#endif\n');
fclose(fileID);
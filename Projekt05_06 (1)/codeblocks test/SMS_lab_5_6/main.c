#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "PID.h"
#include "DMC.h"
#include "DMC_data.h"
float get_y_zad(uint32_t);
float simulate_object(float);

int main()
{
        static float y = 0.0f;
		static float u = 0.0f;


		float e = 0;

		float y_zad = 1000;

		uint32_t k=0;

		float input, output = 2047;

		DMC_type dmc;
		DMC_init(&dmc, DMC_D, DMC_Ke, DMC_Ku, u);

    while(1)
    {
        y_zad = get_y_zad(k);

        input = simulate_object(output);
		y = (input);//-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047
		e = y_zad-y;

		u = DMC_get_control(&dmc, e, 2047, -2048);

		if(u < -2048.0f) u = -2048.0f;
		if(u >  2047.0f) u =  2047.0f;
		output = u;//+2048.0f; // przejscie z -2048 - 2047 do 0 - 4095

		printf("k=%4d;U=%+8.2f;Y=%+8.2f;Yzad=%+8.2f;\n\r",k,u,y,y_zad); // 22 znaki


		Sleep(20);
		k++;
    }
}

float get_y_zad(uint32_t k)
{
    if(k<500) return 0;
    if(k<1000) return 500;
    if(k<1500) return 0;
    if(k<2000) return -500;
    return 0;

}

float simulate_object(float u)
{
    static float y1=0.0, y2=0.0;
    float y;

    float b1=0.0174, a1=1.7255, a2=-0.7429;

    y = b1*u + a1*y1 + a2*y2;

    if(y>2047)
        y = 2047;
    else if(y<-2048)
        y = -2048;

    y2 = y1;
    y1 = y;
    return y;
}

/*



*/


#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

#include "GUI.h"


char* control_mode_tab[] = { " AUTO ", "MANUAL" };
uint8_t control_mode;

char* control_mode_cv_tab[] = { "Tzad", " G1 " };
//uint8_t control_mode_cv;

char* alarm_state_tab[] = { "NO ALARM           ", "T1 FAILURE         ", "T TOO LOW          ", "COMMUNICATION ERROR" };
uint8_t alarm_state;

uint16_t bar_T1_green_area_min = BAR_T1_HEIGHT-4;
uint16_t bar_T1_green_area_max = 1;

/* tablica na dane tymczasowe */
char tab[10];

/* 
*   Inicjacja GUI
*/
void GUI_init()
{
    control_mode = CONTROL_MODE_AUTO;
    //control_mode_cv = CONTROL_MODE_CV_AUTO;
    alarm_state = ALARM_NONE;
}

/*
*   Przerysowanie wszystkich elementów
*/
void GUI_repaint_all()
{    
    BSP_LCD_SetFont(&Font12);

    /* rysowanie obszaru słupków */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(BAR_G1_X0, BAR_G1_Y0-12-2, "G1", LEFT_MODE);  /* napis "G1" powyżej słupka G1 */
    BSP_LCD_DisplayStringAt(BAR_T1_X0, BAR_T1_Y0-12-2, "T1", LEFT_MODE);  /* napis "T1" powyżej słupka T1 */
    //BSP_LCD_DisplayStringAt(BAR_W1_X0, BAR_W1_Y0-12-2, "W1", LEFT_MODE);  /* napis "W1" powyżej słupka W1 */	
    //GUI_display_G1_value(50.0);
    //GUI_display_W1_value(50.0);
    //GUI_display_T1_value(51.3);	
    GUI_display_T1_limits(60.0, 30.0);
    GUI_display_T1_setpoint(48.2);
	
    /* rysowanie obszaru wizualizacji modelu */ /* DODAJ PODPISY */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(MODEL_VIEW_BACKGROUND_X0-20, MODEL_VIEW_BACKGROUND_Y0-22, "CONTROLLED OBJECT VIEW", LEFT_MODE );
    /* szary prostokąt tła */
    BSP_LCD_SetTextColor(MODEL_VIEW_BACKGROUND_COLOR);
    BSP_LCD_FillRect(MODEL_VIEW_BACKGROUND_X0, MODEL_VIEW_BACKGROUND_Y0, MODEL_VIEW_BACKGROUND_WIDTH, MODEL_VIEW_BACKGROUND_HEIGHT);
    /* niebieski prostokąt W1 */
    BSP_LCD_SetTextColor(MODEL_VIEW_W1_COLOR);
    BSP_LCD_FillRect(MODEL_VIEW_W1_X0, MODEL_VIEW_W1_Y0, MODEL_VIEW_W1_WIDTH, MODEL_VIEW_W1_HEIGHT);
    /* pomarańczowy prostokąt T1 */
    BSP_LCD_SetTextColor(MODEL_VIEW_T1_COLOR);
    BSP_LCD_FillRect(MODEL_VIEW_T1_X0, MODEL_VIEW_T1_Y0, MODEL_VIEW_T1_WIDTH, MODEL_VIEW_T1_HEIGHT);
    /* lekko żółty prostokąt G1 */
    BSP_LCD_SetTextColor(MODEL_VIEW_G1_COLOR);
    BSP_LCD_FillRect(MODEL_VIEW_G1_X0, MODEL_VIEW_G1_Y0, MODEL_VIEW_G1_WIDTH, MODEL_VIEW_G1_HEIGHT);
    /* pomarańczowy prostokąt T5 */
    BSP_LCD_SetTextColor(MODEL_VIEW_T5_COLOR);
    BSP_LCD_FillRect(MODEL_VIEW_T5_X0, MODEL_VIEW_T5_Y0, MODEL_VIEW_T5_WIDTH, MODEL_VIEW_T5_HEIGHT);
		/* podpisy na bloczkach modelu */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(MODEL_VIEW_W1_X0+15, MODEL_VIEW_W1_Y0+50, "W1", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MODEL_VIEW_T1_X0+15, MODEL_VIEW_T1_Y0+15, "T1", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MODEL_VIEW_G1_X0+15, MODEL_VIEW_G1_Y0+32, "G1", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MODEL_VIEW_T5_X0+15, MODEL_VIEW_T5_Y0+15, "T5", LEFT_MODE);

    /* rysowanie obszaru przycisków sterujących */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    /* tekst "adjustment" */
    BSP_LCD_DisplayStringAt(TEXT_ADJUSTMENT_X0, TEXT_ADJUSTMENT_Y0, "ADJUSTMENT:", LEFT_MODE);
    /* tekst pod "adjustment" - aktualnie sterowana wartość */
    BSP_LCD_DisplayStringAt(TEXT_CV_X0, TEXT_CV_Y0, (uint8_t*)control_mode_cv_tab[control_mode], LEFT_MODE);
	/* przyciski precyzyjnej zmiany wartości zadanej */ 
    BSP_LCD_DisplayStringAt(TEXT_SMALL_STEP_X0, TEXT_SMALL_STEP_Y0, "STEP BY 0.1:", LEFT_MODE );   
	BSP_LCD_DrawRect(BT_CV_MINUS_SMALL_X0, BT_CV_MINUS_SMALL_Y0, BT_CV_MINUS_SMALL_WIDTH, BT_CV_MINUS_SMALL_HEIGHT);
	BSP_LCD_DisplayStringAt(BT_CV_MINUS_SMALL_X0+10, BT_CV_MINUS_SMALL_Y0+10, "-", LEFT_MODE);
	BSP_LCD_DrawRect(BT_CV_PLUS_SMALL_X0, BT_CV_PLUS_SMALL_Y0, BT_CV_PLUS_SMALL_WIDTH, BT_CV_PLUS_SMALL_HEIGHT);
	BSP_LCD_DisplayStringAt(BT_CV_PLUS_SMALL_X0+10, BT_CV_PLUS_SMALL_Y0+10, "+", LEFT_MODE);
	/* przyciski zgrubnej zmiany wartości zadanej */
    BSP_LCD_DisplayStringAt(TEXT_BIG_STEP_X0, TEXT_BIG_STEP_Y0, "STEP BY 2.0:  ", LEFT_MODE ); 
	BSP_LCD_DrawRect(BT_CV_MINUS_BIG_X0, BT_CV_MINUS_BIG_Y0, BT_CV_MINUS_BIG_WIDTH, BT_CV_MINUS_BIG_HEIGHT);
	BSP_LCD_DisplayStringAt(BT_CV_MINUS_BIG_X0+10, BT_CV_MINUS_BIG_Y0+10, "-", LEFT_MODE);
	BSP_LCD_DrawRect(BT_CV_PLUS_BIG_X0, BT_CV_PLUS_BIG_Y0, BT_CV_PLUS_BIG_WIDTH, BT_CV_PLUS_BIG_HEIGHT);
	BSP_LCD_DisplayStringAt(BT_CV_PLUS_BIG_X0+10, BT_CV_PLUS_BIG_Y0+10, "+", LEFT_MODE);
    /* tekst control mode nad przyciskiem auto/manual */
    BSP_LCD_DisplayStringAt(TEXT_CONTROL_X0, TEXT_CONTROL_Y0,  "CONTROL MODE:", LEFT_MODE);
    /* przycisk auto/manual */
	BSP_LCD_DrawRect(BT_CONTROL_X0, BT_CONTROL_Y0, BT_CONTROL_WIDTH, BT_CONTROL_HEIGHT);
		GUI_display_auto_manual(control_mode);
    
    /* rysowanie obszaru alarmów */
    /* napis "ALARMS:" */
		BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(TEXT_ALARMS_X0, TEXT_ALARMS_Y0, "ALARMS:", LEFT_MODE);
    /* pole wyświetlania alarmów */
    if(alarm_state==ALARM_NONE)
        BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
    else
        BSP_LCD_SetBackColor(TEXT_ALARMS_BACKGROUND);
    BSP_LCD_DisplayStringAt(TEXT_ALARMS_FIELD_X0, TEXT_ALARMS_FIELD_Y0, (uint8_t*)alarm_state_tab[alarm_state], LEFT_MODE);
		BSP_LCD_SetFont(&Font12);
}


/*
*   Wyswietlenie zmierzonej wartości G1
*/
void GUI_display_G1_value(float value)
{
    float y;
    /* zielony obszar bez alarmowania */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(BAR_G1_X0+1, BAR_G1_Y0+1, BAR_G1_WIDTH-2, BAR_G1_HEIGHT-2);
    /* wskaźnik wartości */
    y = BAR_G1_Y0+1+(BAR_G1_HEIGHT-2-3)/2-value*(BAR_G1_HEIGHT-2-3)/(VALUE_G1_MAX-VALUE_G1_MIN);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_FillRect(BAR_G1_X0+1, (uint16_t)y, BAR_G1_WIDTH-2, 3);
    /* wartość sterowania pod słupkiem */
    sprintf(tab, "%+2.1f", value);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(VALUE_G1_X0, VALUE_G1_Y0, (uint8_t*)tab, LEFT_MODE);
	
    BSP_LCD_DrawRect(BAR_G1_X0, BAR_G1_Y0, BAR_G1_WIDTH-1, BAR_G1_HEIGHT-1);
}

/*
*   Wyświetlenie zmierzonej wartości T1
*/
void GUI_display_T1_value(float value)
{ 
    float y;

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(BAR_T1_X0+1, BAR_T1_Y0+1, BAR_T1_WIDTH-2, BAR_T1_HEIGHT-2);
    /* zielony obszar bez alarmowania */
    //BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
    //BSP_LCD_FillRect(BAR_T1_X0+1, 1+bar_T1_green_area_max, BAR_T1_WIDTH-2, bar_T1_green_area_min);
    /* wskaźnik wartości */
    //y = BAR_T1_Y0+1+(BAR_T1_HEIGHT-2-3)-value*(BAR_T1_HEIGHT-2-3)/(VALUE_T1_MAX-VALUE_T1_MIN);
		y = (BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX)*value - VALUE_T1_MAX*(BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_FillRect(BAR_T1_X0+1, (uint16_t)y+1+BAR_T1_Y0, BAR_T1_WIDTH-2, 3);
    /* wartość sterowania pod słupkiem */
    sprintf(tab, "%2.1f", value);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(VALUE_T1_X0, VALUE_T1_Y0, (uint8_t*)tab, LEFT_MODE);
	
    BSP_LCD_DrawRect(BAR_T1_X0, BAR_T1_Y0, BAR_T1_WIDTH-1, BAR_T1_HEIGHT-1);
}

/*
*   Wyświeltenie limitów T1
*/
void GUI_display_T1_limits(float max, float min)
{
    //bar_T1_green_area_min = (BAR_T1_HEIGHT-2-3)/2-min*(BAR_T1_HEIGHT-2-3)/(VALUE_T1_MAX-VALUE_T1_MIN); 
    //bar_T1_green_area_max = (BAR_T1_HEIGHT-2-3)/2-max*(BAR_T1_HEIGHT-2-3)/(VALUE_T1_MAX-VALUE_T1_MIN); 
		bar_T1_green_area_min = (BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX)*min - VALUE_T1_MAX*(BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX);
		bar_T1_green_area_max = (BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX)*max - VALUE_T1_MAX*(BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX);

    
    /* czyszczenie obszaru na prawo od słupka T1 */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(BAR_T1_X0+BAR_T1_WIDTH+1, BAR_T1_Y0-15, 35, BAR_T1_HEIGHT+15);
    /* górny limit alarmowania */
    sprintf(tab, "%+2.1f", max);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt( BAR_T1_X0+BAR_T1_WIDTH+5, BAR_T1_Y0+2+bar_T1_green_area_max-15, (uint8_t*)tab, LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_DrawHLine(BAR_T1_X0+BAR_T1_WIDTH+1, BAR_T1_Y0+2+bar_T1_green_area_max, 40);
    /* dolny limit alarmowania */
    sprintf(tab, "%+2.1f", min);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt( BAR_T1_X0+BAR_T1_WIDTH+5, BAR_T1_Y0+2+bar_T1_green_area_min-15, (uint8_t*)tab, LEFT_MODE);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_DrawHLine(BAR_T1_X0+BAR_T1_WIDTH+1, BAR_T1_Y0+2+bar_T1_green_area_min, 40);

}

/*
*   Wyswietlenie wartości zadanej T1
*/
void GUI_display_T1_setpoint(float value)
{
    float y;
    /* czyszczenie obszaru na lewo od słupka T1 */
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(BAR_T1_X0-35, BAR_T1_Y0-15, 35, BAR_T1_HEIGHT+15+1);
    /* wskaźnik wartości zadanej na lewo od słupka T1 */
    //y = BAR_T1_Y0+1+(BAR_T1_HEIGHT-2)-value*(BAR_T1_HEIGHT-2)/(VALUE_T1_MAX-VALUE_T1_MIN);
		y = (BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX)*value - VALUE_T1_MAX*(BAR_T1_HEIGHT)/(VALUE_T1_MIN-VALUE_T1_MAX);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_DrawHLine(BAR_T1_X0-35, y+BAR_T1_Y0+2, 35-1);
    /* wskaźnik wartości zmierzonej T1 */
    sprintf(tab, "%+2.1f", value);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt( BAR_T1_X0-35, y-15+BAR_T1_Y0+2, (uint8_t*)tab, LEFT_MODE);

}

/*
*   Wyświetlenie wartości W1
*/
void GUI_display_W1_value(float value)
{
    float y;
    /* zielony obszar bez alarmowania */
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
    //BSP_LCD_FillRect(BAR_W1_X0+1, BAR_W1_Y0+1, BAR_W1_WIDTH-2, BAR_W1_HEIGHT-2);
    /* wskaźnik wartości */
    y = BAR_W1_Y0+1+(BAR_W1_HEIGHT-2-3)/2-value*(BAR_W1_HEIGHT-2-3)/(VALUE_W1_MAX-VALUE_W1_MIN);
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_FillRect(BAR_W1_X0+1, (uint16_t)y, BAR_W1_WIDTH-2, 3);
    /* wartość sterowania pod słupkiem */
    sprintf(tab, "%+2.1f", value);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(VALUE_W1_X0, VALUE_W1_Y0, (uint8_t*)tab, LEFT_MODE);

}

/*
* Wyświetlenie trybu sterowania auto/manual 
*/
void GUI_display_auto_manual(uint8_t mode)
{
    //if( control_mode == mode )
    //    return;
    control_mode = mode;
    //control_mode_cv = mode;

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    /* przycisk auto/manual */
    BSP_LCD_DisplayStringAt(BT_CONTROL_X0+20, BT_CONTROL_Y0+10, (uint8_t*)control_mode_tab[control_mode], LEFT_MODE);
    
    /* tekst pod "adjustment" - aktualnie sterowana wartość */
    BSP_LCD_DisplayStringAt(TEXT_CV_X0, TEXT_CV_Y0, (uint8_t*)control_mode_cv_tab[control_mode], LEFT_MODE);
}

/* 
*   Wyświetlenie alarmów
*   Wielokrotne wywołanie z tym samym argumentem nie powoduje kolejnych operacji LCD
*/
void GUI_display_alarm(uint8_t alarm)
{
    //if( alarm == alarm_state )
    //    return;
    alarm_state = alarm;
	
		if( alarm_state==ALARM_MODBUS_COM_ERROR )
		{
			BSP_LCD_SetFont(&Font24);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_FillRect(0, 0, 480, 272);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetBackColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAtLine(3, "FATAL ERROR!");
			BSP_LCD_DisplayStringAtLine(5, "MODBUS COMMUNICATION FAILED!!");
			return;
		}
		BSP_LCD_SetFont(&Font16);
    /* pole wyświetlania alarmów */
    if(alarm_state==ALARM_NONE)
        BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
    else
        BSP_LCD_SetBackColor(TEXT_ALARMS_BACKGROUND);

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(TEXT_ALARMS_FIELD_X0, TEXT_ALARMS_FIELD_Y0, (uint8_t*)alarm_state_tab[alarm_state], LEFT_MODE); 
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);   
		BSP_LCD_SetFont(&Font12);
}

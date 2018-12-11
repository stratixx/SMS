#ifndef GUI_H
#define GUI_H

/* LCD is 480x272 */


/* tryby zadawania wartości zadanej auto/manual */
//extern char* control_mode_tab[];
//extern uint8_t control_mode;
#define CONTROL_MODE_AUTO 			0
#define CONTROL_MODE_MANUAL 		1

/* sterowana wartość */
//extern char* control_mode_cv_tab[];
//extern uint8_t control_mode_cv;
//#define CONTROL_MODE_CV_AUTO		0
//#define CONTROL_MODE_CV_MANUAL		1

/* alarmy */
//extern char* alarm_state_tab[];
//extern uint8_t alarm_state;
#define ALARM_NONE 				    0	
#define ALARM_T1_OUT_OF_RANGE 	    1
#define ALARM_T_TOO_LOW				2
#define ALARM_MODBUS_COM_ERROR		3


/* słupek wartości G1 */
#define BAR_G1_X0	    15
#define BAR_G1_Y0	    25
#define BAR_G1_WIDTH	16
#define BAR_G1_HEIGHT	131

/* wartość G1 pod słupkiem G1 */
#define VALUE_G1_X0	    	15
#define VALUE_G1_Y0	    	160
#define VALUE_G1_MAX	    50.0
#define VALUE_G1_MIN	    -50.0
#define VALUE_G1_FONT_SIZE  12

/* słupka wartości T1 */
#define BAR_T1_X0	    70
#define BAR_T1_Y0	    25
#define BAR_T1_WIDTH	16
#define BAR_T1_HEIGHT	131

/* wartość T1 pod słupkiem T1 */
#define VALUE_T1_X0	    70
#define VALUE_T1_Y0	    160
#define VALUE_T1_MAX	99.9
#define VALUE_T1_MIN	20.0
#define VALUE_T1_FONT   12

/* pasek wartości W1 */
#define BAR_W1_X0	    130
#define BAR_W1_Y0	    25
#define BAR_W1_WIDTH	16
#define BAR_W1_HEIGHT	131

/* wartość W1 pod paskiem W1 */
#define VALUE_W1_X0	    130
#define VALUE_W1_Y0	    160
#define VALUE_W1_MAX	99.9
#define VALUE_W1_MIN	0.0
#define VALUE_W1_FONT   12

/* tekst określający obecnie sterowaną wartość, G1 lub Tzad */
#define TEXT_CV_X0	    400
#define TEXT_CV_Y0	    30
#define TEXT_CV_FONT    12

/* tekst "adjustment" nad tesktem określającym obecnie sterowaną wartość */
#define TEXT_ADJUSTMENT_X0	    383
#define TEXT_ADJUSTMENT_Y0	    15
#define TEXT_ADJUSTMENT_FONT    12

/* Tekst "SMALL STEP:" nad przyciskami precyzyjnej zmiany wartości sterowanej */
#define TEXT_SMALL_STEP_X0	    387
#define TEXT_SMALL_STEP_Y0	    52
#define TEXT_SMALL_STEP_FONT    12

/* Tekst "BIG STEP:  " nad przyciskami grubnej zmiany wartości sterowanej */
#define TEXT_BIG_STEP_X0	    392
#define TEXT_BIG_STEP_Y0	    52
#define TEXT_BIG_STEP_FONT      12

/* przycisk precyzyjnego zwiększenia wartości sterowania */
#define BT_CV_PLUS_SMALL_X0	        430
#define BT_CV_PLUS_SMALL_Y0	        66
#define BT_CV_PLUS_SMALL_WIDTH	    33
#define BT_CV_PLUS_SMALL_HEIGHT	    33

/* przycisk zgrubnego zwiększenia wartości sterowania */
#define BT_CV_PLUS_BIG_X0	        430
#define BT_CV_PLUS_BIG_Y0   	    122
#define BT_CV_PLUS_BIG_WIDTH	    33
#define BT_CV_PLUS_BIG_HEIGHT	    33

/* przycisk precyzyjnego zmniejszenia wartości sterowania */
#define BT_CV_MINUS_SMALL_X0    	382
#define BT_CV_MINUS_SMALL_Y0	    66
#define BT_CV_MINUS_SMALL_WIDTH	    33
#define BT_CV_MINUS_SMALL_HEIGHT	33

/* przycisk zgrubnego zmniejszenia wartości sterowania */
#define BT_CV_MINUS_BIG_X0	        382
#define BT_CV_MINUS_BIG_Y0	        122
#define BT_CV_MINUS_BIG_WIDTH	    33
#define BT_CV_MINUS_BIG_HEIGHT	    33

/* text "CONTROL MODE:" powyżej przycisku wybierania trybu sterowania AUTO/MANUAL */
#define TEXT_CONTROL_X0	            366
#define TEXT_CONTROL_Y0	            170
#define TEXT_CONTROL_FONT           12

/* przycisk wybierania trybu sterowania AUTO/MANUAL */
#define BT_CONTROL_X0	            360
#define BT_CONTROL_Y0	            185
#define BT_CONTROL_WIDTH	        112
#define BT_CONTROL_HEIGHT	        33

/* text "ALARMS" powyżej pola wypisywania alarmów */
#define TEXT_ALARMS_X0	            8
#define TEXT_ALARMS_Y0	            234
#define TEXT_ALARMS_FONT            12
#define TEXT_ALARMS_BACKGROUND	    LCD_COLOR_LIGHTRED

/* text zawierający aktualny alarm */
#define TEXT_ALARMS_FIELD_X0	            8
#define TEXT_ALARMS_FIELD_Y0	            252
#define TEXT_ALARMS_FIELD_FONT            12

/* struktury modelu obiektu sterowanego */
/* prostokąt szarego tła */
#define MODEL_VIEW_BACKGROUND_X0	    210
#define MODEL_VIEW_BACKGROUND_Y0	    35
#define MODEL_VIEW_BACKGROUND_FONT      12
#define MODEL_VIEW_BACKGROUND_WIDTH     120
#define MODEL_VIEW_BACKGROUND_HEIGHT	120
#define MODEL_VIEW_BACKGROUND_COLOR		LCD_COLOR_GRAY

/* prostokąt W1 */
#define MODEL_VIEW_W1_X0	    192
#define MODEL_VIEW_W1_Y0	    44
#define MODEL_VIEW_W1_FONT      12
#define MODEL_VIEW_W1_WIDTH     38
#define MODEL_VIEW_W1_HEIGHT	95
#define MODEL_VIEW_W1_COLOR		LCD_COLOR_BLUE

/* prostokąt T1 */
#define MODEL_VIEW_T1_X0	    240
#define MODEL_VIEW_T1_Y0	    70
#define MODEL_VIEW_T1_FONT      12
#define MODEL_VIEW_T1_WIDTH     38
#define MODEL_VIEW_T1_HEIGHT	38
#define MODEL_VIEW_T1_COLOR		LCD_COLOR_ORANGE

/* prostokąt G1 */
#define MODEL_VIEW_G1_X0	    286
#define MODEL_VIEW_G1_Y0	    53
#define MODEL_VIEW_G1_FONT      12
#define MODEL_VIEW_G1_WIDTH     38
#define MODEL_VIEW_G1_HEIGHT	75
#define MODEL_VIEW_G1_COLOR	LCD_COLOR_LIGHTYELLOW

/* prostokąt T5 */
#define MODEL_VIEW_T5_X0	    240
#define MODEL_VIEW_T5_Y0	    165
#define MODEL_VIEW_T5_FONT      12
#define MODEL_VIEW_T5_WIDTH     38
#define MODEL_VIEW_T5_HEIGHT	38
#define MODEL_VIEW_T5_COLOR	LCD_COLOR_ORANGE


/* 
*   Inicjacja GUI
*/
void GUI_init(void);

/*
*   Przerysowanie wszystkich elementów
*/
void GUI_repaint_all(void);

/*
*   Wyswietlenie zmierzonej wartości G1
*/
void GUI_display_G1_value(float);

/*
*   Wyświetlenie zmierzonej wartości T1
*/
void GUI_display_T1_value(float);

/*
*   Wyświeltenie limitów T1
*/
void GUI_display_T1_limits(float max, float min);

/*
*   Wyswietlenie wartości zadanej T1
*/
void GUI_display_T1_setpoint(float);

/*
*   Wyświetlenie wartości W1
*/
void GUI_display_W1_value(float);

/*
* Wyświetlenie trybu sterowania auto/manual 
*/
void GUI_display_auto_manual(uint8_t);

/* 
*   Wyświetlenie alarmów
*   Wielokrotne wywołanie z tym samym argumentem nie powoduje kolejnych operacji LCD
*/
void GUI_display_alarm(uint8_t);

#endif

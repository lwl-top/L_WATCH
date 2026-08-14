/**
 * @file lv_port_indev_templ.c
 *
 */

 /*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev_template.h"
#include "lvgl/lvgl.h"
//#include "./BSP/TOUCH/touch.h"
#include "CST816T.h"
#include "i2c.h"
#include "lvgl_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "spi.h"
#include "gpio.h"

/*********************
 *      DEFINES
 *********************/
extern TaskHandle_t LV_DEMOTask_Handler;       /*  */
extern TaskHandle_t regular_time_Task_Handler;
extern TaskHandle_t RTC_Task_Handler;
extern TaskHandle_t SENSOR_Task_Handler;
extern TaskHandle_t SCREEN_WAKE_Task_Handler;           /*  */

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y);

static void keypad_init(void);
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static bool keypad_is_pressed(void);
static void keypad_get_key(uint8_t * key);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;
lv_indev_t * indev_keypad;
lv_group_t * group;
/**********************
 *      MACROS
 **********************/
static bool close=false;
static bool key_pressed = false;
volatile uint8_t key_num=0;
volatile uint8_t key_flag=0;
extern screen_state_t current_state;
bool current_mode=false;
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;
    static lv_indev_drv_t key_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad if you have*/
    touchpad_init();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
	
	 /*------------------
     * keypad
    * -----------------*/
	
		keypad_init();
	  lv_indev_drv_init(&key_drv);
    key_drv.type = LV_INDEV_TYPE_KEYPAD;
    key_drv.read_cb = keypad_read;
    indev_keypad = lv_indev_drv_register(&key_drv);

    /*Set keypad group focus mode*/
    group = lv_group_create();
    lv_indev_set_group(indev_keypad, group);
}

/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void touchpad_init(void)
{
//    tp_dev.init();
	MX_I2C1_Init();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
	
    /*Save the pressed coordinates and the state*/
    if(touchpad_is_pressed()) 
    {
        touchpad_get_xy(&last_x, &last_y);
        data->state = LV_INDEV_STATE_PR;
			  reset_screen_timeout();
    } 
    else 
    {
        data->state = LV_INDEV_STATE_REL;
    }    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;
}

/*Return true is the touchpad is pressed*/
static bool touchpad_is_pressed(void)
{
	if(CST816T_ReadReg(CST816T_Register_FingerNum))
	{
		 return true;
	}
	return false;

}

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    /*Your code comes here*/
	
    (*x) = CST816T_GetXpos();
    (*y) = CST816T_GetYpos();
}
	
	 /*------------------
     * keypad
    * -----------------*/
static void keypad_init(void)
{
		key_init();
}
static bool enter_fag=false;
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
	    uint8_t last_key = 0;
    /*Get the current state of the key*/
	if(keypad_is_pressed()) {			 
		    keypad_get_key(&last_key);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }   
    data->key = last_key;
}

static bool keypad_is_pressed(void)
{
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET||HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        return true;
    }
		else {return false;}
}
static void keypad_get_key(uint8_t * key)
{
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET&&current_state==SCREEN_ACTIVE) {
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET);
       (*key)=LV_KEY_UP;
    }
    else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET&&current_state==SCREEN_ACTIVE) {
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
       (*key)=LV_KEY_DOWN;
    }
		else{(*key)=0;}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    if(GPIO_Pin == GPIO_PIN_12)
    {
			 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET&&HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
				{
					 NVIC_SystemReset();
				}
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET&&key_flag==0)
        {	
					key_num=LV_KEY_SHORT;
        }	
				else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET&&key_flag==1)
				{
					 key_num=LV_KEY_LONG;
				}
				 else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET&&key_flag==2)
				{
					 NVIC_SystemReset();
				}
     
    }
}
#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif

#include "lvgl_demo.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include <time.h>

#include "MYI2C.h"
#include "CST816T.h"
#include "stdlib.h"
#include "W25Q64.h"
#include "PCF8563.h"
#include "tim.h"
#include "queue.h"
#include "lcd_init.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "mpu6050.h"
#include "MPU6050_I2C.h"
#include "BME280.h"
#include "HMC.h"
#include <math.h>
#include "gpio.h"
#include "MySPI.h"
#include "main.h"
#include "semphr.h"

#define START_TASK_PRIO     1           /* 开始任务优先级 */
#define START_STK_SIZE      128         /*  */
TaskHandle_t StartTask_Handler;         /*  */
void start_task(void *pvParameters);    /*  */


#define regular_time_TASK_PRIO        4         /* 时间任务优先级 */
#define regular_time_STK_SIZE        256         /*  */
TaskHandle_t regular_time_Task_Handler;           /* 时间任务句柄 */
void regular_time_task(void *pvParameters);      /*  */

#define LV_DEMO_TASK_PRIO   6           /* lvgl任务优先级 */
#define LV_DEMO_STK_SIZE    1024        /*  */
TaskHandle_t LV_DEMOTask_Handler;       /* lvgl任务句柄 */
void lv_demo_task(void *pvParameters);  /*  */

#define RTC_TASK_PRIO       5           /* 时钟任务优先级 */
#define RTC_STK_SIZE        256         /*  */
TaskHandle_t RTC_Task_Handler;           /* 时钟任务句柄  */
void RTC_task(void *pvParameters);      /* */

#define SENSOR_TASK_PRIO       3          /* 传感器任务优先级 */
#define SENSOR_STK_SIZE        256         /*  */
TaskHandle_t SENSOR_Task_Handler;           /* 传感器任务句柄 */
void SENSOR_task(void *pvParameters);      /*  */


#define SCREEN_WAKE_TASK_PRIO       2          /* 屏幕唤醒任务优先级 */
#define SCREEN_WAKE_STK_SIZE        256         /*  */
TaskHandle_t SCREEN_WAKE_Task_Handler;           /* 屏幕唤醒任务句柄 */
void SCREEN_WAKE_task(void *pvParameters);      /*  */

#define SCREEN_sleep_wake_TASK_PRIO       7          /* 屏幕唤醒任务优先级 */
#define SCREEN_sleep_wake_STK_SIZE        256         /*  */
TaskHandle_t SCREEN_sleep_wake_Task_Handler;           /* 屏幕唤醒任务句柄 */
void SCREEN_sleep_wake_task(void *pvParameters);      /*  */

static lv_obj_t *obj1;
static lv_obj_t *main_cont;
static lv_obj_t*wallpaper;
static lv_obj_t*next_main_cont;
static lv_obj_t * next_cont;
static lv_obj_t * circle;
static lv_obj_t * Calendar;
static lv_obj_t * calculator;
static lv_obj_t * WAllpaper;
static lv_obj_t * Watch_dial;
static lv_obj_t * About;
static lv_obj_t * Stopwatch;
static lv_obj_t * setting;
static lv_obj_t * Envirnment;
static lv_obj_t * Alarm_clock;
static lv_obj_t * Compass;
static lv_obj_t * calendaR;
static lv_obj_t * calendaR_cont;
static lv_obj_t * calculator_show;
static lv_obj_t * wallpaperer_show;
static lv_obj_t * about_show;
static lv_obj_t * Stop_watch_show;
static lv_obj_t * setting_show ;
static lv_obj_t * scr;
static lv_obj_t * clock_scr;
static lv_obj_t * compass_scr;
static lv_obj_t * shutdown_screen = NULL;
static lv_obj_t * time_label; 
static lv_obj_t * label2;
static lv_obj_t * label4;
static lv_obj_t * label6;
static lv_obj_t * time_appear_label;
static lv_obj_t * time_appear_label_sec;
static lv_obj_t * time_appear;
static lv_obj_t * week_appear;
static lv_obj_t * temperature_label ;
static lv_timer_t * vibration_timer = NULL;
static int vibration_tick_count = 0;
static bool vibration_on = false;
static lv_obj_t * step_label;
static int16_t time_pos_data_x=0;
static int16_t time_pos_data_y=88;
static int16_t day_pos_data_x=-20;
static int16_t day_pos_data_y=48;
static int16_t week_pos_data_x=25;
static int16_t week_pos_data_y=48;
LV_IMG_DECLARE(KLL2);
static lv_obj_t *img;
static lv_obj_t *game_2048_board_cont;
#define LIGHT_BG      lv_color_hex(0xFFFFFF)  
#define LIGHT_TEXT    lv_color_hex(0x000000)  
#define DARK_BG       lv_color_hex(0x333333)  
#define DARK_TEXT     lv_color_hex(0xFFFFFF)  
#define EYE_PROTECT_COLOR lv_color_hex(0x5E5E35)   

#define EYE_PROTECT_OPA  50  

static unsigned char Year, Month, Day, Hour, Min, Sec, Week;
extern unsigned char Timelist[];

screen_state_t current_state = SCREEN_ACTIVE;
static lv_timer_t* dim_timer = NULL;
static lv_timer_t* off_timer = NULL;
static lv_timer_t* sleep_timer = NULL;
static lv_timer_t* sensor_timer = NULL;
extern lv_group_t * group;
typedef struct
{
    int32_t temperature;   // 单位：0.01 °C
    uint32_t humidity;     // 单位：0.001 %RH
    uint32_t pressure;     // 单位：Pa
} BME280_Data_t;
QueueHandle_t DianLiang_queue;
QueueHandle_t temperature_queue;
QueueHandle_t temperature_only_queue;
QueueHandle_t HMC_queue;
QueueHandle_t STEP_queue;
QueueHandle_t SLEEP_queue;
SemaphoreHandle_t xTextModeSemaphore = NULL;
#define ALPHA 0.8
   int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData)
{
    return ALPHA * lastData + (1 - ALPHA) * newData;
}
#define M_PI 3.14159265358

static void calendar_event_GESTURE_handler(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
					lv_scr_load(next_main_cont);
          lv_obj_del_delayed(calendaR,15);
					calendaR = NULL;
					lv_group_focus_obj(next_main_cont); 
        }
}
static void calendar_event_key_handler(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							lv_scr_load(next_main_cont);
							lv_obj_del_delayed(calendaR,15);
							calendaR = NULL;
							lv_group_focus_obj(next_main_cont); 
						}
				}
}
void Calendar_show()
{
    calendaR = lv_obj_create(NULL);  
   lv_obj_remove_style_all(calendaR);                 
    calendaR_cont = lv_calendar_create(calendaR);  
   lv_obj_set_size(calendaR_cont,220,255);                       
   lv_obj_align(calendaR_cont,LV_ALIGN_CENTER,0,0);             
   lv_obj_add_event_cb(calendaR,calendar_event_GESTURE_handler,LV_EVENT_GESTURE,NULL); 
   lv_obj_add_event_cb(calendaR,calendar_event_key_handler,LV_EVENT_KEY,NULL); 

   lv_calendar_set_today_date(calendaR_cont,Year+2000,Month,Day);         
   static lv_calendar_date_t highlighted_days[3];
   highlighted_days[0].year  = Year+2000;
   highlighted_days[0].month = Month;
   highlighted_days[0].day   = Day;

   highlighted_days[1].year  = Year+2000;
   highlighted_days[1].month = Month;
   highlighted_days[1].day   = Day+1;

   highlighted_days[2].year  = Year+2000;
   highlighted_days[2].month = Month;
   highlighted_days[2].day   = Day+2;

   lv_calendar_set_highlighted_dates(calendaR_cont,highlighted_days,3);   
   lv_calendar_header_dropdown_create(calendaR_cont);        
   lv_calendar_set_showed_date(calendaR_cont,Year+2000,Month);                   
    lv_scr_load(calendaR);
	  lv_group_add_obj(group,calendaR);
}

typedef struct {
    char input_buf[32];    
    double prev_num;        
    char current_op;        
    bool new_input;         
    lv_obj_t *display;      
} CalcState;
static CalcState calc_state = {
    .input_buf = "0",
    .prev_num = 0,
    .current_op = 0,
    .new_input = true,
    .display = NULL
};
static void calculate() {
    double num = atof(calc_state.input_buf);
	
    switch(calc_state.current_op) {
        case '+':
            calc_state.prev_num += num;
            break;
        case '-':
            calc_state.prev_num -= num;
            break;
        case '*':
            calc_state.prev_num *= num;
            break;
        case '/':
            if (num == 0) {
                strcpy(calc_state.input_buf, "Error");
                calc_state.new_input = true;
                return;
            }
            calc_state.prev_num /= num;
            break;
        default:  
            calc_state.prev_num = num;
            break;
    }
    lv_snprintf(calc_state.input_buf, sizeof(calc_state.input_buf), "%g", calc_state.prev_num);
}
static void btn_event_handler(lv_event_t * e) {
    lv_obj_t * target = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(target, 0);
    const char * text = lv_label_get_text(label);

    if (strcmp(text, "C") == 0) {        
        strcpy(calc_state.input_buf, "0");
        calc_state.prev_num = 0;
        calc_state.current_op = 0;
        calc_state.new_input = true;
    }
    else if (strcmp(text, "=") == 0) {   
        if (calc_state.current_op) {
            calculate();
            calc_state.current_op = 0;
            calc_state.new_input = true;
        }
    }
    else if (strchr("+-*/", text[0])) {  
        if (calc_state.current_op) {
            calculate();
        }
        calc_state.current_op = text[0];
        calc_state.prev_num = atof(calc_state.input_buf);
        calc_state.new_input = true;
    }
    else {                               
        if (calc_state.new_input) {
            strcpy(calc_state.input_buf, text[0] == '.' ? "0." : text);
            calc_state.new_input = false;
        } else {
            if (text[0] == '.' && strchr(calc_state.input_buf, '.')) return;

            if (strlen(calc_state.input_buf) < sizeof(calc_state.input_buf) - 1) {
                strcat(calc_state.input_buf, text);
            }
        }
    }

    lv_label_set_text(calc_state.display, calc_state.input_buf);
}

lv_obj_t * create_btn(lv_obj_t * parent, const char * text, lv_color_t bg_color) {
    lv_obj_t * btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 50, 40);
    lv_obj_add_event_cb(btn, btn_event_handler, LV_EVENT_CLICKED, NULL);

    
    lv_obj_set_style_bg_color(btn, bg_color, LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(btn, lv_color_darken(bg_color, LV_OPA_20), LV_STATE_PRESSED);

    
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return btn;
}

static void calculator_show_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
          lv_scr_load(next_main_cont);
					lv_obj_del_delayed(calculator_show,15);
					calculator_show=NULL;
					lv_group_focus_obj(next_main_cont); 
       }
}
static void calculator_show_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
					lv_scr_load(next_main_cont);            
				 lv_obj_del_delayed(calculator_show,15);
				 calculator_show=NULL;
					lv_group_focus_obj(next_main_cont); 
						}
				}
}
void calculator_create(void) {
    
    calculator_show=lv_obj_create(NULL);
    lv_obj_add_event_cb(calculator_show, calculator_show_event_cb, LV_EVENT_GESTURE, NULL);
	  lv_obj_add_event_cb(calculator_show, calculator_show_key_event_cb, LV_EVENT_KEY, NULL);

    calc_state.display = lv_label_create(calculator_show);
    lv_obj_set_size(calc_state.display, 160, 25);
    lv_obj_align(calc_state.display, LV_ALIGN_TOP_MID,0,5);
    lv_label_set_text(calc_state.display, "0");
    lv_obj_set_style_text_align(calc_state.display, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(calc_state.display, &lv_font_montserrat_20, 0);

  
    lv_obj_t * btn_cont = lv_obj_create(calculator_show);
    lv_obj_set_size(btn_cont, 240, 250);
    lv_obj_align_to(btn_cont, calc_state.display, LV_ALIGN_OUT_BOTTOM_MID, 0, 3);

    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(btn_cont, 5, 0);

  
    create_btn(btn_cont, "7", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "8", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "9", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "*", lv_palette_main(LV_PALETTE_BLUE));

    create_btn(btn_cont, "4", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "5", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "6", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "-", lv_palette_main(LV_PALETTE_BLUE));

    create_btn(btn_cont, "1", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "2", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "3", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, "+", lv_palette_main(LV_PALETTE_BLUE));

    create_btn(btn_cont, "0", lv_palette_main(LV_PALETTE_GREY));
    create_btn(btn_cont, ".", lv_palette_main(LV_PALETTE_GREY));
		create_btn(btn_cont, "/", lv_palette_main(LV_PALETTE_BLUE)); 
		create_btn(btn_cont, "=", lv_palette_main(LV_PALETTE_BLUE));  
    create_btn(btn_cont, "C", lv_palette_main(LV_PALETTE_RED));

    lv_scr_load(calculator_show);
		lv_group_add_obj(group,calculator_show);

}

static void about_show_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
				lv_scr_load(next_main_cont);
				lv_obj_del_delayed(about_show,15);
					about_show=NULL;
				lv_group_focus_obj(next_main_cont); 
        }
}
static void about_show_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
						  	lv_scr_load(next_main_cont);
							 lv_obj_del_delayed(about_show,15);
							 about_show=NULL;
							lv_group_focus_obj(next_main_cont); 
						}
				}
}
LV_FONT_DECLARE(HEITi_14);
LV_FONT_DECLARE(HEITi_16);
void About_show(void)
{
		about_show=lv_obj_create(NULL);
		lv_obj_add_event_cb(about_show, about_show_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(about_show, about_show_key_event_cb, LV_EVENT_KEY, NULL);

		lv_obj_set_style_bg_color(about_show,lv_color_hex(0xE5E5E5),0);
	  lv_obj_set_flex_flow(about_show, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(about_show, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	
		lv_obj_t*title=lv_label_create(about_show);
		lv_label_set_text(title,"About");
		lv_obj_set_style_text_color(title,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title,LV_ALIGN_TOP_MID,0,5);


    lv_obj_t* main_watch=lv_obj_create(about_show);
    lv_obj_set_size(main_watch,210,70);
    lv_obj_set_style_bg_color(main_watch,lv_color_hex(0x317FF0),0);
    lv_obj_align(main_watch,LV_ALIGN_CENTER,0,-70);
    lv_obj_set_style_radius(main_watch,10,0);


    lv_obj_t*title1=lv_label_create(main_watch);
		lv_label_set_text(title1,"WriST WatCH");
		lv_obj_set_style_text_color(title1,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(title1, &lv_font_montserrat_20, 0);
    lv_obj_align(title1,LV_ALIGN_CENTER,-25,-10);
    lv_obj_t*title2=lv_label_create(main_watch);
		lv_label_set_text(title2,"watch 1.1");//正式版本
		lv_obj_set_style_text_color(title2,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(title2, &lv_font_montserrat_14, 0);
    lv_obj_align(title2,LV_ALIGN_CENTER,-60,12);

		lv_obj_t*title3=lv_label_create(main_watch);
		lv_label_set_text(title3,"正式版本");//正式版本
		lv_obj_set_style_text_color(title3,lv_color_hex(0xffffff),0);
    lv_obj_set_style_text_font(title3, &HEITi_14, 0);
    lv_obj_align(title3,LV_ALIGN_CENTER,-5,12);

    lv_obj_t* second_watch=lv_obj_create(about_show);
    lv_obj_set_size(second_watch,210,140);
    lv_obj_set_style_bg_color(second_watch,lv_color_hex(0xffffff),0);
    lv_obj_align(second_watch,LV_ALIGN_CENTER,0,42);
    lv_obj_set_style_radius(second_watch,10,0);

		lv_obj_t*watch_name=lv_label_create(second_watch);//手表名称
		lv_label_set_text(watch_name,"手表名称");
    lv_obj_set_style_text_color(watch_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(watch_name, &HEITi_16, 0);
    lv_obj_align(watch_name,LV_ALIGN_TOP_LEFT,5,0);

    lv_obj_t*model_name=lv_label_create(second_watch);//手表型号
    lv_label_set_text(model_name,"手表型号");
    lv_obj_set_style_text_color(model_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(model_name, &HEITi_16, 0);
    lv_obj_align(model_name,LV_ALIGN_TOP_RIGHT,-22,0);

    lv_obj_t*store_name=lv_label_create(second_watch);//运行内存
    lv_label_set_text(store_name,"运行内存");
    lv_obj_set_style_text_color(store_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(store_name, &HEITi_16, 0);
    lv_obj_align(store_name,LV_ALIGN_TOP_LEFT,5,60);

    lv_obj_t*watch_son_name=lv_label_create(second_watch);
    lv_label_set_text(watch_son_name,"L-WatCH");
    lv_obj_set_style_text_color(watch_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(watch_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(watch_son_name,LV_ALIGN_TOP_LEFT,5,30);


    lv_obj_t*model_son_name=lv_label_create(second_watch);
    lv_label_set_text(model_son_name,"STM32F411");
    lv_obj_set_style_text_color(model_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(model_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(model_son_name,LV_ALIGN_TOP_RIGHT,-3,30);

    lv_obj_t*store_son_name=lv_label_create(second_watch);
    lv_label_set_text(store_son_name,"128K SRAM");
    lv_obj_set_style_text_color(store_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(store_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(store_son_name,LV_ALIGN_TOP_LEFT,5,90);

    lv_obj_t* third_watch=lv_obj_create(about_show);
    lv_obj_set_size(third_watch,210,140);
    lv_obj_set_style_bg_color(third_watch,lv_color_hex(0xffffff),0);
    lv_obj_align(third_watch,LV_ALIGN_CENTER,0,190);
    lv_obj_set_style_radius(third_watch,10,0);

    lv_obj_t*system_name=lv_label_create(third_watch);//运行系统
    lv_label_set_text(system_name,"运行系统");
    lv_obj_set_style_text_color(system_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(system_name, &HEITi_16, 0);
    lv_obj_align(system_name,LV_ALIGN_TOP_LEFT,5,0);

		lv_obj_t*size_name=lv_label_create(third_watch);//屏幕尺寸
    lv_label_set_text(size_name,"屏幕尺寸");
    lv_obj_set_style_text_color(size_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(size_name, &HEITi_16, 0);
    lv_obj_align(size_name,LV_ALIGN_TOP_MID,46,0);

    lv_obj_t*interface_name=lv_label_create(third_watch);//使用界面
    lv_label_set_text(interface_name,"用户界面");
    lv_obj_set_style_text_color(interface_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(interface_name, &HEITi_16, 0);
    lv_obj_align(interface_name,LV_ALIGN_TOP_LEFT,5,60);

    lv_obj_t*system_son_name=lv_label_create(third_watch);
    lv_label_set_text(system_son_name,"FreeRTOS");
    lv_obj_set_style_text_color(system_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(system_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(system_son_name,LV_ALIGN_TOP_LEFT,5,30);


    lv_obj_t*size_son_name=lv_label_create(third_watch);
    lv_label_set_text(size_son_name,"1.83");
    lv_obj_set_style_text_color(size_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(size_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(size_son_name,LV_ALIGN_TOP_MID,30,30);
		
		lv_obj_t*size_son_name1=lv_label_create(third_watch);
    lv_label_set_text(size_son_name1,"寸");
    lv_obj_set_style_text_color(size_son_name1,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(size_son_name1, &HEITi_16, 0);
    lv_obj_align(size_son_name1,LV_ALIGN_TOP_MID,56,30);


    lv_obj_t*interface_son_name=lv_label_create(third_watch);
    lv_label_set_text(interface_son_name,"LVGL");
    lv_obj_set_style_text_color(interface_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(interface_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(interface_son_name,LV_ALIGN_TOP_LEFT,5,90);

    lv_obj_t* fourth_watch=lv_obj_create(about_show);
    lv_obj_set_size(fourth_watch,210,73);
    lv_obj_set_style_bg_color(fourth_watch,lv_color_hex(0xffffff),0);
    lv_obj_align(fourth_watch,LV_ALIGN_CENTER,0,305);
    lv_obj_set_style_radius(fourth_watch,10,0);

    lv_obj_t*design_name=lv_label_create(fourth_watch);//总设计者
    lv_label_set_text(design_name,"设计开发者");
    lv_obj_set_style_text_color(design_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(design_name, &HEITi_16, 0);
    lv_obj_align(design_name,LV_ALIGN_TOP_LEFT,5,0);

    lv_obj_t*design_son_name=lv_label_create(fourth_watch);
    lv_label_set_text(design_son_name,"RA_L_W_L");
    lv_obj_set_style_text_color(design_son_name,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(design_son_name, &lv_font_montserrat_16, 0);
    lv_obj_align(design_son_name,LV_ALIGN_TOP_LEFT,5,25);

	  lv_scr_load(about_show);
		lv_group_add_obj(group,about_show);

}
lv_obj_t*SELECT_WALLPAPAER_BTN;
typedef enum {
    WALLPAPER_STATIC,
    WALLPAPER_DYNAMIC
} WallpaperType;

typedef struct {
    const char* name;
    uint32_t data;  // 静态壁纸地址或动态壁纸首帧地址
    WallpaperType type;
    uint32_t frame_count;  // 动态壁纸帧数 (仅动态壁纸使用)
    uint32_t frame_delay;  // 帧间隔ms (仅动态壁纸使用)
    uint32_t* frames;      // 动态壁纸各帧地址数组 (仅动态壁纸使用)
} WallpaperItem;

static int current_selected_index = 0; // 当前选中项的索引
uint8_t current_wallpaper = 0;

WallpaperItem wallpapers[] = {
    {""},
    {"Just Do It", 0x001000,WALLPAPER_STATIC},
    {"Evil Eye",  0x001000+sizeof(lv_img_header_t)+1*245*287*2,WALLPAPER_STATIC},
    {"Stephen Curry",0x001000+2*sizeof(lv_img_header_t)+1*245*287*2+1*245*289*2,WALLPAPER_STATIC},
    {"Satoru Gojo",0x001000+3*sizeof(lv_img_header_t)+2*245*287*2+1*245*289*2,WALLPAPER_STATIC},
	  {"Aerial Spider",0x001000+4*sizeof(lv_img_header_t)+2*245*287*2+1*245*289*2+243*285*2,WALLPAPER_STATIC},
		{"Azure Gaze", 0x001000+5*sizeof(lv_img_header_t)+2*245*287*2+1*245*289*2+243*285*2+243*284*2,WALLPAPER_STATIC},
    {"Iron Man", 0x001000+6*sizeof(lv_img_header_t)+837914,WALLPAPER_STATIC},
    {"Eyes of Duality",0x001000+7*sizeof(lv_img_header_t)+976424,WALLPAPER_STATIC},
    {"Time Jerry",0x001000+8*sizeof(lv_img_header_t)+1117054,WALLPAPER_STATIC},
	  {"Ethereal Goku",0x001000+9*sizeof(lv_img_header_t)+1257684,WALLPAPER_STATIC},
	  {"McTime Query", 0x001000+10*sizeof(lv_img_header_t)+1396194,WALLPAPER_STATIC},
	  {"Wealth Quest",0x001000+11*sizeof(lv_img_header_t)+1536824,WALLPAPER_STATIC},
	  {"Celestial Gaze",0x001000+12*sizeof(lv_img_header_t)+1675334,WALLPAPER_STATIC},
	  {"SpongBob&Patrick",0x001000+13*sizeof(lv_img_header_t)+1813358,WALLPAPER_STATIC},
	  {"Nordic Snowscape",0x001000+14*sizeof(lv_img_header_t)+1952840,WALLPAPER_STATIC},
		{"Spidey's Focus",0x001000+15*sizeof(lv_img_header_t)+2091350,WALLPAPER_STATIC},
	  {"Cosmic Harmony",0x001000+16*sizeof(lv_img_header_t)+2229860,WALLPAPER_STATIC},
	  {"Demon Blade",0x001000+17*sizeof(lv_img_header_t)+2368370,WALLPAPER_STATIC},
		{"Shade of Obito",0x001000+18*sizeof(lv_img_header_t)+2506880,WALLPAPER_STATIC},
	  {"Lake Reflection",0x001000+19*sizeof(lv_img_header_t)+2645390,WALLPAPER_STATIC},
	  {"Snowy Rose",0x001000+20*sizeof(lv_img_header_t)+2783900,WALLPAPER_STATIC},
    {"Animated Clouds", 0x001000+21*sizeof(lv_img_header_t)+2922411, WALLPAPER_DYNAMIC,36, 80, 
     (uint32_t[]){0x001000+21*sizeof(lv_img_header_t)+2922411,0x001000+22*sizeof(lv_img_header_t)+2922411+1*245*287*2, 
		0x001000+23*sizeof(lv_img_header_t)+2922411+2*245*287*2,0x001000+24*sizeof(lv_img_header_t)+2922411+3*245*287*2,
		0x001000+25*sizeof(lv_img_header_t)+2922411+4*245*287*2,0x001000+26*sizeof(lv_img_header_t)+2922411+5*245*287*2, 
		0x001000+27*sizeof(lv_img_header_t)+2922411+6*245*287*2,0x001000+28*sizeof(lv_img_header_t)+2922411+7*245*287*2,
	  0x001000+29*sizeof(lv_img_header_t)+2922411+8*245*287*2,0x001000+30*sizeof(lv_img_header_t)+2922411+9*245*287*2, 
		0x001000+31*sizeof(lv_img_header_t)+2922411+10*245*287*2,0x001000+32*sizeof(lv_img_header_t)+2922411+11*245*287*2,
		0x001000+33*sizeof(lv_img_header_t)+2922411+12*245*287*2,0x001000+34*sizeof(lv_img_header_t)+2922411+13*245*287*2,
		0x001000+35*sizeof(lv_img_header_t)+2922411+14*245*287*2,0x001000+36*sizeof(lv_img_header_t)+2922411+15*245*287*2,
		0x001000+37*sizeof(lv_img_header_t)+2922411+16*245*287*2,0x001000+38*sizeof(lv_img_header_t)+2922411+17*245*287*2,
		0x001000+39*sizeof(lv_img_header_t)+2922411+18*245*287*2,0x001000+40*sizeof(lv_img_header_t)+2922411+19*245*287*2,
		0x001000+41*sizeof(lv_img_header_t)+2922411+20*245*287*2,0x001000+42*sizeof(lv_img_header_t)+2922411+21*245*287*2,
		0x001000+43*sizeof(lv_img_header_t)+2922411+22*245*287*2,0x001000+44*sizeof(lv_img_header_t)+2922411+23*245*287*2,
		0x001000+45*sizeof(lv_img_header_t)+2922411+24*245*287*2,0x001000+46*sizeof(lv_img_header_t)+2922411+25*245*287*2,
		0x001000+47*sizeof(lv_img_header_t)+2922411+26*245*287*2,0x001000+48*sizeof(lv_img_header_t)+2922411+27*245*287*2,
		0x001000+49*sizeof(lv_img_header_t)+2922411+28*245*287*2,0x001000+50*sizeof(lv_img_header_t)+2922411+29*245*287*2,
		0x001000+51*sizeof(lv_img_header_t)+2922411+30*245*287*2,0x001000+52*sizeof(lv_img_header_t)+2922411+31*245*287*2,
		0x001000+53*sizeof(lv_img_header_t)+2922411+32*245*287*2,0x001000+54*sizeof(lv_img_header_t)+2922411+33*245*287*2,
		0x001000+55*sizeof(lv_img_header_t)+2922411+34*245*287*2,0x001000+56*sizeof(lv_img_header_t)+2922411+35*245*287*2,
	
		}}
};


static void scroll_event(lv_event_t * e) {
    lv_obj_t * scroll_view = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_SCROLL_END) {
        lv_coord_t scroll_y = lv_obj_get_scroll_y(scroll_view);
        lv_coord_t center_y = lv_obj_get_height(scroll_view)/2 + scroll_y;

        lv_coord_t min_dist = INT16_MAX;
        int selected_index = 1;

        uint32_t child_cnt = lv_obj_get_child_cnt(scroll_view);
        for(uint32_t i = 0; i < child_cnt; i++) {
            lv_obj_t * child = lv_obj_get_child(scroll_view, i);
            if(lv_obj_check_type(child, &lv_label_class)) {
                lv_coord_t child_y = lv_obj_get_y(child);
                lv_coord_t child_h = lv_obj_get_height(child);
                lv_coord_t child_center = child_y + child_h/2;
                lv_coord_t dist = LV_ABS(child_center - center_y);

                if(dist < min_dist) {
                    min_dist = dist;
                    selected_index = i;
                }
            }
        }
        current_selected_index = selected_index;
        if(SELECT_WALLPAPAER_BTN) {
            lv_obj_clear_state(SELECT_WALLPAPAER_BTN, LV_STATE_DISABLED);
        }
    }
}

static lv_timer_t* anim_timer = NULL;
static uint8_t current_frame = 0;
static WallpaperItem* current_dynamic_wp = NULL;
typedef struct {
    lv_img_header_t header; 
    uint32_t data_size;   
     uint32_t data;  
} img_dsc_t1;
img_dsc_t1 flash_img={
  .header.cf = LV_IMG_CF_TRUE_COLOR,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 240,
  .header.h = 280,
  .data_size =240*280 *2,
  .data=0x001000,
};

static void set_wallpaper_frame(uint32_t addr) {
	 if (!addr) return; // 防止 NULL 地址
	  flash_img.data =addr;
	  lv_img_cache_invalidate_src(&flash_img);  
    lv_img_set_src(img, &flash_img);
}

static void dynamic_wp_anim_cb(lv_timer_t* timer) {
			if(!current_dynamic_wp) return;
    current_frame = (current_frame + 1) % current_dynamic_wp->frame_count;
    set_wallpaper_frame(current_dynamic_wp->frames[current_frame]);
	if (current_frame == current_dynamic_wp->frame_count - 1) {
        lv_timer_pause(anim_timer);
    }
}

static void play_dynamic_wallpaper(WallpaperItem* wp) {
    if(anim_timer) {
        lv_timer_del(anim_timer);
        anim_timer = NULL;
    }  
    if(wp->type != WALLPAPER_DYNAMIC) return;
    current_dynamic_wp = wp;
    current_frame = 0;
    set_wallpaper_frame(wp->frames[0]);
    anim_timer = lv_timer_create(dynamic_wp_anim_cb, wp->frame_delay, NULL);
}


static void apply_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {      
    WallpaperItem* selected = &wallpapers[current_selected_index];
    if(selected->type == WALLPAPER_STATIC) {	

			lv_img_cache_invalidate_src(&flash_img);
			if(anim_timer) {
                lv_timer_del(anim_timer);
                anim_timer = NULL;
            }
			flash_img.data =wallpapers[current_selected_index].data;	
			lv_img_set_src(img, &flash_img);	
				switch (current_selected_index)
			{	
				case 1: { lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
										time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 2: { lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
					lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 3: { lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 108);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,68);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,68);
										time_pos_data_x=0;
										time_pos_data_y=108;
										day_pos_data_x=-20;
										day_pos_data_y=68;
										week_pos_data_x=25;
										week_pos_data_y=68;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xC7C7C7),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xC7C7C7),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xC7C7C7),0);break;}
				case 4: { lv_obj_align(time_label, LV_ALIGN_CENTER,35, 108);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,23,68);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,68,68);
					          time_pos_data_x=35;
										time_pos_data_y=108;
										day_pos_data_x=23;
										day_pos_data_y=68;
										week_pos_data_x=68;
										week_pos_data_y=68;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 5: { lv_obj_align(time_label, LV_ALIGN_CENTER,44,-98);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,43,-58);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,88,-58);
										time_pos_data_x=44;
										time_pos_data_y=-98;
										day_pos_data_x=43;
										day_pos_data_y=-58;
										week_pos_data_x=88;
										week_pos_data_y=-58;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xEB3324),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xEB3324),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xEB3324),0);break;}
				
				case 6: { lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER, 25,48);	
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 7: { lv_obj_align(time_label, LV_ALIGN_CENTER,35, 108);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,23,68);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,68,68);
					          time_pos_data_x=35;
										time_pos_data_y=108;
										day_pos_data_x=23;
										day_pos_data_y=68;
										week_pos_data_x=68;
										week_pos_data_y=68;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 8: { lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 9: { lv_obj_align(time_label, LV_ALIGN_CENTER,44,-63);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,43,-103);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,88,-103);
					          time_pos_data_x=44;
										time_pos_data_y=-63;
										day_pos_data_x=43;
										day_pos_data_y=-103;
										week_pos_data_x=88;
										week_pos_data_y=-103;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 10:{ lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 108);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,68);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,68);	 
					          time_pos_data_x=0;
									 	time_pos_data_y=108;
										day_pos_data_x=-20;
										day_pos_data_y=68;
										week_pos_data_x=25;
										week_pos_data_y=68;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xCFCFCF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xCFCFCF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xCFCFCF),0);break;}
				case 11:{ lv_obj_align(time_label, LV_ALIGN_CENTER,44,-48);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,23,-88);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,68,-88);
					          time_pos_data_x=44;
										time_pos_data_y=-48;
										day_pos_data_x=23;
										day_pos_data_y=-88;
										week_pos_data_x=68;
										week_pos_data_y=-88;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFF3727),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFF3727),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFF3727),0);break;}
				case 12:{ lv_obj_align(time_label, LV_ALIGN_CENTER,44,-68);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,23,-108);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,68,-108);
					          time_pos_data_x=44;
										time_pos_data_y=-68;
										day_pos_data_x=23;
										day_pos_data_y=-108;
										week_pos_data_x=68;
										week_pos_data_y=-108;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xE3E3E3),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xE3E3E3),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xE3E3E3),0);break;}
				case 13:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,-48);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,-88);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,-88);
					          time_pos_data_x=0;
										time_pos_data_y=-48;
										day_pos_data_x=-20;
										day_pos_data_y=-88;
										week_pos_data_x=25;
										week_pos_data_y=-88;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 14:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,-48);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,-88);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,-88);
					          time_pos_data_x=0;
										time_pos_data_y=-48;
										day_pos_data_x=-20;
										day_pos_data_y=-88;
										week_pos_data_x=25;
										week_pos_data_y=-88;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0x58C0C2),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0x58C0C2),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0x58C0C2),0);break;}
				case 15:{ lv_obj_align(time_label, LV_ALIGN_CENTER,-25,-63);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-45,-103);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,0,-103);
					          time_pos_data_x=-25;
										time_pos_data_y=-63;
										day_pos_data_x=-45;
										day_pos_data_y=-103;
										week_pos_data_x=0;
										week_pos_data_y=-103;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 16:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 17:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 18:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 19:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 20:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,-68);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,-108);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,-108);
					          time_pos_data_x=0;
										time_pos_data_y=-68;
										day_pos_data_x=-20;
										day_pos_data_y=-108;
										week_pos_data_x=25;
										week_pos_data_y=-108;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}
				case 21:{ lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
									lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
									lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
					          time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
				  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	        lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
				  lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);break;}		
			}				
    }
    else {
      play_dynamic_wallpaper(selected);
			lv_obj_align(time_label, LV_ALIGN_CENTER,0,88);
			lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
			lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
			              time_pos_data_x=0;
										time_pos_data_y=88;
										day_pos_data_x=-20;
										day_pos_data_y=48;
										week_pos_data_x=25;
										week_pos_data_y=48;
			lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0);
	    lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
			lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);
    }

    lv_scr_load(wallpaper);
		lv_obj_del_delayed(wallpaperer_show,10);
		wallpaperer_show=NULL;
		lv_group_focus_obj(wallpaper);
    }
}

static void wallpaperer_show_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
				lv_scr_load(next_main_cont);       
				lv_obj_del_delayed(wallpaperer_show,15);
				wallpaperer_show=NULL;
			  lv_group_focus_obj(next_main_cont); 
        }
}
static void wallpaperer_show_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
					lv_scr_load(next_main_cont);						
					lv_obj_del_delayed(wallpaperer_show,15);
							 wallpaperer_show=NULL;
					lv_group_focus_obj(next_main_cont); 
						}
				}
}

LV_FONT_DECLARE(playlist);
LV_FONT_DECLARE(Facon);

 void Wallpaper_show(void)
 {
    wallpaperer_show=lv_obj_create(NULL);
    lv_obj_add_event_cb(wallpaperer_show, wallpaperer_show_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(wallpaperer_show, wallpaperer_show_key_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_style_bg_color(wallpaperer_show,lv_color_hex(0x000000),0);
    lv_obj_t * title = lv_label_create(wallpaperer_show);
    lv_label_set_text(title, "WALLPAPER");
    lv_obj_set_style_text_color(title,lv_color_hex(0x0fff00),0);
    lv_obj_set_style_text_font(title, &Facon, 0);
    lv_obj_align(title,LV_ALIGN_TOP_MID,0,30);

    lv_obj_t * title1 = lv_label_create(wallpaperer_show);
    lv_label_set_text(title1, "SELECT");
    lv_obj_set_style_text_color(title1,lv_color_hex(0x0fff00),0);
    lv_obj_set_style_text_font(title1, &Facon, 0);
    lv_obj_align(title1,LV_ALIGN_TOP_MID,0,70);


    lv_obj_t * scroll_view = lv_obj_create(wallpaperer_show);
    lv_obj_set_style_border_width(scroll_view, 0, LV_PART_MAIN);//移除边框样式
    lv_obj_set_style_bg_opa(scroll_view, LV_OPA_TRANSP, 0);//透明背景
    lv_obj_set_style_width(scroll_view, 0, LV_PART_SCROLLBAR); // 宽度设为0
    lv_obj_set_size(scroll_view, 180, 90);  // 设置固定大小
    lv_obj_align(scroll_view,LV_ALIGN_CENTER,0,20);
    lv_obj_set_flex_flow(scroll_view, LV_FLEX_FLOW_COLUMN);  // 垂直排列内容
    lv_obj_set_style_pad_all(scroll_view, 10, 0);  // 设置内边距
    lv_obj_set_scroll_dir(scroll_view, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(scroll_view, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(scroll_view, LV_SCROLLBAR_MODE_OFF);
    lv_obj_scroll_to_view(scroll_view, LV_ANIM_OFF);

    lv_obj_t * color_region = lv_obj_create(scroll_view);
    lv_obj_set_size(color_region, 180, 30);  // 设置区域大小
    lv_obj_center(color_region);
    lv_obj_set_style_radius(color_region,10,0);
    lv_obj_set_style_bg_color(color_region,lv_color_hex(0xFF5656), 0);
    lv_obj_set_style_border_width(color_region, 0, 0);  // 移除边框
    lv_obj_add_flag(color_region, LV_OBJ_FLAG_IGNORE_LAYOUT | LV_OBJ_FLAG_FLOATING);//添加遮罩层

    lv_obj_t * first_label = NULL;
    for(int i = 1; i <23;i++) {
        lv_obj_t * label = lv_label_create(scroll_view);
        lv_obj_set_style_text_align(label,LV_TEXT_ALIGN_CENTER,0);
        lv_label_set_text_fmt(label, "%s", wallpapers[i].name);
        lv_obj_set_width(label, lv_pct(100));  // 设置宽度与父容器相同
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label,lv_color_hex(0xffffff),0);
        lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
       if(i == 1) first_label = label;
    }
        if(first_label) {
            lv_obj_scroll_to_view(first_label, LV_ANIM_OFF);
        }

     SELECT_WALLPAPAER_BTN=lv_btn_create(wallpaperer_show);
     lv_obj_set_style_bg_color(SELECT_WALLPAPAER_BTN, lv_color_hex(0x0FE7F2), 0);
     lv_obj_align(SELECT_WALLPAPAER_BTN,LV_ALIGN_CENTER,0,100);
     lv_obj_set_size(SELECT_WALLPAPAER_BTN,100,35);
     lv_obj_t*wallpaper_label=lv_label_create(SELECT_WALLPAPAER_BTN);//创建壁纸界面的显示文本
     lv_obj_set_style_text_align(wallpaper_label,LV_TEXT_ALIGN_CENTER,0);
     lv_label_set_text_fmt(wallpaper_label, "Apply", 0);
     lv_obj_set_style_text_color(wallpaper_label,lv_color_hex(0x303030),0);
     lv_obj_set_width(wallpaper_label, lv_pct(100));

		 lv_obj_add_state(SELECT_WALLPAPAER_BTN, LV_STATE_DISABLED);

		 lv_obj_add_event_cb(SELECT_WALLPAPAER_BTN, apply_btn_event_cb, LV_EVENT_CLICKED, NULL);
		 lv_obj_add_event_cb(scroll_view, scroll_event, LV_EVENT_SCROLL, NULL);
		 lv_obj_add_event_cb(scroll_view, scroll_event, LV_EVENT_SCROLL_END, NULL);
     lv_scr_load(wallpaperer_show);
		lv_group_add_obj(group,wallpaperer_show);

 }
static lv_obj_t * ZHUANGSHI;
static lv_obj_t * time_label3;
static lv_obj_t * msec_label;
static lv_obj_t * start_btn;
static lv_obj_t * reset_btn;
static lv_obj_t * arc;
static uint32_t elapsed_time = 0;
static bool is_running = false;
static uint32_t start_time;
static void update_time_display(uint32_t ms) {
    uint32_t minutes = ms / 60000;
    uint32_t seconds = (ms % 60000) / 1000;
    uint32_t msec = ms % 1000;

    lv_label_set_text_fmt(time_label3, "%02d:%02d", minutes, seconds);
    lv_label_set_text_fmt(msec_label, ".%03d", msec);
    uint8_t current_second = seconds % 60;
    lv_arc_set_value(arc, current_second * 6);  // 360°/60秒 = 6°/秒

}

static void start_stop_btn_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        is_running = !is_running;
        if(is_running) {
					  start_time = xTaskGetTickCount() * portTICK_PERIOD_MS; 
					  lv_obj_set_style_bg_color(start_btn, lv_color_hex(0xFF5555), 0);
            lv_label_set_text(lv_obj_get_child(start_btn, 0), "STOP");
        } else {
            lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x55FF55), 0);
            lv_label_set_text(lv_obj_get_child(start_btn, 0), "START");
					  elapsed_time += (xTaskGetTickCount() * portTICK_PERIOD_MS)-start_time;
        }
    }
}

static void reset_btn_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        elapsed_time = 0;
        update_time_display(0);
    }
}

static void Stop_watch_show_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
				is_running=false;
				lv_scr_load(next_main_cont);      
				lv_obj_del_delayed(Stop_watch_show,15);
					Stop_watch_show=NULL;
				lv_group_focus_obj(next_main_cont); 
        }
}
static void Stop_watch_show_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							is_running=false;
							lv_scr_load(next_main_cont);
							lv_obj_del_delayed(Stop_watch_show,15);
							Stop_watch_show=NULL;
							lv_group_focus_obj(next_main_cont); 
						}
				}
}
LV_FONT_DECLARE(MIAObiao);
LV_FONT_DECLARE(MIAObiaoZS);
LV_FONT_DECLARE(MIAOBiao_msec);
void create_stopwatch_ui() {
	  Stop_watch_show=lv_obj_create(NULL);
	  lv_obj_set_style_bg_color(Stop_watch_show,lv_color_black(),0);
    lv_obj_add_event_cb(Stop_watch_show, Stop_watch_show_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(Stop_watch_show, Stop_watch_show_key_event_cb, LV_EVENT_KEY,NULL);
    arc = lv_arc_create(Stop_watch_show);
    lv_obj_set_size(arc, 205, 205);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, -32);
    lv_arc_set_range(arc, 0,360);

    lv_arc_set_bg_angles(arc,0,360);
    lv_arc_set_value(arc, 0);
    lv_arc_set_rotation(arc,270);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB); // 移除旋钮样式

    lv_obj_set_style_arc_width(arc, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);

    lv_obj_set_style_arc_width(arc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
		
    ZHUANGSHI = lv_label_create(Stop_watch_show);
    lv_obj_set_style_text_font(ZHUANGSHI, &MIAObiaoZS, 0);
    lv_obj_set_style_text_color(ZHUANGSHI, lv_color_white(), 0);
    lv_label_set_text(ZHUANGSHI, "∞");
    lv_obj_align(ZHUANGSHI, LV_ALIGN_TOP_MID, 0, 65);

    time_label3 = lv_label_create(Stop_watch_show);
    lv_obj_set_style_text_font(time_label3, &MIAObiao, 0);
    lv_obj_set_style_text_color(time_label3, lv_color_white(), 0);
    lv_label_set_text(time_label3, "00:00");
    lv_obj_align(time_label3, LV_ALIGN_CENTER, 0, -30);

    msec_label = lv_label_create(Stop_watch_show);
    lv_obj_set_style_text_font(msec_label, &MIAOBiao_msec, 0);
    lv_obj_set_style_text_color(msec_label, lv_color_hex(0xAAAAAA), 0);
    lv_label_set_text(msec_label, ".000");
    lv_obj_align_to(msec_label, time_label3, LV_ALIGN_OUT_BOTTOM_MID, 0, 13);

    start_btn = lv_btn_create(Stop_watch_show);
    lv_obj_set_size(start_btn, 120, 50);
    lv_obj_align(start_btn, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x55FF55), 0);
    lv_obj_set_style_radius(start_btn, 25, 0);

    lv_obj_t * start_label = lv_label_create(start_btn);
    lv_label_set_text(start_label, "START");
    lv_obj_center(start_label);

    lv_obj_add_event_cb(start_btn, start_stop_btn_cb, LV_EVENT_ALL, NULL);

    reset_btn = lv_btn_create(Stop_watch_show);
    lv_obj_set_size(reset_btn, 120, 50);
    lv_obj_align(reset_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_set_style_bg_color(reset_btn, lv_color_hex(0x5555FF), 0);
    lv_obj_set_style_radius(reset_btn, 25, 0);

    lv_obj_t * reset_label = lv_label_create(reset_btn);
    lv_label_set_text(reset_label, "RESET");
    lv_obj_center(reset_label);

    lv_obj_add_event_cb(reset_btn, reset_btn_cb, LV_EVENT_ALL, NULL);
		lv_scr_load(Stop_watch_show);
		lv_group_add_obj(group,Stop_watch_show);

}

static void pulse_anim_cb(void *obj, int32_t v)
{
    lv_obj_set_style_arc_opa(obj, v, LV_PART_MAIN);
}

void create_pulsing_hud_ring(lv_obj_t *parent)
{
    lv_obj_t *ring = lv_arc_create(parent);
    lv_obj_remove_style(ring, NULL, LV_PART_INDICATOR);  
    lv_obj_set_size(ring, 200, 200);
    lv_obj_center(ring);
    lv_arc_set_bg_angles(ring, 0, 360);
    lv_arc_set_rotation(ring, 0);
    lv_arc_set_value(ring, 0);

    lv_obj_remove_style(ring, NULL, LV_PART_KNOB);

    lv_obj_set_style_arc_color(ring, lv_color_hex(0x00ffff), LV_PART_MAIN);
    lv_obj_set_style_arc_width(ring, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring, LV_OPA_70, LV_PART_MAIN);


    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ring);
    lv_anim_set_exec_cb(&a, pulse_anim_cb);
    lv_anim_set_time(&a, 1000);
    lv_anim_set_playback_time(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_values(&a, LV_OPA_20, LV_OPA_70);
    lv_anim_start(&a);
}
void create_particle_effect(lv_obj_t *parent, int count)
{
    for(int i = 0; i < count; i++) {
        lv_obj_t *dot = lv_obj_create(parent);
        lv_obj_set_size(dot, 4, 4);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x00ffff), 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);

        int x = rand() % 230;
        int y = rand() % 240;
        lv_obj_set_pos(dot, x, y);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, dot);
        lv_anim_set_values(&a, y, y + (rand() % 40 + 20));
        lv_anim_set_time(&a, rand() % 4000 + 3000);
        lv_anim_set_playback_time(&a, rand() % 2000 + 2000);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_start(&a);
    }
}
 LV_FONT_DECLARE(Environment_HZ);
static lv_obj_t *create_data_panel(lv_obj_t *parent, const char *label_txt, lv_color_t color, lv_coord_t x_ofs, lv_coord_t y_ofs)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, 75, 150);
		lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF); 
		lv_obj_set_scroll_dir(panel, LV_DIR_NONE);               

    lv_obj_set_style_radius(panel, 10, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_70, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_border_color(panel, color, 0);
    lv_obj_set_style_shadow_color(panel, color, 0);
    lv_obj_set_style_shadow_width(panel, 20, 0);
    lv_obj_set_style_shadow_opa(panel, LV_OPA_30, 0);
    lv_obj_align(panel, LV_ALIGN_CENTER, x_ofs, y_ofs);

    lv_obj_t *val_label = lv_label_create(panel);
    lv_label_set_text(val_label, "--");
    lv_obj_set_style_text_color(val_label, color, 0);
    lv_obj_set_style_text_font(val_label, &lv_font_montserrat_20, 0);
    lv_obj_align(val_label, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t *unit_label = lv_label_create(panel);
    lv_label_set_text(unit_label, label_txt);
    lv_obj_set_style_text_color(unit_label, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(unit_label, &Environment_HZ, 0);
    lv_obj_align(unit_label, LV_ALIGN_CENTER, 0, 30);

    return val_label;  
}

static void scr_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
					 lv_scr_load(next_main_cont);
           lv_timer_del(sensor_timer);
					 sensor_timer=NULL;
           lv_obj_del_delayed(scr,15);
					 scr=NULL;
					 lv_group_focus_obj(next_main_cont); 
        }
}
static void scr_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {							
						   lv_scr_load(next_main_cont);
	             lv_timer_del(sensor_timer);
							 sensor_timer=NULL;
							 lv_obj_del_delayed(scr,15);
							 scr=NULL;
							lv_group_focus_obj(next_main_cont); 
						}
				}
}
static lv_obj_t *temp_label;
static lv_obj_t *hum_label;
static lv_obj_t *press_label;

static void update_sensor_data_cb(lv_timer_t *timer)
	{
			BME280_Data_t data;
			if (xQueueReceive(temperature_queue, &data, 0) == pdTRUE)
			{
					char buf[16];
					sprintf(buf, "%.2f°C", data.temperature / 100.0);
					lv_label_set_text(temp_label, buf);
					sprintf(buf, "%.1f%%", data.humidity / 1024.0);
					lv_label_set_text(hum_label, buf);
					sprintf(buf, "%dhpa", data.pressure / 100);
					lv_label_set_text(press_label, buf);
			}
	}

void create_futuristic_weather_ui(void)
{
    scr =lv_obj_create(NULL);
	  lv_obj_add_event_cb(scr,scr_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(scr,scr_key_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0d0d0d), 0);
    lv_obj_set_style_bg_grad_color(scr, lv_color_hex(0x111133), 0);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_VER, 0);
	
    create_pulsing_hud_ring(scr);
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ENVIRONMENT");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ffff), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_letter_space(title, 2, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    temp_label =create_data_panel(scr, "温度", lv_color_hex(0xff4444), -78, 45);
    hum_label=create_data_panel(scr, "湿度",  lv_color_hex(0x44ddff), 0, 45);
    press_label=create_data_panel(scr, "气压", lv_color_hex(0xbb66ff), 78, 45);


    create_particle_effect(scr, 30);
    lv_scr_load(scr);
		lv_group_add_obj(group,scr);
	 if(sensor_timer) {lv_timer_del(sensor_timer); sensor_timer=NULL; }
   sensor_timer = lv_timer_create(update_sensor_data_cb, 200, NULL);
}

LV_IMG_DECLARE(indicator2); 

static int angle_deg = 0;
static lv_obj_t *img_pointer;
static lv_obj_t *label_dir;
static lv_obj_t *label_angle_only;
static lv_timer_t*compass_timer=NULL;
const char* get_direction_text(int angle) {
    const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return dirs[((angle + 22) % 360) / 45];
}

static void compass_timer_cb(lv_timer_t *timer) {
   float heading;
    if(xQueueReceive(HMC_queue, &heading,0)) {
        lv_img_set_angle(img_pointer, (int)heading * 10);  
        char buf[32];
        snprintf(buf, sizeof(buf), "%s %d°", get_direction_text((int)heading), (int)heading);
        lv_label_set_text(label_dir, buf);
    }
}


static void compass_scr_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
						lv_scr_load(setting_show);
            lv_timer_del(compass_timer);
						compass_timer=NULL;
						lv_obj_del_delayed(compass_scr,15);
					  compass_scr=NULL;
						lv_group_focus_obj(setting_show); 
        }
}
static void compass_scr_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						  uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
								lv_scr_load(setting_show);
                lv_timer_del(compass_timer);
							  compass_timer=NULL;
								lv_obj_del_delayed(compass_scr,15);
							  compass_scr=NULL;
								lv_group_focus_obj(setting_show); 
						}
				}
}
void create_compass_screen(void) {
    compass_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(compass_scr, lv_color_hex(0x000010), 0);
    lv_obj_set_style_border_width(compass_scr, 0, 0);
	
    lv_obj_add_event_cb(compass_scr,compass_scr_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(compass_scr,compass_scr_key_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_t *arc = lv_arc_create(compass_scr);
    lv_obj_remove_style(arc, NULL, LV_PART_INDICATOR); 
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);      
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x00ffff), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 3, LV_PART_MAIN);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_set_size(arc, 180, 180);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *label = lv_label_create(compass_scr);
    lv_label_set_text(label, "N");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0,-128);
    lv_obj_t *label1 = lv_label_create(compass_scr);
    lv_label_set_text(label1, "E");
    lv_obj_set_style_text_font(label1, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label1, lv_color_hex(0xffffff), 0);
    lv_obj_align(label1, LV_ALIGN_CENTER,0,50);
     lv_obj_t *label2 = lv_label_create(compass_scr);
    lv_label_set_text(label2, "S");
    lv_obj_set_style_text_font(label2, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label2, lv_color_hex(0xffffff), 0);
    lv_obj_align(label2, LV_ALIGN_CENTER, -88,-40);
     lv_obj_t *label3 = lv_label_create(compass_scr);
    lv_label_set_text(label3, "W");
    lv_obj_set_style_text_font(label3, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label3, lv_color_hex(0xffffff), 0);
    lv_obj_align(label3, LV_ALIGN_CENTER, 88,-40);

    img_pointer = lv_img_create(compass_scr);
    lv_img_set_src(img_pointer, &indicator2);
    lv_img_set_pivot(img_pointer,10,64);
    lv_img_set_angle(img_pointer, 0);
    lv_obj_align(img_pointer, LV_ALIGN_CENTER, 0, -40);

    label_dir = lv_label_create(compass_scr);
    lv_obj_set_style_text_font(label_dir, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_dir, lv_color_hex(0x00ffff), 0);
    lv_obj_align(label_dir, LV_ALIGN_CENTER, 0, 80);
    lv_label_set_text(label_dir, "N 0°");

    lv_scr_load(compass_scr);
		lv_group_add_obj(group,compass_scr);

	if(compass_timer){lv_timer_del(compass_timer);compass_timer=NULL;}
    // 启动定时器：100ms 刷新一次
    compass_timer=lv_timer_create(compass_timer_cb, 40, NULL);
}

#define MAX_ALARMS 4

typedef struct {
    int hour;
    int minute;
    bool enabled;
    lv_obj_t *container;
    lv_obj_t *switch_obj;
} alarm_item_t;

static alarm_item_t alarms[MAX_ALARMS];
static int alarm_count = 0;

static lv_obj_t *clock_next_scr;
static lv_obj_t *label_time;
static lv_obj_t *alarm_list;
static lv_obj_t *roller_hour;
static lv_obj_t *roller_minute;
static lv_obj_t *popup;
static lv_obj_t *btn_add;
static lv_timer_t * update_timer = NULL;

static void update_time_cb(lv_timer_t *timer)
{
	  char time_str[9];
	  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
    Hour,Min,Sec);
    lv_label_set_text(label_time, time_str);
}

static void switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    int id = (int)(intptr_t)lv_event_get_user_data(e);
    if (id < alarm_count)
    alarms[id].enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
}

static void delete_alarm_cb(lv_event_t *e)
{
    int id = (int)(intptr_t)lv_event_get_user_data(e);
    if (id >= alarm_count) return;

    lv_obj_del(alarms[id].container);

    for (int i = id; i < alarm_count - 1; ++i) {
        alarms[i] = alarms[i + 1];
        lv_obj_set_user_data(alarms[i].switch_obj, (void *)(intptr_t)i);
    }

    alarm_count--;
}

static void refresh_alarm_list_ui(void)
{
    if (!alarm_list) return;
    lv_obj_clean(alarm_list);

    for (int i = 0; i < alarm_count; ++i) {
        alarm_item_t *a = &alarms[i];

        lv_obj_t *row = lv_obj_create(alarm_list);
        lv_obj_set_size(row, lv_pct(100), 50);
        lv_obj_set_style_pad_all(row, 10, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x111122), 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(row, 5, 0);

        char time_buf[16];
        snprintf(time_buf, sizeof(time_buf), "%02d:%02d", a->hour, a->minute);
        lv_obj_t *label = lv_label_create(row);
        lv_label_set_text(label, time_buf);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(0x00ffff), 0);
        lv_obj_set_width(label, 100);
        lv_obj_align(label,LV_ALIGN_CENTER,-40,0);

        lv_obj_t *sw = lv_switch_create(row);
        lv_obj_align(sw,LV_ALIGN_CENTER,0,0);
        lv_obj_add_event_cb(sw, switch_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)i);
        if (a->enabled) lv_obj_add_state(sw, LV_STATE_CHECKED);

        lv_obj_t *btn_del = lv_btn_create(row);
        lv_obj_set_size(btn_del, 50, 40);
        lv_obj_align(btn_del,LV_ALIGN_CENTER,65,0);
        lv_obj_t *lbl = lv_label_create(btn_del);
        lv_label_set_text(lbl, "DEL");
        lv_obj_center(lbl);
        lv_obj_add_event_cb(btn_del, delete_alarm_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        a->switch_obj = sw;
        a->container = row;
    }
}

static void add_alarm_to_list(int hour, int minute, bool enabled)
{
    if (alarm_count >= MAX_ALARMS) {
        return;
    }
    alarms[alarm_count].hour = hour;
    alarms[alarm_count].minute = minute;
    alarms[alarm_count].enabled = enabled;
    alarm_count++;
    refresh_alarm_list_ui();
}

static void close_popup(void)
{
    if (popup) {
        lv_obj_del(popup);
        popup = NULL;
    }
}

static void popup_confirm_cb(lv_event_t *e)
{
    int hour = lv_roller_get_selected(roller_hour);
    int minute = lv_roller_get_selected(roller_minute);

    add_alarm_to_list(hour, minute, true);
    close_popup();
}

static void show_add_alarm_popup(lv_event_t *e)
{
    if (alarm_count >= MAX_ALARMS) {
        return;
    }

    popup = lv_obj_create(clock_scr);
    lv_obj_set_size(popup, 220, 180);
    lv_obj_center(popup);
    lv_obj_set_style_bg_color(popup, lv_color_hex(0x222233), 0);
    lv_obj_set_style_radius(popup, 10, 0);
    lv_obj_set_style_pad_all(popup, 10, 0);
    lv_obj_set_flex_flow(popup, LV_FLEX_FLOW_COLUMN);

    roller_hour = lv_roller_create(popup);
    lv_roller_set_options(roller_hour,
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n"
        "13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23",
        LV_ROLLER_MODE_NORMAL);
    lv_obj_set_width(roller_hour, 80);

    roller_minute = lv_roller_create(popup);
    lv_roller_set_options(roller_minute,
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n"
        "13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n"
        "26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n"
        "39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n"
        "52\n53\n54\n55\n56\n57\n58\n59",
        LV_ROLLER_MODE_NORMAL);
    lv_obj_set_width(roller_minute, 80);

    lv_obj_t *btn = lv_btn_create(popup);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Set");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn, popup_confirm_cb, LV_EVENT_CLICKED, NULL);
}


static void clock_scr_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
           lv_scr_load(next_main_cont);
					 lv_timer_del(update_timer);
           update_timer = NULL;  // 防止悬空指针   
					 lv_obj_del_delayed(clock_scr,15);
					 clock_scr=NULL;
					 lv_group_focus_obj(next_main_cont); 
        }
}
static void clock_scr_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							  lv_scr_load(next_main_cont);
							  lv_timer_del(update_timer);
								update_timer = NULL;  // 防止悬空指针
							  lv_obj_del_delayed(clock_scr,15);
							  clock_scr=NULL;
					   		lv_group_focus_obj(next_main_cont); 
						}
				}
}
void create_alarm_clock_show(void)
{
    clock_scr=lv_obj_create(NULL);
	  lv_obj_set_style_bg_color(clock_scr, lv_color_hex(0x000010), 0);
	
	  lv_obj_add_event_cb(clock_scr,clock_scr_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(clock_scr,clock_scr_key_event_cb, LV_EVENT_KEY, NULL);
	
    clock_next_scr = lv_obj_create(clock_scr);
    lv_obj_set_style_bg_color(clock_next_scr, lv_color_hex(0x000010), 0);
    lv_obj_set_size(clock_next_scr,240,284);
	  lv_obj_set_style_border_width(clock_next_scr, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(clock_next_scr, 0, LV_PART_MAIN);

    label_time = lv_label_create(clock_next_scr);
    lv_obj_set_style_text_font(label_time, &MIAObiao, 0);
    lv_obj_set_style_text_color(label_time, lv_color_hex(0x00ffff), 0);
    lv_obj_align(label_time, LV_ALIGN_TOP_MID, 0, 10);

    btn_add = lv_btn_create(clock_next_scr);
    lv_obj_set_size(btn_add,160,40);
    lv_obj_align(btn_add, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_t *label = lv_label_create(btn_add);
    lv_label_set_text(label, "+ Add Alarm");
    lv_obj_center(label);
    lv_obj_add_event_cb(btn_add, show_add_alarm_popup, LV_EVENT_CLICKED, NULL);

    alarm_list = lv_obj_create(clock_next_scr);
    lv_obj_set_size(alarm_list, lv_pct(100), lv_pct(50));
    lv_obj_align(alarm_list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_scroll_dir(alarm_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(alarm_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_all(alarm_list, 5, 0);
    lv_obj_set_style_bg_color(alarm_list, lv_color_hex(0x000015), 0);
    lv_obj_set_layout(alarm_list, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(alarm_list, LV_FLEX_FLOW_COLUMN);

    refresh_alarm_list_ui();
		
 if(update_timer)
 {
	 lv_timer_del(update_timer);
	 update_timer=NULL;
 }
	 update_timer=lv_timer_create(update_time_cb, 1000, NULL);
    lv_scr_load(clock_scr);
		lv_group_add_obj(group,clock_scr);
}

LV_FONT_DECLARE(Time);
LV_FONT_DECLARE(Dyh);

#define TIME_DEFAULT_X    20
#define TIME_DEFAULT_Y    20

lv_obj_t *time_label_free;
lv_obj_t *img_display;
lv_obj_t *time_pos_set;
lv_obj_t *day_pos_set;
lv_obj_t *week_pos_set;
lv_obj_t *color_popup;
lv_obj_t *color_wheel;
lv_obj_t *saturation_slider;
lv_obj_t *brightness_slider;
lv_obj_t *triangle_btn;
lv_obj_t *color_preview;

static lv_point_t drag_offset;
static bool drag_enabled = false;
static int current_mode = 0;
static lv_color_hsv_t current_hsv = {0, 100, 100};  

static void drag_event_cb(lv_event_t *e);
static void color_panel_create(void);
static void triangle_btn_cb(lv_event_t *e);
static void update_text_color(void);

static void update_text_color(void)
{
    lv_color_t c = lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v);
    lv_obj_set_style_text_color(time_pos_set, c, LV_STATE_DEFAULT);
	  lv_obj_set_style_text_color(day_pos_set, c, LV_STATE_DEFAULT);
	  lv_obj_set_style_text_color(week_pos_set, c, LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(time_label,c,LV_STATE_DEFAULT);  
	  lv_obj_set_style_text_color(time_appear,c,LV_STATE_DEFAULT);  
	  lv_obj_set_style_text_color(week_appear,c,LV_STATE_DEFAULT);  

}

static void color_wheel_cb(lv_event_t *e)
{
    lv_obj_t *wheel = lv_event_get_target(e);
    if (wheel == NULL) return;

    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(wheel);
    current_hsv.h = hsv.h;
    current_hsv.s = lv_slider_get_value(saturation_slider);
    current_hsv.v = lv_slider_get_value(brightness_slider);
	
	  lv_color_t c = lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v);

	 if (color_preview) {
        lv_obj_set_style_bg_color(color_preview, c, LV_STATE_DEFAULT);
    }
}

static void saturation_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    current_hsv.s = lv_slider_get_value(slider);
	  lv_color_t c = lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v);
	 if (color_preview) {
        lv_obj_set_style_bg_color(color_preview, c, LV_STATE_DEFAULT);
    }

    lv_obj_t *value_label = (lv_obj_t*)lv_event_get_user_data(e);
    if (value_label) {
        lv_label_set_text_fmt(value_label, "%d%%", current_hsv.s);
    }
}

static void brightness_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    current_hsv.v = lv_slider_get_value(slider);
	  lv_color_t c = lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v);
 if (color_preview) {
        lv_obj_set_style_bg_color(color_preview, c, LV_STATE_DEFAULT);
    }
    lv_obj_t *value_label = (lv_obj_t*)lv_event_get_user_data(e);
    if (value_label) {
        lv_label_set_text_fmt(value_label, "%d%%", current_hsv.v);
    }
}

static void color_confirm_cb(lv_event_t *e)
{
  update_text_color();
}

static void preset_color_cb(lv_event_t *e)
{
    lv_color_t *color = (lv_color_t*)lv_event_get_user_data(e);
    lv_color_t c = *color;

    current_hsv = lv_color_rgb_to_hsv(c.ch.red, c.ch.green_h, c.ch.blue);
		lv_color_t a = lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v);
	 if (color_preview) {
        lv_obj_set_style_bg_color(color_preview, a, LV_STATE_DEFAULT);
    }
    if (saturation_slider) {
        lv_slider_set_value(saturation_slider, current_hsv.s, LV_ANIM_OFF);
    }
    if (brightness_slider) {
        lv_slider_set_value(brightness_slider, current_hsv.v, LV_ANIM_OFF);
    }
}

static lv_color_t* color_ptrs[16] = {NULL};
static void free_preset_colors(void)
{
    for(int i = 0; i < 16; i++) {
        if(color_ptrs[i] != NULL) {
            lv_mem_free(color_ptrs[i]);
            color_ptrs[i] = NULL;
        }
    }
}

static void color_popup_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							  free_preset_colors();
							  lv_scr_load(triangle_btn);
							  lv_obj_del_delayed(color_popup,15);
							  color_popup=NULL;
					   		lv_group_focus_obj(triangle_btn); 
						}
				}
}

static void color_panel_create(void)
{

    color_popup = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(color_popup, lv_color_hex(0x2c3e50), LV_STATE_DEFAULT);
		lv_obj_add_event_cb(color_popup,color_popup_key_event_cb, LV_EVENT_KEY, NULL);
    color_wheel = lv_colorwheel_create(color_popup, false);
    lv_obj_set_size(color_wheel, 90, 90);
    lv_obj_set_pos(color_wheel, 10, 10);
    lv_colorwheel_set_mode(color_wheel, LV_COLORWHEEL_MODE_HUE);
    lv_obj_set_style_pad_all(color_wheel,5 , LV_PART_KNOB);
    lv_obj_t *sat_label = lv_label_create(color_popup);
    lv_label_set_text(sat_label, "SAT:");
    lv_obj_set_style_text_color(sat_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_pos(sat_label, 100, 5);

    saturation_slider = lv_slider_create(color_popup);
    lv_obj_set_size(saturation_slider, 80, 35);
    lv_obj_set_pos(saturation_slider, 105, 25);
    lv_slider_set_range(saturation_slider, 0, 100);
    lv_slider_set_value(saturation_slider, current_hsv.s, LV_ANIM_OFF);

    lv_obj_t *sat_value = lv_label_create(color_popup);
    lv_label_set_text_fmt(sat_value, "%d%%", current_hsv.s);
    lv_obj_set_style_text_color(sat_value, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_pos(sat_value, 175, 5);

    lv_obj_t *bright_label = lv_label_create(color_popup);
    lv_label_set_text(bright_label, "BRI:");
    lv_obj_set_style_text_color(bright_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_pos(bright_label, 100, 65);

    brightness_slider = lv_slider_create(color_popup);
    lv_obj_set_size(brightness_slider, 80,35);
    lv_obj_set_pos(brightness_slider, 105, 85);
    lv_slider_set_range(brightness_slider, 0, 100);
    lv_slider_set_value(brightness_slider, current_hsv.v, LV_ANIM_OFF);

    lv_obj_t *bright_value = lv_label_create(color_popup);
    lv_label_set_text_fmt(bright_value, "%d%%", current_hsv.v);
    lv_obj_set_style_text_color(bright_value, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_pos(bright_value, 175, 65);

    color_preview = lv_obj_create(color_popup);
    lv_obj_set_size(color_preview, 200, 30);
    lv_obj_align(color_preview,LV_ALIGN_CENTER,0,70);
    lv_obj_set_style_bg_color(color_preview, lv_color_hsv_to_rgb(current_hsv.h, current_hsv.s, current_hsv.v), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(color_preview, 1, LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(color_preview, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_style_radius(color_preview, 5, LV_STATE_DEFAULT);



    lv_color_t preset_colors[] = {
        LV_COLOR_MAKE(255, 0, 0),     // 纯红
        LV_COLOR_MAKE(255, 128, 0),   // 橙色
        LV_COLOR_MAKE(255, 255, 0),   // 黄色
        LV_COLOR_MAKE(128, 255, 0),   // 黄绿
        LV_COLOR_MAKE(0, 255, 0),     // 纯绿
        LV_COLOR_MAKE(0, 255, 128),   // 青绿
        LV_COLOR_MAKE(0, 255, 255),   // 青色
        LV_COLOR_MAKE(0, 128, 255),   // 天蓝
        LV_COLOR_MAKE(0, 0, 255),     // 纯蓝
        LV_COLOR_MAKE(128, 0, 255),   // 紫色
        LV_COLOR_MAKE(255, 0, 255),   // 品红
        LV_COLOR_MAKE(255, 0, 128),   // 粉红
        LV_COLOR_MAKE(255, 255, 255), // 白色
        LV_COLOR_MAKE(192, 192, 192), // 银色
        LV_COLOR_MAKE(128, 128, 128), // 灰色
        LV_COLOR_MAKE(0, 0, 0),       // 黑色
    };

    for(int i = 0; i < 16; i++) {
        lv_obj_t *btn = lv_btn_create(color_popup);
        lv_obj_set_size(btn, 28, 28);
        lv_obj_set_pos(btn, 10 + (i % 8) * 26, 135 + (i / 8) * 0);
        lv_obj_set_style_bg_color(btn, preset_colors[i], LV_STATE_DEFAULT);
        lv_obj_set_style_radius(btn, 4, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, lv_color_white(), LV_STATE_DEFAULT);
         color_ptrs[i] = (lv_color_t*)lv_mem_alloc(sizeof(lv_color_t));
        *color_ptrs[i] = preset_colors[i];
        lv_obj_add_event_cb(btn, preset_color_cb, LV_EVENT_CLICKED, color_ptrs[i]);
    }
    lv_obj_t *confirm_btn = lv_btn_create(color_popup);
    lv_obj_set_size(confirm_btn, 100, 35);
    lv_obj_align(confirm_btn, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(confirm_btn, lv_palette_main(LV_PALETTE_GREEN), LV_STATE_DEFAULT);

    lv_obj_t *btn_label = lv_label_create(confirm_btn);
    lv_label_set_text(btn_label, "Confrim");
    lv_obj_center(btn_label);

    lv_obj_add_event_cb(color_wheel, color_wheel_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(saturation_slider, saturation_cb, LV_EVENT_VALUE_CHANGED, sat_value);
    lv_obj_add_event_cb(brightness_slider, brightness_cb, LV_EVENT_VALUE_CHANGED, bright_value);
    lv_obj_add_event_cb(confirm_btn, color_confirm_cb, LV_EVENT_CLICKED, NULL);

		lv_scr_load(color_popup);
		lv_group_add_obj(group,color_popup);
}



static void drag_event_cb_time(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (obj == NULL) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;

    lv_point_t p;
    static lv_coord_t offset_x = 0;
    static lv_coord_t offset_y = 0;

    lv_coord_t scr_w = lv_disp_get_hor_res(NULL);
    lv_coord_t scr_h = lv_disp_get_ver_res(NULL);

    lv_coord_t obj_w = lv_obj_get_width(obj);
    lv_coord_t obj_h = lv_obj_get_height(obj);
    switch(lv_event_get_code(e)) {
        case LV_EVENT_PRESSED: {
            lv_coord_t obj_x = lv_obj_get_x(obj);
            lv_coord_t obj_y = lv_obj_get_y(obj);
            
            lv_indev_get_point(indev, &p);
            
            offset_x = p.x - obj_x;
            offset_y = p.y - obj_y;          
            break;
        }

        case LV_EVENT_PRESSING: {
            lv_indev_get_point(indev, &p);
            lv_coord_t new_x = p.x - offset_x;
            lv_coord_t new_y = p.y - offset_y;

            if (new_x < 0) new_x = 0;
            if (new_y < 0) new_y = 0;
            if (new_x + obj_w > scr_w) new_x = scr_w - obj_w;
            if (new_y + obj_h > scr_h) new_y = scr_h - obj_h;

            lv_coord_t obj_align_x = new_x - (scr_w / 2 - obj_w / 2);
            lv_coord_t obj_align_y = new_y - (scr_h / 2 - obj_h / 2);
            lv_obj_align(obj, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            lv_obj_align(time_label, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            break;
        }

        case LV_EVENT_RELEASED:
            break;

        default:
            break;
    }
}


static void drag_event_cb_day(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (obj == NULL) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;

    lv_point_t p;
    static lv_coord_t offset_x = 0;
    static lv_coord_t offset_y = 0;

    lv_coord_t scr_w = lv_disp_get_hor_res(NULL);
    lv_coord_t scr_h = lv_disp_get_ver_res(NULL);

    lv_coord_t obj_w = lv_obj_get_width(obj);
    lv_coord_t obj_h = lv_obj_get_height(obj);
    switch(lv_event_get_code(e)) {
        case LV_EVENT_PRESSED: {
            lv_coord_t obj_x = lv_obj_get_x(obj);
            lv_coord_t obj_y = lv_obj_get_y(obj);
            
            lv_indev_get_point(indev, &p);
            
            offset_x = p.x - obj_x;
            offset_y = p.y - obj_y;          
            break;
        }

        case LV_EVENT_PRESSING: {
            lv_indev_get_point(indev, &p);
            lv_coord_t new_x = p.x - offset_x;
            lv_coord_t new_y = p.y - offset_y;

            if (new_x < 0) new_x = 0;
            if (new_y < 0) new_y = 0;
            if (new_x + obj_w > scr_w) new_x = scr_w - obj_w;
            if (new_y + obj_h > scr_h) new_y = scr_h - obj_h;

            lv_coord_t obj_align_x = new_x - (scr_w / 2 - obj_w / 2);
            lv_coord_t obj_align_y = new_y - (scr_h / 2 - obj_h / 2);
            lv_obj_align(obj, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            lv_obj_align(time_appear, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            break;
        }

        case LV_EVENT_RELEASED:
            break;

        default:
            break;
    }
}

static void drag_event_cb_week(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (obj == NULL) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == NULL) return;

    lv_point_t p;
    static lv_coord_t offset_x = 0;
    static lv_coord_t offset_y = 0;

    lv_coord_t scr_w = lv_disp_get_hor_res(NULL);
    lv_coord_t scr_h = lv_disp_get_ver_res(NULL);

    lv_coord_t obj_w = lv_obj_get_width(obj);
    lv_coord_t obj_h = lv_obj_get_height(obj);
    switch(lv_event_get_code(e)) {
        case LV_EVENT_PRESSED: {
            lv_coord_t obj_x = lv_obj_get_x(obj);
            lv_coord_t obj_y = lv_obj_get_y(obj);
            
            lv_indev_get_point(indev, &p);
            
            offset_x = p.x - obj_x;
            offset_y = p.y - obj_y;          
            break;
        }

        case LV_EVENT_PRESSING: {
            lv_indev_get_point(indev, &p);
            lv_coord_t new_x = p.x - offset_x;
            lv_coord_t new_y = p.y - offset_y;

            if (new_x < 0) new_x = 0;
            if (new_y < 0) new_y = 0;
            if (new_x + obj_w > scr_w) new_x = scr_w - obj_w;
            if (new_y + obj_h > scr_h) new_y = scr_h - obj_h;

            lv_coord_t obj_align_x = new_x - (scr_w / 2 - obj_w / 2);
            lv_coord_t obj_align_y = new_y - (scr_h / 2 - obj_h / 2);
            lv_obj_align(obj, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            lv_obj_align(week_appear, LV_ALIGN_CENTER, obj_align_x, obj_align_y);
            break;
        }
        case LV_EVENT_RELEASED:
            break;

        default:
            break;
    }
}

static void time_label_free_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
								lv_scr_load(triangle_btn);
								lv_group_focus_obj(triangle_btn); 
						}
				}
}

void time_clock_create(void)
{
   time_label_free=lv_obj_create(NULL);
	 img_display = lv_img_create(time_label_free);
	 lv_obj_set_size(img_display, 240,284);  
	 lv_img_cache_invalidate_src(&flash_img);	
	 lv_img_set_src(img_display,&flash_img);
	 lv_obj_align(img_display, LV_ALIGN_CENTER, 0,0);
	
	 time_pos_set=lv_label_create(time_label_free);
	 lv_label_set_text(time_pos_set, "00:00");
	 lv_obj_set_style_text_font(time_pos_set, &Time,0); // 
   lv_obj_set_style_text_color(time_pos_set, lv_color_hex(0xFFFFFF),0); //
	 lv_obj_align(time_pos_set, LV_ALIGN_CENTER,time_pos_data_x,time_pos_data_y);
	
	 lv_obj_add_flag(time_pos_set, LV_OBJ_FLAG_CLICKABLE);
	 lv_obj_clear_flag(time_pos_set, LV_OBJ_FLAG_SCROLLABLE); 
	 lv_obj_clear_flag(time_label_free, LV_OBJ_FLAG_SCROLLABLE); 

   lv_obj_add_event_cb(time_pos_set, drag_event_cb_time, LV_EVENT_PRESSED, NULL);
   lv_obj_add_event_cb(time_pos_set, drag_event_cb_time, LV_EVENT_PRESSING, NULL);
   lv_obj_add_event_cb(time_pos_set, drag_event_cb_time, LV_EVENT_RELEASED, NULL);
	
	 day_pos_set=lv_label_create(time_label_free);
	 lv_label_set_text(day_pos_set, "00");
	 lv_obj_set_style_text_font(day_pos_set, &Dyh,0); // 
   lv_obj_set_style_text_color(day_pos_set, lv_color_hex(0xFFFFFF),0); //
	 lv_obj_align(day_pos_set, LV_ALIGN_CENTER,day_pos_data_x,day_pos_data_y);

	 lv_obj_add_flag(day_pos_set, LV_OBJ_FLAG_CLICKABLE);
	 lv_obj_clear_flag(day_pos_set, LV_OBJ_FLAG_SCROLLABLE); 
	 lv_obj_clear_flag(day_pos_set, LV_OBJ_FLAG_SCROLLABLE); 

   lv_obj_add_event_cb(day_pos_set, drag_event_cb_day, LV_EVENT_PRESSED, NULL);
   lv_obj_add_event_cb(day_pos_set, drag_event_cb_day, LV_EVENT_PRESSING, NULL);
   lv_obj_add_event_cb(day_pos_set, drag_event_cb_day, LV_EVENT_RELEASED, NULL);
	 
	 
	 week_pos_set=lv_label_create(time_label_free);
	 lv_label_set_text(week_pos_set, "周一");
	 lv_obj_set_style_text_font(week_pos_set, &Dyh,0); // 
   lv_obj_set_style_text_color(week_pos_set, lv_color_hex(0xFFFFFF),0); //
	 lv_obj_align(week_pos_set,LV_ALIGN_CENTER,week_pos_data_x,week_pos_data_y);

	 lv_obj_add_flag(week_pos_set, LV_OBJ_FLAG_CLICKABLE);
	 lv_obj_clear_flag(week_pos_set, LV_OBJ_FLAG_SCROLLABLE); 
	 lv_obj_clear_flag(week_pos_set, LV_OBJ_FLAG_SCROLLABLE); 

   lv_obj_add_event_cb(week_pos_set, drag_event_cb_week, LV_EVENT_PRESSED, NULL);
   lv_obj_add_event_cb(week_pos_set, drag_event_cb_week, LV_EVENT_PRESSING, NULL);
   lv_obj_add_event_cb(week_pos_set, drag_event_cb_week, LV_EVENT_RELEASED, NULL);
		
	 lv_obj_add_event_cb(time_label_free, time_label_free_key_event_cb,LV_EVENT_KEY, 0);
	 lv_group_add_obj(group,time_label_free);
}


static void top_btn_cb(lv_event_t *e)
{
   lv_event_code_t code = lv_event_get_code(e);
   lv_obj_t * obj = lv_event_get_target(e);
   if(code == LV_EVENT_CLICKED) {
	    color_panel_create();
		 	lv_group_focus_obj(color_popup); 
   }
}
static void mid_btn_cb(lv_event_t *e)
{
   lv_event_code_t code = lv_event_get_code(e);
   lv_obj_t * obj = lv_event_get_target(e);
   if(code == LV_EVENT_CLICKED) {
		  lv_scr_load(time_label_free);
			lv_group_focus_obj(time_label_free); 
   }
}


static void triangle_btn_key_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							  lv_scr_load(next_main_cont);
							  lv_obj_del_delayed(time_label_free,5);
							  lv_obj_del_delayed(triangle_btn,15);		
							  time_label_free=NULL;
							  triangle_btn=NULL;
					   		lv_group_focus_obj(next_main_cont); 
						}
				}
}
static void create_watch_dial(void)
{
    triangle_btn = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(triangle_btn, lv_color_hex(0x2c3e50), LV_STATE_DEFAULT);
		lv_obj_add_event_cb(triangle_btn,triangle_btn_key_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_t *top_btn = lv_btn_create(triangle_btn);
    lv_obj_set_size(top_btn, 240, 130);
    lv_obj_align(top_btn, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_btn, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_DEFAULT);

    lv_obj_t *top_label = lv_label_create(top_btn);
    lv_label_set_text(top_label, "COLOR_SET");
	  lv_obj_set_style_text_font(top_label, &lv_font_montserrat_20, 0);
    lv_obj_center(top_label);

    lv_obj_t *mid_btn = lv_btn_create(triangle_btn);
    lv_obj_set_size(mid_btn, 240, 130);
    lv_obj_align(mid_btn, LV_ALIGN_BOTTOM_MID,0, 0);
    lv_obj_set_style_bg_color(mid_btn, lv_palette_main(LV_PALETTE_GREEN), LV_STATE_DEFAULT);

    lv_obj_t *mid_label = lv_label_create(mid_btn);
    lv_label_set_text(mid_label, "POS_SET");
	  lv_obj_set_style_text_font(mid_label, &lv_font_montserrat_20, 0);
    lv_obj_center(mid_label);

    // 绑定事件
    lv_obj_add_event_cb(top_btn, top_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(mid_btn, mid_btn_cb, LV_EVENT_CLICKED, NULL);
		lv_scr_load(triangle_btn);
		lv_group_add_obj(group,triangle_btn);

}

LV_FONT_DECLARE(Setting_HZ);

static lv_obj_t * led_indicator;
static bool wrist_raise_enabled = false;
static lv_obj_t * hand_cont ;

static void btn_on_event_cb(lv_event_t * e);
static void btn_off_event_cb(lv_event_t * e);

static void btn_on_event_cb(lv_event_t * e)
{
    lv_obj_t * status_label = (lv_obj_t *)lv_event_get_user_data(e);

    wrist_raise_enabled = true;
    lv_label_set_text(status_label, "OPEN");

    lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x00FFC8), 0);
    lv_obj_set_style_shadow_color(led_indicator, lv_color_hex(0x00FFC8), 0);
    lv_obj_set_style_shadow_width(led_indicator, 20, 0);
    lv_obj_set_style_shadow_spread(led_indicator, 5, 0);
    lv_obj_set_style_shadow_opa(led_indicator, LV_OPA_40, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, led_indicator);
    lv_anim_set_values(&a, 0, 360);
    lv_anim_set_time(&a, 500);
    lv_anim_start(&a);
}

static void btn_off_event_cb(lv_event_t * e)
{
    lv_obj_t * status_label = (lv_obj_t *)lv_event_get_user_data(e);

    wrist_raise_enabled = false;
    lv_label_set_text(status_label, "CLOSE");

    lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x3A3A5A), 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, led_indicator);
    lv_anim_set_values(&a, 360, 0);
    lv_anim_set_time(&a, 500);
    lv_anim_start(&a);
}


static void hand_cont_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {	
        lv_scr_load(setting_show);
        lv_obj_del_delayed(hand_cont,15);
					hand_cont=NULL;
				lv_group_focus_obj(setting_show); 
        }
}

static void hand_cont_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
								lv_scr_load(setting_show);
								lv_obj_del_delayed(hand_cont,15);
							 hand_cont=NULL;
								lv_group_focus_obj(setting_show); 
						}
				}
}

void create_tech_ui(void)
{
    hand_cont = lv_obj_create(NULL);
		lv_obj_add_event_cb(hand_cont, hand_cont_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(hand_cont, hand_cont_KEY_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_size(hand_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(hand_cont, lv_color_hex(0x0A0A1A), 0);
    lv_obj_set_style_bg_opa(hand_cont, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(hand_cont, 0, 0);
    lv_obj_set_flex_flow(hand_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(hand_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);


    led_indicator = lv_obj_create(hand_cont);
    lv_obj_set_size(led_indicator, 30, 30);
    lv_obj_set_style_radius(led_indicator, LV_RADIUS_CIRCLE, 0);
    if(wrist_raise_enabled) {
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x00FFC8), 0);
        lv_obj_set_style_shadow_color(led_indicator, lv_color_hex(0x00FFC8), 0);
        lv_obj_set_style_shadow_width(led_indicator, 20, 0);
        lv_obj_set_style_shadow_spread(led_indicator, 5, 0);
        lv_obj_set_style_shadow_opa(led_indicator, LV_OPA_40, 0);
    } else {
        lv_obj_set_style_bg_color(led_indicator, lv_color_hex(0x3A3A5A), 0);
    }

    lv_obj_t * status_label = lv_label_create(hand_cont);
    lv_label_set_text(status_label, wrist_raise_enabled ? "OPEN" : "CLOSE");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);

    lv_obj_t * btn_cont = lv_obj_create(hand_cont);
    lv_obj_set_size(btn_cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_remove_style_all(btn_cont);
    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(btn_cont, 20, 0);

    lv_obj_t * btn_on = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_on, LV_PCT(48), 50);
    lv_obj_set_style_bg_color(btn_on, lv_color_hex(0x1A1A3A), 0);
    lv_obj_set_style_radius(btn_on, 10, 0);
    lv_obj_set_style_border_color(btn_on, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_border_width(btn_on, 2, 0);
    lv_obj_set_style_border_opa(btn_on, LV_OPA_70, 0);
    lv_obj_set_style_shadow_color(btn_on, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_shadow_width(btn_on, 15, 0);
    lv_obj_set_style_shadow_spread(btn_on, 5, 0);
    lv_obj_set_style_shadow_opa(btn_on, LV_OPA_30, 0);
    lv_obj_set_style_text_color(btn_on, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_pad_all(btn_on, 10, 0);

    lv_obj_set_style_bg_color(btn_on, lv_color_hex(0x00C8FF), LV_STATE_PRESSED);
    lv_obj_set_style_text_color(btn_on, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(btn_on, 25, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(btn_on, 10, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(btn_on, LV_OPA_50, LV_STATE_PRESSED);

    lv_obj_add_event_cb(btn_on, btn_on_event_cb, LV_EVENT_CLICKED, status_label);

    lv_obj_t * btn_on_label = lv_label_create(btn_on);
    lv_label_set_text(btn_on_label, "开");
    lv_obj_set_style_text_font(btn_on_label,&Setting_HZ,0);
    lv_obj_center(btn_on_label);

    lv_obj_t * btn_off = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_off, LV_PCT(48), 50);
    lv_obj_set_style_bg_color(btn_off, lv_color_hex(0x1A1A3A), 0);
    lv_obj_set_style_radius(btn_off, 10, 0);
    lv_obj_set_style_border_color(btn_off, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_border_width(btn_off, 2, 0);
    lv_obj_set_style_border_opa(btn_off, LV_OPA_70, 0);
    lv_obj_set_style_shadow_color(btn_off, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_shadow_width(btn_off, 15, 0);
    lv_obj_set_style_shadow_spread(btn_off, 5, 0);
    lv_obj_set_style_shadow_opa(btn_off, LV_OPA_30, 0);
    lv_obj_set_style_text_color(btn_off, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_pad_all(btn_off, 10, 0);

    lv_obj_set_style_bg_color(btn_off, lv_color_hex(0x00C8FF), LV_STATE_PRESSED);
    lv_obj_set_style_text_color(btn_off, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(btn_off, 25, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(btn_off, 10, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(btn_off, LV_OPA_50, LV_STATE_PRESSED);

    lv_obj_add_event_cb(btn_off, btn_off_event_cb, LV_EVENT_CLICKED, status_label);

    lv_obj_t * btn_off_label = lv_label_create(btn_off);
    lv_label_set_text(btn_off_label, "关");
    lv_obj_set_style_text_font(btn_off_label,&Setting_HZ,0);
    lv_obj_center(btn_off_label);

    lv_obj_t * decor = lv_obj_create(hand_cont);
    lv_obj_set_size(decor, LV_PCT(90), 2);
    lv_obj_set_style_bg_color(decor, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_bg_opa(decor, LV_OPA_50, 0);
    lv_obj_set_style_shadow_color(decor, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_shadow_width(decor, 10, 0);
    lv_obj_set_style_shadow_opa(decor, LV_OPA_30, 0);
		
		lv_scr_load(hand_cont);
		lv_group_add_obj(group,hand_cont);
}




#define COLOR_BG lv_color_hex(0x0F1A2A)
#define COLOR_PRIMARY lv_color_hex(0x00F5FF)
#define COLOR_SECONDARY lv_color_hex(0x9D00FF)
#define COLOR_ACCENT lv_color_hex(0x00FF9D)
#define COLOR_TEXT lv_color_hex(0xFFFFFF)

static lv_obj_t *time_lab;
static lv_obj_t *date_label;
static lv_obj_t *year_roller;
static lv_obj_t *month_roller;
static lv_obj_t *day_roller;
static lv_obj_t *hour_roller;
static lv_obj_t *minute_roller;
static lv_obj_t *second_roller;
static lv_obj_t *week_roller;
static unsigned char month,day,hour,minute,second,week;
static uint16_t year;
void update_time_display1() {

    static char time_str[10];
    static char date_str[12];

    uint8_t year_idx = lv_roller_get_selected(year_roller);
     year =  year_idx+2020; 
    uint8_t month_idx = lv_roller_get_selected(month_roller);
     month = month_idx+1; 
    uint8_t day_idx = lv_roller_get_selected(day_roller);
     day = day_idx+1; 
     hour = lv_roller_get_selected(hour_roller);
     minute = lv_roller_get_selected(minute_roller);
     second = lv_roller_get_selected(second_roller);
     uint8_t week_idx= lv_roller_get_selected(week_roller);
     week = week_idx;

    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, minute, second);
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", year, month, day);

    lv_label_set_text(time_lab, time_str);
    lv_label_set_text(date_label, date_str);

}

static void confirm_btn_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        update_time_display1();
			  PCF8563_WriteTime(0x00, 0x20);
  		  PCF8563_WriteTime(0x08, year-2000);
				PCF8563_WriteTime(0x07, month);
				PCF8563_WriteTime(0x05, day);
				PCF8563_WriteTime(0x04, hour);
				PCF8563_WriteTime(0x03, minute);
				PCF8563_WriteTime(0x02, second);
				PCF8563_WriteTime(0x06, week);
			  PCF8563_WriteTime(0x00, 0x00); 
        lv_obj_t * btn = lv_event_get_target(e);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, btn);
        lv_anim_set_values(&a, 0, 10);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_time(&a, 100);
        lv_anim_set_playback_time(&a, 100);
        lv_anim_start(&a);
    }
}

static void sync_btn_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
       
        lv_roller_set_selected(year_roller, Year+2000-2020, LV_ANIM_ON);
        lv_roller_set_selected(month_roller, Month-1, LV_ANIM_ON);
        lv_roller_set_selected(day_roller,   Day-1, LV_ANIM_ON);
        lv_roller_set_selected(hour_roller,  Hour, LV_ANIM_ON);
        lv_roller_set_selected(minute_roller, Min, LV_ANIM_ON);
        lv_roller_set_selected(second_roller, Sec, LV_ANIM_ON);
		    lv_roller_set_selected(week_roller, Week-1, LV_ANIM_ON);

        update_time_display1();
    }
}
lv_obj_t *time_data_set_cont;
static void time_data_set_event_handler(lv_event_t*e)
{
      lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        lv_scr_load(setting_show);
        lv_obj_del_delayed(time_data_set_cont,15);
					time_data_set_cont=NULL;
				lv_group_focus_obj(setting_show); 
        }
}
static void time_data_set_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
								lv_scr_load(setting_show);
								lv_obj_del_delayed(time_data_set_cont,15);
							 time_data_set_cont=NULL;
								lv_group_focus_obj(setting_show); 
						}
				}
}
void create_time_setting_ui(void) {
     time_data_set_cont = lv_obj_create(NULL);
    lv_obj_add_event_cb(time_data_set_cont, time_data_set_event_handler, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(time_data_set_cont,time_data_set_KEY_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_size(time_data_set_cont, 240,284);
    lv_obj_center(time_data_set_cont);
    lv_obj_set_style_bg_color(time_data_set_cont, COLOR_BG, 0);
    lv_obj_set_style_border_color(time_data_set_cont, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(time_data_set_cont, 2, 0);
    lv_obj_set_style_radius(time_data_set_cont, 10, 0);
    lv_obj_set_style_pad_all(time_data_set_cont, 10, 0);

    time_lab = lv_label_create(time_data_set_cont);
    lv_obj_set_style_text_font(time_lab, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(time_lab, COLOR_PRIMARY, 0);
    lv_obj_set_style_text_opa(time_lab, LV_OPA_COVER, 0);
    lv_obj_set_style_text_align(time_lab, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(time_lab, LV_ALIGN_TOP_MID, 0, 0);

	  lv_obj_set_style_shadow_color(time_lab, COLOR_PRIMARY, 0);
    lv_obj_set_style_shadow_width(time_lab, 15, 0);
    lv_obj_set_style_shadow_spread(time_lab, 5, 0);

    date_label = lv_label_create(time_data_set_cont);
    lv_obj_set_style_text_font(date_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(date_label, COLOR_TEXT, 0);
    lv_obj_align(date_label,LV_ALIGN_TOP_MID, 0, 35);


    lv_obj_t*year_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(year_label,"Year");
    lv_obj_set_style_text_color(year_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(year_label, &lv_font_montserrat_20, 0);
    lv_obj_align(year_label, LV_ALIGN_TOP_LEFT, 16, 57);

     lv_obj_t*month_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(month_label,"Month");
    lv_obj_set_style_text_color(month_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(month_label, &lv_font_montserrat_20, 0);
    lv_obj_align(month_label, LV_ALIGN_TOP_MID, 2, 57);

    lv_obj_t*day_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(day_label,"Day");
    lv_obj_set_style_text_color(day_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(day_label, &lv_font_montserrat_20, 0);
    lv_obj_align(day_label, LV_ALIGN_TOP_RIGHT, -14, 57);

    lv_obj_t*hour_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(hour_label,"Hour");
    lv_obj_set_style_text_color(hour_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(hour_label, &lv_font_montserrat_20, 0);
    lv_obj_align(hour_label, LV_ALIGN_CENTER, -73, 18);

    lv_obj_t*min_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(min_label,"Min");
    lv_obj_set_style_text_color(min_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(min_label, &lv_font_montserrat_20, 0);
    lv_obj_align(min_label, LV_ALIGN_CENTER, 0, 18);

    lv_obj_t*sec_label = lv_label_create(time_data_set_cont);
    lv_label_set_text(sec_label,"Sec");
    lv_obj_set_style_text_color(sec_label,lv_color_hex(0x73FBFD), 0);
    lv_obj_set_style_text_font(sec_label, &lv_font_montserrat_20, 0);
    lv_obj_align(sec_label, LV_ALIGN_CENTER, 75, 18);

    year_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(year_roller, "2020\n2021\n2022\n2023\n2024\n2025\n2026\n2027\n2028\n2029\n2030", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(year_roller, 55, 60);
    lv_obj_align(year_roller,LV_ALIGN_TOP_LEFT,8,80);
    lv_roller_set_selected(year_roller, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(year_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(year_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(year_roller, 1, 0);
    lv_obj_set_style_text_color(year_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(year_roller, 5, 0);

    month_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(month_roller, "01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(month_roller, 55, 60);
    lv_obj_align(month_roller,LV_ALIGN_TOP_MID,0,80);
    lv_roller_set_selected(month_roller, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(month_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(month_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(month_roller, 1, 0);
    lv_obj_set_style_text_color(month_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(month_roller, 5, 0);

    day_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(day_roller,
        "01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n"
        "11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n"
        "21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(day_roller, 55, 60);
    lv_roller_set_selected(day_roller,0, LV_ANIM_OFF);
    lv_obj_align(day_roller,LV_ALIGN_TOP_RIGHT,-6,80);
    lv_obj_set_style_bg_color(day_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(day_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(day_roller, 1, 0);
    lv_obj_set_style_text_color(day_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(day_roller, 5, 0);

    hour_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(hour_roller,
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n"
        "12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(hour_roller, 55, 60);
    lv_obj_align(hour_roller,LV_ALIGN_CENTER,-74,58);

    lv_roller_set_selected(hour_roller, 00, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(hour_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(hour_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(hour_roller, 1, 0);
    lv_obj_set_style_text_color(hour_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(hour_roller, 5, 0);

    minute_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(minute_roller,
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n"
        "10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n"
        "20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n"
        "30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n"
        "40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n"
        "50\n51\n52\n53\n54\n55\n56\n57\n58\n59", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(minute_roller, 55,60);
    lv_obj_align(minute_roller,LV_ALIGN_CENTER,0,58);
    lv_roller_set_selected(minute_roller, 00, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(minute_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(minute_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(minute_roller, 1, 0);
    lv_obj_set_style_text_color(minute_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(minute_roller, 5, 0);

    second_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(second_roller,
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n"
        "10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n"
        "20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n"
        "30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n"
        "40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n"
        "50\n51\n52\n53\n54\n55\n56\n57\n58\n59", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(second_roller, 55, 60);
    lv_obj_align(second_roller,LV_ALIGN_CENTER,74,58);
    lv_roller_set_selected(second_roller, 00, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(second_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(second_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(second_roller, 1, 0);
    lv_obj_set_style_text_color(second_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(second_roller, 5, 0);

     week_roller = lv_roller_create(time_data_set_cont);
    lv_roller_set_options(week_roller,
        "01\n02\n03\n04\n05\n06\n07"
        , LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(week_roller, 40, 40);
    lv_obj_align(week_roller,LV_ALIGN_TOP_RIGHT,0,15);
    lv_roller_set_selected(week_roller, 00, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(week_roller, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(week_roller, COLOR_PRIMARY, 0);
    lv_obj_set_style_border_width(week_roller, 1, 0);
    lv_obj_set_style_text_color(week_roller, COLOR_TEXT, 0);
    lv_obj_set_style_radius(week_roller, 5, 0);

    lv_obj_add_event_cb(year_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(month_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(day_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(hour_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(minute_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(second_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(week_roller, (lv_event_cb_t)update_time_display1, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *confirm_btn = lv_btn_create(time_data_set_cont);
    lv_obj_set_size(confirm_btn, 70, 35);
    lv_obj_align(confirm_btn,LV_ALIGN_BOTTOM_LEFT,0,0);
    lv_obj_set_style_bg_color(confirm_btn, COLOR_SECONDARY, 0);
    lv_obj_set_style_radius(confirm_btn, 20, 0);
    lv_obj_t *confirm_label = lv_label_create(confirm_btn);
    lv_label_set_text(confirm_label, "CONFIRM");
    lv_obj_set_style_text_color(confirm_label, COLOR_TEXT, 0);
    lv_obj_center(confirm_label);
    lv_obj_add_event_cb(confirm_btn, confirm_btn_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *sync_btn = lv_btn_create(time_data_set_cont);
    lv_obj_set_size(sync_btn, 70, 35);
    lv_obj_align(sync_btn,LV_ALIGN_BOTTOM_MID,0,0);
    lv_obj_set_style_bg_color(sync_btn, COLOR_PRIMARY, 0);
    lv_obj_set_style_radius(sync_btn, 20, 0);
    lv_obj_t *sync_label = lv_label_create(sync_btn);
    lv_label_set_text(sync_label, "RTC");
    lv_obj_set_style_text_color(sync_label, COLOR_BG, 0);
    lv_obj_center(sync_label);
    lv_obj_add_event_cb(sync_btn, sync_btn_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *cancel_btn = lv_btn_create(time_data_set_cont);
    lv_obj_set_size(cancel_btn, 70, 35);
    lv_obj_align(cancel_btn,LV_ALIGN_BOTTOM_RIGHT,0,0);

    lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x2A3A4A), 0);
    lv_obj_set_style_radius(cancel_btn, 20, 0);
    lv_obj_t *cancel_label = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_label, "CANCEL");
    lv_obj_set_style_text_color(cancel_label, COLOR_TEXT, 0);
    lv_obj_center(cancel_label);

    update_time_display1();
    lv_scr_load(time_data_set_cont);
		lv_group_add_obj(group,time_data_set_cont);
}


#define PASSWORD_LENGTH 4  // 4位密码
static char correct_password[PASSWORD_LENGTH + 1] ;  // 默认密码
static char input_password[PASSWORD_LENGTH + 1] = {0};
static uint8_t input_index = 0;

static lv_obj_t * dot_indicators[PASSWORD_LENGTH];
static lv_obj_t * keyboard;
static lv_obj_t * msg_label;
static lv_obj_t * dots_container;
static lv_obj_t*dots_container_main;
static lv_obj_t * password_set_cont;
static lv_obj_t * password_show_KG_label;

static bool passward_set_flag=false;
static bool passward_show_flag=false;

static void update_dots_display(void) {
    for(int i = 0; i < PASSWORD_LENGTH; i++) {
        lv_obj_t * dot = dot_indicators[i];
        if(i < input_index) {
            lv_obj_set_style_bg_color(dot, lv_color_hex(0xffffff), 0);
            lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        } else {
            lv_obj_set_style_bg_color(dot, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_bg_opa(dot, LV_OPA_30, 0);
        }
    }
}


static void verify_password(void) {
    if(input_index == PASSWORD_LENGTH) {
        if(strncmp(input_password, correct_password, PASSWORD_LENGTH) == 0) {
					  lv_scr_load(main_cont);
						lv_group_focus_obj(main_cont); 
            lv_obj_del_delayed(dots_container_main,10);
					dots_container_main=NULL;
        } else {
					   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
             vTaskDelay(60);
						 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
					   vTaskDelay(60);
					   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
             vTaskDelay(60);
						 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            lv_label_set_text(msg_label, "密码错误");
            lv_obj_set_style_text_color(msg_label, lv_color_hex(0xFF0000), 0);
            memset(input_password, 0, sizeof(input_password));
            input_index = 0;
            update_dots_display();	
        }
    }
}

static void keyboard_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * kb = lv_event_get_target(e);
    lv_keyboard_t * keyboard = (lv_keyboard_t *)kb;

    if(code == LV_EVENT_VALUE_CHANGED) {
        uint16_t btn_id = lv_btnmatrix_get_selected_btn(kb);
        if(btn_id == LV_BTNMATRIX_BTN_NONE) return;

        const char * txt = lv_btnmatrix_get_btn_text(kb, btn_id);

        if(txt && txt[0] != '\0'&&passward_set_flag==false&&passward_show_flag==true) {
            if(isdigit(txt[0])) {
                if(input_index < PASSWORD_LENGTH) {
									lv_label_set_text(msg_label, "请输入密码:");
								  lv_obj_set_style_text_color(msg_label,lv_color_hex(0xffffff),0);
                    input_password[input_index++] = txt[0];  
								     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                     vTaskDelay(50);
								     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                    update_dots_display();

                     if(input_index == PASSWORD_LENGTH) {
                        verify_password();
                    }
                }

            }
            else if(strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
                if(input_index > 0) {
                    input_index--;
                    update_dots_display();
                    lv_label_set_text(msg_label, "");
                }
            }
        }

        if(passward_set_flag==true)
        {
                if (strcmp(txt, "0") == 0 || strcmp(txt, "1") == 0 || strcmp(txt, "2") == 0 ||
                    strcmp(txt, "3") == 0 || strcmp(txt, "4") == 0 || strcmp(txt, "5") == 0 ||
                    strcmp(txt, "6") == 0 || strcmp(txt, "7") == 0 || strcmp(txt, "8") == 0 ||
                    strcmp(txt, "9") == 0) {
                    if (input_index < PASSWORD_LENGTH) {
                        correct_password[input_index++] = txt[0];  
                        update_dots_display();
                        correct_password[input_index] = '\0';  
                        if(input_index==PASSWORD_LENGTH)
                        {
                        passward_set_flag=false;
												lv_label_set_text(password_show_KG_label,"密码已开启");
	                      lv_obj_set_style_text_color(password_show_KG_label,lv_color_hex(0x75FA61),0);
                        lv_scr_load(password_set_cont);
												lv_obj_del_delayed(dots_container_main,10);
												dots_container_main=NULL;
												lv_group_focus_obj(password_set_cont); 
                        }
                    }
                }
            else if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
                if (input_index > 0) {
                    input_index--;
                    update_dots_display();
                    correct_password[input_index] = '\0';
                }
            }
        }
    }
}
static void dots_container_main_event_handler(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT&&passward_set_flag==true&&passward_show_flag==true) {
        lv_scr_load(password_set_cont);
        passward_set_flag=false;
				passward_show_flag=false;       
				lv_obj_del(dots_container_main);
				dots_container_main=NULL;
				lv_group_focus_obj(password_set_cont); 
        }
				else if(lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT&&passward_set_flag==false&&passward_show_flag==true){
				lv_scr_load(wallpaper); 
			  lv_obj_del_delayed(dots_container_main,15);
				dots_container_main=NULL;
				lv_group_focus_obj(wallpaper); 
				}
}
static void dots_container_main_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
					uint32_t key = lv_event_get_key(e);
					if(key == LV_KEY_DOWN&&passward_set_flag==true&&passward_show_flag==true) {
					lv_scr_load(password_set_cont);
          passward_set_flag=false;
					passward_show_flag=false;		
					lv_obj_del(dots_container_main);
					dots_container_main=NULL;
					lv_group_focus_obj(password_set_cont); 
			 }
				else if(key == LV_KEY_DOWN&&passward_set_flag==false&&passward_show_flag==true){
				lv_scr_load(wallpaper); 
			  lv_obj_del_delayed(dots_container_main,15);
				dots_container_main=NULL;
				lv_group_focus_obj(wallpaper); 
				}
		}
}

LV_FONT_DECLARE(password_show);
void create_password_screen(void)
{ 
    dots_container_main=lv_obj_create(NULL);
	  lv_obj_add_event_cb(dots_container_main, dots_container_main_event_handler, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(dots_container_main,dots_container_main_KEY_event_cb, LV_EVENT_KEY, NULL);
    lv_obj_set_style_bg_color(dots_container_main, lv_color_hex(0x000000), 0);
    dots_container = lv_obj_create(dots_container_main);
    lv_obj_set_size(dots_container, 200, 50);
    lv_obj_set_style_bg_color(dots_container, lv_color_hex(0x000000), 0);
    lv_obj_align(dots_container, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_border_width(dots_container, 0, 0);  
    lv_obj_set_style_outline_width(dots_container, 0, 0); 
    lv_obj_set_style_shadow_width(dots_container, 0, 0);  

    for(int i = 0; i < PASSWORD_LENGTH; i++) {
        dot_indicators[i] = lv_obj_create(dots_container);
        lv_obj_set_size(dot_indicators[i], 13, 13);
        lv_obj_align(dot_indicators[i], LV_ALIGN_CENTER, -30+i*22, 0);
        lv_obj_set_style_radius(dot_indicators[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot_indicators[i], lv_color_hex(0xffffff), 0);
        lv_obj_set_style_bg_opa(dot_indicators[i], LV_OPA_30, 0);
        lv_obj_set_style_border_width(dot_indicators[i], 0, 0);
   }

    msg_label = lv_label_create(dots_container_main);
    lv_label_set_text(msg_label, "");
	  lv_obj_set_style_text_font(msg_label,&Setting_HZ,0);
    lv_obj_align(msg_label, LV_ALIGN_TOP_MID, 4, 30);
    lv_obj_set_style_text_color(msg_label, lv_color_hex(0xFF0000), 0);

    static const char * num_map[] = {
        "1", "2", "3", "\n",
        "4", "5", "6", "\n",
        "7", "8", "9", "\n",
        " ", "0",LV_SYMBOL_BACKSPACE, ""
    };
static const lv_btnmatrix_ctrl_t kb_ctrl_num1[] = {
    LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5,
    LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 5
};

    keyboard = lv_keyboard_create(dots_container_main);

    lv_obj_set_size(keyboard, 240, 142);
    lv_obj_set_style_bg_color(keyboard,lv_color_hex(0x000000),0);
    lv_obj_align(keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_map(keyboard, LV_KEYBOARD_MODE_NUMBER, num_map, kb_ctrl_num1);
    lv_keyboard_set_mode(keyboard,  LV_KEYBOARD_MODE_NUMBER);
    lv_obj_add_event_cb(keyboard, keyboard_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    memset(input_password, 0, sizeof(input_password));
    input_index = 0;
    update_dots_display();
    lv_scr_load(dots_container_main);
	  lv_group_add_obj(group,dots_container_main);
}


static void passward_btn_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * parent = lv_obj_get_parent(btn);

    if(code == LV_EVENT_CLICKED) {
        const char * btn_text = lv_label_get_text(lv_obj_get_child(btn, 0));
        if(strstr(btn_text, "密码设置")) {
            passward_set_flag=true;
					  passward_show_flag=true;
           create_password_screen();		
					lv_group_focus_obj(dots_container_main); 
				 	lv_label_set_text(msg_label, "请设置密码:");
		      lv_obj_set_style_text_color(msg_label,lv_color_hex(0xffffff),0);
        }
        else if(strstr(btn_text, "密码清除")) {
            passward_set_flag=false;
					  passward_show_flag=false;
						lv_label_set_text(password_show_KG_label,"密码未开启");
						lv_obj_set_style_text_color(password_show_KG_label,lv_color_hex(0xffffff),0);
        }
    }
}
static lv_obj_t * password_btn_enable;
static lv_obj_t * password_btn_cancel;
static void password_set_cont_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
        lv_scr_load(setting_show);
        lv_obj_del_delayed(password_set_cont,2);
					password_set_cont=NULL;
				lv_group_focus_obj(setting_show); 
        }
}
static void password_set_cont_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							lv_scr_load(setting_show);
							lv_obj_del_delayed(password_set_cont,2);
							 password_set_cont=NULL;
							lv_group_focus_obj(setting_show); 
						 }
				}
}
 void PASSWORD_SETTING(void)
 {
    password_set_cont = lv_obj_create(NULL);
	  lv_obj_add_event_cb(password_set_cont, password_set_cont_event_cb, LV_EVENT_GESTURE, NULL);
	 	lv_obj_add_event_cb(password_set_cont, password_set_cont_KEY_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_size(password_set_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(password_set_cont, lv_color_hex(0x0A0A20), 0);
    lv_obj_set_style_border_width(password_set_cont, 0, 0);
    lv_obj_set_flex_flow(password_set_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(password_set_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(password_set_cont, 20, 0);
	 
	 password_show_KG_label=lv_label_create(password_set_cont);
	 lv_obj_set_size(password_show_KG_label,200,50);
	 lv_label_set_text(password_show_KG_label,"");
	 lv_obj_align(password_show_KG_label,LV_ALIGN_TOP_MID,0,-50);
	 lv_obj_set_style_text_align(password_show_KG_label,LV_TEXT_ALIGN_CENTER,0);
	 lv_obj_set_style_text_color(password_show_KG_label,lv_color_hex(0xffffff),0);
	 lv_obj_set_style_text_font(password_show_KG_label,&password_show,0);
	 if(passward_show_flag)
	 {
		lv_label_set_text(password_show_KG_label,"密码已开启");
	  lv_obj_set_style_text_color(password_show_KG_label,lv_color_hex(0x75FA61),0);
	 }
	 else{
		lv_label_set_text(password_show_KG_label,"密码未开启");
	  lv_obj_set_style_text_color(password_show_KG_label,lv_color_hex(0xffffff),0);
	 }


    password_btn_enable = lv_btn_create(password_set_cont);
    lv_obj_set_size(password_btn_enable, 200, 60);
		lv_obj_set_style_radius(password_btn_enable, 15,0);
    lv_obj_set_style_bg_opa(password_btn_enable, LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(password_btn_enable, 2,0);
    lv_obj_set_style_border_color(password_btn_enable, lv_color_hex(0x5A5AFF),0);
    lv_obj_set_style_shadow_width(password_btn_enable, 10,0);
    lv_obj_set_style_shadow_color(password_btn_enable, lv_color_hex(0x5A5AFF),0);
    lv_obj_set_style_text_color(password_btn_enable, lv_color_hex(0xFFFFFF),0);
    lv_obj_set_style_pad_all(password_btn_enable, 15,0);
		
    lv_obj_set_style_shadow_width(password_btn_enable, 15,LV_STATE_FOCUSED);
    lv_obj_set_style_shadow_spread(password_btn_enable, 5,LV_STATE_FOCUSED);
    lv_obj_set_style_shadow_color(password_btn_enable, lv_color_hex(0x7F7FFF),LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(password_btn_enable, lv_color_hex(0x7F7FFF),LV_STATE_FOCUSED);
   
	  lv_obj_set_style_shadow_width(password_btn_enable, 5,LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(password_btn_enable, 0,LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(password_btn_enable, lv_color_hex(0x3A3AFF),LV_STATE_PRESSED);
    lv_obj_set_style_border_color(password_btn_enable, lv_color_hex(0x3A3AFF),LV_STATE_PRESSED);
    lv_obj_set_style_text_color(password_btn_enable, lv_color_hex(0xCCCCFF),LV_STATE_PRESSED);
    lv_obj_add_event_cb(password_btn_enable, passward_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_enable_label = lv_label_create(password_btn_enable);
    lv_label_set_text(btn_enable_label,  "密码设置");
    lv_obj_set_style_text_font(btn_enable_label, &Setting_HZ, 0);
    lv_obj_center(btn_enable_label);

    password_btn_cancel = lv_btn_create(password_set_cont);
    lv_obj_set_size(password_btn_cancel, 200, 60);
		lv_obj_set_style_radius(password_btn_cancel, 15,0);
    lv_obj_set_style_bg_opa(password_btn_cancel, LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(password_btn_cancel, 2,0);
    lv_obj_set_style_border_color(password_btn_cancel, lv_color_hex(0x5A5AFF),0);
    lv_obj_set_style_shadow_width(password_btn_cancel, 10,0);
    lv_obj_set_style_shadow_color(password_btn_cancel, lv_color_hex(0x5A5AFF),0);
    lv_obj_set_style_text_color(password_btn_cancel, lv_color_hex(0xFFFFFF),0);
    lv_obj_set_style_pad_all(password_btn_cancel, 15,0);
		
    lv_obj_set_style_shadow_width(password_btn_cancel, 15,LV_STATE_FOCUSED);
    lv_obj_set_style_shadow_spread(password_btn_cancel, 5,LV_STATE_FOCUSED);
    lv_obj_set_style_shadow_color(password_btn_cancel, lv_color_hex(0x7F7FFF),LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(password_btn_cancel, lv_color_hex(0x7F7FFF),LV_STATE_FOCUSED);
   
	  lv_obj_set_style_shadow_width(password_btn_cancel, 5,LV_STATE_PRESSED);
    lv_obj_set_style_shadow_spread(password_btn_cancel, 0,LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(password_btn_cancel, lv_color_hex(0x3A3AFF),LV_STATE_PRESSED);
    lv_obj_set_style_border_color(password_btn_cancel, lv_color_hex(0x3A3AFF),LV_STATE_PRESSED);
    lv_obj_set_style_text_color(password_btn_cancel, lv_color_hex(0xCCCCFF),LV_STATE_PRESSED);
    lv_obj_add_event_cb(password_btn_cancel, passward_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_cancel_label = lv_label_create(password_btn_cancel);
    lv_label_set_text(btn_cancel_label,  "密码清除");
    lv_obj_set_style_text_font(btn_cancel_label, &Setting_HZ, 0);
    lv_obj_center(btn_cancel_label);
		lv_scr_load(password_set_cont);
		lv_group_add_obj(group,password_set_cont);
}

 
#define TECH_BG_COLOR lv_color_hex(0x0a0a1a)
#define TECH_PRIMARY_COLOR lv_color_hex(0x6c4dff)
#define TECH_SECONDARY_COLOR lv_color_hex(0x00f0ff)
#define TECH_TEXT_COLOR lv_color_hex(0xffffff)
static uint16_t last_seconds=60;
static void slider_CL_event_cb(lv_event_t * e);
static void btn_event_cb(lv_event_t * e);
LV_FONT_DECLARE(LIGHT_hz);
static void slider_CL_event_cb(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    lv_obj_t * label = lv_event_get_user_data(e);

    int32_t seconds = lv_slider_get_value(slider);
    char buf[32];

    if(seconds < 60) {
			  last_seconds=seconds;
        lv_snprintf(buf, sizeof(buf), "当前设置: %d秒", seconds);
    } else {
				last_seconds=seconds;
        lv_snprintf(buf, sizeof(buf), "当前设置: %d分钟", seconds/60);
    }

    lv_label_set_text(label, buf);
}
static void btn_event_cb(lv_event_t * e) {
	 lv_timer_set_period(dim_timer,last_seconds*1000); 
}
static lv_obj_t*display_time_show;
static void display_time_show_event_cb(lv_event_t*e)
{
      lv_indev_t* indev = lv_event_get_indev(e);
	    lv_obj_t* target = lv_event_get_target(e);
			if (lv_obj_check_type(target, &lv_slider_class)) {
						return;
				}
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
						lv_scr_load(setting_show);
					  lv_obj_del_delayed(display_time_show,15);
					display_time_show=NULL;
						lv_group_focus_obj(setting_show); 
        }
}
static void display_time_show_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
						lv_scr_load(setting_show);
					  lv_obj_del_delayed(display_time_show,15);
							 display_time_show=NULL;
						lv_group_focus_obj(setting_show); 
						 }
				}
}

LV_FONT_DECLARE(Sun);
void create_display_time(void) {
	
	    char buff[32];

	   display_time_show=lv_obj_create(NULL);
		 lv_obj_add_event_cb(display_time_show, display_time_show_event_cb, LV_EVENT_GESTURE, NULL);
		 lv_obj_add_event_cb(display_time_show, display_time_show_KEY_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_style_bg_color(display_time_show, TECH_BG_COLOR, LV_PART_MAIN);

    lv_obj_t * title = lv_label_create(display_time_show);
    lv_label_set_text(title, "LIGHT");
    lv_obj_set_style_text_color(title, TECH_PRIMARY_COLOR, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  	lv_obj_t * icon = lv_label_create(display_time_show);
    lv_label_set_text(icon, LV_SYMBOL_SUN);
    lv_obj_set_style_text_font(icon, &Sun, LV_PART_MAIN); 
    lv_obj_set_style_text_color(icon, lv_color_hex(0xFAED00), 0);
    lv_obj_align_to(icon, title, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    lv_obj_set_style_img_recolor(icon, TECH_SECONDARY_COLOR, 0);
    lv_obj_set_style_img_recolor_opa(icon, LV_OPA_COVER, 0);

    lv_obj_t * current_label = lv_label_create(display_time_show);
		 if(last_seconds < 60) {
		lv_snprintf(buff, sizeof(buff), "当前设置:%d秒", last_seconds);
		 }
		 else
		 {
			 lv_snprintf(buff, sizeof(buff), "当前设置:%d分钟", last_seconds/60);
		 }
    lv_label_set_text(current_label, buff);
    lv_obj_set_style_text_color(current_label, TECH_TEXT_COLOR, 0);
    lv_obj_set_style_text_font(current_label, &LIGHT_hz, 0);
    lv_obj_align(current_label, LV_ALIGN_TOP_MID, 0, 65);
		
    lv_obj_t * slider = lv_slider_create(display_time_show);
		lv_obj_add_flag(slider, LV_OBJ_FLAG_EVENT_BUBBLE); 
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_GESTURE_BUBBLE); 
    lv_obj_set_size(slider, 200, 20);
    lv_slider_set_range(slider, 15, 1800); 
    lv_slider_set_value(slider, last_seconds, LV_ANIM_OFF); 
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 120);
    lv_obj_add_event_cb(slider, slider_CL_event_cb, LV_EVENT_VALUE_CHANGED, current_label);

    lv_obj_set_style_bg_color(slider, lv_color_hex(0x303050), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, TECH_PRIMARY_COLOR, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, TECH_SECONDARY_COLOR, LV_PART_KNOB);
    lv_obj_set_style_outline_color(slider, TECH_SECONDARY_COLOR, LV_PART_KNOB);
    lv_obj_set_style_outline_width(slider, 2, LV_PART_KNOB);
    lv_obj_set_style_shadow_color(slider, TECH_SECONDARY_COLOR, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(slider, 15, LV_PART_KNOB);
    lv_obj_set_style_shadow_spread(slider, 5, LV_PART_KNOB);

    lv_obj_t * min_label = lv_label_create(display_time_show);
    lv_label_set_text(min_label, "15秒");
	  lv_obj_set_style_text_font(min_label, &LIGHT_hz, 0);
    lv_obj_set_style_text_color(min_label, TECH_TEXT_COLOR, 0);
    lv_obj_align_to(min_label, slider, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 13);

    lv_obj_t * max_label = lv_label_create(display_time_show);
    lv_label_set_text(max_label, "30分钟");
		lv_obj_set_style_text_font(max_label, &LIGHT_hz, 0);
    lv_obj_set_style_text_color(max_label, TECH_TEXT_COLOR, 0);
    lv_obj_align_to(max_label, slider, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 13);

    lv_obj_t * btn = lv_btn_create(display_time_show);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 180);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_set_style_bg_color(btn, TECH_PRIMARY_COLOR, 0);
    lv_obj_set_style_bg_grad_color(btn, TECH_SECONDARY_COLOR, 0);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_shadow_color(btn, TECH_SECONDARY_COLOR, 0);
    lv_obj_set_style_shadow_width(btn, 20, 0);
    lv_obj_set_style_radius(btn, 25, 0);

    lv_obj_t * btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "确认");
		lv_obj_set_style_text_font(btn_label, &LIGHT_hz, 0);
    lv_obj_set_style_text_color(btn_label, lv_color_black(), 0);
    lv_obj_center(btn_label);

    lv_obj_t * line1 = lv_line_create(display_time_show);
    static lv_point_t line_points1[] = {{0, 0}, {50, 0}};
    lv_line_set_points(line1, line_points1, 2);
    lv_obj_set_style_line_color(line1, TECH_PRIMARY_COLOR, 0);
    lv_obj_set_style_line_width(line1, 2, 0);
    lv_obj_align(line1, LV_ALIGN_TOP_LEFT, 20, 100);

    lv_obj_t * line2 = lv_line_create(display_time_show);
    static lv_point_t line_points2[] = {{0, 0}, {50, 0}};
    lv_line_set_points(line2, line_points2, 2);
    lv_obj_set_style_line_color(line2, TECH_PRIMARY_COLOR, 0);
    lv_obj_set_style_line_width(line2, 2, 0);
    lv_obj_align(line2, LV_ALIGN_TOP_RIGHT, -20, 100);
		lv_scr_load(display_time_show);
		lv_group_add_obj(group,display_time_show);
}


#define SIZE 4
#define TILE_SIZE 40
#define TILE_MARGIN 5
#define BOARD_SIZE (TILE_SIZE * SIZE + TILE_MARGIN * (SIZE + 1)) // 225px total

typedef struct {
    lv_obj_t *cont;
    lv_obj_t *label;
    int value;
} tile_t;

static tile_t tiles[SIZE][SIZE];
static int board[SIZE][SIZE];
static int score = 0;
static lv_obj_t *score_label;

static void tile_set_pos(tile_t *tile, int row, int col) {
    lv_coord_t x = TILE_MARGIN + col * (TILE_SIZE + TILE_MARGIN);
    lv_coord_t y = TILE_MARGIN + row * (TILE_SIZE + TILE_MARGIN);
    lv_obj_set_pos(tile->cont, x, y);
}

static void update_ui(void) {
    char buf[8];
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            tile_t *tile = &tiles[i][j];
            int value = board[i][j];
            tile->value = value;

            if (value == 0) {
                lv_label_set_text(tile->label, "");
                lv_obj_set_style_bg_color(tile->cont, lv_color_hex(0xcdc1b4), 0);
            } else {
                snprintf(buf, sizeof(buf), "%d", value);
                lv_label_set_text(tile->label, buf);

                lv_color_t bg_color;
                switch (value) {
                    case 2: bg_color = lv_color_hex(0xeee4da); break;
                    case 4: bg_color = lv_color_hex(0xede0c8); break;
                    case 8: bg_color = lv_color_hex(0xf2b179); break;
                    case 16: bg_color = lv_color_hex(0xf59563); break;
                    case 32: bg_color = lv_color_hex(0xf67c5f); break;
                    case 64: bg_color = lv_color_hex(0xf65e3b); break;
                    case 128: bg_color = lv_color_hex(0xedcf72); break;
                    case 256: bg_color = lv_color_hex(0xedcc61); break;
                    case 512: bg_color = lv_color_hex(0xedc850); break;
                    case 1024: bg_color = lv_color_hex(0xedc53f); break;
                    case 2048: bg_color = lv_color_hex(0xedc22e); break;
                    default: bg_color = lv_color_hex(0x3c3a32); break;
                }

                lv_obj_set_style_bg_color(tile->cont, bg_color, 0);

                if (value <= 4)
                    lv_obj_set_style_text_color(tile->label, lv_color_hex(0x776e65), 0);
                else
                    lv_obj_set_style_text_color(tile->label, lv_color_hex(0xf9f6f2), 0);
            }

            tile_set_pos(tile, i, j);
        }
    }

    char score_text[32];
    snprintf(score_text, sizeof(score_text), "Score: %d", score);
    lv_label_set_text(score_label, score_text);
}

static void add_random_tile(void) {
    int empty[SIZE * SIZE][2];
    int count = 0;

    for (int i = 0; i < SIZE; i++)
        for (int j = 0; j < SIZE; j++)
            if (board[i][j] == 0)
                empty[count][0] = i, empty[count++][1] = j;

    if (count > 0) {
        int r = rand() % count;
        int val = (rand() % 10 < 9) ? 2 : 4;
        board[empty[r][0]][empty[r][1]] = val;
    }
}

static bool slide_left(void) {
    bool moved = false;
    for (int i = 0; i < SIZE; i++) {
        int last_merge = -1;
        for (int j = 1; j < SIZE; j++) {
            if (board[i][j] == 0) continue;
            int k = j;
            while (k > 0 && board[i][k - 1] == 0) {
                board[i][k - 1] = board[i][k];
                board[i][k] = 0;
                k--;
                moved = true;
            }
            if (k > 0 && board[i][k - 1] == board[i][k] && last_merge != k - 1) {
                board[i][k - 1] *= 2;
                score += board[i][k - 1];
                board[i][k] = 0;
                last_merge = k - 1;
                moved = true;
            }
        }
    }
    return moved;
}

static bool slide_right(void) {
    bool moved = false;
    for (int i = 0; i < SIZE; i++) {
        int last_merge = SIZE;
        for (int j = SIZE - 2; j >= 0; j--) {
            if (board[i][j] == 0) continue;
            int k = j;
            while (k < SIZE - 1 && board[i][k + 1] == 0) {
                board[i][k + 1] = board[i][k];
                board[i][k] = 0;
                k++;
                moved = true;
            }
            if (k < SIZE - 1 && board[i][k + 1] == board[i][k] && last_merge != k + 1) {
                board[i][k + 1] *= 2;
                score += board[i][k + 1];
                board[i][k] = 0;
                last_merge = k + 1;
                moved = true;
            }
        }
    }
    return moved;
}

static bool slide_up(void) {
    bool moved = false;
    for (int j = 0; j < SIZE; j++) {
        int last_merge = -1;
        for (int i = 1; i < SIZE; i++) {
            if (board[i][j] == 0) continue;
            int k = i;
            while (k > 0 && board[k - 1][j] == 0) {
                board[k - 1][j] = board[k][j];
                board[k][j] = 0;
                k--;
                moved = true;
            }
            if (k > 0 && board[k - 1][j] == board[k][j] && last_merge != k - 1) {
                board[k - 1][j] *= 2;
                score += board[k - 1][j];
                board[k][j] = 0;
                last_merge = k - 1;
                moved = true;
            }
        }
    }
    return moved;
}

static bool slide_down(void) {
    bool moved = false;
    for (int j = 0; j < SIZE; j++) {
        int last_merge = SIZE;
        for (int i = SIZE - 2; i >= 0; i--) {
            if (board[i][j] == 0) continue;
            int k = i;
            while (k < SIZE - 1 && board[k + 1][j] == 0) {
                board[k + 1][j] = board[k][j];
                board[k][j] = 0;
                k++;
                moved = true;
            }
            if (k < SIZE - 1 && board[k + 1][j] == board[k][j] && last_merge != k + 1) {
                board[k + 1][j] *= 2;
                score += board[k + 1][j];
                board[k][j] = 0;
                last_merge = k + 1;
                moved = true;
            }
        }
    }
    return moved;
}

static bool is_game_over(void) {
    for (int i = 0; i < SIZE; i++)
        for (int j = 0; j < SIZE; j++) {
            if (board[i][j] == 0) return false;
            if (j < SIZE - 1 && board[i][j] == board[i][j + 1]) return false;
            if (i < SIZE - 1 && board[i][j] == board[i + 1][j]) return false;
        }
    return true;
}

static void on_touch_event(lv_event_t *e) {
    static lv_point_t start;
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
        lv_indev_get_point(lv_indev_get_act(), &start);
    } else if (code == LV_EVENT_RELEASED) {
        lv_point_t end;
        lv_indev_get_point(lv_indev_get_act(), &end);

        int dx = end.x - start.x;
        int dy = end.y - start.y;
        bool moved = false;

        if (abs(dx) > abs(dy)) {
            moved = dx > 10 ? slide_right() : dx < -10 ? slide_left() : false;
        } else {
            moved = dy > 10 ? slide_down() : dy < -10 ? slide_up() : false;
        }

        if (moved) {
            add_random_tile();
            update_ui();
        }

        if (is_game_over()) {
            lv_label_set_text(score_label, "Game Over");
        }
    }
}

static void restart_cb(lv_event_t *e) {
    memset(board, 0, sizeof(board));
    score = 0;
    add_random_tile();
    add_random_tile();
    update_ui();
}


static void btn_return_cb(lv_event_t *e) {
	 lv_event_code_t code = lv_event_get_code(e);  
	if(code == LV_EVENT_CLICKED) {
		lv_scr_load(setting_show);
    lv_obj_del_delayed(game_2048_board_cont,15);
		game_2048_board_cont=NULL;
		lv_group_focus_obj(setting_show);
    }
        
}

static void game_2048_board_cont_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							lv_scr_load(setting_show);
							lv_obj_del_delayed(game_2048_board_cont,15);
							 game_2048_board_cont=NULL;
							lv_group_focus_obj(setting_show);
						 }
				}
}
void game_2048_create(void) {
    srand(lv_tick_get());
    game_2048_board_cont = lv_obj_create(NULL);
		lv_obj_add_event_cb(game_2048_board_cont, game_2048_board_cont_KEY_event_cb, LV_EVENT_KEY, NULL);

   	lv_obj_set_style_bg_color(game_2048_board_cont,lv_color_hex(0xE0E0E0),0);
	  lv_obj_t*board_cont=lv_obj_create(game_2048_board_cont);
    lv_obj_set_size(board_cont, 220, 220);
	  lv_obj_set_style_radius(board_cont,20, 0);
    lv_obj_align(board_cont, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(board_cont, lv_color_hex(0xbbada0), 0);
    lv_obj_clear_flag(board_cont, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            tile_t *t = &tiles[i][j];
            t->cont = lv_obj_create(board_cont);
            lv_obj_set_size(t->cont, TILE_SIZE, TILE_SIZE);
            lv_obj_set_style_radius(t->cont, 4, 0);
            lv_obj_set_style_bg_color(t->cont, lv_color_hex(0xcdc1b4), 0);
            lv_obj_set_style_border_width(t->cont, 0, 0);
            lv_obj_clear_flag(t->cont, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_clear_flag(t->cont, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(t->cont, LV_OBJ_FLAG_SCROLL_ELASTIC);
            lv_obj_clear_flag(t->cont, LV_OBJ_FLAG_SCROLL_MOMENTUM);
            lv_obj_clear_flag(t->cont, LV_OBJ_FLAG_CLICKABLE);

            t->label = lv_label_create(t->cont);
            lv_label_set_text(t->label, "");
            lv_obj_set_style_text_font(t->label, &lv_font_montserrat_14, 0);
            lv_obj_center(t->label);
        }
    }

    score_label = lv_label_create(game_2048_board_cont);
    lv_obj_align(score_label, LV_ALIGN_TOP_MID, 0, BOARD_SIZE +23);
    lv_label_set_text(score_label, "Score: 0");

    lv_obj_t *btn = lv_btn_create(game_2048_board_cont);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_size(btn, 80, 35);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Restart");
    lv_obj_center(btn_label);
    lv_obj_add_event_cb(btn, restart_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(board_cont, on_touch_event, LV_EVENT_ALL, NULL);
		
		lv_obj_t *btn_return = lv_btn_create(game_2048_board_cont);
    lv_obj_align(btn_return, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_size(btn_return, 80, 35);
    lv_obj_t *btn_return_label = lv_label_create(btn_return);
    lv_label_set_text(btn_return_label, "Return");
    lv_obj_center(btn_return_label);
    lv_obj_add_event_cb(btn_return, btn_return_cb, LV_EVENT_CLICKED, NULL);
		
    restart_cb(NULL);  
		lv_scr_load(game_2048_board_cont);
		lv_group_add_obj(group,game_2048_board_cont);
}


#define SCREEN_W 240
#define SCREEN_H 280
#define GRID_SIZE 3
#define CELL_SIZE (SCREEN_W / GRID_SIZE)

typedef enum { EMPTY=0, PLAYER1=1, PLAYER2=2 } CellState;

static lv_obj_t *board_circle[GRID_SIZE][GRID_SIZE];
static CellState board_state[GRID_SIZE][GRID_SIZE];
static lv_obj_t *label_info;
static bool player1_turn = false; 
static bool game_over = false;

static void draw_checkmark(lv_obj_t *cell) {
    lv_obj_clean(cell);

    lv_obj_set_style_bg_color(cell, lv_color_hex(0x007700), 0); 
    lv_obj_set_style_border_width(cell, 3, 0);
    lv_obj_set_style_border_color(cell, lv_color_white(), 0);

    lv_obj_t *label = lv_label_create(cell);
    lv_label_set_text(label, LV_SYMBOL_OK);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
}

static void draw_circle(lv_obj_t *cell) {
    lv_obj_clean(cell);

    lv_obj_set_style_bg_color(cell, lv_color_hex(0x000077), 0); 
    lv_obj_set_style_border_width(cell, 3, 0);
    lv_obj_set_style_border_color(cell, lv_color_white(), 0);

    lv_obj_t *label = lv_label_create(cell);
    lv_label_set_text(label, "O");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
}

static bool check_win(CellState player) {
    for (int i = 0; i < GRID_SIZE; i++) {
        if (board_state[i][0] == player &&
            board_state[i][1] == player &&
            board_state[i][2] == player) return true;
        if (board_state[0][i] == player &&
            board_state[1][i] == player &&
            board_state[2][i] == player) return true;
    }
    if (board_state[0][0] == player &&
        board_state[1][1] == player &&
        board_state[2][2] == player) return true;
    if (board_state[0][2] == player &&
        board_state[1][1] == player &&
        board_state[2][0] == player) return true;

    return false;
}

static void cell_event_cb1(lv_event_t *e) {
    if (game_over) return;

    lv_obj_t *cell = lv_event_get_target(e);
    int r = -1, c = -1;

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (board_circle[i][j] == cell) {
                r = i;
                c = j;
                break;
            }
        }
        if (r != -1) break;
    }
    if (r == -1 || c == -1) return;          
    if (board_state[r][c] != EMPTY) return;  

    if (player1_turn) {
        board_state[r][c] = PLAYER1;
        draw_checkmark(cell);
    } else {
        board_state[r][c] = PLAYER2;
        draw_circle(cell);
    }

    if (check_win(board_state[r][c])) {
        lv_label_set_text_fmt(label_info, "Player %d Wins!", player1_turn ? 1 : 2);
        game_over = true;
        return;
    }

    bool draw = true;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (board_state[i][j] == EMPTY) {
                draw = false;
                break;
            }
        }
        if (!draw) break;
    }
    if (draw) {
        lv_label_set_text(label_info, "Draw!");
        game_over = true;
        return;
    }
    player1_turn = !player1_turn;
    lv_label_set_text_fmt(label_info, "Player %d Turn", player1_turn ? 1 : 2);
}

static void reset_game_cb(lv_event_t *e) {
    game_over = false;
    player1_turn = false;  
    lv_label_set_text(label_info, "Player 2 Turn");

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            board_state[i][j] = EMPTY;
            lv_obj_clean(board_circle[i][j]);
            lv_obj_set_style_bg_color(board_circle[i][j], lv_color_hex(0xeeeeee), 0);
            lv_obj_set_style_border_color(board_circle[i][j], lv_color_hex(0x999999), 0);
            lv_obj_set_style_border_width(board_circle[i][j], 2, 0);
        }
    }
}
static lv_obj_t*tic_tac_scr;
static void tic_tac_scr_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
           lv_scr_load(setting_show);
           lv_obj_del_delayed(tic_tac_scr,15);
					tic_tac_scr=NULL;
					 lv_group_focus_obj(setting_show);
        }
}
static void tic_tac_scr_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							 lv_scr_load(setting_show);
							 lv_obj_del_delayed(tic_tac_scr,15);
							 tic_tac_scr=NULL;
							 lv_group_focus_obj(setting_show);
						 }
				}
}
void tic_tac_toe_two_player_start() {
	
	  tic_tac_scr=lv_obj_create(NULL);
    lv_obj_set_style_bg_color(tic_tac_scr, lv_color_hex(0xdddddd), 0);
	  lv_obj_add_event_cb(tic_tac_scr,tic_tac_scr_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(tic_tac_scr, tic_tac_scr_KEY_event_cb, LV_EVENT_KEY, NULL);

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            lv_obj_t *cell = lv_obj_create(tic_tac_scr);
            lv_obj_set_size(cell, CELL_SIZE - 15, CELL_SIZE - 15);
            lv_obj_set_style_bg_color(cell, lv_color_hex(0xeeeeee), 0);
            lv_obj_set_style_border_color(cell, lv_color_hex(0x999999), 0);
            lv_obj_set_style_border_width(cell, 2, 0);
            lv_obj_set_pos(cell, j * CELL_SIZE + 7, i * CELL_SIZE + 30);
            lv_obj_add_event_cb(cell, cell_event_cb1, LV_EVENT_CLICKED, NULL);

            board_circle[i][j] = cell;
            board_state[i][j] = EMPTY;
        }
    }

    label_info = lv_label_create(tic_tac_scr);
    lv_label_set_text(label_info, "Player 2 Turn");
    lv_obj_set_style_text_font(label_info, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_info, lv_color_black(), 0);
    lv_obj_align(label_info, LV_ALIGN_TOP_MID, 0, 5);

    lv_obj_t *btn_reset = lv_btn_create(tic_tac_scr);
    lv_obj_set_size(btn_reset, 100, 32);
    lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_add_event_cb(btn_reset, reset_game_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label_btn = lv_label_create(btn_reset);
    lv_label_set_text(label_btn, "Restart");
    lv_obj_set_style_text_color(label_btn, lv_color_black(), 0);
    lv_obj_center(label_btn);

    game_over = false;
    player1_turn = false;  
		lv_scr_load(tic_tac_scr);
		lv_group_add_obj(group,tic_tac_scr);
}



static lv_obj_t *bluetooth_screen;
static lv_obj_t * text_area;

void serial_add_log(const char * message)
{
    if(text_area != NULL) {
          if (strlen(lv_textarea_get_text(text_area)) >= 40) {
            lv_textarea_set_text(text_area, ""); // 清空
        }
        lv_textarea_add_text(text_area, message);
        lv_textarea_add_text(text_area, "\n");
        lv_textarea_set_cursor_pos(text_area, LV_TEXTAREA_CURSOR_LAST);
			}
}
static void bluetooth_screen_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
           lv_scr_load(setting_show);
           lv_obj_del_delayed(bluetooth_screen,15);
					 bluetooth_screen=NULL;
					 lv_group_focus_obj(setting_show);
        }
}
static void bluetooth_screen_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
							 lv_scr_load(setting_show);
							 lv_obj_del_delayed(bluetooth_screen,15);
							 bluetooth_screen=NULL;
							 lv_group_focus_obj(setting_show);
						 }
				}
}
void bt_control_center_create(void) {
    bluetooth_screen = lv_obj_create(NULL);
	  lv_obj_add_event_cb(bluetooth_screen,bluetooth_screen_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(bluetooth_screen, bluetooth_screen_KEY_event_cb, LV_EVENT_KEY, NULL);
	
    lv_obj_set_size(bluetooth_screen,LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(bluetooth_screen, lv_color_hex(0xE8F4FD), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(bluetooth_screen, lv_color_hex(0xD4E8F5), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(bluetooth_screen, LV_GRAD_DIR_VER, LV_STATE_DEFAULT);


    text_area = lv_textarea_create(bluetooth_screen);
    lv_obj_set_size(text_area, 220, 180);
    lv_obj_align(text_area, LV_ALIGN_TOP_MID, 0, 15);

  	lv_obj_set_style_bg_color(text_area, lv_color_hex(0x0A0E1A), 0);
    lv_obj_set_style_bg_opa(text_area, LV_OPA_70, 0);
    lv_obj_set_style_radius(text_area, 12, 0);
    lv_obj_set_style_border_width(text_area, 1, 0);
    lv_obj_set_style_border_color(text_area, lv_color_hex(0x2A2F3E), 0);

    lv_obj_set_style_text_color(text_area, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(text_area, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_letter_space(text_area, 1, 0);
    lv_obj_set_style_text_line_space(text_area, 4, 0);

    lv_obj_set_style_bg_color(text_area, lv_color_hex(0xFFFFFF), LV_PART_CURSOR);
    lv_obj_set_style_bg_opa(text_area, LV_OPA_70, LV_PART_CURSOR);
    lv_textarea_set_cursor_click_pos(text_area, true);

    lv_obj_set_style_bg_color(text_area, lv_color_hex(0xFFFFFF), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(text_area, LV_OPA_40, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(text_area, 4, LV_PART_SCROLLBAR);


    lv_obj_t*open_btn=lv_btn_create(bluetooth_screen);
    lv_obj_set_size(open_btn, 100, 40);
    lv_obj_align(open_btn, LV_ALIGN_BOTTOM_LEFT, 10, -18);
    lv_obj_set_style_bg_color(open_btn, lv_color_hex(0x636363), 0);
     lv_obj_set_style_bg_opa(open_btn, LV_OPA_70, 0);
    lv_obj_set_style_radius(open_btn, 5, 0);

    lv_obj_t * start_label = lv_label_create(open_btn);
    lv_label_set_text(start_label, "OPEN");
    lv_obj_center(start_label);

    lv_obj_t*clear_btn=lv_btn_create(bluetooth_screen);
    lv_obj_set_size(clear_btn, 100, 40);
    lv_obj_align(clear_btn, LV_ALIGN_BOTTOM_LEFT,130, -18);
    lv_obj_set_style_bg_color(clear_btn, lv_color_hex(0x3282F6), 0);
    lv_obj_set_style_bg_opa(clear_btn, LV_OPA_70, 0);
    lv_obj_set_style_radius(clear_btn, 5, 0);

    lv_obj_t * clear_label = lv_label_create(clear_btn);
    lv_label_set_text(clear_label, "CLEAR");
    lv_obj_center(clear_label);
    lv_scr_load(bluetooth_screen);
		lv_group_add_obj(group,bluetooth_screen);
}


static void time_date_btn_event_handler(lv_event_t * e)
{
     lv_event_code_t code = lv_event_get_code(e);
      if(code == LV_EVENT_CLICKED) {
        create_time_setting_ui();
				lv_group_focus_obj(time_data_set_cont);
   }
}
static void hand_btn_event_handler(lv_event_t*e)
{
	lv_event_code_t code = lv_event_get_code(e);
      if(code == LV_EVENT_CLICKED) {
          create_tech_ui();
					lv_group_focus_obj(hand_cont);
   }
}

static void Bluetooth_btn_event_handler(lv_event_t*e)
{
		lv_event_code_t code = lv_event_get_code(e);
      if(code == LV_EVENT_CLICKED) {
				bt_control_center_create();
				lv_group_focus_obj(bluetooth_screen);
   }
}

static void Passward_btn_event_handler(lv_event_t*e)
{
	lv_event_code_t code = lv_event_get_code(e);
      if(code == LV_EVENT_CLICKED) {
          PASSWORD_SETTING();
					lv_group_focus_obj(password_set_cont);
   }
}

static void Display_time_btn_event_handler(lv_event_t*e)
{
	lv_event_code_t code = lv_event_get_code(e);
      if(code == LV_EVENT_CLICKED) {
					create_display_time();
					lv_group_focus_obj(display_time_show);
			}
}

static void game_2048_btn_event_handler(lv_event_t * e)
{
   lv_event_code_t code = lv_event_get_code(e);
   lv_obj_t * obj = lv_event_get_target(e);
   if(code == LV_EVENT_CLICKED) {
		  game_2048_create();
		 	lv_group_focus_obj(game_2048_board_cont);
   }
}

static void Circle_OK_btn_event_handler(lv_event_t * e)
{
   lv_event_code_t code = lv_event_get_code(e);
   lv_obj_t * obj = lv_event_get_target(e);
   if(code == LV_EVENT_CLICKED) {
		tic_tac_toe_two_player_start();
		lv_group_focus_obj(tic_tac_scr);

   }
}

static void Compass_btn_event_handler(lv_event_t * e)
{
   lv_event_code_t code = lv_event_get_code(e);
   lv_obj_t * obj = lv_event_get_target(e);
   if(code == LV_EVENT_CLICKED) {
		create_compass_screen();
  	lv_group_focus_obj(compass_scr);
   }
}

static void setting_show_event_cb(lv_event_t * e)
{
        lv_indev_t* indev = lv_event_get_indev(e);
        if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
           lv_scr_load(next_main_cont);
           lv_obj_del_delayed(setting_show,15);
					 setting_show=NULL;
					 lv_group_focus_obj(next_main_cont); 
        }
}

static void setting_show_KEY_event_cb(lv_event_t * e)
{
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						 if(key == LV_KEY_DOWN) {
              lv_scr_load(next_main_cont);
              lv_obj_del_delayed(setting_show,15);
							setting_show=NULL;
							lv_group_focus_obj(next_main_cont); 
						}
				}
}

LV_FONT_DECLARE(Clock);
LV_FONT_DECLARE(Hand);
LV_FONT_DECLARE(QRcode);
LV_FONT_DECLARE(Display);
LV_FONT_DECLARE(Password);
LV_FONT_DECLARE(ComPass);
LV_FONT_DECLARE(tic_tac);
LV_FONT_DECLARE(Game_HZ);

void SETTING_SHOW(void)
{
	  setting_show=lv_obj_create(NULL);
    lv_obj_t* list1 = lv_list_create(setting_show);
	  lv_obj_add_event_cb(setting_show,setting_show_event_cb, LV_EVENT_GESTURE, NULL);
		lv_obj_add_event_cb(setting_show,setting_show_KEY_event_cb, LV_EVENT_KEY, NULL);

    lv_obj_set_style_bg_color(list1,lv_color_hex(0xE3E3E3),0);
    lv_obj_set_size(list1, 240, 280);
    lv_obj_center(list1);
    lv_obj_set_style_pad_row(list1, 5, 0);

    lv_obj_t*time_date_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(time_date_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(time_date_btn, 0, 0);  
    lv_obj_set_style_outline_width(time_date_btn, 0, 0);
    lv_obj_set_style_shadow_width(time_date_btn, 0, 0);  
    lv_obj_set_style_bg_color(time_date_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(time_date_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(time_date_btn,210, 65); 
    lv_obj_set_flex_flow(time_date_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(time_date_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * icon_cont = lv_obj_create(time_date_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0); 
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0x60D2D4),0);
    lv_obj_set_size(icon_cont, 45, 45);

    lv_obj_t * icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, LV_SYMBOL_CLOCK);
    lv_obj_set_style_text_font(icon, &Clock, 0);
    lv_obj_center(icon);
 
    lv_obj_t * time_set_label = lv_label_create(time_date_btn);
    lv_label_set_text(time_set_label, "时间日期设置");
    lv_obj_set_style_text_color(time_set_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(time_set_label, &Setting_HZ, 0);
    lv_obj_set_style_pad_left(time_set_label, 16, 0);
    lv_obj_add_event_cb(time_date_btn, time_date_btn_event_handler, LV_EVENT_CLICKED, NULL);


    lv_obj_t*hand_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(hand_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hand_btn, 0, 0);  
    lv_obj_set_style_outline_width(hand_btn, 0, 0); 
    lv_obj_set_style_shadow_width(hand_btn, 0, 0);  // 无阴影
    lv_obj_set_style_bg_color(hand_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(hand_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(hand_btn,210, 65); // 设置按钮大小
    lv_obj_set_flex_flow(hand_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hand_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    icon_cont = lv_obj_create(hand_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  // 无边框
    lv_obj_set_style_outline_width(icon_cont, 0, 0); // 无轮廓（聚焦时）
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  // 无阴影
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);	
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0xEA3FF7),0);
    lv_obj_set_size(icon_cont, 45, 45);
    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, LV_SYMBOL_HAND);
    lv_obj_set_style_text_font(icon, &Hand, 0);
    lv_obj_center(icon);
    lv_obj_t * raise_label = lv_label_create(hand_btn);
    lv_label_set_text(raise_label, "抬腕亮屏");
    lv_obj_set_style_text_color(raise_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(raise_label, &Setting_HZ, 0);
    lv_obj_set_style_pad_left(raise_label, 16, 0);
    lv_obj_add_event_cb(hand_btn, hand_btn_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t*Bluetooth_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(Bluetooth_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(Bluetooth_btn, 0, 0);  
    lv_obj_set_style_outline_width(Bluetooth_btn, 0, 0); 
    lv_obj_set_style_shadow_width(Bluetooth_btn, 0, 0);  
    lv_obj_set_style_bg_color(Bluetooth_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(Bluetooth_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(Bluetooth_btn,210, 65); 
    //lv_obj_set_style_bg_color(btn,lv_color_hex(0xffffff),0);
    lv_obj_set_flex_flow(Bluetooth_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(Bluetooth_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    icon_cont = lv_obj_create(Bluetooth_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0); 
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);		
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0x3282F6),0);
    lv_obj_set_size(icon_cont, 45, 45);

    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_20, 0);
    lv_obj_center(icon);

    lv_obj_t * bluetooth_label = lv_label_create(Bluetooth_btn);
    lv_label_set_text(bluetooth_label, "蓝牙设置");
    lv_obj_set_style_text_color(bluetooth_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(bluetooth_label, &Setting_HZ, 0);
    lv_obj_set_style_pad_left(bluetooth_label, 16, 0);
    lv_obj_add_event_cb(Bluetooth_btn, Bluetooth_btn_event_handler, LV_EVENT_CLICKED, NULL);


    lv_obj_t*Display_time_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(Display_time_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(Display_time_btn, 0, 0);  
    lv_obj_set_style_outline_width(Display_time_btn, 0, 0);
    lv_obj_set_style_shadow_width(Display_time_btn, 0, 0);  
    lv_obj_set_style_bg_color(Display_time_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(Display_time_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(Display_time_btn,210, 65); 
    //lv_obj_set_style_bg_color(btn,lv_color_hex(0xffffff),0);
    lv_obj_set_flex_flow(Display_time_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(Display_time_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    icon_cont = lv_obj_create(Display_time_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0); 
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0xE8E64D),0);
	  lv_obj_set_size(icon_cont, 45, 45);

    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, LV_SYMBOL_DISPLAY);
	  lv_obj_set_style_text_font(icon, &Display, 0);
    lv_obj_center(icon);

    lv_obj_t * display_label = lv_label_create(Display_time_btn);
    lv_label_set_text(display_label, "常亮时间");
    lv_obj_set_style_text_color(display_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(display_label, &Setting_HZ, 0);
    lv_obj_set_style_pad_left(display_label, 16, 0);
	  lv_obj_add_event_cb(Display_time_btn, Display_time_btn_event_handler, LV_EVENT_CLICKED, NULL);

		lv_obj_t*Compass_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(Compass_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(Compass_btn, 0, 0);  
    lv_obj_set_style_outline_width(Compass_btn, 0, 0); 
    lv_obj_set_style_shadow_width(Compass_btn, 0, 0);  
    lv_obj_set_style_bg_color(Compass_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(Compass_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(Compass_btn,210, 65); 
    lv_obj_set_flex_flow(Compass_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(Compass_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    icon_cont = lv_obj_create(Compass_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0); 
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  // 无阴影
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);	
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0x00108A),0);
    lv_obj_set_size(icon_cont, 45, 45);

    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon,LV_SYMBOL_COMPASS);
    lv_obj_set_style_text_font(icon, &ComPass, 0);
    lv_obj_align(icon,LV_ALIGN_CENTER,0,0);
    
    lv_obj_t * compass_label = lv_label_create(Compass_btn);
    lv_label_set_text(compass_label, "指南针");
    lv_obj_set_style_text_color(compass_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(compass_label, &Game_HZ, 0);
    lv_obj_set_style_pad_left(compass_label, 16, 0);
    lv_obj_add_event_cb(Compass_btn, Compass_btn_event_handler, LV_EVENT_CLICKED, NULL);

		lv_obj_t*Circle_OK_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(Circle_OK_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(Circle_OK_btn, 0, 0);  
    lv_obj_set_style_outline_width(Circle_OK_btn, 0, 0); 
    lv_obj_set_style_shadow_width(Circle_OK_btn, 0, 0);  
    lv_obj_set_style_bg_color(Circle_OK_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(Circle_OK_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(Circle_OK_btn,210, 65); 
    lv_obj_set_flex_flow(Circle_OK_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(Circle_OK_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    icon_cont = lv_obj_create(Circle_OK_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0);
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);	
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0x65D642),0);
    lv_obj_set_size(icon_cont, 45, 45);
 
    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon,LV_SYMBOL_TIC);
    lv_obj_set_style_text_font(icon, &tic_tac, 0);
    lv_obj_align(icon,LV_ALIGN_CENTER,0,0);

    lv_obj_t * code_label = lv_label_create(Circle_OK_btn);
    lv_label_set_text(code_label, "井字棋");
    lv_obj_set_style_text_color(code_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(code_label, &Game_HZ, 0);
    lv_obj_set_style_pad_left(code_label, 16, 0);
    lv_obj_add_event_cb(Circle_OK_btn, Circle_OK_btn_event_handler, LV_EVENT_CLICKED, NULL);
		
		lv_obj_t* game_2048_btn= lv_btn_create(list1);
    lv_obj_set_style_bg_opa(game_2048_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(game_2048_btn, 0, 0);  
    lv_obj_set_style_outline_width(game_2048_btn, 0, 0);
    lv_obj_set_style_shadow_width(game_2048_btn, 0, 0);  
    lv_obj_set_style_bg_color(game_2048_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(game_2048_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(game_2048_btn,210, 65); 
    lv_obj_set_layout(game_2048_btn, LV_LAYOUT_FLEX);
		lv_obj_set_flex_flow(game_2048_btn, LV_FLEX_FLOW_ROW);

    lv_obj_set_flex_align(game_2048_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    icon_cont = lv_obj_create(game_2048_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0);
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0xF08650),0);
	  lv_obj_set_size(icon_cont, 45, 45);

    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, "2048");
	  lv_obj_set_style_text_font(icon, &lv_font_montserrat_16, 0);
    lv_obj_center(icon);
		
		 lv_obj_t *game_label = lv_label_create(game_2048_btn);
    lv_label_set_text(game_label, "2048");
    lv_obj_set_style_text_color(game_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(game_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_pad_left(game_label, 16, 0);
		
    lv_obj_t *game_2048_label = lv_label_create(game_2048_btn);
    lv_label_set_text(game_2048_label, "游戏");
    lv_obj_set_style_text_color(game_2048_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(game_2048_label, &Game_HZ, 0);
    lv_obj_add_event_cb(game_2048_btn, game_2048_btn_event_handler, LV_EVENT_CLICKED, NULL);
		
		
		
		lv_obj_t*Password_btn = lv_btn_create(list1);
    lv_obj_set_style_bg_opa(Password_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(Password_btn, 0, 0);  
    lv_obj_set_style_outline_width(Password_btn, 0, 0); 
    lv_obj_set_style_shadow_width(Password_btn, 0, 0); 
    lv_obj_set_style_bg_color(Password_btn, lv_color_hex(0x000000), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(Password_btn, LV_OPA_20, LV_STATE_PRESSED);  // 20% 透明度
    lv_obj_set_size(Password_btn,210, 65); 
    lv_obj_set_flex_flow(Password_btn, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(Password_btn, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  	
    icon_cont = lv_obj_create(Password_btn);
    lv_obj_set_style_border_width(icon_cont, 0, 0);  
    lv_obj_set_style_outline_width(icon_cont, 0, 0); 
    lv_obj_set_style_shadow_width(icon_cont, 0, 0);  
    lv_obj_set_style_radius(icon_cont,LV_RADIUS_CIRCLE,0);
		lv_obj_set_style_bg_opa(icon_cont,LV_OPA_COVER,0);
		lv_obj_set_style_pad_all(icon_cont,1,0);
		lv_obj_set_style_text_color(icon_cont,lv_color_white(),0);
		lv_obj_set_style_bg_color(icon_cont,lv_color_hex(0x0023F5),0);
	  lv_obj_set_size(icon_cont, 45, 45);
	  
    icon = lv_label_create(icon_cont);
    lv_label_set_text(icon, LV_SYMBOL_PASSWORD);
	  lv_obj_set_style_text_font(icon, &Password, 0);
    lv_obj_center(icon);
    
    lv_obj_t * password_label = lv_label_create(Password_btn);
    lv_label_set_text(password_label, "密码设置");
    lv_obj_set_style_text_color(password_label,lv_color_hex(0x000000),0);
    lv_obj_set_style_text_font(password_label, &Setting_HZ, 0);
    lv_obj_set_style_pad_left(password_label, 16, 0);
    lv_obj_add_event_cb(Password_btn, Passward_btn_event_handler, LV_EVENT_CLICKED, NULL);
		
    lv_scr_load(setting_show);
		lv_group_add_obj(group,setting_show);
}



static void cont_gesture_event_cb(lv_event_t* e) {
    lv_indev_t* indev = lv_event_get_indev(e);
    lv_obj_t* panel = lv_event_get_user_data(e);

    if (lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT) {
      lv_scr_load_anim(main_cont,LV_SCR_LOAD_ANIM_FADE_ON,100,0,false);
          lv_group_focus_obj(main_cont); 
    }
}
static void cont_key_event_cb(lv_event_t* e) {
     lv_event_code_t code = lv_event_get_code(e);
    if (code==LV_EVENT_KEY) {
			uint32_t key = lv_event_get_key(e);
						if(key == LV_KEY_DOWN) {			
      lv_scr_load_anim(main_cont,LV_SCR_LOAD_ANIM_FADE_ON,100,0,false);
				 lv_group_focus_obj(main_cont); 
        }  
    }
}

void circle_event_cb(lv_event_t*e) {
 lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e); 
    if(event_code == LV_EVENT_CLICKED) { 

			lv_area_t btn_coords;
      lv_obj_get_coords(btn, &btn_coords);
      if(btn==Calendar)
      {
        Calendar_show();
				lv_group_focus_obj(calendaR); 
      }
      else if(btn==calculator)
      {
				calculator_create();
				lv_group_focus_obj(calculator_show);
      }
			else if(btn==WAllpaper)
      {
        Wallpaper_show();
				lv_group_focus_obj(wallpaperer_show);
      }
			else if(btn==Stopwatch)
			{
				create_stopwatch_ui();
				lv_group_focus_obj(Stop_watch_show);
			}
			else if(btn==Envirnment)
			{
				create_futuristic_weather_ui();
				lv_group_focus_obj(scr);
			}
			else if(btn==Alarm_clock)
			{
				create_alarm_clock_show();
				lv_group_focus_obj(clock_scr);	
			}
			else if(btn==setting)
			{
				SETTING_SHOW();
				lv_group_focus_obj(setting_show);
			}
			else if(btn==Watch_dial)
			{
				create_watch_dial();
        time_clock_create();
				lv_group_focus_obj(triangle_btn); 
			}
			else if(btn==About)
      {
        About_show();
				lv_group_focus_obj(about_show);
      }

    }
}
void circular(void)
{
    circle = lv_obj_create(next_main_cont);
    lv_obj_remove_style_all(circle);                 
    lv_obj_set_size(circle, 68, 68);
    lv_obj_set_style_radius(circle,34,0);
    lv_obj_align(circle,LV_ALIGN_CENTER, -73, 0);
    
    lv_obj_set_style_border_width(circle, 2, 0);
    lv_obj_set_style_border_color(circle,lv_color_hex(0x75F94D), 0);
    lv_obj_move_foreground(circle); // 
    lv_obj_clear_flag(circle, LV_OBJ_FLAG_CLICKABLE); 
}
static lv_obj_t *item_name = NULL;

void item_main(void)
{
    lv_obj_t*item_cont=lv_obj_create(next_main_cont);
    lv_obj_set_size(item_cont,96,44);
  	lv_obj_clear_flag(item_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(item_cont,LV_ALIGN_CENTER,60,0);
    lv_obj_set_style_bg_color(item_cont,lv_color_hex(0x000000),0);
    lv_obj_set_style_border_width(item_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(item_cont, 0, LV_PART_MAIN);
    item_name=lv_label_create(item_cont);
	  lv_label_set_text(item_name,"Calendar");
	  lv_obj_set_style_text_color(item_name,lv_color_hex(0xEA3FF7),0);
	  lv_obj_set_style_text_font(item_name, &lv_font_montserrat_16, 0);
    lv_obj_align(item_name, LV_ALIGN_CENTER, 0, 0);

}

typedef enum {
    BTN_CALENDAR = 0,
    BTN_CALCULATOR,
    BTN_ENVIRONMENT,
    BTN_WALLPAPER,
    BTN_STOPWATCH,
    BTN_ALARM_CLOCK,
    BTN_SETTINGS,
	  BTN_WATCH_DIAL,
    BTN_ABOUT,
    BTN_COUNT  // 按钮总数
} ButtonIndex;

static void update_selected_button(uint8_t btn_index) {
    const char *text = NULL;

    switch (btn_index) {
        case BTN_CALENDAR:    text = "Calendar"; break;
        case BTN_CALCULATOR:  text = "Calculator"; break;
        case BTN_ENVIRONMENT: text = "Ambience"; break;
        case BTN_WALLPAPER:   text = "Wallpaper"; break;
        case BTN_STOPWATCH:   text = "Stopwatch"; break;
        case BTN_ALARM_CLOCK: text = "Alarm Clock"; break;
        case BTN_SETTINGS:    text = "Settings"; break;
			  case BTN_WATCH_DIAL:  text = "Watch Dial";break;
        case BTN_ABOUT:       text = "About"; break;
        default:              text = "Select"; break;
    }

    if (item_name) {
        lv_label_set_text(item_name, text);
    }
}

static void scroll_event_cb(lv_event_t *e) {
    lv_obj_t *cont = lv_event_get_target(e);  
    lv_coord_t cont_height = lv_obj_get_height(cont);
		lv_coord_t scroll_y = lv_obj_get_scroll_y(cont);
		lv_coord_t center_y = scroll_y + cont_height / 2;

    uint8_t btn_index = 0;
    lv_obj_t *child = lv_obj_get_child(cont, btn_index);  
    while (child) {
        lv_coord_t child_y = lv_obj_get_y(child);  
        lv_coord_t child_height = lv_obj_get_height(child);  

        if (child_y <= center_y && (child_y + child_height) > center_y) {
            update_selected_button(btn_index);  
            break;  
        }
        btn_index++; 
        child = lv_obj_get_child(cont, btn_index);  
    }
}


LV_IMG_DECLARE(SETTING);
LV_IMG_DECLARE(calendar);
LV_IMG_DECLARE(Calculator);
LV_IMG_DECLARE(compass);
LV_IMG_DECLARE(environment);
LV_IMG_DECLARE(wallper);
LV_IMG_DECLARE(stopwatch);
LV_IMG_DECLARE(about);
LV_IMG_DECLARE(dial);
LV_IMG_DECLARE(alarm_clock);

void page_end(void)
{
    next_main_cont=lv_obj_create(NULL);
    next_cont = lv_obj_create(next_main_cont);

	  lv_obj_set_style_bg_color(next_main_cont,lv_color_hex(0x000000),0);
	  
		lv_obj_set_scroll_dir(next_main_cont, LV_DIR_ALL);  // ????????
		lv_obj_set_scrollbar_mode(next_main_cont, LV_SCROLLBAR_MODE_AUTO); // ???????
			
		lv_obj_set_size(next_cont, 160, 284);
		lv_obj_set_style_border_width(next_cont, 0, LV_PART_MAIN);
		lv_obj_set_style_shadow_width(next_cont, 0, LV_PART_MAIN);
	
    lv_obj_set_style_bg_color(next_cont,lv_color_hex(0x000000),0);
		lv_obj_align(next_cont,LV_ALIGN_CENTER,-15,0);
    lv_obj_set_flex_flow(next_cont, LV_FLEX_FLOW_COLUMN );//????????????
		
    lv_obj_add_event_cb(next_cont, scroll_event_cb, LV_EVENT_SCROLL, NULL);

    lv_obj_set_scroll_dir(next_cont, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(next_cont, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(next_cont, LV_SCROLLBAR_MODE_OFF);
	  lv_obj_scroll_to_view(next_cont, LV_ANIM_OFF);
     
		 
		 
        Calendar= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(Calendar,35,0);
        lv_obj_set_size(Calendar,64,64);
        lv_obj_set_style_clip_corner(Calendar, true, 0);       
        lv_imgbtn_set_src(Calendar,LV_IMGBTN_STATE_RELEASED,NULL,&calendar,NULL);
        lv_obj_add_event_cb(Calendar, circle_event_cb, LV_EVENT_CLICKED, 0);

        calculator= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(calculator,35,0);
        lv_obj_set_size(calculator,64,64);
        lv_obj_set_style_clip_corner(calculator, true, 0);       
        lv_imgbtn_set_src(calculator,LV_IMGBTN_STATE_RELEASED,NULL,&Calculator,NULL);
        lv_obj_add_event_cb(calculator, circle_event_cb, LV_EVENT_CLICKED, 0);

        Envirnment= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(Envirnment,35,0);
        lv_obj_set_size(Envirnment,64,64);
        lv_obj_set_style_clip_corner(Envirnment, true, 0);       
        lv_imgbtn_set_src(Envirnment,LV_IMGBTN_STATE_RELEASED,NULL,&environment,NULL);
				lv_obj_add_event_cb(Envirnment, circle_event_cb, LV_EVENT_CLICKED, 0);

				
        WAllpaper= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(WAllpaper,35,0);
        lv_obj_set_size(WAllpaper,64,64);
        lv_obj_set_style_clip_corner(WAllpaper, true, 0);       
        lv_imgbtn_set_src(WAllpaper,LV_IMGBTN_STATE_RELEASED,NULL,&wallper,NULL);
        lv_obj_add_event_cb(WAllpaper, circle_event_cb, LV_EVENT_CLICKED, 0);

        Stopwatch= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(Stopwatch,35,0);
        lv_obj_set_size(Stopwatch,64,64);
        lv_obj_set_style_clip_corner(Stopwatch, true, 0);       
        lv_imgbtn_set_src(Stopwatch,LV_IMGBTN_STATE_RELEASED,NULL,&stopwatch,NULL);
				lv_obj_add_event_cb(Stopwatch, circle_event_cb, LV_EVENT_CLICKED, 0);

        Alarm_clock= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(Alarm_clock,35,0);
        lv_obj_set_size(Alarm_clock,64,64);
        lv_obj_set_style_clip_corner(Alarm_clock, true, 0);      
        lv_imgbtn_set_src(Alarm_clock,LV_IMGBTN_STATE_RELEASED,NULL,&alarm_clock,NULL);
				lv_obj_add_event_cb(Alarm_clock, circle_event_cb, LV_EVENT_CLICKED, 0);

        setting= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(setting,35,0);
        lv_obj_set_size(setting,64,64);
        lv_obj_set_style_clip_corner(setting, true, 0);       
        lv_imgbtn_set_src(setting,LV_IMGBTN_STATE_RELEASED,NULL,&SETTING,NULL);
        lv_obj_add_event_cb(setting, circle_event_cb, LV_EVENT_CLICKED, 0);
				
				Watch_dial= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(Watch_dial,35,0);
        lv_obj_set_size(Watch_dial,64,64);
        lv_obj_set_style_clip_corner(Watch_dial, true, 0);       
        lv_imgbtn_set_src(Watch_dial,LV_IMGBTN_STATE_RELEASED,NULL,&dial,NULL);
        lv_obj_add_event_cb(Watch_dial, circle_event_cb, LV_EVENT_CLICKED, 0);

        About= lv_imgbtn_create(next_cont);
        lv_obj_set_style_radius(About,35,0);
        lv_obj_set_size(About,64,64);
        lv_obj_set_style_clip_corner(About, true, 0);       
        lv_imgbtn_set_src(About,LV_IMGBTN_STATE_RELEASED,NULL,&about,NULL);
        lv_obj_add_event_cb(About, circle_event_cb, LV_EVENT_CLICKED, 0);


     lv_event_send(next_cont, LV_EVENT_SCROLL, NULL);

     lv_obj_scroll_to_view(lv_obj_get_child(next_cont, 0), LV_ANIM_OFF);
		 lv_obj_add_event_cb(next_main_cont, cont_gesture_event_cb, LV_EVENT_GESTURE, 0);
		 lv_obj_add_event_cb(next_main_cont, cont_key_event_cb, LV_EVENT_KEY, 0);

}

void panel_slide_anim(lv_obj_t* obj, int32_t y) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_time(&a, 150);
    lv_anim_set_values(&a, lv_obj_get_y(obj), y);
    lv_anim_start(&a);
}


static void wallpaper_gesture_event_cb(lv_event_t* e) {
    lv_indev_t* indev = lv_event_get_indev(e);   
     if (lv_indev_get_gesture_dir(indev) == LV_DIR_LEFT&&passward_show_flag==true) {
        create_password_screen();
			 	lv_group_focus_obj(dots_container_main); 
    }
		else if (lv_indev_get_gesture_dir(indev) == LV_DIR_LEFT&&passward_show_flag==false) {
        lv_scr_load(main_cont);
				lv_group_focus_obj(main_cont); 
    }
}

static void wallpaper_key_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
		 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						if(key == LV_KEY_UP&&passward_show_flag==true) {
							create_password_screen();
						  lv_group_focus_obj(dots_container_main); 
						}	
						else if(key == LV_KEY_UP&&passward_show_flag==false) {
				      lv_scr_load(main_cont);
							lv_group_focus_obj(main_cont); 
						}
				}
}


static void main_gesture_event_cb(lv_event_t* e) {
	  lv_indev_t* indev = lv_event_get_indev(e);
    lv_obj_t* panel = lv_event_get_user_data(e);
    lv_obj_t* target = lv_event_get_target(e);
	
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_LEFT) {
      lv_scr_load_anim(next_main_cont,LV_SCR_LOAD_ANIM_FADE_ON,100,0,false);	
				lv_group_focus_obj(next_main_cont); 
    }
    else if(lv_indev_get_gesture_dir(indev) == LV_DIR_RIGHT)
    {
			    lv_scr_load(wallpaper);
				  lv_group_focus_obj(wallpaper); 
    }
     else if (lv_indev_get_gesture_dir(indev) == LV_DIR_BOTTOM) {
        panel_slide_anim(panel, 0);  // 
    } else if (lv_indev_get_gesture_dir(indev) == LV_DIR_TOP) {
        panel_slide_anim(panel, -170);  // 
    }
	}

static void main_key_event_cb(lv_event_t* e) {
		lv_event_code_t code = lv_event_get_code(e);
			 if(code == LV_EVENT_KEY) {
						uint32_t key = lv_event_get_key(e);
						if(key == LV_KEY_UP) {	
      lv_scr_load_anim(next_main_cont,LV_SCR_LOAD_ANIM_FADE_ON,100,0,false);	
						lv_group_focus_obj(next_main_cont); 
						}	
						else if(key == LV_KEY_DOWN) {
							lv_scr_load_anim(wallpaper,LV_SCR_LOAD_ANIM_OVER_TOP,20,0,false);	
							lv_group_focus_obj(wallpaper); 
						}
				}
}

static void slider_close_event_cb(lv_event_t * e) {
    lv_obj_t * Slider = lv_event_get_target(e);  
    int32_t brightness = lv_slider_get_value(Slider);  
		if(brightness>=95)
		{
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		}
}

void show_shutdown_screen() {

  	shutdown_screen = lv_obj_create(NULL);
    lv_obj_set_size(shutdown_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(shutdown_screen, lv_color_black(), 0);

    lv_obj_t * label = lv_label_create(shutdown_screen);
    lv_label_set_text(label, "Slide to power off");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);

    lv_obj_t *slider = lv_slider_create(shutdown_screen);
    lv_obj_set_size(slider, LV_PCT(80), 35);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_width(slider,15,LV_PART_KNOB);
    lv_obj_set_style_height(slider,15,LV_PART_KNOB);

    lv_slider_set_range(slider, 0, 100);
    lv_obj_set_style_radius(slider, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);

    lv_obj_set_style_bg_color(slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_white(), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_white(), LV_PART_KNOB);

    lv_obj_add_event_cb(slider, slider_close_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
		lv_scr_load(shutdown_screen);
		lv_group_add_obj(group,shutdown_screen);
		lv_group_focus_obj(shutdown_screen);
}
	
static bool bluetooth_enabled = false;
static void event_cb_ly(lv_event_t*e)
{


    lv_obj_t *btn = lv_event_get_target(e);
    bluetooth_enabled = !bluetooth_enabled;
    if(bluetooth_enabled) {
    lv_obj_set_style_text_color(lv_obj_get_child(btn,0),lv_color_hex(0xffffff),LV_STATE_DEFAULT);
    }
    else{
    lv_obj_set_style_text_color(lv_obj_get_child(btn, 0),lv_color_hex(0x595959),LV_STATE_DEFAULT);
    }
}

static void event_cb_Gj(lv_event_t*e)
{
 lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
       	 show_shutdown_screen();
    }

}

static bool SETTINGS_ENABLE = false;
static void event_cb_SZ(lv_event_t*e)
{
	lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
				SETTING_SHOW();
				lv_group_focus_obj(setting_show);
    }
}

static bool EYES_ENABLE = false;
static void event_cb_HY(lv_event_t*e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    EYES_ENABLE = !EYES_ENABLE;

    
    if(EYES_ENABLE) {
lv_obj_set_style_text_color(lv_obj_get_child(btn,0),lv_color_hex(0xffffff),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(main_cont,EYE_PROTECT_COLOR, 0);
lv_obj_set_style_bg_opa(main_cont,EYE_PROTECT_OPA,0);
    }
    else{
lv_obj_set_style_text_color(lv_obj_get_child(btn,0),lv_color_hex(0x000000),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(main_cont,lv_color_hex(0xF5F5F5), 0);
lv_obj_set_style_bg_opa(main_cont,50,0);
    }

}
void set_screen_brightness(uint8_t brightness) {
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,brightness);
}

lv_obj_t * slider;
lv_obj_t* btn_AS;
lv_obj_t*light_show;
static uint8_t last_brightness=49;
static uint8_t last_last_brightness;
static void slider_event_cb(lv_event_t * e) {
		lv_indev_t* indev = lv_event_get_indev(e);
    lv_obj_t * Slider = lv_event_get_target(e);  
    int32_t brightness = lv_slider_get_value(Slider);  
    lv_obj_t *btn = lv_event_get_target(e);
		if(brightness<6)
		{
			brightness=6;
		}		
		if(btn==slider)
		{
    set_screen_brightness((uint8_t)(brightness));	
		last_brightness=brightness;
		}
    else if(btn==btn_AS) {
     lv_scr_load(light_show);
		 set_screen_brightness(99);
    }
    else if(lv_indev_get_gesture_dir(indev) ==LV_DIR_RIGHT&&btn==light_show){
		lv_scr_load(main_cont);
    set_screen_brightness((uint8_t)(last_brightness));
    }
}

static bool BATTERY_EMPTY_ENABLE = false;
 static void event_cb_SD(lv_event_t*e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    BATTERY_EMPTY_ENABLE = !BATTERY_EMPTY_ENABLE;

    if(BATTERY_EMPTY_ENABLE) {
    lv_obj_set_style_text_color(lv_obj_get_child(btn,0),lv_color_hex(0x75FA61),LV_STATE_DEFAULT);
    last_last_brightness=last_brightness;
	  set_screen_brightness(10);
		last_brightness=10;
		lv_obj_add_state(slider, LV_STATE_DISABLED);
    }
    else{
   lv_obj_set_style_text_color(lv_obj_get_child(btn,0),lv_color_hex(0x595959),LV_STATE_DEFAULT);
	 set_screen_brightness((uint8_t)(last_last_brightness));
	 last_brightness=last_last_brightness;
	 lv_obj_clear_state(slider, LV_STATE_DISABLED);
    }
}

 void LIGHT(void)
{
	 light_show=lv_obj_create(NULL);
	 lv_obj_set_style_bg_color(light_show,lv_color_hex(0xffffff),0);
	 lv_obj_add_event_cb(light_show, slider_event_cb, LV_EVENT_GESTURE, 0);
}
 
static void create_lwatch_animation(lv_obj_t *parent);
static void bounce2_ready_cb(lv_anim_t *a);
static void char_anim_exec_cb(void *var, int32_t v);

static lv_obj_t* letter_labels[7]; 
static const char* lwatch_text = "L_WATCH";

static void welcome_anim_ready_cb(lv_anim_t *a) {
    create_lwatch_animation(lv_obj_get_parent((lv_obj_t *)a->var));
}

static int completed_letters = 0;
lv_obj_t*anim;

static void color_fade_anim_cb(void *var, int32_t v) {
    lv_color_t target_color = lv_color_hex(0x2E78E3); 
    lv_color_t new_color = lv_color_mix(target_color, lv_color_white(), v);
    
    for(int i = 0; i < strlen(lwatch_text); i++) {
        lv_obj_set_style_text_color(letter_labels[i], new_color, 0);
    }
}

static void delay_anim_complete_cb(void *var, int32_t v) {
	
	lv_scr_load(wallpaper);
  lv_obj_del(anim);
	set_screen_brightness(49);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void delay_anim_cb(void *var, int32_t v) {
	
	      lv_anim_t color_anim;
        lv_anim_init(&color_anim);
        lv_anim_set_exec_cb(&color_anim, color_fade_anim_cb);
        lv_anim_set_time(&color_anim, 2000); 
        lv_anim_set_values(&color_anim, 0, 255);
        lv_anim_start(&color_anim);
	
			  lv_anim_t final_delay;
				lv_anim_init(&final_delay);
				lv_anim_set_exec_cb(&final_delay, delay_anim_complete_cb);
				lv_anim_set_time(&final_delay, 5000);
				lv_anim_set_values(&final_delay, 0, 1);
				lv_anim_start(&final_delay);
}
static void bounce2_ready_cb(lv_anim_t *a) {
	   completed_letters++;
    
    if(completed_letters == strlen(lwatch_text)) {
        lv_anim_t delay_anim;
        lv_anim_init(&delay_anim);
        lv_anim_set_exec_cb(&delay_anim, delay_anim_cb);
        lv_anim_set_time(&delay_anim, 700);
        lv_anim_set_values(&delay_anim, 0, 1);
        lv_anim_start(&delay_anim);
    }
}

static void bounce1_ready_cb(lv_anim_t *a) {
    lv_obj_t *label = (lv_obj_t *)a->var;
    lv_obj_t *parent = lv_obj_get_parent(label);
    int16_t center_y = lv_obj_get_height(parent)/2 - lv_obj_get_height(label)/2+25;
    
    lv_anim_t bounce2;
    lv_anim_init(&bounce2);
    lv_anim_set_var(&bounce2, label);
    lv_anim_set_exec_cb(&bounce2, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&bounce2, center_y + 20, center_y);
    lv_anim_set_time(&bounce2, 150);
    lv_anim_set_path_cb(&bounce2, lv_anim_path_bounce);
	  lv_anim_set_ready_cb(&bounce2, bounce2_ready_cb);
    lv_anim_start(&bounce2);
}

static void fall_anim_ready_cb(lv_anim_t *a) {
    lv_obj_t *label = (lv_obj_t *)a->var;
    lv_obj_t *parent = lv_obj_get_parent(label);
    int16_t center_y = lv_obj_get_height(parent)/2 - lv_obj_get_height(label)/2+25;
    
    lv_anim_t bounce1;
    lv_anim_init(&bounce1);
    lv_anim_set_var(&bounce1, label);
    lv_anim_set_exec_cb(&bounce1, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_values(&bounce1, center_y + 25, center_y);
    lv_anim_set_time(&bounce1, 200);
    lv_anim_set_path_cb(&bounce1, lv_anim_path_bounce);
    lv_anim_set_ready_cb(&bounce1, bounce1_ready_cb);
    lv_anim_start(&bounce1);
}
LV_FONT_DECLARE(Welcome);
LV_FONT_DECLARE(LWatch);

void create_welcome_animation(lv_obj_t *parent) {
    lv_obj_t *welcome_label = lv_label_create(parent);
    lv_label_set_text(welcome_label, "WELCOME!");
   	lv_obj_set_style_text_color(welcome_label,lv_color_hex(0xffffff),0); 
	  lv_obj_set_style_text_font(welcome_label,&Welcome,0);
	  lv_obj_align(welcome_label, LV_ALIGN_CENTER, 5, -30); // 上移一点给L_WATCH留空间

    lv_anim_t welcome_anim;
    lv_anim_init(&welcome_anim);
    lv_anim_set_var(&welcome_anim, welcome_label);
    lv_anim_set_exec_cb(&welcome_anim, (lv_anim_exec_xcb_t)lv_obj_set_style_opa);
    lv_anim_set_values(&welcome_anim, LV_OPA_COVER,LV_OPA_COVER); // 保持不透明
    lv_anim_set_time(&welcome_anim, 1000);
    lv_anim_set_delay(&welcome_anim, 600);
    lv_anim_set_ready_cb(&welcome_anim, welcome_anim_ready_cb);
    lv_anim_start(&welcome_anim);
}

void create_lwatch_animation(lv_obj_t *parent) {
	   completed_letters = 0;  
    int letter_count = strlen(lwatch_text);
    int total_width = 0;
    int letter_widths[7] = {0}; 
    
    
    for(int i = 0; i < letter_count; i++) {
        lv_obj_t *temp_label = lv_label_create(parent);
        lv_label_set_text_fmt(temp_label, "%c", lwatch_text[i]);
        lv_obj_set_style_text_font(temp_label, &LWatch, 0);
			 	lv_obj_set_style_text_color(temp_label,lv_color_hex(0xffffff),0); 
        lv_obj_update_layout(temp_label); 
        letter_widths[i] = lv_obj_get_width(temp_label);
        total_width += letter_widths[i];
        lv_obj_del(temp_label);
    }
    
    int start_x = (lv_obj_get_width(parent) - total_width) / 2;
    int current_x = start_x;
    
    for(int i = 0; i < letter_count; i++) {
        letter_labels[i] = lv_label_create(parent);
        lv_label_set_text_fmt(letter_labels[i], "%c", lwatch_text[i]);
        lv_obj_set_style_text_font(letter_labels[i], &LWatch, 0);
			 	lv_obj_set_style_text_color(letter_labels[i],lv_color_hex(0xffffff),0); 
        
        lv_obj_set_pos(letter_labels[i], current_x, -lv_obj_get_height(letter_labels[i]));
        current_x += letter_widths[i]+2;
        lv_anim_t fall_anim;
        lv_anim_init(&fall_anim);
        lv_anim_set_var(&fall_anim, letter_labels[i]);
        lv_anim_set_exec_cb(&fall_anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_values(&fall_anim, -lv_obj_get_height(letter_labels[i]), 
                          lv_obj_get_height(parent)/2 - lv_obj_get_height(letter_labels[i])/2 + 70);
        lv_anim_set_time(&fall_anim, 300);
        lv_anim_set_delay(&fall_anim, i * 100); 
        lv_anim_set_path_cb(&fall_anim, lv_anim_path_ease_out);
        lv_anim_set_ready_cb(&fall_anim, fall_anim_ready_cb);
        lv_anim_start(&fall_anim);
    }
}

void app_main() {
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	set_screen_brightness(99);
	anim=lv_obj_create(NULL);
	lv_obj_set_style_bg_color(anim,lv_color_hex(0x000000),0);
  create_welcome_animation(anim);
	lv_scr_load(anim);
}
typedef struct {
    lv_img_header_t header; 
    uint32_t data_size;    
     uint8_t * data;  
} img_dsc_t;

typedef struct {
    lv_img_header_t header;  
    uint32_t img_addr;       
} w25q64_img_ctx_t;

static lv_res_t w25q64_img_info_cb(lv_img_decoder_t *decoder,const void *src,lv_img_header_t*header)
{
	if(src!=&flash_img)return LV_RES_INV;
	  if(flash_img.data == 0 || flash_img.data == NULL){
        return LV_RES_INV;
    }
   lv_img_header_t head;
	 W25Q64_ReadData((uint8_t*)&head,(uint32_t)flash_img.data,sizeof(lv_img_header_t));
	 header->w= head.w;
	 header->h= head.h;
	 header->cf= head.cf;
	return LV_RES_OK;
}
static lv_res_t w25q64_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc) {
	    if(dsc->src != &flash_img) return LV_RES_INV;
	 if(flash_img.data == NULL || flash_img.data == 0){
        return LV_RES_INV;
    }
    w25q64_img_ctx_t *ctx = lv_mem_alloc(sizeof(w25q64_img_ctx_t));
    if(!ctx) return LV_RES_INV;
    ctx->img_addr = (uint32_t)flash_img.data; 

    W25Q64_ReadData((uint8_t*)&ctx->header, ctx->img_addr, sizeof(lv_img_header_t));
    dsc->user_data = ctx;
    dsc->img_data = NULL; 
    return LV_RES_OK;
}
static lv_res_t w25q64_decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc,
                                         lv_coord_t x, lv_coord_t y, lv_coord_t len, uint8_t *buf) {
																					 																																					 
		w25q64_img_ctx_t *ctx = (w25q64_img_ctx_t *)dsc->user_data;
    if (!ctx) return LV_RES_INV;

    uint32_t pixel_size = 2; 
    uint32_t offset = sizeof(lv_img_header_t) + (y * ctx->header.w + x) * pixel_size;
    W25Q64_ReadData(buf, ctx->img_addr + offset, len * pixel_size);

    return LV_RES_OK;
}

static void w25q64_img_close(lv_img_decoder_t*decoder,lv_img_decoder_dsc_t*dsc)
{
	 if (dsc->user_data) {
        lv_mem_free(dsc->user_data);
        dsc->user_data = NULL;
    }
}

void lvgl_img_decoder_init(void) {
    lv_img_decoder_t *decoder = lv_img_decoder_create();
  	lv_img_decoder_set_info_cb(decoder,w25q64_img_info_cb);
    lv_img_decoder_set_open_cb(decoder, w25q64_decoder_open);
    lv_img_decoder_set_read_line_cb(decoder, w25q64_decoder_read_line);
	  lv_img_decoder_set_close_cb(decoder,w25q64_img_close);
}
 
void next_panel(void)
{	  
   flash_img.data=0x001000;
	 wallpaper=lv_obj_create(NULL);
	 img = lv_img_create(wallpaper);
	 lv_obj_set_size(img, 240,284);  
	 lv_img_cache_invalidate_src(&flash_img);	
	 lv_img_set_src(img,&flash_img);
	 lv_obj_align(img, LV_ALIGN_CENTER, 0,0);
   lv_obj_add_event_cb(wallpaper, wallpaper_gesture_event_cb,LV_EVENT_GESTURE, 0);
	 lv_obj_add_event_cb(wallpaper, wallpaper_key_event_cb,LV_EVENT_KEY, 0);
}

void line(void)
{
       static lv_point_t line_points[] = {
    {0, 60},    
    {240, 60} 
};
    lv_obj_t*line_top_1=lv_line_create(main_cont);
    lv_line_set_points(line_top_1,line_points,2);

     static lv_point_t line_points1[] = {
    {160, 0},    
    {160, 60}  
};
    lv_obj_t*line_top_2=lv_line_create(main_cont);
    lv_line_set_points(line_top_2,line_points1,2);

      static lv_point_t line_points2[] = {
    {0, 220},    
    {240, 220}  
};
    lv_obj_t*line_bot_1=lv_line_create(main_cont);
    lv_line_set_points(line_bot_1,line_points2,2);

   static lv_point_t line_points3[] = {
    {70, 220},    
    {70, 284}  
};
    lv_obj_t*line_bot_2=lv_line_create(main_cont);
    lv_line_set_points(line_bot_2,line_points3,2);

   static lv_point_t line_points4[] = {
    {125, 220},   
    {125, 284}  
};
    lv_obj_t*line_bot_3=lv_line_create(main_cont);
    lv_line_set_points(line_bot_3,line_points4,2);
      static lv_point_t line_points6[] = {
    {180, 220},    
    {180, 284}  
};
    lv_obj_t*line_top_15=lv_line_create(main_cont);
    lv_line_set_points(line_top_15,line_points6,2);
}

void motor_set_state(bool on)
{
    if (on) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    } else {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }
}

static void vibration_timer_cb(lv_timer_t *timer)
{
    vibration_tick_count++;

    if (vibration_tick_count >= 60) { // 500ms * 60 = 30s
        motor_set_state(false);
        lv_timer_del(timer);
        vibration_timer = NULL;
        vibration_tick_count = 0;
        return;
    }

    vibration_on = !vibration_on;
    motor_set_state(vibration_on);
}

static void update_time(lv_timer_t *timer) {
		Read_PCF8563time();
 		Year = Timelist[0];
		Month = Timelist[1];
		Day = Timelist[2];
		Hour = Timelist[3];
		Min = Timelist[4];
		Sec = Timelist[5];
		Week = Timelist[6];	
    char time_str[6];char year_str[6],month_str[6],day_str[6],sec_str[4];
    snprintf(time_str, sizeof(time_str), "%02d:%02d",
             Hour,Min);
		snprintf(sec_str, sizeof(sec_str), "%02d",
             Sec);	
    snprintf(year_str, sizeof(year_str), "%04d",
             Year+2000);
		snprintf(month_str, sizeof(month_str), "%02d",
             Month);
		snprintf(day_str, sizeof(day_str), "%02d",
             Day);	

		lv_label_set_text(time_appear_label_sec, sec_str);
    lv_label_set_text(time_label, time_str);
		lv_label_set_text(time_appear_label, time_str);


	  lv_label_set_text(label2, year_str);
    lv_label_set_text(label4, month_str);
	  lv_label_set_text(label6, day_str);  
		lv_label_set_text(time_appear, day_str);
		switch (Week) {
			case 1:
				  lv_label_set_text(week_appear, "周一");
			break;
			case 2:
				  lv_label_set_text(week_appear, "周二");
			break;
			case 3:
				  lv_label_set_text(week_appear, "周三");
			break;
			case 4:
				  lv_label_set_text(week_appear, "周四");
			break;
			case 5:
				  lv_label_set_text(week_appear, "周五");
			break;
			case 6:
				  lv_label_set_text(week_appear, "周六");
			break;
			case 7:
				  lv_label_set_text(week_appear, "周日");
			break;
     }
		    for (int i = 0; i < alarm_count; ++i) {
        if (alarms[i].enabled &&
            Hour== alarms[i].hour &&
            Min== alarms[i].minute &&
            Sec== 0) {
							if (!vibration_timer) {
            vibration_tick_count = 0;
            vibration_on = false;
            vibration_timer = lv_timer_create(vibration_timer_cb, 500, NULL);
            }
        }
    }
		int32_t temp_val;
    if(xQueueReceive(temperature_only_queue, &temp_val, 0) == pdTRUE){
				char buf[4];
				sprintf(buf, "%d°C", temp_val/100);
				lv_label_set_text(temperature_label, buf);  
     }		
		uint16_t received_steps;
	  if (xQueueReceive(STEP_queue, &received_steps, 0) == pdTRUE) {
        char step_buf[32];
        sprintf(step_buf, "%dStep", received_steps);
				lv_label_set_text(step_label, step_buf);  
     }
		 
}


LV_IMG_DECLARE(TKR1);
LV_IMG_DECLARE(TKR2);
LV_IMG_DECLARE(TKR3);
LV_IMG_DECLARE(TKR4);


LV_IMG_DECLARE(TKR11);
LV_IMG_DECLARE(TKR12);
LV_IMG_DECLARE(TKR13);
LV_IMG_DECLARE(TKR14);
LV_IMG_DECLARE(TKR15);
LV_IMG_DECLARE(TKR15);


void GIF(void)
{
	static const lv_img_dsc_t * anim_imgs[3] = {
			&TKR2,
			&TKR3,
			&TKR4,
	};
	lv_obj_t * animimg = lv_animimg_create(main_cont);
	lv_obj_remove_style_all(animimg);
  lv_obj_align(animimg,LV_ALIGN_CENTER,-70,30);
	lv_animimg_set_src(animimg,(lv_img_dsc_t**)anim_imgs,3);
	lv_animimg_set_duration(animimg, 290);
	lv_animimg_set_repeat_count(animimg, LV_ANIM_REPEAT_INFINITE);
	lv_animimg_start(animimg);
}

LV_IMG_DECLARE(rocket);
LV_IMG_DECLARE(temperature);
LV_IMG_DECLARE(step);

//LV_FONT_DECLARE(Time);
LV_FONT_DECLARE(lv_font_30);
LV_FONT_DECLARE(Italic);
//LV_FONT_DECLARE(Dyh);
LV_FONT_DECLARE(playist);
//壁纸的时间
void RTC_show_time(void)
{
  time_label = lv_label_create(img);
  lv_obj_set_style_text_font(time_label, &Time,0); // 
  lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF),0); // 
  lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 88);
		
	time_appear = lv_label_create(img);
	lv_obj_set_style_text_color(time_appear,lv_color_hex(0xFFFFFF),0);
	lv_obj_align(time_appear,LV_ALIGN_CENTER,-20,48);
  lv_obj_set_style_text_font(time_appear, &Dyh, LV_PART_MAIN);
	
	week_appear = lv_label_create(img);
	lv_obj_set_style_text_color(week_appear,lv_color_hex(0xFFFFFF),0);
	lv_obj_align(week_appear,LV_ALIGN_CENTER,25,48);
  lv_obj_set_style_text_font(week_appear, &Dyh, LV_PART_MAIN);

  lv_timer_create(update_time, 250, NULL);
	update_time(NULL);
}
LV_FONT_DECLARE(TIMEAppear);
LV_FONT_DECLARE(TIME_Appear_sec);

lv_obj_t* battery_level;
lv_obj_t*DiangLiang_label;
lv_obj_t* DianLiang;
lv_obj_t*CD;
void main_panel(void)
{
  main_cont=lv_obj_create(NULL);
  lv_obj_clear_flag(main_cont,LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(main_cont, lv_color_hex(0xF0F0F0), 0);
	
	
	  lv_obj_t*Rocket=lv_img_create(main_cont);
    lv_obj_set_size(Rocket,32,32);
    lv_obj_align(Rocket,LV_ALIGN_CENTER,-95,-112);
    lv_img_set_src(Rocket,&rocket);
    lv_obj_set_style_bg_color(Rocket,lv_color_hex(0xffffff),0);

    lv_obj_t*temp=lv_img_create(main_cont);
    lv_obj_set_size(temp,32,32);
    lv_obj_align(temp,LV_ALIGN_CENTER,70,110);
    lv_img_set_src(temp,&temperature);
    lv_obj_set_style_bg_color(temp,lv_color_hex(0xffffff),0);

  	lv_obj_t*Step=lv_img_create(main_cont);
    lv_obj_set_size(Step,32,32);
    lv_obj_align(Step,LV_ALIGN_CENTER,20,50);
    lv_img_set_src(Step,&step);
    lv_obj_set_style_bg_color(Step,lv_color_hex(0xffffff),0);
		
   DianLiang=lv_label_create(main_cont);
   lv_label_set_text(DianLiang, LV_SYMBOL_BATTERY_EMPTY);
   lv_obj_set_style_text_font(DianLiang, &lv_font_montserrat_20, LV_PART_MAIN);
   lv_obj_align(DianLiang,LV_ALIGN_CENTER,70,-120);

  battery_level = lv_bar_create(DianLiang);
	lv_obj_set_size(battery_level, 20 , 8); 
	lv_obj_set_style_radius(battery_level, 0, LV_PART_MAIN);
	lv_obj_set_style_radius(battery_level, 0, LV_PART_INDICATOR); 
	lv_obj_set_style_bg_opa(battery_level, 0, LV_PART_MAIN); 
	lv_obj_center(battery_level);
	lv_bar_set_range(battery_level, 0, 100);
	lv_bar_set_value(battery_level, 100, LV_ANIM_ON); 
	lv_obj_set_style_bg_color(battery_level, lv_color_hex(0x00FF00),LV_PART_INDICATOR);

  DiangLiang_label=lv_label_create(main_cont);
	lv_obj_set_style_text_color(DiangLiang_label,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(DiangLiang_label, &lv_font_montserrat_20, 0);
  lv_obj_align(DiangLiang_label,LV_ALIGN_CENTER,75,-100);

	CD=lv_label_create(DianLiang);
	lv_label_set_text(CD, LV_SYMBOL_CHARGE);
	lv_obj_set_style_text_font(CD, &lv_font_montserrat_8, LV_PART_MAIN);
	lv_obj_set_align(CD,LV_ALIGN_CENTER);
	lv_obj_set_style_bg_color(battery_level, lv_color_hex(0x2E77E0),LV_PART_INDICATOR);
  lv_obj_add_flag(CD, LV_OBJ_FLAG_HIDDEN);

  time_appear_label = lv_label_create(main_cont);
  lv_obj_set_style_text_color(time_appear_label,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(time_appear_label, &TIMEAppear, 0);
  lv_obj_align(time_appear_label,LV_ALIGN_CENTER,0,-35);

	time_appear_label_sec = lv_label_create(main_cont);
  lv_obj_set_style_text_color(time_appear_label_sec,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(time_appear_label_sec, &TIME_Appear_sec, 0);
  lv_obj_align(time_appear_label_sec,LV_ALIGN_CENTER,74,-27);

  lv_obj_t *label0 = lv_label_create(main_cont);
  lv_label_set_text(label0, "No pain No gain");
  lv_obj_align(label0,LV_ALIGN_CENTER,-20,-127);
  lv_obj_set_style_text_font(label0, &Italic, LV_PART_MAIN);
	lv_obj_t *label1 = lv_label_create(main_cont);
  lv_label_set_text(label1, "Self_Discipline");
  lv_obj_align(label1,LV_ALIGN_CENTER,-20,-107);
  lv_obj_set_style_text_font(label1, &playist, LV_PART_MAIN);

  label2 = lv_label_create(main_cont);
  lv_obj_set_style_text_color(label2,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label2, &lv_font_montserrat_16, 0);
  lv_obj_align(label2,LV_ALIGN_BOTTOM_LEFT,10,-25);

  lv_obj_t * label3 = lv_label_create(main_cont);
  lv_label_set_text(label3, "年");
  lv_obj_set_style_text_color(label3,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label3, &lv_font_30, 0);
  lv_obj_align(label3,LV_ALIGN_BOTTOM_LEFT,50,-25);
	
	label4 = lv_label_create(main_cont);
  lv_obj_set_style_text_color(label4,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label4, &lv_font_montserrat_16, 0);
  lv_obj_align(label4,LV_ALIGN_BOTTOM_LEFT,80,-24);
	
	lv_obj_t * label5 = lv_label_create(main_cont);
  lv_label_set_text(label5, "月");
  lv_obj_set_style_text_color(label5,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label5, &lv_font_30, 0);
  lv_obj_align(label5,LV_ALIGN_BOTTOM_LEFT,102,-25);
	
	label6 = lv_label_create(main_cont);
  lv_obj_set_style_text_color(label6,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label6, &lv_font_montserrat_16, 0);
  lv_obj_align(label6,LV_ALIGN_BOTTOM_LEFT,135,-24);
	
	lv_obj_t * label7 = lv_label_create(main_cont);
  lv_label_set_text(label7, "日");
  lv_obj_set_style_text_color(label7,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(label7, &lv_font_30, 0);
  lv_obj_align(label7,LV_ALIGN_BOTTOM_LEFT,155,-25);
	
	temperature_label= lv_label_create(main_cont);
  lv_label_set_text(temperature_label, "");
  lv_obj_set_style_text_color(temperature_label,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(temperature_label, &lv_font_montserrat_16, 0);
  lv_obj_align(temperature_label,LV_ALIGN_BOTTOM_LEFT,198,-25);
	
	step_label= lv_label_create(main_cont);
  lv_label_set_text(step_label, "0Step");
  lv_obj_set_style_text_color(step_label,lv_color_hex(0x000000),0);
  lv_obj_set_style_text_font(step_label, &lv_font_montserrat_16, 0);
  lv_obj_align(step_label,LV_ALIGN_CENTER,74,50);
		
}
static volatile bool is_display_awake = false;
static volatile bool wake_ing=false;
static uint8_t SLEEP_FLAG=0;

void sleep_timer_callback(lv_timer_t * timer)
{			
	    lv_timer_pause(sleep_timer);
			SLEEP_FLAG=1;
			xQueueSend(SLEEP_queue,&SLEEP_FLAG,0);
}
extern uint8_t rx_buffer[1]; 
 void reset_screen_timeout_key(void)
{

    if(current_state == SCREEN_ACTIVE || current_state == SCREEN_DIM)
    {
        set_screen_brightness(0);			
			  lv_scr_load(wallpaper);
        lv_group_focus_obj(wallpaper);


        if(dim_timer)  lv_timer_pause(dim_timer);
        if(off_timer)  lv_timer_pause(off_timer);

				if(shutdown_screen)    { lv_obj_del_delayed(shutdown_screen,20); shutdown_screen=NULL; }
				if(dots_container_main){ if(passward_set_flag==true&&passward_show_flag==true){passward_set_flag=false;passward_show_flag=false;} lv_obj_del_delayed(dots_container_main,10); dots_container_main=NULL;}
        if(hand_cont)          { lv_obj_del_delayed(hand_cont,10); hand_cont=NULL; }
        if(password_set_cont)  { lv_obj_del_delayed(password_set_cont,10); password_set_cont=NULL; }
        if(display_time_show)  { lv_obj_del_delayed(display_time_show,10); display_time_show=NULL; }
        if(game_2048_board_cont) { lv_obj_del_delayed(game_2048_board_cont,10); game_2048_board_cont=NULL; }
        if(tic_tac_scr)        { lv_obj_del_delayed(tic_tac_scr,10); tic_tac_scr=NULL; }
        if(bluetooth_screen)   { lv_obj_del_delayed(bluetooth_screen,10); bluetooth_screen=NULL; }
        if(color_popup)        { free_preset_colors(); lv_obj_del_delayed(color_popup,10); color_popup=NULL; }
        if(time_label_free)    { lv_obj_del_delayed(time_label_free,10); time_label_free=NULL; }

        if(compass_scr)        { lv_timer_del(compass_timer); compass_timer=NULL; lv_obj_del_delayed(compass_scr,10); compass_scr=NULL; }
        if(time_data_set_cont) { lv_obj_del_delayed(time_data_set_cont,10); time_data_set_cont=NULL; }
        if(calendaR)           { lv_obj_del_delayed(calendaR,10); calendaR=NULL; }
        if(calculator_show)    { lv_obj_del_delayed(calculator_show,10); calculator_show=NULL; }
        if(wallpaperer_show)   { lv_obj_del_delayed(wallpaperer_show,10); wallpaperer_show=NULL; }

        if(Stop_watch_show)    { is_running=false; lv_obj_del_delayed(Stop_watch_show,10); Stop_watch_show=NULL; }
        if(setting_show)       { lv_obj_del_delayed(setting_show,10); setting_show=NULL; }
        if(scr)                { lv_timer_del(sensor_timer); sensor_timer=NULL; lv_obj_del_delayed(scr,10); scr=NULL; }
        if(clock_scr)          { lv_timer_del(update_timer); update_timer=NULL; lv_obj_del_delayed(clock_scr,10); clock_scr=NULL; }
        if(triangle_btn)       { lv_obj_del_delayed(triangle_btn,10); triangle_btn=NULL; }
        if(about_show)         { lv_obj_del_delayed(about_show,10); about_show=NULL; }

        current_state = SCREEN_OFF;
        if(sleep_timer) { lv_timer_del(sleep_timer); sleep_timer=NULL; }
        sleep_timer = lv_timer_create(sleep_timer_callback, 2000, NULL);
    }
		  else if(current_state == SCREEN_OFF)
			{
				if(wake_ing)
						{	
				 __HAL_RCC_TIM1_CLK_ENABLE();  
				 __HAL_RCC_USART1_CLK_ENABLE(); 
				 __HAL_RCC_ADC1_CLK_ENABLE();   
							
				 MX_USART1_UART_Init();
				 HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
				 MX_ADC1_Init();
				 MX_I2C1_Init(); 
				 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

					MySPI_W_SS(0);
					vTaskDelay(pdMS_TO_TICKS(1)); 	
					MySPI_SwapByte(0XAB);
					vTaskDelay(pdMS_TO_TICKS(1)); 	
					MySPI_W_SS(1);							
					vTaskDelay(pdMS_TO_TICKS(50)); 	

					MPU6050_WakeUp();
					BME280_wake();
					HMC_wake();
					enter_cst816t_reset();

					LCD_CS_Clr();
					LCD_WR_REG(0x11); //Sleep out 	
					LCD_CS_Set();
					vTaskDelay(pdMS_TO_TICKS(150)); 	

					if (eTaskGetState(LV_DEMOTask_Handler) == eSuspended) {
									vTaskResume(LV_DEMOTask_Handler);
									}			
					if (eTaskGetState(regular_time_Task_Handler) == eSuspended) {
									vTaskResume(regular_time_Task_Handler);
									}		
					if (eTaskGetState(RTC_Task_Handler) == eSuspended) {			
									vTaskResume(RTC_Task_Handler);		
									}		
					if (eTaskGetState(SENSOR_Task_Handler) == eSuspended) {		
									vTaskResume(SENSOR_Task_Handler);		
									}		
					if (eTaskGetState(SCREEN_WAKE_Task_Handler) == eSuspended) {		
									vTaskResume(SCREEN_WAKE_Task_Handler);		
									}		
						wake_ing=false;
				}			
									
			set_screen_brightness(last_brightness);
				current_state=SCREEN_ACTIVE; 	
				if(dim_timer)  { lv_timer_reset(dim_timer); lv_timer_resume(dim_timer); }
        if(anim_timer) { lv_timer_reset(anim_timer); lv_timer_resume(anim_timer); }
        if(off_timer)   lv_timer_pause(off_timer);
        if(sleep_timer) lv_timer_pause(sleep_timer);
			}
}

void reset_screen_timeout(void) {
    if(current_state == SCREEN_DIM) {      
        set_screen_brightness(last_brightness);
        current_state = SCREEN_ACTIVE;
    } else if(current_state == SCREEN_OFF) {
        set_screen_brightness(last_brightness);
        current_state = SCREEN_ACTIVE;
    }  
		if(dim_timer)
		{
    lv_timer_reset(dim_timer);
		lv_timer_resume(dim_timer);
		}
		if(off_timer)
		{
    lv_timer_pause(off_timer);
		}
		if(sleep_timer)
		{
	  lv_timer_pause(sleep_timer);
		}
}

void off_timer_callback(lv_timer_t * timer) {
	  lv_scr_load(wallpaper);
	  lv_group_focus_obj(wallpaper); 	
		set_screen_brightness(0); 
		    if(shutdown_screen)    { lv_obj_del_delayed(shutdown_screen,20); shutdown_screen=NULL; }
				if(dots_container_main){ if(passward_set_flag==true&&passward_show_flag==true){passward_set_flag=false;passward_show_flag=false;} lv_obj_del_delayed(dots_container_main,10); dots_container_main=NULL;}
        if(hand_cont)          { lv_obj_del_delayed(hand_cont,10); hand_cont=NULL; }
        if(password_set_cont)  { lv_obj_del_delayed(password_set_cont,10); password_set_cont=NULL; }
        if(display_time_show)  { lv_obj_del_delayed(display_time_show,10); display_time_show=NULL; }
        if(game_2048_board_cont) { lv_obj_del_delayed(game_2048_board_cont,10); game_2048_board_cont=NULL; }
        if(tic_tac_scr)        { lv_obj_del_delayed(tic_tac_scr,10); tic_tac_scr=NULL; }
        if(bluetooth_screen)   { lv_obj_del_delayed(bluetooth_screen,10); bluetooth_screen=NULL; }
        if(color_popup)        { free_preset_colors(); lv_obj_del_delayed(color_popup,10); color_popup=NULL; }
        if(time_label_free)    { lv_obj_del_delayed(time_label_free,10); time_label_free=NULL; }
        

        if(compass_scr)        { lv_timer_del(compass_timer); compass_timer=NULL; lv_obj_del_delayed(compass_scr,10); compass_scr=NULL; }
        if(time_data_set_cont) { lv_obj_del_delayed(time_data_set_cont,10); time_data_set_cont=NULL; }
        if(calendaR)           { lv_obj_del_delayed(calendaR,10); calendaR=NULL; }
        if(calculator_show)    { lv_obj_del_delayed(calculator_show,10); calculator_show=NULL; }
        if(wallpaperer_show)   { lv_obj_del_delayed(wallpaperer_show,10); wallpaperer_show=NULL; }

        if(Stop_watch_show)    { is_running=false; lv_obj_del_delayed(Stop_watch_show,10); Stop_watch_show=NULL; }
        if(setting_show)       { lv_obj_del_delayed(setting_show,10); setting_show=NULL; }
        if(scr)                { lv_timer_del(sensor_timer); sensor_timer=NULL; lv_obj_del_delayed(scr,10); scr=NULL; }
				if(clock_scr)          { lv_timer_del(update_timer); update_timer=NULL; lv_obj_del_delayed(clock_scr,10); clock_scr=NULL; }
        if(triangle_btn)       { lv_obj_del_delayed(triangle_btn,10); triangle_btn=NULL; }        
				if(about_show)         { lv_obj_del_delayed(about_show,10); about_show=NULL; }

       current_state = SCREEN_OFF;

				if(sleep_timer != NULL)
					{	
							lv_timer_del(sleep_timer);
							sleep_timer = NULL;
					}
				sleep_timer = lv_timer_create(sleep_timer_callback, 2000, NULL);
				 lv_timer_pause(dim_timer);	
				 lv_timer_pause(off_timer);			
}
void dim_timer_callback(lv_timer_t * timer) {
    current_state = SCREEN_DIM;
    set_screen_brightness(10);
	
	if(off_timer != NULL)
					{	
							lv_timer_del(off_timer);
							off_timer = NULL;
					}
		 off_timer = lv_timer_create(off_timer_callback, 5000, NULL);
		 lv_timer_pause(dim_timer);
		 if(sleep_timer)
		 {
		 lv_timer_pause(sleep_timer);
		 }
}
void init_screen_timeout(void) {
    dim_timer = lv_timer_create(dim_timer_callback, last_seconds*1000, NULL);
	  off_timer=NULL;
		sleep_timer=NULL;

}
void focus(void)
{
		lv_group_add_obj(group, main_cont);
	  lv_group_add_obj(group, wallpaper);
	  lv_group_add_obj(group, next_main_cont);

    lv_group_focus_obj(wallpaper); 
}

static uint32_t button_press_time = 0;
static bool button_pressed = false;


void battery_update_timer(lv_timer_t *timer) {
      uint8_t precent;
			  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==0)
				{
			lv_obj_clear_flag(CD, LV_OBJ_FLAG_HIDDEN); 
			lv_obj_set_style_bg_color(battery_level, lv_color_hex(0x2E77E0),LV_PART_INDICATOR);
				}
				else
				{
		  lv_obj_add_flag(CD, LV_OBJ_FLAG_HIDDEN);
			lv_obj_set_style_bg_color(battery_level, lv_color_hex(0x00FF00),LV_PART_INDICATOR);
				}
				
    if (xQueueReceive(DianLiang_queue, &precent, 0) == pdTRUE) { // 非阻塞读取
				lv_bar_set_value(battery_level, precent, LV_ANIM_ON);
    	  lv_label_set_text_fmt(DiangLiang_label, "%d%%",precent);
			}

}
void init_ui() {
   lv_timer_create(battery_update_timer,2000, NULL);
}



#define TECH_BLUE       lv_color_hex(0x00a8ff)
#define TECH_CYAN       lv_color_hex(0x00fff0)
#define TECH_PURPLE     lv_color_hex(0x9c27b0)
#define DARK_BE         lv_color_hex(0x0a0a1a)
#define PARTICLE_COLOR  lv_color_hex(0x00a8ff)
#define PARTICLE_COUNT  30

static lv_obj_t * charging_screen;
static lv_obj_t * arc;
static lv_obj_t * percent_label;
static lv_obj_t * status_label;
static lv_obj_t * charge_icon;

static lv_anim_t anim_icon;
static lv_anim_t anim_particles;

typedef struct {
    lv_obj_t * obj;
    float speed;
    float x, y;
    float start_x;
} particle_t;

static particle_t particles[PARTICLE_COUNT];

// 圆环动画回调（保持不变）
static void arc_anim_cb(void * var, int32_t v) {
    lv_arc_set_value(var, v);
    lv_label_set_text_fmt(percent_label, "%d%%", v);
    if(v < 20)
        lv_obj_set_style_arc_color(arc, lv_color_hex(0xff0000), LV_PART_INDICATOR);
    else if(v < 50)
        lv_obj_set_style_arc_color(arc, lv_color_hex(0xffa500), LV_PART_INDICATOR);
    else
        lv_obj_set_style_arc_color(arc, TECH_CYAN, LV_PART_INDICATOR);
}

static void icon_breath_anim_cb(void * var, int32_t v) {
    lv_obj_t * icon = (lv_obj_t *)var;
    int32_t phase = v % 1024;
    int32_t alpha = (phase < 512) ? phase : 1023 - phase;
    alpha = 80 + (alpha / 4);
    lv_color_t color = lv_color_mix(TECH_BLUE, TECH_PURPLE, alpha);
    lv_obj_set_style_text_color(icon, color, 0);
    lv_obj_set_style_text_opa(icon, LV_OPA_COVER - (alpha / 6), 0);
}

static void particles_anim_cb(void * var, int32_t v) {
    LV_UNUSED(var);
    LV_UNUSED(v);

    int screen_w = lv_obj_get_width(charging_screen);
    int screen_h = lv_obj_get_height(charging_screen);
    int center_x = screen_w / 2;

    lv_coord_t icon_x = lv_obj_get_x(charge_icon);
    lv_coord_t icon_y = lv_obj_get_y(charge_icon);
    lv_coord_t icon_w = lv_obj_get_width(charge_icon);
    lv_coord_t icon_h = lv_obj_get_height(charge_icon);
    int icon_center_x = icon_x + icon_w / 2;
    int icon_bottom_y = icon_y + icon_h;

    for(int i = 0; i < PARTICLE_COUNT; i++) {
        particle_t * p = &particles[i];

        p->y -= p->speed / 25.0f;
        float t = 0;
        if(p->y < screen_h && p->y > icon_bottom_y + 10) {
            t = (screen_h - p->y) / (screen_h - (icon_bottom_y + 10));
            if(t > 1) t = 1;
            if(t < 0) t = 0;
        } else if(p->y <= icon_bottom_y + 10) {
            t = 1.0f;  // 完全靠近图标下方位置
        }
        // 横坐标从start_x线性插值到icon_center_x（竖直汇聚）
        p->x = p->start_x + (icon_center_x - p->start_x) * t;
        // 到达图标下方附近，粒子消失并重置
        if(p->y < icon_bottom_y) {
            // 重新随机初始位置，范围在圆环底部稍上方，宽度取充电屏幕宽度
            p->y = screen_h - 40 + (rand() % 20); // 比屏幕底部往上40~60像素位置
            p->start_x = center_x - 100 + (rand() % 200); // 圆环底部宽范围散开
            p->x = p->start_x;
            p->speed = 15 + rand() % 10; // 速度随机，保证流畅
        }
        lv_obj_set_pos(p->obj, (int)p->x, (int)p->y);
    }
}

void create_charging_screenn(void) {
    if(charging_screen) {
        lv_anim_del(charge_icon, NULL);
        lv_obj_del(charging_screen);
    }

    charging_screen = lv_obj_create(NULL);
    lv_obj_set_size(charging_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(charging_screen, DARK_BE, 0);
    lv_obj_set_style_border_width(charging_screen, 0, 0);
    lv_obj_set_style_pad_all(charging_screen, 0, 0);
		lv_obj_clear_flag(charging_screen, LV_OBJ_FLAG_SCROLLABLE);
    for(int i = 0; i < 20; i++) {
        lv_obj_t * line = lv_line_create(charging_screen);
        lv_point_t points[] = {{0, i * 20}, {lv_obj_get_width(charging_screen), i * 20}};
        lv_line_set_points(line, points, 2);
        lv_obj_set_style_line_color(line, lv_color_hex(0x1a1a2e), 0);
        lv_obj_set_style_line_width(line, 1, 0);
        lv_obj_set_style_line_opa(line, LV_OPA_50, 0);
    }

    arc = lv_arc_create(charging_screen);
    lv_obj_set_size(arc, 200, 200);
    lv_obj_center(arc);
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 0);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_rotation(arc, 270);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x1a1a2e), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, TECH_CYAN, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);

    percent_label = lv_label_create(charging_screen);
    lv_label_set_text(percent_label, "0%");
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(percent_label, lv_color_white(), 0);
    lv_obj_center(percent_label);

    status_label = lv_label_create(charging_screen);
    lv_label_set_text(status_label, "CHARGING");
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(status_label, TECH_BLUE, 0);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_set_style_text_opa(status_label, LV_OPA_0, 0);

    charge_icon = lv_label_create(charging_screen);
    lv_label_set_text(charge_icon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(charge_icon, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(charge_icon, TECH_BLUE, 0);
    lv_obj_align(charge_icon, LV_ALIGN_TOP_MID, 0, 80);
		lv_scr_load(charging_screen);

}

void close_charging_screen(void) {
    if(charging_screen) {
        lv_anim_del(charging_screen, NULL);
        for(int i = 0; i < PARTICLE_COUNT; i++) {
            if (particles[i].obj) {
                lv_obj_del(particles[i].obj);
                particles[i].obj = NULL;
            }
        }
        lv_obj_del(charging_screen);
        charging_screen = NULL;
    }
}
static lv_obj_t*sada;
void create_power_display(void)
{
	sada=lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(sada, &lv_font_montserrat_20, 0);
   lv_obj_set_style_text_color(sada, TECH_BLUE, 0);
	lv_obj_center(sada);
}



BaseType_t xReturn;
void lvgl_demo(void)
{
    lv_init();                                        
    lv_port_disp_init();                             
    lv_port_indev_init();                              
	  PCF8563_Init();
   	W25Q64_Init();	
    lvgl_img_decoder_init();
	  MPU6050_Init();  
		BME280_Init();
	  HMC_Init();
		DianLiang_queue=xQueueCreate(1,sizeof(uint8_t));
 		temperature_queue=xQueueCreate(3,sizeof(BME280_Data_t));
 		temperature_only_queue=xQueueCreate(2,sizeof(int32_t));
	  HMC_queue=xQueueCreate(1,sizeof(float));
	  STEP_queue=xQueueCreate(1,sizeof(uint16_t));
	  SLEEP_queue=xQueueCreate(1,sizeof(uint8_t));
	 xTextModeSemaphore = xSemaphoreCreateBinary();
	  xTaskCreate((TaskFunction_t )start_task,           
                (const char*    )"start_task",         
                (uint16_t       )START_STK_SIZE,       
                (void*          )NULL,                 
                (UBaseType_t    )START_TASK_PRIO,       
                (TaskHandle_t*  )&StartTask_Handler);   

    vTaskStartScheduler();                             
}

void start_task(void *pvParameters)
{
    pvParameters = pvParameters;
  	taskENTER_CRITICAL();          
    xTaskCreate((TaskFunction_t )lv_demo_task,
                (const char*    )"lv_demo_task",
                (uint16_t       )LV_DEMO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LV_DEMO_TASK_PRIO,
                (TaskHandle_t*  )&LV_DEMOTask_Handler);
								
								
		xTaskCreate((TaskFunction_t )regular_time_task,
                (const char*    )"regular_time_task",
                (uint16_t       )regular_time_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )regular_time_TASK_PRIO,
                (TaskHandle_t*  )&regular_time_Task_Handler);

    /*  */
    xTaskCreate((TaskFunction_t )RTC_task,
                (const char*    )"RTC_task",
                (uint16_t       )RTC_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )RTC_TASK_PRIO,
                (TaskHandle_t*  )&RTC_Task_Handler);
								
								    /*  */
    xReturn=xTaskCreate((TaskFunction_t )SENSOR_task,
                (const char*    )"SENSOR_task",
                (uint16_t       )SENSOR_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )SENSOR_TASK_PRIO,
                (TaskHandle_t*  )&SENSOR_Task_Handler);
								
    xReturn=xTaskCreate((TaskFunction_t )SCREEN_WAKE_task,
                (const char*    )"SCREEN_WAKE_task",
                (uint16_t       )SCREEN_WAKE_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )SCREEN_WAKE_TASK_PRIO,
                (TaskHandle_t*  )&SCREEN_WAKE_Task_Handler);
								
	  xReturn=xTaskCreate((TaskFunction_t )SCREEN_sleep_wake_task,
                (const char*    )"SCREEN_sleep_wake_task",
                (uint16_t       )SCREEN_sleep_wake_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )SCREEN_sleep_wake_TASK_PRIO,
                (TaskHandle_t*  )&SCREEN_sleep_wake_Task_Handler);


    taskEXIT_CRITICAL();          
    vTaskDelete(StartTask_Handler); /* 删除开始任务句柄 */
}

LV_FONT_DECLARE(Sun);
LV_FONT_DECLARE(Lightbulb);
static volatile bool screen_should_wake;
void lv_demo_task(void *pvParameters)
{
   pvParameters = pvParameters;

app_main(); 
next_panel();
init_screen_timeout();
main_panel();
RTC_show_time();
GIF();
line();
obj1=lv_obj_create(main_cont);
lv_obj_set_size(obj1,240,170);
lv_obj_set_y(obj1,-170);  // 
lv_obj_set_style_bg_color(obj1,lv_color_hex(0x8A8A8A),LV_STATE_DEFAULT);
lv_obj_set_style_radius(obj1, 10, 0);
lv_obj_set_style_bg_opa(obj1,50,LV_STATE_DEFAULT);
	
lv_obj_t* btn_ly=lv_btn_create(obj1);
lv_obj_set_size(btn_ly,40,52);
lv_obj_align(btn_ly,LV_ALIGN_CENTER,-84,-40);
lv_obj_set_style_bg_color(btn_ly,lv_color_hex(0xFAFAFA),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_ly,lv_color_hex(0xffffff),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_ly,lv_color_hex(0x3282F6),LV_STATE_CHECKED);
lv_obj_add_flag(btn_ly,LV_OBJ_FLAG_CHECKABLE);
lv_obj_t* LanYa=lv_label_create(btn_ly);
lv_label_set_text(LanYa, LV_SYMBOL_BLUETOOTH);
lv_obj_set_style_text_font(LanYa, &lv_font_montserrat_20, LV_PART_MAIN);
lv_obj_align(LanYa,LV_ALIGN_CENTER,0,0);
lv_obj_set_style_text_color(LanYa,lv_color_hex(0x595959),LV_STATE_DEFAULT);
lv_obj_t* btn_Gj=lv_btn_create(obj1);
lv_obj_set_size(btn_Gj,40,52);
lv_obj_align(btn_Gj,LV_ALIGN_CENTER,16,35);
lv_obj_set_style_bg_color(btn_Gj,lv_color_hex(0xFF1E27),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_Gj,lv_color_hex(0xD61921),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_Gj,lv_color_hex(0xD61921),LV_STATE_CHECKED);
lv_obj_add_flag(btn_Gj,LV_OBJ_FLAG_CLICKABLE);
lv_obj_t* GuanJi=lv_label_create(btn_Gj);
lv_label_set_text(GuanJi, LV_SYMBOL_POWER);
lv_obj_set_style_text_font(GuanJi, &lv_font_montserrat_20, LV_PART_MAIN);
lv_obj_set_align(GuanJi,LV_ALIGN_CENTER);
lv_obj_set_style_text_color(GuanJi,lv_color_hex(0xffffff),LV_STATE_DEFAULT);

lv_obj_t* btn_SD=lv_btn_create(obj1);
lv_obj_set_size(btn_SD,40,52);
lv_obj_align(btn_SD,LV_ALIGN_CENTER,-34,-40);
lv_obj_set_style_bg_color(btn_SD,lv_color_hex(0xFAFAFA),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_SD,lv_color_hex(0xffffff),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_SD,lv_color_hex(0x3487FF),LV_STATE_CHECKED);
lv_obj_add_flag(btn_SD,LV_OBJ_FLAG_CHECKABLE);
lv_obj_t* ShenDian=lv_label_create(btn_SD);
lv_label_set_text(ShenDian, LV_SYMBOL_BATTERY_EMPTY);
lv_obj_set_style_text_font(ShenDian, &lv_font_montserrat_20, LV_PART_MAIN);
lv_obj_set_align(ShenDian,LV_ALIGN_CENTER);
lv_obj_set_style_text_color(ShenDian,lv_color_hex(0x595959),LV_STATE_DEFAULT);
lv_obj_t*ChongDian=lv_label_create(ShenDian);
lv_label_set_text(ChongDian, LV_SYMBOL_CHARGE );
lv_obj_set_style_text_font(ChongDian, &lv_font_montserrat_8, LV_PART_MAIN);
lv_obj_set_align(ChongDian,LV_ALIGN_CENTER);
lv_obj_t* btn_SZ=lv_btn_create(obj1);
lv_obj_set_size(btn_SZ,40,52);
lv_obj_align(btn_SZ,LV_ALIGN_CENTER,-34,35);
lv_obj_set_style_bg_color(btn_SZ,lv_color_hex(0xFAFAFA),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_SZ,lv_color_hex(0xffffff),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_SZ,lv_color_hex(0x3282F6),LV_STATE_CHECKED);
lv_obj_add_flag(btn_SZ,LV_OBJ_FLAG_CLICKABLE);
lv_obj_t* SheZhi=lv_label_create(btn_SZ);
lv_label_set_text(SheZhi, LV_SYMBOL_SETTINGS);
lv_obj_set_style_text_font(SheZhi, &lv_font_montserrat_20, LV_PART_MAIN);
lv_obj_set_align(SheZhi,LV_ALIGN_CENTER);
lv_obj_set_style_text_color(SheZhi,lv_color_hex(0x595959),LV_STATE_DEFAULT);
lv_obj_t* btn_HY=lv_btn_create(obj1);
lv_obj_set_size(btn_HY,40,52);
lv_obj_align(btn_HY,LV_ALIGN_CENTER,16,-40);
lv_obj_set_style_bg_color(btn_HY,lv_color_hex(0xFAFAFA),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_HY,lv_color_hex(0xffffff),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_HY,lv_color_hex(0x3282F6),LV_STATE_CHECKED);
lv_obj_add_flag(btn_HY,LV_OBJ_FLAG_CHECKABLE);
lv_obj_t* HuYan=lv_label_create(btn_HY);
lv_label_set_text(HuYan, LV_SYMBOL_EYE_CLOSE);
lv_obj_set_style_text_font(HuYan, &lv_font_montserrat_20, LV_PART_MAIN);
lv_obj_set_align(HuYan,LV_ALIGN_CENTER);
lv_obj_set_style_text_color(HuYan,lv_color_hex(0x595959),LV_STATE_DEFAULT);
btn_AS=lv_btn_create(obj1);
lv_obj_set_size(btn_AS,40,52);
lv_obj_align(btn_AS,LV_ALIGN_CENTER,-84,35);
lv_obj_set_style_bg_color(btn_AS,lv_color_hex(0xFAFAFA),LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(btn_AS,lv_color_hex(0xffffff),LV_STATE_PRESSED);
lv_obj_set_style_bg_color(btn_AS,lv_color_hex(0x3282F6),LV_STATE_CHECKED);
lv_obj_add_flag(btn_AS,LV_OBJ_FLAG_CLICKABLE);
lv_obj_t* AnSe=lv_label_create(btn_AS);
lv_label_set_text(AnSe, LV_SYMBOL_LIGHTBULB);
lv_obj_set_style_text_font(AnSe, &Lightbulb, LV_PART_MAIN);
lv_obj_set_align(AnSe,LV_ALIGN_CENTER);
lv_obj_set_style_text_color(AnSe,lv_color_hex(0x595959),LV_STATE_DEFAULT);

slider = lv_slider_create(obj1);
lv_obj_set_size(slider,58,125);
lv_obj_align(slider,LV_ALIGN_CENTER,75,-3);
lv_slider_set_range(slider, 0, 99);  // 设置范围为0-100%
lv_slider_set_value(slider, 49, LV_ANIM_OFF);  // 初始值设为50%
lv_obj_set_style_radius(slider, 12, LV_PART_MAIN);
lv_obj_set_style_bg_color(slider, lv_color_hex(0x474747), LV_PART_MAIN);

lv_obj_set_style_bg_color(slider, lv_color_hex(0xFAFAFA), LV_PART_INDICATOR);
lv_obj_set_style_radius(slider,0,LV_PART_INDICATOR);

lv_obj_set_style_bg_opa(slider,LV_OPA_TRANSP,LV_PART_KNOB);
lv_obj_set_style_pad_all(slider, 0, LV_PART_KNOB);
lv_obj_set_style_radius(slider, 0, LV_PART_KNOB);
lv_obj_set_style_width(slider,0,LV_PART_KNOB);
lv_obj_set_style_height(slider,0,LV_PART_KNOB);

lv_obj_t*SUN=lv_label_create(slider);
lv_label_set_text(SUN, LV_SYMBOL_SUN);
lv_obj_set_style_text_font(SUN, &Sun, LV_PART_MAIN);
lv_obj_align(SUN,LV_ALIGN_CENTER,0,35);
lv_obj_set_style_text_color(SUN,lv_color_hex(0x595959),LV_STATE_DEFAULT);

lv_obj_add_event_cb(main_cont, main_gesture_event_cb, LV_EVENT_GESTURE, obj1);
lv_obj_add_event_cb(main_cont, main_key_event_cb, LV_EVENT_KEY,0);


lv_obj_add_event_cb(btn_ly,event_cb_ly,LV_EVENT_VALUE_CHANGED,NULL);
lv_obj_add_event_cb(btn_Gj,event_cb_Gj,LV_EVENT_CLICKED,NULL);
lv_obj_add_event_cb(btn_SD,event_cb_SD,LV_EVENT_VALUE_CHANGED,NULL);
lv_obj_add_event_cb(btn_SZ,event_cb_SZ,LV_EVENT_CLICKED,NULL);
lv_obj_add_event_cb(btn_HY,event_cb_HY,LV_EVENT_VALUE_CHANGED,NULL);
lv_obj_add_event_cb(btn_AS,slider_event_cb,LV_EVENT_CLICKED,NULL);
lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, 0);
page_end();
item_main();
LIGHT();
focus();
init_ui();

while(1)
    {
        lv_timer_handler(); 
        vTaskDelay(5);
    }
}
void regular_time_task(void *pvParameters)
{
	 pvParameters = pvParameters;
	 while(1)
    {
			if(is_running)
			{
				uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
				uint32_t total_time = elapsed_time +(current_time-start_time);
				update_time_display(total_time);
				vTaskDelay(pdMS_TO_TICKS(50));
			}
			else{
			 vTaskDelay(pdMS_TO_TICKS(500));
			}
    }
}
void RTC_task(void *pvParameters)
{
	 pvParameters = pvParameters;	
	 static uint8_t precent;
	 while(1)
    {
			precent=PowerCalculate();
			xQueueSend(DianLiang_queue,&precent,0);
      vTaskDelay(pdMS_TO_TICKS(4000)); 
    }
}
extern char rx_data[100];    
extern volatile uint8_t rx_index;

void SENSOR_task(void *pvParameters)
{
  pvParameters = pvParameters;	
	static uint32_t last_step_time = 0;
  static float peak_value = 0;
  static float acc_filtered = 0;
	
	static int16_t GX;			
	static int16_t lastDatas[2] = {0};
	static uint8_t raise_counter = 0;
	BME280_Data_t data;
	int16_t mag_data[3];          
  float heading = 0;         
	MPU6050_t mpu;
	uint16_t step_count = 0;
	float prev_magnitude = 0;
	uint8_t state = 0;
	 while(1)
    {
			if(wrist_raise_enabled)
			{
			MPU6050_GetData(&GX);
			GX=Com_Filter_LowPass(GX,lastDatas[0]);
   	  lastDatas[0]= GX;
			  if (GX>850) {
            raise_counter++;
            if (raise_counter >= 4) {  	
                raise_counter = 0;
							  screen_should_wake = true;
            }
        } else {
					  screen_should_wake = false;
            raise_counter = 0;
        }
			}
    int32_t raw_temp = BME280_ReadRawTemperature();
    data.temperature = BME280_CompensateTemperature(raw_temp);
		data.temperature = data.temperature - 7.0f;
    int32_t raw_hum = BME280_ReadRawHumidity();
    data.humidity = BME280_CompensateHumidity(raw_hum);
    int32_t raw_press = BME280_ReadRawPressure();
    data.pressure = BME280_CompensatePressure(raw_press);
    xQueueSend(temperature_queue, &data, 0);  	
		xQueueSend(temperature_only_queue, &data.temperature, 0); 
		
	  HMC_READ_DATA(mag_data);
    float x = mag_data[0]-480;
		float y = mag_data[1]-82;
		heading = atan2(-y, x) * 180.0f / M_PI;
if (heading < 0) heading += 360.0f;  
	  xQueueSend(HMC_queue, &heading, 0); 	

		MPU6050_Read_Accel(&mpu);
		float ax = mpu.Accel_X / 2048.0f;
    float ay = mpu.Accel_Y / 2048.0f;
    float az = mpu.Accel_Z / 2048.0f;
	  float acc_mag_raw  = sqrtf(ax*ax + ay*ay + az*az) - 1.0f;
		acc_filtered = acc_filtered * 0.7f + acc_mag_raw * 0.3f;
	  float acc_mag = acc_filtered;
		
		float threshold = 0.25;
		uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
		
   if (state == 0 && acc_mag > threshold && acc_mag > prev_magnitude) {
    state = 1;
    peak_value = acc_mag;  
		}
		if (state == 1 && acc_mag < prev_magnitude) {
				if(peak_value > 0.3f && (current_time - last_step_time) > 250) {
						step_count++;
						xQueueSend(STEP_queue, &step_count, 0);
						last_step_time = current_time;
				}
				state = 0;
		}
		prev_magnitude = acc_mag;
		
		taskENTER_CRITICAL();
		 if (xSemaphoreTake(xTextModeSemaphore, 0) == pdTRUE&&bluetooth_screen!=NULL)
        {			
          serial_add_log(rx_data);
					rx_index=0;
					memset(rx_data,0,sizeof(rx_data));	
        }				
		taskEXIT_CRITICAL();
    vTaskDelay(pdMS_TO_TICKS(80)); 
    }
}


void SCREEN_WAKE_task(void *pvParameters)
{
		 pvParameters = pvParameters;
	while(1)
	{
	 if(screen_should_wake&&is_display_awake)
					{
							screen_should_wake = false;
							is_display_awake=false;
					   	wake_ing=false;
									MySPI_W_SS(0);
									vTaskDelay(pdMS_TO_TICKS(1));
									MySPI_SwapByte(0XAB);
									vTaskDelay(pdMS_TO_TICKS(1));
									MySPI_W_SS(1);
									vTaskDelay(pdMS_TO_TICKS(50));
									LCD_CS_Clr();
									LCD_WR_REG(0x11); //Sleep out 	
									LCD_CS_Set();
									vTaskDelay(pdMS_TO_TICKS(80));
								__HAL_RCC_TIM1_CLK_ENABLE();  
								__HAL_RCC_USART1_CLK_ENABLE(); 
								__HAL_RCC_ADC1_CLK_ENABLE();   	
									MX_USART1_UART_Init();
								  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
									MX_ADC1_Init();
									MX_I2C1_Init();
									HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
									enter_cst816t_reset();
									BME280_wake();	
									HMC_wake();
				  if (eTaskGetState(LV_DEMOTask_Handler) == eSuspended) {
									vTaskResume(LV_DEMOTask_Handler);
									}		
					if (eTaskGetState(regular_time_Task_Handler) == eSuspended) {
									vTaskResume(regular_time_Task_Handler);
									}		
					if (eTaskGetState(RTC_Task_Handler) == eSuspended) {			
									vTaskResume(RTC_Task_Handler);		
									}			
					reset_screen_timeout();  
					}					
			 vTaskDelay(pdMS_TO_TICKS(200)); 
			}
}
extern volatile uint8_t key_num ;

void SCREEN_sleep_wake_task(void *pvParameters)
{
			 pvParameters = pvParameters;
	while(1)
	{
  if (xQueueReceive(SLEEP_queue, &SLEEP_FLAG,0) == pdTRUE) { 
			if(SLEEP_FLAG==1)
			{
			  	SLEEP_FLAG=0;
					if(!wake_ing)
				{
				vTaskSuspend(regular_time_Task_Handler);
				vTaskSuspend(RTC_Task_Handler);		
				vTaskSuspend(LV_DEMOTask_Handler); 
					
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
				HAL_UART_DeInit(&huart1);
				HAL_ADC_DeInit(&hadc1);
				
				LCD_CS_Clr();
				LCD_WR_REG(0x10); //Sleep in
				LCD_CS_Set();
				vTaskDelay(pdMS_TO_TICKS(120));	

				MySPI_W_SS(0);
				vTaskDelay(pdMS_TO_TICKS(1));	
				MySPI_SwapByte(0xB9);
				vTaskDelay(pdMS_TO_TICKS(1));	
				MySPI_W_SS(1);
				vTaskDelay(pdMS_TO_TICKS(50));	

				HMC_sleep();
				BME280_sleep();
				enter_cst816t_sleep();

				__HAL_RCC_TIM1_CLK_DISABLE(); 
				__HAL_RCC_USART1_CLK_DISABLE(); 
				__HAL_RCC_ADC1_CLK_DISABLE();   	
				wake_ing=true;	
				is_display_awake=true;	
				if(wrist_raise_enabled==false)
				 {		
					MPU6050_Sleep();
					vTaskSuspend(SENSOR_Task_Handler);
					vTaskSuspend(SCREEN_WAKE_Task_Handler);
					taskENTER_CRITICAL();
					 __disable_irq();
					HAL_SuspendTick(); 	
					HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON ,PWR_STOPENTRY_WFI);	//等待外部中断
				 SystemClock_Config(); 			       
				 HAL_ResumeTick();	
				 __enable_irq();
				 taskEXIT_CRITICAL();
	       }	
			 }
		 }
	 }
	if(key_num==LV_KEY_SHORT)
	{		
		key_num=0;
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		reset_screen_timeout_key();
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
	else if(key_num==LV_KEY_LONG)
	{
		 key_num=0;
		show_shutdown_screen();
	}
		vTaskDelay(pdMS_TO_TICKS(60));
	}
}

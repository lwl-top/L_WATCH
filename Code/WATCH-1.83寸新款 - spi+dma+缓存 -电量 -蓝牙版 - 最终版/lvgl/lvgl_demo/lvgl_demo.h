/**
 ****************************************************************************************************
 * @file        lvgl_demo.h
 * @author      ÕıµãÔ­×ÓÍÅ¶Ó(ALIENTEK)
 * @version     V1.0
 * @date        2022-01-11
 * @brief       LVGL V8²Ù×÷ÏµÍ³ÒÆÖ² ÊµÑé
 * @license     Copyright (c) 2020-2032, ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾
 ****************************************************************************************************
 * @attention
 *
 * ÊµÑéÆ½Ì¨:ÕıµãÔ­×Ó °¢²¨ÂŞ F429¿ª·¢°å
 * ÔÚÏßÊÓÆµ:www.yuanzige.com
 * ¼¼ÊõÂÛÌ³:www.openedv.com
 * ¹«Ë¾ÍøÖ·:www.alientek.com
 * ¹ºÂòµØÖ·:openedv.taobao.com
 *
 ****************************************************************************************************
 */


#ifndef __LVGL_DEMO_H
#define __LVGL_DEMO_H

void lvgl_demo(void);
// çŠ¶æ€å®šä¹‰
typedef enum {
    SCREEN_ACTIVE,
    SCREEN_DIM,
    SCREEN_OFF
} screen_state_t;
extern screen_state_t current_state;

#endif


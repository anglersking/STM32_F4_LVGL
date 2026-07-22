/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include "LCD/lcd_init.h"
#include "LCD/lcd.h"
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/
/* 从 lcd_init.h 获取实际分辨率，USE_HORIZONTAL=2 时 LCD_W=320 LCD_H=172 */
#ifndef MY_DISP_HOR_RES
    #define MY_DISP_HOR_RES    LCD_W
#endif

#ifndef MY_DISP_VER_RES
    #define MY_DISP_VER_RES    LCD_H
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    lv_display_set_flush_cb(disp, disp_flush);

    /* One buffer for partial rendering */
    static lv_color_t buf_1_1[MY_DISP_HOR_RES * 35];
    lv_display_set_buffers(disp, buf_1_1, NULL, sizeof(buf_1_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    LCD_Init();
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map)
{
    if(disp_flush_enabled) {
        int32_t x, y;
        uint16_t *color_p = (uint16_t *)px_map;

        /* 设置 LCD 写入区域 */
        LCD_Address_Set(area->x1, area->y1, area->x2, area->y2);

        /* 逐像素写入缓冲区数据 */
        for(y = area->y1; y <= area->y2; y++) {
            for(x = area->x1; x <= area->x2; x++) {
                LCD_WR_DATA(*color_p++);
            }
        }
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_display_flush_ready(disp_drv);
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif

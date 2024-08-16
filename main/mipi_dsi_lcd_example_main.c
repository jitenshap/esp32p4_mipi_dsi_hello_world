/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "graphics.h"
static const char *TAG = "MAIN";

void app_main(void)
{
    lv_display_t* display = lcd_init();
    char buf[128] = {'\0'};
    int count = 0;
    lv_obj_t *scr = NULL;
    lv_obj_t *label = NULL;
    if (lvgl_lock(-1)) {
        scr = lv_disp_get_scr_act(display);
        label = lv_label_create(scr);
        lv_style_t st;
        lv_style_init(&st);
        lv_style_set_text_font(&st, &lv_font_montserrat_48);
        lvgl_unlock();
    }
    while(1)
    {
        sprintf(buf, "Hello world!\ncount: %d", count);
        count ++;
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1)) {
            lv_label_set_text(label, buf);
            lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
            //  Release the mutex
            lvgl_unlock();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

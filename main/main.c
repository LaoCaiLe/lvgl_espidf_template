/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lcd.h"
#include "lcd_init.h"
#include "lvgl/lvgl.h"  
#include "lv_port_disp.h"
#include "lv_demos.h"
#include "lv_examples.h"
#include "esp_timer.h"

static esp_timer_handle_t lvgl_timer_handle = NULL;

 static IRAM_ATTR void lv_timer_cb(void *arg)
{
    lv_tick_inc(1);
}

static esp_timer_create_args_t lvgl_timer = {
    .callback = &lv_timer_cb,
    .arg = NULL,
    .name ="lvgl_timer",
    .dispatch_method = ESP_TIMER_TASK
};

esp_err_t _lv_timer_create(void)
{
    esp_err_t err = esp_timer_create(&lvgl_timer, &lvgl_timer_handle);
    err = esp_timer_start_periodic(lvgl_timer_handle, 1000); // 1毫秒回调
    
    return err;
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    // configure_led();
    LCD_Init();
    _lv_timer_create();
    lv_init();
    lv_port_disp_init();
    // lv_example_anim_3();
    lv_demo_music();
    while(1)
    {
        // lv_tick_inc(1);
        lv_task_handler();
        // vTaskDelay(5);
    }
}

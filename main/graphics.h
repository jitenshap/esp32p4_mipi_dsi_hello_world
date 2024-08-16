#include "freertos/semphr.h"
#include "lvgl.h"

#define MIPI_DSI_DPI_CLK_MHZ  80
#define MIPI_DSI_LCD_H_RES    600
#define MIPI_DSI_LCD_V_RES    1024
#define MIPI_DSI_LCD_HSYNC    40
#define MIPI_DSI_LCD_HBP      140
#define MIPI_DSI_LCD_HFP      40
#define MIPI_DSI_LCD_VSYNC    4
#define MIPI_DSI_LCD_VBP      16
#define MIPI_DSI_LCD_VFP      16

#define MIPI_DSI_LANE_NUM          2    // 2 data lanes
#define MIPI_DSI_LANE_BITRATE_MBPS 1000 // 1Gbps
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define PIN_NUM_BK_LIGHT                26
#define PIN_NUM_LCD_RST                 27
#define LVGL_DRAW_BUF_LINES    200 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

extern SemaphoreHandle_t lvgl_api_mux;
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);
void bsp_set_lcd_backlight(uint32_t level);
lv_display_t* lcd_init();
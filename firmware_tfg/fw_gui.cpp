#include "fw_gui.hpp"
#include "fw_pid.hpp"
#include "fw_sensorblock.hpp"

#include <lvgl.h>
#include <TFT_eSPI.h>

/*** GUI private constants ******************************************/

static const uint16_t screenWidth  = TFT_WIDTH;
static const uint16_t screenHeight = TFT_HEIGHT;
static /* const */ uint16_t calData[] = {444, 3356, 366, 3336, 1};

/****************************************** GUI private constants ***/

/*** Firmware global variables **************************************/

extern FW::PidController t_pid;    // Tcold PID controller
extern FW::SensorBlock   snsrblk;  // Analog sensor block

/************************************** Firmware global variables ***/

/*** GUI private variables ******************************************/

static lv_disp_draw_buf_t draw_buf;        // Display driver configuration
static lv_color_t buf[screenWidth * 10];   // Image buffer

static TFT_eSPI tft {screenWidth, screenHeight};  // TFT_eSPI interface

static lv_obj_t *spinbox;                  // Temperature selection spinbox
static lv_obj_t *led1;                     // Overheating warning led
static lv_obj_t *chart;                    // Temperature chart
static lv_chart_series_t *ser_t_hot;
static lv_chart_series_t *ser_t_cold;

static lv_timer_t *chart_timer;            // Chart update timer

/****************************************** GUI private variables ***/ 

/*** LVGL tactile screen portability private functions **************/

// Display driver adapter
static void msp2087_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

// Touchpad driver adapter
static void msp2087_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    bool touched = tft.getTouch(&touchX, &touchY);
    if (!touched)
        data->state = LV_INDEV_STATE_REL;
    else
    {
        data->state   = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

/********************** LVGL tactile screen portability functions ***/

/*** GUI callbacks **************************************************/

static void lv_spinbox_increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT)
        lv_spinbox_increment(spinbox);
}

static void lv_spinbox_decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT)
        lv_spinbox_decrement(spinbox);
}

static void on_update_chart(lv_timer_t *timer)
{
    lv_chart_set_next_value(chart, ser_t_hot , (int32_t)snsrblk.t_hot());
    lv_chart_set_next_value(chart, ser_t_cold, (int32_t)snsrblk.t_cold());
}

static void lv_spinbox_value_event_cb(lv_event_t * e)
{
    t_pid.reference((double)lv_spinbox_get_value(spinbox) / 10.0);
}

/************************************************** GUI callbacks ***/

/*** GUI setup ******************************************************/

static void lvgl_setup(void)
{
    tft.begin();         // TFT init
    tft.setRotation(0);  // Landscape orientation

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    tft.setTouch(calData);

    lv_init();

    // Initialize display driver
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = screenWidth;
    disp_drv.ver_res  = screenHeight;
    disp_drv.flush_cb = msp2087_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize input device driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = msp2087_touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

static void create_gui()
{
    // Create temperature selection spinbox
    spinbox = lv_spinbox_create(lv_scr_act());
    lv_spinbox_set_range(spinbox, -100, 500);
    lv_spinbox_set_digit_format(spinbox, 3, 2);
    lv_spinbox_step_prev(spinbox);
    lv_obj_set_width(spinbox, 55);
    lv_obj_align(spinbox, LV_ALIGN_TOP_MID, 0, 5);
    lv_spinbox_set_value(spinbox, (int32_t)(t_pid.reference() * 10));
    lv_obj_add_event_cb(spinbox,lv_spinbox_value_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_coord_t h = lv_obj_get_height(spinbox);

    // Create temperature increment button
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 3*h/4, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

    // Create temperature decrement button
    btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 3*h/4, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    // Create overheating warning LED and switch it OFF
    led1 = lv_led_create(lv_scr_act());
    lv_obj_align_to(led1, btn, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_led_set_brightness(led1, 150);
    lv_led_set_color(led1, lv_palette_main(LV_PALETTE_RED));
    lv_led_off(led1);

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Overheat");
    lv_obj_align_to(label, led1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    // Create temperature chart
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 90, 140);
    lv_obj_align_to(chart, spinbox, LV_ALIGN_OUT_BOTTOM_MID, 15, 45);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -10, 50); 
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 7, 2, true, 35);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);

    // Add temperture series
    ser_t_hot  = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    ser_t_cold = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    // Update chart every 1s
    chart_timer = lv_timer_create(on_update_chart, 1000,  nullptr);
}

void gui_begin(void)
{
    lvgl_setup();
    create_gui();
}

/****************************************************** GUI setup ***/

/*** GUI update *****************************************************/

void gui_update(void)
{
    lv_timer_handler_run_in_period(5);  // run lv_timer_handler() every 5ms
}

/***************************************************** GUI update ***/

void gui_notify(op_status_t cause)
{
    switch (cause)
    {
        case OS_NORMAL:
            lv_led_off(led1);
            break;
        case OS_BADSNSR:
            // TODO: inform user unrecoverable fault (reset needed)
            break;
        case OS_OVRCURRENT:
            // TODO: inform user unrecoverable fault (reset needed)
            break;
        case OS_OVRHEATED:
            lv_led_on(led1);
            break;
        default:
            // TODO: inform user unrecoverable internal error (reset needed)
            break;
    }
}

/**
 * @file lv_port_indev_templ.c
 *
 */

/*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "common.h"
#include "lv_port_indev.h"
#include "lvgl.h"
#include "ft6236.h"
#include <ulog.h>

/*********************
 *      DEFINES
 *********************/

//touch define
#define REST_PIN GET_PIN(D, 3)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;
lv_indev_t * indev_mouse;
lv_indev_t * indev_keypad;
lv_indev_t * indev_encoder;
lv_indev_t * indev_button;

rt_device_t touch;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    /*Initialize your touchpad if you have*/
    touchpad_init();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void touchpad_init(void)
{
    struct rt_touch_config cfg;
    cfg.dev_name = "i2c2";
    rt_hw_ft6236_init("touch", &cfg, REST_PIN);
    touch = rt_device_find("touch");
    rt_device_open(touch, RT_DEVICE_FLAG_RDONLY);

    struct rt_touch_info info;
    rt_device_control(touch, RT_TOUCH_CTRL_GET_INFO, &info);
    LOG_I("type       :%d", info.type);
    LOG_I("vendor     :%d", info.vendor);
    LOG_I("point_num  :%d", info.point_num);
    LOG_I("range_x    :%d", info.range_x);
    LOG_I("range_y    :%d", info.range_y);
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    struct rt_touch_data *read_data;
    read_data = (struct rt_touch_data *)rt_calloc(1, sizeof(struct rt_touch_data));
    rt_device_read(touch, 0, read_data, 1);
    if (read_data->event == RT_TOUCH_EVENT_DOWN)
    {
        rt_kprintf("down x: %03d y: %03d", read_data->x_coordinate, read_data->y_coordinate);
        rt_kprintf(" t: %d\n", read_data->timestamp);
        data->state = LV_INDEV_STATE_PR;
    }else if (read_data->event == RT_TOUCH_EVENT_MOVE)
    {
        rt_kprintf("move x: %03d y: %03d", read_data->x_coordinate, read_data->y_coordinate);
        rt_kprintf(" t: %d\n", read_data->timestamp);
        data->state = LV_INDEV_STATE_PR;
    }else {
        data->state = LV_INDEV_STATE_REL;
    }


/*    if (read_data->event == RT_TOUCH_EVENT_UP)
    {
      rt_kprintf("up   x: %03d y: %03d", read_data->x_coordinate, read_data->y_coordinate);
      rt_kprintf(" t: %d\n\n", read_data->timestamp);
    }*/

    /*Set the last pressed coordinates*/
    data->point.x = read_data->x_coordinate;
    data->point.y = read_data->y_coordinate;
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif

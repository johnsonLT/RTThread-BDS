/**
 * @file lv_demo_rtt_music.h
 *
 */

#ifndef LV_DEMO_RTT_MUSIC_H
#define LV_DEMO_RTT_MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <lvgl.h>
//lvgl music define
#define LV_USE_DEMO_RTT_MUSIC   1

#if LV_USE_DEMO_RTT_MUSIC

/*********************
 *      DEFINES
 *********************/

#if LV_DEMO_RTT_MUSIC_LARGE
#  define LV_DEMO_MUSIC_HANDLE_SIZE  40
#else
#  define LV_DEMO_MUSIC_HANDLE_SIZE  20
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_demo_music(void);
const char * _lv_demo_music_get_title(uint32_t track_id);
const char * _lv_demo_music_get_artist(uint32_t track_id);
const char * _lv_demo_music_get_genre(uint32_t track_id);
uint32_t _lv_demo_music_get_track_length(uint32_t track_id);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DEMO_RTT_MUSIC*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_RTT_MUSIC_H*/

#ifndef   NFC_SPK_SSD1306_H
#define   NFC_SPK_SSD1306_H

#include <app_timer.h>
#include <ssd1306_oled.h>
#include "nrf_drv_twi.h"
#include "fonts.c"
//#include "DSEG7_Classic_Bold_33.c"
//#include "DejaVu_LGC_Serif_33.c"
//#include "Roboto_Black_34.c"
//#include "seg7_classic_bold_33.h"

//#define SEVEN_SEGMENT_BITMAP_POINTER      (DSEG7_Classic_Bold_33)
//#define WIDTH_FRAME_SEVEN_SEGMENT         (SEVEN_SEGMENT_BITMAP_POINTER[0])
//#define HEIGHT_FRAME_SEVEN_SEGMENT        (SEVEN_SEGMENT_BITMAP_POINTER[1])
//#define FIRST_CHAR_SEVEN_SEGMENT          (SEVEN_SEGMENT_BITMAP_POINTER[2])
//#define NUM_OF_CHAR_SEVEN_SEGMENT         (SEVEN_SEGMENT_BITMAP_POINTER[3])

//#define TEXT_BITMAP_POINTER                                   (DejaVu_LGC_Serif_33)
#define WIDTH_FRAME_FONT(TEXT_BITMAP_POINTER)                 (TEXT_BITMAP_POINTER[0])
#define HEIGHT_FRAME_FONT(TEXT_BITMAP_POINTER)                (TEXT_BITMAP_POINTER[1])
#define FIRST_CHAR_FONT(TEXT_BITMAP_POINTER)                  (TEXT_BITMAP_POINTER[2])
#define NUM_OF_CHAR_FONT(TEXT_BITMAP_POINTER)                 (TEXT_BITMAP_POINTER[3])

//typedef struct 
//{
//    char text[50];
//    uint8_t text_length;
//    uint8_t font_size;
//    uint8_t pixel_per_move;
//}text_t;

typedef struct 
{
    char text[50];
    uint8_t text_length;
    const uint8_t *font;
}text_t;


typedef enum {
    ALIGN_H_LEFT,
    ALIGN_H_CENTER,
    ALIGN_H_RIGHT
}align_horizontal_t;

typedef enum{
    ALIGN_V_TOP,
    ALIGN_V_CENTER,
    ALIGN_V_BOTTOM
}align_vertical_t;


void nfc_spk_ssd1306_init(nrf_drv_twi_t * p_twi_master);
void start_moving_text(const text_t *p_text, uint8_t pixel_per_move);
void stop_moving_text();
void move_text();

//static void timer_handler(void *p_context);

void draw_text(int16_t x, int16_t y, const text_t *p_text, uint16_t color);
void draw_text_2(const text_t *p_text, uint16_t color, align_horizontal_t align_horizontal, align_vertical_t align_vertical);
//void draw_seven_segment(int16_t x, int16_t y, uint8_t c, uint16_t color);
//void draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color);



#endif //NFC_SPK_SSD1306_H
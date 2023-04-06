#include "nfc_spk_ssd1306.h"
#include "main.h"
#include "eeprom.h"
//#include "DSEG7_Classic_Bold_33.c"
//#include "Roboto_Black_34.h"

static text_t m_text;
static uint16_t m_text_pixel_length = 0;
static bool is_moving = false;

static int m_text_pos = 0;
static uint8_t m_pixel_per_move = 0;
//static uint16_t m_interval_moving_text = INTERVAL_MOVING_TEXT;

//#define USE_5x7_FONT

void nfc_spk_ssd1306_init(nrf_drv_twi_t * p_twi_master)
{
    ret_code_t err;
    //err = app_timer_create(&m_timer_ssd1306, APP_TIMER_MODE_SINGLE_SHOT, timer_handler);

    if (err != NRF_SUCCESS)
    {
        printf("Failed to create timer in function %s", __func__);
        APP_ERROR_CHECK(err);
    }

    ssd1306_init_i2c_2(p_twi_master);
}

//uint16_t interval_moving_text
void start_moving_text(const text_t *p_text, uint8_t pixel_per_move)
{
    if (is_moving || p_text == NULL) return;
    if (p_text->font == NULL) return;

    memcpy(&m_text, p_text, sizeof(text_t));
    m_text_pos = 0;
    m_pixel_per_move = pixel_per_move;

    const uint8_t first_char = FIRST_CHAR_FONT(m_text.font);

    m_text_pixel_length = 0;
    for (int i = 0; i < m_text.text_length; i++)
    {
        char c = m_text.text[i];
        uint16_t char_idx = c - first_char + 1;
        uint8_t offset_char_meta_info = char_idx * 4;

        uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((m_text.font + offset_char_meta_info));
        uint16_t  char_width = m_text.font[offset_char_meta_info + 3];

        m_text_pixel_length += char_width;
    }
    //m_interval_moving_text = interval_moving_text;
    
    move_text();

    is_moving = true;
    //app_timer_start(m_timer_ssd1306, APP_TIMER_TICKS(interval_moving_text), NULL);
}

void stop_moving_text()
{
    is_moving = false;
}

void move_text()
{
    if (!is_moving) return;
    ssd1306_clear_display();

    m_text_pos++;

    const uint8_t *font = m_text.font;
    uint8_t font_frame_width = WIDTH_FRAME_FONT(font);
    uint8_t font_frame_height = HEIGHT_FRAME_FONT(font);

    //// m_text.text_length * font_frame_width / m_pixel_per_move - 4
    if (m_text_pos > m_text_pixel_length / m_pixel_per_move + (SSD1306_LCDWIDTH + m_pixel_per_move - 1 ) / m_pixel_per_move) m_text_pos = 0;

    //uint8_t space_counter = 0;
    //for (int i = 0; i < m_text.text_length; i++)
    //{
    //    if (m_text.text[i] == ' ') 
    //    {
    //        space_counter++; continue;
    //    }
          
#ifdef  USE_5x7_FONT
        ssd1306_draw_char(SSD1306_LCDWIDTH - m_text_pos * m_text.pixel_per_move + i * m_text.font_size * 6, SSD1306_LCDHEIGHT / 2, m_text.text[i], WHITE, BLACK, m_text.font_size);
#else
        //draw_char(SSD1306_LCDWIDTH - m_text_pos * m_pixel_per_move + i * font_frame_width *5/6 - space_counter * font_frame_width / 4, (SSD1306_LCDHEIGHT - HEIGHT_FRAME_TEXT) / 2, m_text.text[i], WHITE);

#endif
    //}
    printf("Move text\n");
    draw_text(SSD1306_LCDWIDTH - m_text_pos * m_pixel_per_move, (SSD1306_LCDHEIGHT - font_frame_height) / 2, &m_text, WHITE);
    //draw_text(OFFSET_X_DISPLAY, (SSD1306_LCDHEIGHT - font_frame_height) / 2, &m_text, WHITE);
    ssd1306_display();
}

void draw_text(int16_t x, int16_t y, const text_t *p_text, uint16_t color)
{
    if (p_text == NULL) return;
    if (p_text->font == NULL) return;

    const uint8_t *font = p_text->font;
    const uint8_t text_length = p_text->text_length;
    const uint8_t first_char = FIRST_CHAR_FONT(font);
    const uint8_t num_of_char_available = NUM_OF_CHAR_FONT(font);
    const uint8_t height_frame_font = HEIGHT_FRAME_FONT(font);
    const uint8_t width_frame_font = WIDTH_FRAME_FONT(font);

    printf("Draw text text_length %d, first_char %d, num_of_char_available %d, height_frame_font %d, width_frame_font %d\n", text_length, first_char, num_of_char_available, height_frame_font, width_frame_font);
    
    uint16_t next_x = x;
    for (int i = 0; i < text_length; i++)
    {
        char c = p_text->text[i];
        uint16_t char_idx = c - first_char + 1;
        uint8_t offset_char_meta_info = char_idx * 4;

        uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((font + offset_char_meta_info));
    
        uint16_t  num_of_bytes = font[offset_char_meta_info + 2];
        uint16_t  char_width = font[offset_char_meta_info + 3];

        if (c == ' ')
        {
            next_x = next_x + char_width;
            continue;
        }

        uint8_t bytesWidth = (char_width + 7) / 8;
        uint16_t  char_height = num_of_bytes / bytesWidth;

        uint16_t offset_font = (num_of_char_available + 1) * 4 + offset_char_bytes_map;

        printf("i %d, c %d, offset_char_bytes_map %d, num_of_bytes %d, char_width %d, char_height %d, offset_font %d\n", i, c, offset_char_bytes_map, num_of_bytes, char_width, char_height, offset_font);

        ssd1306_draw_bitmap_3(next_x, y, char_width, height_frame_font, font, offset_font, num_of_bytes, color);
        next_x += char_width;
    }
}

void draw_text_2(const text_t *p_text, uint16_t color, align_horizontal_t align_horizontal, align_vertical_t align_vertical)
{
    if (p_text == NULL) return;
    if (p_text->font == NULL) return;

    const uint8_t *font = p_text->font;
    const uint8_t text_length = p_text->text_length;
    const uint8_t first_char = FIRST_CHAR_FONT(font);
    const uint8_t num_of_char_available = NUM_OF_CHAR_FONT(font);
    const uint8_t height_frame_font = HEIGHT_FRAME_FONT(font);
    const uint8_t width_frame_font = WIDTH_FRAME_FONT(font);

    uint16_t text_pixel_width = 0;
    for (int i = 0; i < p_text->text_length; i++)
    {
        char c = p_text->text[i];
        uint16_t char_idx = c - first_char + 1;
        uint8_t offset_char_meta_info = char_idx * 4;

        uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((p_text->font + offset_char_meta_info));
        uint16_t  char_width = p_text->font[offset_char_meta_info + 3];

        text_pixel_width += char_width;
    }
    
    uint16_t offset_x;
    if (align_horizontal == ALIGN_H_LEFT)
    {
        offset_x = OFFSET_X_DISPLAY;
    }
    
    else if (align_horizontal == ALIGN_H_CENTER)
    {
        offset_x = (SSD1306_LCDWIDTH - text_pixel_width) / 2;
    }

    else if (align_horizontal == ALIGN_H_RIGHT)
    {
        offset_x = SSD1306_LCDWIDTH - text_pixel_width ;//- OFFSET_X_DISPLAY;
    }


    uint16_t offset_y;
    if (align_vertical == ALIGN_V_TOP)
    {
        offset_y = 0;
    }
    
    else if (align_vertical == ALIGN_V_CENTER)
    {
        offset_y = (SSD1306_LCDHEIGHT - height_frame_font) / 2;
    }

    else if (align_vertical == ALIGN_V_BOTTOM)
    {
        offset_y = SSD1306_LCDHEIGHT - height_frame_font - OFFSET_Y_DISPLAY;
    }

    uint16_t next_x = offset_x;
    for (int i = 0; i < p_text->text_length; i++)
    {
        char c = p_text->text[i];
        uint16_t char_idx = c - first_char + 1;
        uint8_t offset_char_meta_info = char_idx * 4;

        uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((font + offset_char_meta_info));
    
        uint16_t  num_of_bytes = font[offset_char_meta_info + 2];
        uint16_t  char_width = font[offset_char_meta_info + 3];

        if (c == ' ')
        {
            next_x = next_x + char_width;
            continue;
        }

        uint8_t bytesWidth = (char_width + 7) / 8;
        uint16_t  char_height = num_of_bytes / bytesWidth;

        uint16_t offset_font = (num_of_char_available + 1) * 4 + offset_char_bytes_map;

        printf("i %d, c %d, offset_char_bytes_map %d, num_of_bytes %d, char_width %d, char_height %d, offset_font %d\n", i, c, offset_char_bytes_map, num_of_bytes, char_width, char_height, offset_font);

        ssd1306_draw_bitmap_3(next_x, offset_y, char_width, height_frame_font, font, offset_font, num_of_bytes, color);
        next_x += char_width;
    }
}

//void draw_seven_segment(int16_t x, int16_t y, uint8_t c, uint16_t color)
//{
//    uint16_t char_idx = c - FIRST_CHAR_SEVEN_SEGMENT + 1;
//    uint8_t offset_char_meta_info = char_idx * 4;

//    uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((SEVEN_SEGMENT_BITMAP_POINTER + offset_char_meta_info));
    
//    uint16_t  num_of_bytes = SEVEN_SEGMENT_BITMAP_POINTER[offset_char_meta_info + 2];
//    uint16_t  char_width = SEVEN_SEGMENT_BITMAP_POINTER[offset_char_meta_info + 3];

//    uint8_t bytesWidth = (char_width + 7) / 8;
//    uint16_t  char_height = num_of_bytes / bytesWidth;

//    uint16_t offset_font = (NUM_OF_CHAR_SEVEN_SEGMENT + 1) * 4 + offset_char_bytes_map;

//    printf("offset_char_bytes_map %d, num_of_bytes %d, char_width %d, char_height %d, offset_font %d", offset_char_bytes_map, num_of_bytes, char_width, char_height, offset_font);

//    ssd1306_draw_bitmap_3(x, y, char_width, HEIGHT_FRAME_SEVEN_SEGMENT, SEVEN_SEGMENT_BITMAP_POINTER, offset_font, num_of_bytes, color);
    //ssd1306_draw_bitmap(x, y, SEVEN_SEGMENT_BITMAP_POINTER + offset_font, char_width, char_height, WHITE);


    //uint16_t  char_width = 0x1B;
    //uint16_t  num_of_bytes = 0x78;
    //uint8_t bytesWidth = (char_width + 7) / 8;
    //uint16_t  char_height = num_of_bytes / bytesWidth;

    //ssd1306_draw_bitmap_2(x, y, seven_segment, char_width, char_height, WHITE);
    //uint16_t bytesWidth = (char_width + 7)/8;

    //int k = 0;
    //for (int i = 0; i < num_of_bytes; i++)
    //{
    //    uint8_t data = seven_segment[i];
    //    for (int j = 0; j < 8; j++)
    //    {
    //        if ((data >> j) & 0x01)  ssd1306_draw_pixel(x + j + 8 * (i % bytesWidth), y + k, WHITE);
    //    }
    //    printf("i bytesWidth %d\n",i % bytesWidth);
    //    if (i % bytesWidth ==  bytesWidth - 1) {
    //        printf("k %d\n",k);
    //        k++;
    //    }
        
    //}
    
    
//}

//void draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color)
//{
//    uint16_t char_idx = c - FIRST_CHAR_TEXT + 1;
//    uint8_t offset_char_meta_info = char_idx * 4;

//    uint16_t offset_char_bytes_map = CONVERT_8BIT_ARRAY_TO_16BIT((TEXT_BITMAP_POINTER + offset_char_meta_info));
    
//    uint16_t  num_of_bytes = TEXT_BITMAP_POINTER[offset_char_meta_info + 2];
//    uint16_t  char_width = TEXT_BITMAP_POINTER[offset_char_meta_info + 3];

//    uint8_t bytesWidth = (char_width + 7) / 8;
//    uint16_t  char_height = num_of_bytes / bytesWidth;

//    uint16_t offset_font = (NUM_OF_CHAR_TEXT + 1) * 4 + offset_char_bytes_map;

//    printf("offset_char_bytes_map %d, num_of_bytes %d, char_width %d, char_height %d, offset_font %d", offset_char_bytes_map, num_of_bytes, char_width, char_height, offset_font);

//    ssd1306_draw_bitmap_3(x, y, char_width, HEIGHT_FRAME_TEXT, TEXT_BITMAP_POINTER, offset_font, num_of_bytes, color);
//}
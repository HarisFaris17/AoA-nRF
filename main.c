#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
//#include "nrf52833.h"
#include "radio_config.h"
#include "sdk_config.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_button.h"
#include "boards.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_wdt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#include "app_button.h"
#include "app_timer.h"
#include "app_scheduler.h"

//#include "eeprom.h"

#include "nfc_spk_eeprom3.h"
#include "nfc_spk_ssd1306.h"
//#include "adafruit_pn532.h"
//#include "nrf_log_default_backends.h"
#include "nfc_spk_pn532.h"

#include "comp.h"
#include "app_util_platform.h"

#define BUZZER 31


#define STORE_VAR_START_ADDR_FLASH          0x3e000                                       /// starting address to store data in flash
#define STORE_VAR_END_ADDR_FLASH            0x3ffff                                       /// end address (exclusive) to store data in flash
#define OFFSET_ADDR_COUNTER                 0x0                                           /// offset address of counter reference to STORE_VAR_START_ADDR_FLASH in flash to store the counter 
#define OFFSET_ADDR_LENGTH_RFID_ID          (OFFSET_ADDR_COUNTER+4)                       /// offset address of length of RFID id in flash reference to OFFSET_ADDR_COUNTER (note : the counter is 4-byte, i.e. uint32_t)
#define OFFSET_ADDR_RFID_ID                 (OFFSET_ADDR_LENGTH_RFID_ID+4)                /* offset address of RFID id in flash reference OFFSET_ADDR_LENGTH_RFID_ID 
                                                                                          (note : one can't write only 1 byte to the flash, the flash only allow 
                                                                                          at least a page of bytes, our hypothesis one page is 4 bytes @ ref to
                                                                                          nrf_storage_nvmc*/


#define MAX_SPK_COUNT_PER_NFC               5
#define MAX_NFC_ID_COUNT                    5

static uint32_t                   packet;                    /**< Packet to transmit. */
static uint8_t                    payload[PAYLOAD_LENGTH];
static bool                       active_send_packet = false;

// @brief Possible state of display
typedef enum
{
    IDLE,
    CHOOSE_SPK,
    CONTINUE,
    START,
    COUNTING,
    ASK_DONE,
    COUNTING_DONE,
} state_type_t;

typedef enum {
    ADV_COUNTING,
    ADV_COUNTING_DONE,
}adv_type_t;


// @brief current display state
static state_type_t m_state;

static uint8_t m_choose_spk = 0;

// @brief object to store the received nfc
nfc_a_tag_info m_nfc_tag;

// @brief object to store current active nfc
static active_nfc_t m_active_nfc;

// @brief object of twi
static nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);

// @brief object timer
APP_TIMER_DEF(m_timer);

// @brief object timer to count how long button has been pressed
//APP_TIMER_DEF(m_button_timer); 


// @brief object timer for advertising that counting is done
APP_TIMER_DEF(m_timer_advertising);

// @brief object timer for repeated advertising
APP_TIMER_DEF(m_timer_repeated_advertising);

// @brief 
APP_TIMER_DEF(m_timer_ssd1306);

// @brief
APP_TIMER_DEF(m_timer_reset);

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
APP_TIMER_DEF(m_timer_automatic_counting_up);
#endif

#ifdef ACTIVATE_ANDON
APP_TIMER_DEF(m_timer_andon);
#endif
// @brief
//APP_TIMER_DEF(m_timer_pn532);

static uint32_t m_ticks_counter_reset = 0;
static bool     m_reset_counting = false;

static nrf_wdt_rr_register_t m_wdt_channel_id;

// @brief counter to count how many counting done advertisement has been sent.
static uint8_t m_counter_counting_done = 0;

// @brief counter to count how many counting advertisement has been sent
static uint8_t m_counter_counting = 0;

#define CMD_PAYLOAD_LENGTH      3
static char CMD_DAT[] = "DAT";
static char CMD_DONE[] = "DON";

#ifndef PN532_PRESENT

uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};

    #define CHANGE_NFC_TAG_TO_DEADBEEF()    m_active_nfc.active = true;                                     \
                                            m_active_nfc.nfc_id_len = 4;                                    \
                                            memcpy(m_active_nfc.nfc_id, deadbeef, m_active_nfc.nfc_id_len);
#endif

// =================Declarations=================

static inline uint16_t addr_eeprom_spk_with_counter(uint16_t spk_id);
                                        
static void update_display_counter();

static void update_display_choose_spk();

static void display_pn532_err();

static void display_eeprom_err();

//static void display_continue();

static void button_init();

static void timer_handler(void * p_context);

static void timer_button_handler(void *p_context);

static void timer_advertising_handler(void * p_context);

static void timer_repeated_advertising_handler(void * p_context);

static void timer_ssd1306_handler(void * p_context);

//static void timer_pn532_handler(void * p_context);

static void timer_repeated_advertising_start();

static void timer_repeated_advertising_stop();

static void timer_reset_counter_handler(void * p_context);

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
static void timer_automatic_counter_up(void * p_context);

static void start_automatic_counter_up();

static void stop_automatic_counter_up();

#endif

//static void irq_event_handler();

static void update_display_state(state_type_t state);

static void display_ask_done();

static void display_counting_done();

static void display_continue();

static void display_choose_spk();

static void display_init();

static void start_ask_done();

//static void extend_ask_done();

static void stop_ask_done();

static void startup_spk_counter_eeprom();

static void save_current_counter();

static void toggle_two_times_buzzer();   

static void update_packet_app_data();

static void wdt_event_handler();

static void start_move_ssd1306(const text_t *p_text, uint8_t pixel_per_move);

static void stop_move_ssd1306();
        

void fill_packet()
{
    // the S0, LENGTH, S1 all are byte alligned, which means if they are set to be less than a byte, the memory for each byte still byte separated
    // this is S0 field
    payload[0] = 0b00000010;  // S0, it is occupy 1 byte, it is representing the pdutype, txadd, rxadd

    // this is LENGTH field. Only 6 last significant bits being used in on-air packet, but the field will be stored as a byte.
    payload[1] = 0b00100101;  // representing LENGTH field the Payload contains 6 bytes advA and 31 bytes advData. hence total is 37
    //payload[1] = 0b00011011;

    // this is S1 field. even though the S1 only 2 last signifacant bit being used in on-aie packet, S1 still stored as a byte.
    payload[2] = 0x00;


    uint32_t addr1_32t = NRF_FICR->DEVICEADDR[0];
    uint32_t addr2_32t = NRF_FICR->DEVICEADDR[1];

    uint8_t addr1[4] ;//= NRF_FICR->DEVICEADDR[0];
    uint8_t addr2[4] ;//= (NRF_FICR->DEVICEADDR[1]);

    memcpy(addr1, &addr1_32t, 4);
    memcpy(addr2, &addr2_32t, 4);

    NRF_LOG_HEXDUMP_INFO(addr1, 4);
    NRF_LOG_HEXDUMP_INFO(addr2, 2);

    uint8_t mask_change_to_static = 0xC0;
    addr2[1] = addr2[1] | mask_change_to_static;
    NRF_LOG_INFO("After Changed");

    NRF_LOG_HEXDUMP_INFO(addr2, 2);

    memcpy(&(payload[3]), addr1, 4);
    memcpy(&(payload[7]), addr2, 2);

    uint8_t flags_section[3] = {0x02, 0x01, 0x06};

    memcpy(&(payload[9]), flags_section, 3);

    //uint8_t manuf_section[] = {0x1B, 0XFF, 0x39, 0x06, 0xCA, 0xFB, 0x00, 0x14, 0x00, 0x00, 
    //                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    //                           0x00, 0x00, 0x54, 0x3F, 0xB7, 0x48, 0x88, 0x5D};
    uint8_t manuf_section[] = {0x1B, 0XFF, 0x39, 0x06, 0xCA, 0xFB, 0x00, 0x14};//, 0x44, 0x41, 
                               //0x54, 0x23, 0x31, 0x38, 0x23, 0x39, 0x34, 0x23, 0xDA, 0x39,  
                               //0x9B, 0x19, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00};
     //uint8_t manuf_section[] = {0x11, 0XFF, 0x39, 0x06, 0x44, 0x41, 0x54, 0x23, 0x31, 0x38, 
     //                          0x23, 0x39, 0x34, 0x23, 0xDA, 0x39, 0x9B, 0x19};
     //uint8_t manuf_section[] = {0x1B, 0XFF, 0x39, 0x06, 0x44, 0x41, 0x54, 0x23, 0x31, 0x38, 
     //                           0x23, 0x39, 0x34, 0x23, 0xDA, 0x39, 0x9B, 0x19, 0x23, 0x00, 
     //                           0x00, 0x00, 0x54, 0x3F, 0xB7, 0x48, 0x88, 0x5D};

    memcpy(&(payload[12]), manuf_section, sizeof(manuf_section));

    update_packet_app_data();

    NRF_LOG_HEXDUMP_INFO(payload, PAYLOAD_LENGTH);


}


/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    //NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    ///* Wait for the external oscillator to start up */
    //while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    //{
    //    // Do nothing.
    //}

    ///* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    //NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    //NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    //NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    //while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    //{
    //    // Do nothing.
    //}

    nrf_drv_clock_init();
    nrf_drv_clock_hfclk_request(NULL);
    nrf_drv_clock_lfclk_request(NULL);

    while (!nrf_drv_clock_lfclk_is_running())
    {
        // do nothing
    }

    nrf_drv_wdt_config_t config_wdt = NRF_DRV_WDT_DEAFULT_CONFIG;//{.behaviour = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
                                        //.reload_value = NRF_WDT_CONFIG_RELOAD,
                                        //.interrupt_priority = }
    nrf_drv_wdt_init(&config_wdt, wdt_event_handler);
    nrf_drv_wdt_channel_alloc(&m_wdt_channel_id);
    nrf_drv_wdt_enable();
}

//void button_initialization()
//{
//    ret_code_t err;
//    app_button_cfg_t button = {.active_state = APP_BUTTON_ACTIVE_LOW,
//                                .button_handler = button_event_handler,
//                                .pin_no = 15,
//                                .pull_cfg = NRF_GPIO_PIN_PULLUP};
//    static app_button_cfg_t button_cfg[] = {
//                                      {
//                                      .pin_no = 15,
//                                      .active_state = APP_BUTTON_ACTIVE_HIGH,
//                                      .pull_cfg = NRF_GPIO_PIN_NOPULL,
//                                      .button_handler = button_event_handler
//                                      }
//    };
//    uint8_t num_btn = sizeof(button_cfg)/sizeof(app_button_cfg_t);
//    NRF_LOG_INFO("Number of button that wanted to be configured %d", num_btn);
//    err = app_button_init(button_cfg, num_btn, APP_TIMER_TICKS(50));
//    APP_ERROR_CHECK(err);

//    err = app_button_enable();
//    APP_ERROR_CHECK(err);

//}

void send_adv_on_channel(uint8_t channel_index)
{ 
    //NRF_LOG_INFO("Send advertisement %d", channel_index);
    set_freq(channel_index);
    send_packet(payload);
}

void send_adv()
{
    //NRF_RADIO->POWER      = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
    //nrf_delay_ms(1); 
    //NRF_RADIO->POWER      = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;
    ////nrf_delay_ms(20); 
    //nrf_delay_ms(1); 
    //NRF_LOG_INFO("mp_active_nfc pointing to nbbbbbbbbbbbbbbb: %X", get_pointer());
    update_packet_app_data();
    //NRF_LOG_INFO("mp_active_nfc pointing to rtrttrtrtrtrtrtrtrtrtrt: %X", get_pointer());
    send_adv_on_channel(37);
    send_adv_on_channel(38);
    //send_adv_on_channel(36);
    //send_adv_on_channel(35);
    //send_adv_on_channel(34);
    //send_adv_on_channel(33);
    //send_adv_on_channel(32);
    send_adv_on_channel(39);
    //NRF_LOG_INFO("mp_active_nfc pointing to 444444444444444444444444444444: %X", get_pointer());
    nrf_drv_wdt_feed();
}

void utils_setup(void)
{
    //bsp_board_init(BSP_INIT_LEDS);
    ret_code_t err;
    //err = NRF_LOG_INIT(NULL);
    err = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    //APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    button_init();

    nrf_gpio_cfg_output(BUZZER);
    //toggle_two_times_buzzer();

    err = app_timer_init();
    APP_ERROR_CHECK(err);

    nrf_delay_ms(15);

    err = app_timer_create(&m_timer,APP_TIMER_MODE_SINGLE_SHOT, timer_handler);
    APP_ERROR_CHECK(err);
    /* even though ideally we should use APP_TIMER_MODE_REPEATED, since we want to count how many tick when the 
       BUTTON_DONE pressed. but sometimes the board randomly determine who should prioritized first, sometimes it
       is m_detection_delay_timer_id from app_button module and sometimes m_button_timer. When m_button_timer prioritized 
       first and we use timer mode APP_TIMER_MODE_REPEATED and we press the BUTTON_DONE to make the m_button_timer active
       the board doesn't feel the release of BUTTON_DONE. Therefore, the m_button_timer will count ideally to infinite.\
       Therefore it is wise to used APP_TIMER_MODE_SINGLE_SHOT, hence when the handler called the timer m_detection_delay_timer_id
       automatically stopped by the board, hence the event of from BUTTON_DONE from timer m_timer can be fel by the board 
       */

    //err = app_timer_create(&m_button_timer, APP_TIMER_MODE_SINGLE_SHOT, timer_button_handler);
    //APP_ERROR_CHECK(err);

    err = app_timer_create(&m_timer_advertising, APP_TIMER_MODE_SINGLE_SHOT, timer_advertising_handler);
    APP_ERROR_CHECK(err);

    err = app_timer_create(&m_timer_repeated_advertising, APP_TIMER_MODE_SINGLE_SHOT, timer_repeated_advertising_handler);
    APP_ERROR_CHECK(err);

    err = app_timer_create(&m_timer_ssd1306, APP_TIMER_MODE_SINGLE_SHOT, timer_ssd1306_handler);
    APP_ERROR_CHECK(err);

    err = app_timer_create(&m_timer_reset, APP_TIMER_MODE_SINGLE_SHOT, timer_reset_counter_handler);
    APP_ERROR_CHECK(err);

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
    err = app_timer_create(&m_timer_automatic_counting_up, APP_TIMER_MODE_SINGLE_SHOT, timer_automatic_counter_up);
    APP_ERROR_CHECK(err);
#endif
    //err = app_timer_create(&m_timer_pn532, APP_TIMER_MODE_SINGLE_SHOT, timer_pn532_handler);
    //APP_ERROR_CHECK(err);

    //uint8_t nfc_id[] = {0x44, 0x00, 0x11, 0x89};
    //NRF_LOG_INFO("Init advertising");
    //err = adv_init();
    //APP_ERROR_CHECK(err);

    //NRF_LOG_INFO("Advertising data config");
    //err = adv_data_config(1,2,3, nfc_id, 4);
    //APP_ERROR_CHECK(err);

    //NRF_LOG_INFO("Start advertising");
    //err = adv_start_or_update(1,2,3, nfc_id, 4);
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Advertising started");

    //err = nrf_fstorage_init(&m_fstorage,&nrf_fstorage_nvmc,NULL);
    //APP_ERROR_CHECK(err);
}

static void display_idle(){
    //ssd1306_clear_display();
    //char *text_to_be_displayed = "TAP TAG";
    //int space_counter = 0;
    //for (int i = 0; i < strlen(text_to_be_displayed); i++)
    //{
    //    if (text_to_be_displayed[i] == ' ') {
    //        space_counter++;
    //        continue;
    //    }
    //    ssd1306_draw_char(OFFSET_X_DISPLAY + i *SPACING_SIZE_NORMAL - space_counter * SPACING_SIZE_NORMAL/2, OFFSET_Y_DISPLAY, text_to_be_displayed[i], WHITE, BLACK, FONT_SIZE_NORMAL_TEXT);
    //}
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'A', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'P', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, ' ', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 72, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 90, SSD1306_LCDHEIGHT / 2 - 8, 'A', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 108, SSD1306_LCDHEIGHT / 2 - 8, 'G', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_display();
    //ssd1306_start_scroll_right(0x00, 0x07);
    const char text_to_be_displayed[] = "PLEASE TAP IN";//lease Tap In";
    text_t text_object;
    memcpy(text_object.text, text_to_be_displayed, strlen(text_to_be_displayed));
    text_object.text_length = strlen(text_to_be_displayed);
    //text_object.font_size = WIDTH_FRAME_TEXT;
    //text_object.font_size = 30;
    //text_object.pixel_per_move = PIXEL_PER_MOVE;
    text_object.font = DejaVu_LGC_Serif_33;

    //ssd1306_draw_rect(0, 0, SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, WHITE);

    //app_timer_start(&m_timer_ssd1306, APP_TIMER_TICKS(INTERVAL_MOVING_TEXT), NULL);
    //start_moving_text(&text_object, PIXEL_PER_MOVE(DejaVu_LGC_Serif_33));
    start_move_ssd1306(&text_object, PIXEL_PER_MOVE(DejaVu_LGC_Serif_33));
    nrf_drv_wdt_feed();
    //update_display_counter();
}

static void display_start(){
    ssd1306_clear_display();
    char *text_to_be_displayed = "START";
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40, SSD1306_LCDHEIGHT / 2 - 8, 'S', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'A', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'R', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 40 + 72, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_display();
}

static void update_display_counter(){
    // @ this can be optimized
    static int16_t start_header_count = OFFSET_X_DISPLAY;
    ssd1306_clear_display();
    const char *text_header_count = "COUNT";
    text_t text_header_count_object;
    text_header_count_object.text_length = strlen(text_header_count);
    text_header_count_object.font = DejaVu_Serif_11;
    memcpy(text_header_count_object.text, text_header_count, strlen(text_header_count));

    //draw_text(start_header_count, 6, &text_header_count_object, WHITE);
    draw_text_2(&text_header_count_object, WHITE, ALIGN_H_LEFT, ALIGN_V_TOP);
    //draw_text(0,SSD1306_LCDHEIGHT/2, &text_header_count_object, WHITE);
    //ssd1306_fill_rect(0, 10, SSD1306_LCDWIDTH, 10, WHITE);
    counter_t counter = m_active_nfc.counter;

    static int16_t end_header_nfc_id = OFFSET_X_DISPLAY;
    char text_header_nfc_id[MAX_NFC_A_ID_LEN * 2];
    uint8_t nfc_str_length = 0;

    for (int i = 0; i < m_active_nfc.nfc_id_len; i++)
    {
        snprintf(&(text_header_nfc_id[i * 2]), MAX_NFC_A_ID_LEN * 2 - i * 2, "%X", m_active_nfc.nfc_id[i]);
        nfc_str_length += 2;
    }

    //printf("\nText nfc str : %s\n", text_header_nfc_id);
    //NRF_LOG_INFO("Text nfc str : %s\n", text_header_nfc_id);
    //printf("\n\nThis called \n\n\n\n");
    //for (int i = 0; i < nfc_str_length; i++)
    //{
        
    //    ssd1306_draw_char(SSD1306_LCDWIDTH - end_header_nfc_id - (i + 1) * 6, 6, text_header_nfc_id[nfc_str_length - 1 - i], WHITE, BLACK, 1);
    //}

    text_t text_header_nfc_id_object;
    text_header_nfc_id_object.text_length = nfc_str_length;
    text_header_nfc_id_object.font = DejaVu_Serif_11;
    memcpy(text_header_nfc_id_object.text, text_header_nfc_id, nfc_str_length);

    //draw_text(SSD1306_LCDWIDTH - 50 , 6, &text_header_nfc_id_object, WHITE);
    draw_text_2(&text_header_nfc_id_object, WHITE, ALIGN_H_RIGHT, ALIGN_V_TOP);

    // maximum 4 digit counter, since the last element to store '\0' from sprintf
    char counter_string[7];
    uint8_t digits_counter = snprintf(counter_string, 7, "%d", counter);

    text_t text;
    if (digits_counter <= 4) text.font = DSEG7_Classic_Bold_33;
    else if (digits_counter == 5) text.font = DSEG7_Classic_Bold_28;
    else if (digits_counter == 6) text.font = DSEG7_Classic_Bold_25;
    memcpy(text.text, counter_string, digits_counter);
    text.text_length = digits_counter;

    draw_text(SSD1306_LCDWIDTH / 2 - digits_counter * WIDTH_FRAME_FONT(text.font) / 2, OFFSET_Y_DISPLAY, &text, WHITE);
    ssd1306_display();
}

static void update_display_choose_spk()
{
    ssd1306_clear_display();
    char *text_to_be_displayed = "SPK";
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'S', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'P', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'K', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    char choose_spk_string[5];
    uint8_t digits_counter = snprintf(choose_spk_string,5,"%d",m_choose_spk);
    for (int i = 0; i< digits_counter ; i++){
        ssd1306_draw_char(SSD1306_LCDWIDTH / 2 + 10 + i*18, SSD1306_LCDHEIGHT / 2 - 8, choose_spk_string[i], WHITE, BLACK, 3);
    }
    ssd1306_display();
}

static void display_ask_done()
{
    NRF_LOG_INFO("Done counting ?");
    printf("Done counting ?\n\n\n");
    ssd1306_clear_display();
    const char *text_to_be_displayed = "END ?";

    text_t text;
    text.font = DejaVu_LGC_Serif_33;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    //draw_text(OFFSET_X_DISPLAY, OFFSET_Y_DISPLAY, &text, WHITE);
    draw_text_2(&text, WHITE, ALIGN_H_CENTER, ALIGN_V_CENTER);

    ssd1306_display();
}

static void display_counting_done(){
    ssd1306_clear_display();
    const char *text_to_be_displayed = "DONE";
    //for (int i = 0; i < strlen(text_to_be_displayed); i++)
    //{
    //    ssd1306_draw_char(OFFSET_X_DISPLAY + i * SPACING_SIZE_LARGE, OFFSET_Y_DISPLAY, text_to_be_displayed[i], WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //}
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'D', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'O', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'N', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'E', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    text_t text;
    text.font = DejaVu_LGC_Serif_33;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    draw_text_2(&text, WHITE, ALIGN_H_CENTER, ALIGN_V_CENTER);
    ssd1306_display();
}

static void display_continue()
{
    ssd1306_clear_display();
    const char *text_to_be_displayed = "CONTINUE";
    //int space_counter = 0;
    //for (int i = 0; i < strlen(text_to_be_displayed); i++)
    //{
    //    ssd1306_draw_char(OFFSET_X_DISPLAY + i * SPACING_SIZE_LARGE, OFFSET_Y_DISPLAY, text_to_be_displayed[i], WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //}
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58, SSD1306_LCDHEIGHT / 2 - 8, 'C', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 18, SSD1306_LCDHEIGHT / 2 - 8, 'O', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 36, SSD1306_LCDHEIGHT / 2 - 8, 'N', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    //ssd1306_draw_char(SSD1306_LCDWIDTH / 2 - 58 + 54, SSD1306_LCDHEIGHT / 2 - 8, 'T', WHITE, BLACK, FONT_SIZE_LARGE_TEXT);
    text_t text;
    text.font = DejaVu_LGC_Serif_20;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    draw_text_2(&text, WHITE, ALIGN_H_CENTER, ALIGN_V_CENTER);
    ssd1306_display();
}

static void display_init()
{
    ssd1306_clear_display();
    const char *text_to_be_displayed = "INIT...";
    text_t text;
    text.font = DejaVu_LGC_Serif_33;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    draw_text_2(&text, WHITE, ALIGN_H_CENTER, ALIGN_V_CENTER);
    ssd1306_display();
}
//static void display_choose_spk()
//{

//}

static void update_display_state(state_type_t state){
    if  (state == START)
    {
        display_start();
        m_state = START;
    }
    else if(state == COUNTING_DONE)
    {
        display_counting_done();
        m_state = COUNTING_DONE;
    }
    else if(state == IDLE)
    {
        display_idle();
        m_state == IDLE;
    }

    else if (state == COUNTING)
    {
        stop_moving_text();
        update_display_counter();
        m_state = COUNTING;
    }

    else if (state == ASK_DONE)
    {
        display_ask_done();
        m_state = ASK_DONE;
    }

    else if (state == CHOOSE_SPK)
    {
        update_display_choose_spk();
        m_state = CHOOSE_SPK;
    }

    else if (state == CONTINUE)
    {
        display_continue();
        m_state = CONTINUE;
    }
}

static void display_pn532_err()
{
    ssd1306_clear_display();

    const char *text_to_be_displayed = "NFC ERR";
    //int space_counter = 0;
    //for (int i = 0; i < strlen(text_to_be_displayed); i++)
    //{
    //    if (text_to_be_displayed[i] == ' ') {
    //        space_counter++;
    //        continue;
    //    }
    //    ssd1306_draw_char(OFFSET_X_DISPLAY + i *SPACING_SIZE_NORMAL - space_counter * SPACING_SIZE_NORMAL/2, OFFSET_Y_DISPLAY, text_to_be_displayed[i], WHITE, BLACK, FONT_SIZE_NORMAL_TEXT);
    //}
    text_t text;
    text.font = Roboto_Black_34;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    //draw_text(OFFSET_X_DISPLAY, OFFSET_Y_DISPLAY, &text, WHITE);
    draw_text_2(&text, WHITE, ALIGN_H_LEFT, ALIGN_V_CENTER);
    
    ssd1306_display();
}

static void display_eeprom_err()
{
    ssd1306_clear_display();

    const char *text_to_be_displayed = "MEM ERR";

    //int space_counter = 0;
    //for (int i = 0; i < strlen(text_to_be_displayed); i++)
    //{
    //    if (text_to_be_displayed[i] == ' ') {
    //        space_counter++;
    //        continue;
    //    }
    //    ssd1306_draw_char(OFFSET_X_DISPLAY + i *SPACING_SIZE_NORMAL - space_counter * SPACING_SIZE_NORMAL/2, OFFSET_Y_DISPLAY, text_to_be_displayed[i], WHITE, BLACK, FONT_SIZE_NORMAL_TEXT);
    //}
    text_t text;
    text.font = Roboto_Black_34;
    text.text_length = strlen(text_to_be_displayed);
    memcpy(text.text, text_to_be_displayed, strlen(text_to_be_displayed));

    //draw_text(OFFSET_X_DISPLAY, OFFSET_Y_DISPLAY, &text, WHITE);
    draw_text_2(&text, WHITE, ALIGN_H_LEFT, ALIGN_V_CENTER);
    ssd1306_display();
}


ret_code_t i2c_init()
{
    ret_code_t err_code;

    NRF_LOG_INFO("Uninit the TWI");
    NRF_LOG_FLUSH();
    nrf_drv_twi_uninit(&m_twi_master);

    NRF_LOG_INFO("Disable the TWI");
    NRF_LOG_FLUSH();
    //nrf_drv_twi_disable(&m_twi_master);


    nrf_delay_ms(10);

    nrf_gpio_cfg_output(SCL_I2C_PIN);
    nrf_gpio_cfg_output(SDA_I2C_PIN);
    nrf_gpio_pin_set(SCL_I2C_PIN);
    nrf_delay_ms(1);
    nrf_gpio_pin_set(SDA_I2C_PIN);
    nrf_delay_ms(10);

    nrf_gpio_cfg_default(SCL_I2C_PIN);
    nrf_gpio_cfg_default(SDA_I2C_PIN);

    NRF_LOG_INFO("Enable again TWI");
    NRF_LOG_FLUSH();
    //nrf_drv_twi_enable(&m_twi_master);
    *(volatile uint32_t *)0x40003FFC = 0;
    *(volatile uint32_t *)0x40003FFC;
    *(volatile uint32_t *)0x40003FFC = 1;
    nrf_drv_twi_config_t twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    twi_config.scl = SCL_I2C_PIN;
    twi_config.sda = SDA_I2C_PIN;
    printf("Creating I2C..\n");
    nrf_delay_ms(10);

    //@ Note : the event handler of twi should be null (don't know why)
    err_code = nrf_drv_twi_init(&m_twi_master, &twi_config, NULL, NULL);
    if(err_code != NRF_SUCCESS){
      printf("Failed to init TWI, error code : %d\n",err_code);
      return err_code;
    }

    //nrf_drv_twi_enable(&m_twi_master);
    return NRF_SUCCESS;
}


/**
 * @brief Function for detecting a Tag, identifying its Type and reading data from it.
 *
 * This function waits for a Tag to appear in the field. When a Tag is detected, Tag Platform
 * Type (2/4) is identified and appropriate read procedure is run.
 */
//ret_code_t tag_detect_and_read()
//{
//    ret_code_t     err_code;
//    //nfc_a_tag_info t_tag;

//    // Detect a NFC-A Tag in the field and initiate a communication. This function activates
//    // the NFC RF field. If a Tag is present, basic information about detected Tag is returned
//    // in tag info structure.
//    err_code = adafruit_pn532_nfc_a_target_init(&m_nfc_tag, TAG_DETECT_TIMEOUT);
  
//    if (err_code != NRF_SUCCESS)
//    {
//        printf("Error NRF_ERROR_NOT_FOUND\n");
//        return NRF_ERROR_NOT_FOUND;
//    }
//    adafruit_pn532_tag_info_printout(&m_nfc_tag);
    

//    nfc_tag_type_t tag_type = tag_type_identify(m_nfc_tag.sel_res);
//    switch (tag_type)
//    {
//        case NFC_T2T:
//            NRF_LOG_INFO("Type 2 Tag Platform detected. ");
//            return t2t_data_read_and_analyze(&m_nfc_tag);

//        case NFC_T4T:
//            NRF_LOG_INFO("Type 4 Tag Platform detected. ");
//            return t4t_data_read_and_analyze(&m_nfc_tag);

//        default:
//            printf("Tag is not type 2 nor type 4, but it is sufficient to be processed\n");
//            return NRF_SUCCESS;
//            //return NRF_SUCCESS;
//    }
//}




//================Definitions================

static inline uint16_t addr_eeprom_spk_with_counter(uint16_t spk_id)
{
    /* the spk_id starts from 1, but we want to store list of SPK(s) 
        starting at address START_ADDR_DATA in EEPROM
    */
    return ADDR_START_SPK_LIST+(spk_id-1)*BYTES_PER_PAGE;
}

static void button_event_handler(uint8_t pin_no, uint8_t action){
    ret_code_t err;
    //NRF_LOG_INFO("Current m_state ");//, m_state);
    NRF_LOG_INFO("Button event handler called with pin : %d",pin_no);
    printf("Button event handler called");
    //NRF_LOG_FLUSH();
    NRF_LOG_INFO("Current m_state %d", m_state);
    //NRF_LOG_FLUSH();
    //nrf_delay_ms(500);
    //void (*function_ptr)();

    if (!m_active_nfc.active)
    {  
        printf("There is no Tag registered now\n");
        NRF_LOG_INFO("There is no Tag registered now");
        //NRF_LOG_FLUSH();
        return;
    }
    
    //if (action != APP_BUTTON_PUSH)
    //{
    //    return;
    //}
    //if (action != APP_BUTTON_PUSH)
    //{
    //    NRF_LOG_INFO("The button is released");
    //    NRF_LOG_FLUSH();
    //    //save_current_counter();
    //    //printf("The button is not being pushed");
    //    if  (pin_no == BUTTON_DONE)
    //    {
    //        NRF_LOG_INFO("Button ticks counter %d", button_ticks_counter);
    //        if (button_ticks_counter < NUM_OF_TICKS_CHANGE_NFC && m_state == CHOOSE_SPK)
    //        {
    //            NRF_LOG_INFO("SPK %d Selected", m_choose_spk);
    //            m_active_nfc.active = true;
    //            m_active_nfc.spk = m_choose_spk;
    //            m_active_nfc.counter=0; ////////////////// change this
    //            //m_active_nfc.nfc_id_len=0;
    //            //for(int i=0;i<MAX_NFC_A_ID_LEN;i++){
    //            //  m_active_nfc.nfc_id[i] = 0;
    //            //}
    //            eeprom_data data;

    //            //data.length = sizeof(counter_t);
    //            //memcpy(&(data.p_data[OFFSET_COUNTER_IN_SPK]), 
    //            err = eeprom_read_data(&data, addr_eeprom_spk_with_counter(m_choose_spk), sizeof(counter_t));
    //            APP_ERROR_CHECK(err);

    //            NRF_LOG_INFO("Counter : ");
    //            NRF_LOG_HEXDUMP_INFO(data.p_data, sizeof(counter_t));

    //            if (data.p_data[0] == 0xFF && data.p_data[1] == 0xFF && data.p_data[2] == 0xFF && data.p_data[3] == 0xFF)
    //            {
                    
    //                NRF_LOG_INFO("Looks like we are using new EEPROM, hence we will reset the region of memory it to 0");
    //                memset(data.p_data, 0, sizeof(counter_t));
    //                data.length = sizeof(counter_t);
    //                err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_choose_spk));
                
    //            }
    //            else
    //            {
    //                m_active_nfc.counter = CONVERT_8BIT_ARRAY_TO_32BIT(data.p_data);
    //            }
    //            nrf_gpio_pin_clear(BUZZER);
    //            update_display_state(START);
    //            //save_eeprom();
    //            err = app_timer_start(m_timer,
    //                                  APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),
    //                                  &m_state);
    //            APP_ERROR_CHECK(err);
    //            NRF_LOG_FLUSH();
    //        }
    //        else if (button_ticks_counter < NUM_OF_TICKS_CHANGE_NFC && m_state == COUNTING)
    //        {
    //            //m_active_nfc.spk = 0;
    //            //m_active_nfc.counter = 0;
    //            //m_choose_spk = 1;
    //            m_active_nfc.active = false;
    //            m_active_nfc.nfc_id_len = 0;
    //            memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
    //            err = nfc_spk_save();
    //            if (err != NRF_SUCCESS)
    //            {
    //                NRF_LOG_ERROR("Failed to save counting continue in function %s", __func__);
    //                APP_ERROR_CHECK(err);
    //            }
    //            APP_ERROR_CHECK(adv_stop());
    //            update_display_state(CONTINUE);
    //        }
    //        else if (button_ticks_counter >= NUM_OF_TICKS_CHANGE_NFC && m_state == COUNTING)
    //        {
    //            //m_choose_spk = 1;
    //            //m_state = IDLE;
    //            m_active_nfc.active = false;
    //            m_active_nfc.nfc_id_len = 0;
    //            memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
    //            m_active_nfc.spk++;
    //            m_active_nfc.counter = 0;
    //            ret_code_t err;
    //            err = nfc_spk_save();
    //            if (err != NRF_SUCCESS)
    //            {
    //                NRF_LOG_ERROR("Failed to save counting done in function %s", __func__);
    //                APP_ERROR_CHECK(err);
    //            }
    //            APP_ERROR_CHECK(adv_stop());
    //            update_display_state(IDLE);

    //        }
    //        //nrf_delay_ms(10);
    //        //app_timer_stop(m_button_timer);
    //        button_ticks_counter = 0;
    //        continue_m_button_timer = false;
    //    }
    //    return;
    //}



    //if (m_state == CHOOSE_SPK)
    //{
    //    NRF_LOG_INFO("The function pointer is update_display_choose_spk");
    //    NRF_LOG_FLUSH();
    //    switch(pin_no)
    //    {
    //        case BUTTON_COUNTER_UP:
    //            if (m_choose_spk == MAX_NUMBER_OF_SPK) m_choose_spk = 1;
    //            else m_choose_spk++;

    //            NRF_LOG_INFO("Choose SPK up, current SPK %d", m_choose_spk);
    //            NRF_LOG_FLUSH();
    //            //NRF_LOG_INFO("Current counter : %d\n",m_active_nfc.counter);
    //            update_display_choose_spk();
    //            //save_eeprom();
    //            break;

    //        case BUTTON_COUNTER_DOWN:
    //            if (m_choose_spk == 1) m_choose_spk = MAX_NUMBER_OF_SPK;
    //            else m_choose_spk--;

    //            NRF_LOG_INFO("Choose SPK down, current SPK %d", m_choose_spk);
    //            NRF_LOG_FLUSH();
    //            update_display_choose_spk();
    //            break;

    //        case BUTTON_DONE:
    //            continue_m_button_timer = true;
    //            app_timer_start(m_button_timer, APP_TIMER_TICKS(TIMER_TICKS_PER_SHOT), NULL);
    //            break;
    //    }
    //}
    /*else*/ 
    if (action == APP_BUTTON_PUSH)
    {
        if(m_state == COUNTING)
        {
            NRF_LOG_INFO("The function pointer is update_display_choose_spk");
            //NRF_LOG_FLUSH();
            printf("state counting in %s", __func__);

            ret_code_t err;
            eeprom_data data;

            data.length = sizeof(counter_t);
            //function_ptr = update_display_counter;
            switch(pin_no)
            {
                case BUTTON_COUNTER_UP:
                    m_active_nfc.counter++;
                    printf("Counting up %d\n", m_active_nfc.counter);
                    NRF_LOG_INFO("Counter up\n");
                    NRF_LOG_INFO("Current counter : %d\n",m_active_nfc.counter);
                    NRF_LOG_INFO("m_active_nfc.spk : %d",m_active_nfc.spk);
                    NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
                    NRF_LOG_INFO("%X", get_pointer());
                    update_display_counter();
                    send_adv();
                    //save_eeprom();

                    //data.length = sizeof(counter_t);
                    //memset(data.p_data, 0, sizeof(counter_t));
                    //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                    NRF_LOG_INFO("m_active_nfc address : %X", &m_active_nfc);
                    err = nfc_spk_save();
                    if (err != NRF_SUCCESS)
                    {
                        NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                        APP_ERROR_CHECK(err);
                    }

                    //err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
                    APP_ERROR_CHECK(err);

                    err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
                    APP_ERROR_CHECK(err);

                    break;

                case BUTTON_COUNTER_DOWN:
                    NRF_LOG_INFO("%X", get_pointer());
                    printf("Counter down\n");
                    NRF_LOG_INFO("Counter down");
                    if(m_active_nfc.counter<=0)
                    {
                        printf("Current counter is 0 or less\n");
                        NRF_LOG_INFO("Current counter is 0 or less");
                    }

                    else
                    {
                        (m_active_nfc).counter--;
                        printf("Current counter : %d\n",m_active_nfc.counter);
                        update_display_counter();
                        send_adv();
                        //memset(data.p_data, 0, sizeof(counter_t));
                        //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                        //err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
                        err = nfc_spk_save();
                        if (err != NRF_SUCCESS)
                        {
                            NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                            printf("Failed to write last data counter, in function %s\n",__func__);
                            APP_ERROR_CHECK(err);
                        }
                        //save_eeprom();
                        //err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
                        //APP_ERROR_CHECK(err);

                        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
                        APP_ERROR_CHECK(err);
                    }
                    break;
//#ifdef PN532_PRESENT
                case BUTTON_DONE:
                    //err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING_DONE);
                    //APP_ERROR_CHECK(err);
                
                    start_ask_done();

                    toggle_two_times_buzzer();
#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
                    stop_automatic_counter_up();
#endif
                    //continue_m_button_timer = true;
                    //memset(data.p_data, 0, sizeof(counter_t));
                    //ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
                    //err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
                    //if (err != NRF_SUCCESS)
                    //{
                    //    NRF_LOG_ERROR("Failed to write last data counter, in function %s",__func__);
                    //    APP_ERROR_CHECK(err);
                    //}
                    //app_timer_start(m_button_timer, APP_TIMER_TICKS(TIMER_TICKS_PER_SHOT), NULL);
                    //printf("Counting done!\n");
                    //m_active_nfc.active=false;
                    //m_active_nfc.active=0;
                    //m_active_nfc.counter=0;
                    //m_active_nfc.nfc_id_len=0;
                    //for(int i=0;i<MAX_NFC_A_ID_LEN;i++){
                    //  m_active_nfc.nfc_id[i] = 0;
                    //}
                    //nrf_gpio_pin_clear(led);
                    //update_display_state(COUNTING_DONE);
                    ////save_eeprom();
                    //err = app_timer_start(m_timer,
                    //                      APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),
                    //                      &m_state);
                    //APP_ERROR_CHECK(err);
                    break;
//#endif
               
            }
        }
//#ifdef PN532_PRESENT
        else if (m_state == ASK_DONE)
        {
            switch(pin_no)
            {
                case BUTTON_DONE:
                      app_timer_stop(m_timer);
                      app_timer_start(m_timer_reset, APP_TIMER_TICKS(INTERVAL_RESET_TIMER), NULL);
                      m_reset_counting = true;

                      break;

                case BUTTON_COUNTER_UP:
                case BUTTON_COUNTER_DOWN:
                     stop_ask_done();
                     //update_display_state(COUNTING);
                     m_reset_counting = false;
                     app_timer_stop(m_timer_reset);
#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
                     start_automatic_counter_up();
#endif
                     break;

                default:
                     break;
            }
                        
        }
//#endif
        else if (m_state == CONTINUE)
        {
            // do nothing
        }
    }
//#ifdef PN532_PRESENT
    else if (action == APP_BUTTON_RELEASE)
    {
        if (pin_no == BUTTON_DONE && m_state == ASK_DONE && m_reset_counting)
        {
            NRF_LOG_INFO("Pin button_done released");
            stop_ask_done();
            m_reset_counting = false;
            app_timer_stop(m_timer_reset);
        }
        else if (pin_no == BUTTON_DONE && (m_state == COUNTING || m_state == COUNTING_DONE))
        {
            NRF_LOG_INFO("This is called");

        }
    }
//#endif
    
   

}

static void button_init(){
  ret_code_t err;
  static app_button_cfg_t buttons[] = {
                                      { .pin_no = BUTTON_COUNTER_UP,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                        },
                                      {
                                        .pin_no = BUTTON_COUNTER_DOWN,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      },
                                      {
                                        .pin_no = BUTTON_DONE,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      }
#ifdef ACTIVATE_ANDON
                                      ,{
                                        .pin_no = BUTTON_CALLING_1,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      }
                                      ,{
                                        .pin_no = BUTTON_CALLING_2,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      }
                                      ,{
                                        .pin_no = BUTTON_CALLING_4,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      }
                                      ,{
                                        .pin_no = BUTTON_CALLING_8,
                                        .active_state = APP_BUTTON_ACTIVE_HIGH,
                                        .pull_cfg = NRF_GPIO_PIN_PULLUP,//NRF_GPIO_PIN_PULLDOWN,
                                        .button_handler = button_event_handler
                                      }
#endif

  };
  uint8_t button_count = sizeof(buttons)/sizeof(app_button_cfg_t);
  NRF_LOG_INFO("Button count %d",button_count);
  printf("Button count %d\n",button_count);
  err = app_button_init(buttons, button_count, APP_TIMER_TICKS(NUM_OF_TICKS_DETECTION_DELAY_BUTTON));
  APP_ERROR_CHECK(err);

  NRF_LOG_INFO("Button has been initialized");
  printf("Button has been initialized\n");

  err = app_button_enable();
  APP_ERROR_CHECK(err);

  NRF_LOG_INFO("Button has been enabled");
  printf("Button has been enabled\n");
  NRF_LOG_FLUSH();
}

static void after_found(){
    ret_code_t err;
    if (m_active_nfc.active)
    {
        printf("There is active NFC : ");
        NRF_LOG_INFO("There is activated NFC before : ");
        NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        //for(int i=0;i<m_active_nfc.nfc_id_len;i++)
        //{
        //    printf("0x%X ",m_active_nfc.nfc_id[i]);
        //}
        //printf("\n\r");
        if (m_active_nfc.nfc_id_len == m_nfc_tag.nfc_id_len)
        {
            printf("Same length %d %d\n", m_active_nfc.nfc_id_len, m_nfc_tag.nfc_id_len);
            if (!memcmp(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_active_nfc.nfc_id_len))
            {
                
                printf("Same NFC ID detected\n");
                NRF_LOG_ERROR("Same NFC ID detected");
                return;
            }

            for (int i = 0; i<4; i++)
                {
                    printf("0x%X  0x%X\n", m_active_nfc.nfc_id[i],  m_nfc_tag.nfc_id[i]);
                }
            printf("\n");
        }
        
        NRF_LOG_INFO("Continue counting...!");
        memcpy(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_nfc_tag.nfc_id_len);
        update_display_state(CONTINUE);
        app_timer_start(m_timer, APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY), &m_state);
        
    }

    else{
        eeprom_data data;
        
        m_active_nfc.active = true;
        m_active_nfc.counter = 0;
        // later will be changed if the operator that has the NFC select SPK that have been used before
        //m_active_nfc.counter = 0;
        m_active_nfc.nfc_id_len = m_nfc_tag.nfc_id_len;
        memset(m_active_nfc.nfc_id, 0, MAX_NFC_A_ID_LEN);
        memcpy(m_active_nfc.nfc_id, m_nfc_tag.nfc_id, m_nfc_tag.nfc_id_len);

        //for(int i = 0;  i<m_nfc_tag.nfc_id_len; i++)
        //{
        //    m_active_nfc.nfc_id[i] = m_nfc_tag.nfc_id[i];
        //}
        printf("NFC Activated\n");
        NRF_LOG_INFO("NFC Activated : ");
        NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        NRF_LOG_INFO("Current m_active_nfc SPK : %d, counter : %d", m_active_nfc.spk, m_active_nfc.counter);
        //printf("Activated NFC : ");
        //for(int i=0;i<m_active_nfc.nfc_id_len;i++){
        //  printf("0x%X ",m_active_nfc.nfc_id[i]);
        //}
        //printf("\n\r");

        //update_display_state(START);

        //memset(data.p_data, 0, MAX_BYTE_PER_TRX);
        //data.p_data[OFFSET_NFC_ID_LENGTH] = m_active_nfc.nfc_id_len;
        //memcpy(&(data.p_data[OFFSET_NFC_ID]), m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
        //data.length = m_active_nfc.nfc_id_len+1; //NFC ID + NFC ID length
        //err = eeprom_write_data(&data, ADDR_NFC_LENGTH_NFC_ID);
        err = nfc_spk_save();
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to save newly active NFC ID");
            APP_ERROR_CHECK(err);
        }

        stop_move_ssd1306();
        update_display_state(COUNTING);

        //err = adv_start_or_update(m_active_nfc.spk, m_active_nfc.counter, m_active_nfc.nfc_id, m_active_nfc.nfc_id_len, ADV_COUNTING);
        //APP_ERROR_CHECK(err);

        //err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
        //APP_ERROR_CHECK(err);

        //m_choose_spk = 1;
        //update_display_state(CHOOSE_SPK);
        //app_timer_start(m_timer,APP_TIMER_TICKS(DELAY_CHANGE_STATE_DISPLAY),&m_state);

        //save_eeprom();

        //nrf_delay_ms(1000);

        //read_eeprom();
      }

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
      start_automatic_counter_up();
#endif

      toggle_two_times_buzzer();
      timer_repeated_advertising_start();
}

static void timer_handler(void * p_context){
    printf("Timer handler called\n");
    NRF_LOG_INFO("Timer Handler called");
    state_type_t state = *(state_type_t *)p_context;

    if (state == START)
    {
        update_display_state(COUNTING);
    }
    else if (state == COUNTING_DONE)
    {
        
        update_display_state(IDLE);
    }

    else if (state == CONTINUE || state == ASK_DONE)
    {
        update_display_state(COUNTING);

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
        start_automatic_counter_up();
#endif
    }
}

static void start_ask_done()
{
    update_display_state(ASK_DONE);
    app_timer_start(m_timer, APP_TIMER_TICKS(DELAY_ASK_DONE), &m_state);
}

//static void extend_ask_done()
//{
//    app_timer_stop(m_timer);
//    update_display_state(ASK_DONE);
//    app_timer_start(m_timer, APP_TIMER_TICKS(DELAY_ASK_DONE), &m_state);
//}

static void stop_ask_done()
{
    update_display_state(COUNTING);
    app_timer_stop(m_timer);
}

static void startup_spk_counter_eeprom()
{
    ret_code_t err;
    eeprom_data data;

    //get the previous NFC id and NFC length  
    err = eeprom_read_data(&data, ADDR_NFC_LENGTH_NFC_ID, BYTES_PER_PAGE);

    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Falied to retrieve currently active NFC");
        NRF_LOG_FLUSH();
        APP_ERROR_CHECK(err);

    }
    NRF_LOG_HEXDUMP_INFO(data.p_data, 4);
    uint8_t nfc_id_len = data.p_data[OFFSET_NFC_ID_LENGTH];

    if (nfc_id_len == EEPROM_DEFAULT_VALUE)
    {
        //NRF_LOG_INFO("Looks like the EEPROM is new since the NFC id length is 0xFF");
        //NRF_LOG_FLUSH();
        //uint8_t data8bit[] = {0x44, 0x99, 0x88, 0x11};
        //memcpy(m_active_nfc.nfc_id, data8bit, 4);
        //NRF_LOG_HEXDUMP_INFO(data8bit, 4);
        //m_active_nfc.active = true;
        //m_choose_spk = 1;
        //m_active_nfc.nfc_id_len = 4;
        //NRF_LOG_FLUSH();
        //update_display_state(IDLE);
        //nrf_ldelay_ms(2000);
        //update_display_state(CHOOSE_SPK);
        return;
    }

    if (nfc_id_len == 0)
    {
        NRF_LOG_INFO("No active NFC ID");
        NRF_LOG_FLUSH();
        return;
    }

    // counter guard
    if(nfc_id_len > MAX_NFC_A_ID_LEN)
    {
        NRF_LOG_INFO("The length of the retrieved NFC ID exceed MAX_NFC_A_ID_LEN");
        NRF_LOG_FLUSH();
        return;
    }
    
    m_active_nfc.active = true;
    m_active_nfc.nfc_id_len = nfc_id_len;
    memcpy(m_active_nfc.nfc_id, &(data.p_data[OFFSET_NFC_ID]), nfc_id_len);
    /////////////////////////////////////////////////////
    //m_state = CHOOSE_SPK;
    //m_choose_spk = 1;
    //update_display_state(CHOOSE_SPK);
    ////////////////////////////////////////////////////
    NRF_LOG_INFO("There is previously active NFC ID : ");
    NRF_LOG_HEXDUMP_INFO(m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);

    counter_t counter = CONVERT_8BIT_ARRAY_TO_32BIT((&(data.p_data[OFFSET_NFC_ID_LENGTH])));
    uint8_t offset_spk = m_active_nfc.nfc_id_len + OFFSET_NFC_ID + sizeof(counter_t);
    //memcpy(m_active_nfc.nfc_id, &(data.p_data[OFFSET_NFC_ID_LENGTH]), nfc_id_len);
    //m_active_nfc.active = true;
    //m_choose_spk = 0;
    //update_display_state(CHOOSE_SPK);
    //NRF_LOG_FLUSH();


    //memcpy(m_spk_list.indexes_of_spk, &(data.p_data[OFFSET_INDEXES_SPK_LIST]), number_of_spk);
    //m_spk_list.number_of_spk = number_of_spk;

    //0xFF default value
    //nfc_spk_reg_t * p_reg = nfc_spk_get_reg();

    //NRF_LOG_INFO("Default register");
    //default_registery();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Reset register");
    //nfc_spk_reset_register();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("save register comprehensive!");
    //NRF_LOG_FLUSH();
    //err = nfc_spk_save_comprehensive();
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Register saved");
    //NRF_LOG_FLUSH();

    //nfc_spk_retrieve_all();
    
    //NRF_LOG_INFO("0xEE register");
    //nfc_spk_0xff_register();
    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Retrieve comprehensive");
    //err = nfc_spk_retrieve_comprehensive();
    //APP_ERROR_CHECK(err);
    //NRF_LOG_INFO("Comprehensive retreived");
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Count NFC %d", nfc_spk_get_nfc_count());
    //NRF_LOG_FLUSH();

    //NRF_LOG_INFO("Registry size : %d",sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();
    //NRF_LOG_HEXDUMP_INFO(p_reg, sizeof(nfc_spk_reg_t));
    //NRF_LOG_FLUSH();

    //nfc_spk_print_all_active_nfc_spk();
}

static void save_current_counter()
{
    ret_code_t err;
    eeprom_data data;
    NRF_LOG_INFO("Saving current counter : %d to SPK ID %d", m_active_nfc.counter, m_active_nfc.spk);
    ASSIGN_32BIT_TO_8BIT_ARRAY(m_active_nfc.counter, data.p_data);
    data.length = sizeof(counter_t);
    
    err = eeprom_write_data(&data, addr_eeprom_spk_with_counter(m_active_nfc.spk));
    APP_ERROR_CHECK(err);    
}

static void timer_advertising_handler(void * p_context)
{
    //NRF_LOG_INFO("Timer advertising handler");
    ret_code_t err;

    NRF_LOG_INFO("Timer advertising handler called");
    
    err = app_timer_stop(m_timer_advertising);

    nrf_drv_wdt_feed();

    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Failed to stop advertisement when counting done");
        APP_ERROR_CHECK(err);
    }

    if (m_state == COUNTING_DONE)
    {
        m_counter_counting_done++;
        if (m_counter_counting_done >= 10)
        {
            m_counter_counting_done = 0;
            NRF_LOG_INFO("Attempting to reset and save to eeprom");
            err = nfc_spk_reset_and_save();
            NRF_LOG_INFO("Result to reset and save to eeprom");
            if (err != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Error while reset and save to eeprom");
                APP_ERROR_CHECK(err);
            }
            

#ifndef PN532_PRESENT
            CHANGE_NFC_TAG_TO_DEADBEEF();
            update_display_state(COUNTING);
            timer_repeated_advertising_start();

  #if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
            start_automatic_counter_up();
  #endif
#else
            update_display_state(IDLE);
#endif
            
            return;
        }
        // if m_counter_counting_done haven't reach 10, keep advertising counting_done;
        send_adv();
        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
        if (err != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Failed to start advertising in function %s",__func__);
            APP_ERROR_CHECK(err);
        }
        
    }
    else if (m_state == COUNTING)
    {
      // do nothing since the advertising has been stopped above
    }
    
}

static void timer_repeated_advertising_handler(void * p_context)
{
    ret_code_t err;

    err = app_timer_stop(m_timer_repeated_advertising);
    if (err != NRF_SUCCESS)
    {
        printf("Timer failed to stop %s\n", __func__);
        APP_ERROR_CHECK(err);
    }
    
    send_adv();
    //printf("Repeating advertisement\n");
    err = app_timer_start(m_timer_repeated_advertising, APP_TIMER_TICKS(ADVERTISING_REPEATED_INTERVAL), NULL);
    APP_ERROR_CHECK(err);
}

static void timer_ssd1306_handler(void * p_context)
{
    ret_code_t err;
    err = app_timer_stop(m_timer_ssd1306);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Timer in function %s failed to stopped", __func__);
        APP_ERROR_CHECK(err);
    }

    move_text();

    err = app_timer_start(m_timer_ssd1306, APP_TIMER_TICKS(INTERVAL_MOVING_TEXT), NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Timer in function %s failed to start", __func__);
        APP_ERROR_CHECK(err);
    }

    nrf_drv_wdt_feed();
}

static void timer_pn532_handler(void * p_context)
{
    //app_timer_stop(m_timer_pn532);
    //err_code = tag_detect_and_read();
    //switch (err_code)
    //{
    //    case NRF_SUCCESS:
    //        printf("Found\n");
    //        NRF_LOG_INFO("Found");
    //        after_found();
    //        after_read_delay();
            
    //        break;

    //    case NRF_ERROR_NO_MEM:
    //        NRF_LOG_INFO("Declared buffer for T2T is to small to store tag data.");
    //        printf("Declared buffer for T2T is to small to store tag data.\n");
    //        after_read_delay();
    //        break;

    //    case NRF_ERROR_NOT_FOUND:
    //        NRF_LOG_INFO("No Tag found.");
    //        printf("No Tag found.\n");
    //         //No delay here as we want to search for another tag immediately.
    //        break;

    //    case NRF_ERROR_NOT_SUPPORTED:
    //        NRF_LOG_INFO("Tag not supported.");
    //        printf("Tag not supported.\n");
    //        after_read_delay();
    //        break;

    //    default:
    //        NRF_LOG_INFO("Error during tag read.");
    //        printf("Error during tag read.\n");
    //        err_code = adafruit_pn532_field_off();
    //        break;
    //}
}

static void timer_repeated_advertising_start()
{
    ret_code_t err;
    printf("start repeating advertisement\n");
    //NRF_LOG_INFO("mp_active_nfc pointing to meeeeeeeeeeeeeeeeeee: %X", get_pointer());
    send_adv();
    //NRF_LOG_INFO("mp_active_nfc pointing to qqqqqqqqqqqqqqqqqqq: %X", get_pointer());
    err = app_timer_start(m_timer_repeated_advertising, APP_TIMER_TICKS(ADVERTISING_REPEATED_INTERVAL), NULL);
    //NRF_LOG_INFO("mp_active_nfc pointing to zzzzzzzzzzzzzzzzzzzzzz: %X", get_pointer());
    APP_ERROR_CHECK(err);
}

static void timer_repeated_advertising_stop()
{
    ret_code_t err;
    err = app_timer_stop(m_timer_repeated_advertising);
    APP_ERROR_CHECK(err);
}

static void timer_reset_counter_handler(void * p_context)
{
    if (!m_reset_counting)
    {
        NRF_LOG_INFO("Reset couonting stopped");
        stop_ask_done();
        app_timer_stop(m_timer_reset);
        return;
    }

    m_ticks_counter_reset++;
    if (m_ticks_counter_reset >= RESET_HOLD_TIME / INTERVAL_RESET_TIMER)
    {
        app_timer_stop(m_timer_reset);
        m_counter_counting_done = 0;
        app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
        timer_repeated_advertising_stop();
        update_display_state(COUNTING_DONE);
        m_reset_counting = false;
        m_ticks_counter_reset = 0;
        send_adv();
        toggle_two_times_buzzer();
    }

    else
    {
        app_timer_start(m_timer_reset, APP_TIMER_TICKS(INTERVAL_RESET_TIMER), NULL);
    }

}

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
static void timer_automatic_counter_up(void * p_context)
{
    ret_code_t err;
    err = app_timer_stop(m_timer_automatic_counting_up);
    APP_ERROR_CHECK(err);

    button_event_handler(BUTTON_COUNTER_UP, APP_BUTTON_PUSH);

    err = app_timer_start(m_timer_automatic_counting_up, APP_TIMER_TICKS(INTERVAL_AUTOMATIC_COUNTER_UP), NULL);
    APP_ERROR_CHECK(err);
}

static void start_automatic_counter_up()
{
    ret_code_t err;
    err = app_timer_start(m_timer_automatic_counting_up, APP_TIMER_TICKS(INTERVAL_AUTOMATIC_COUNTER_UP), NULL);
    APP_ERROR_CHECK(err);
}

static void stop_automatic_counter_up()
{
    ret_code_t err;
    err = app_timer_stop(m_timer_automatic_counting_up);
    APP_ERROR_CHECK(err);
}

#endif

static void irq_event_handler()
{
    
}

static void toggle_two_times_buzzer()
{
    nrf_gpio_pin_set(BUZZER);
    nrf_delay_ms(100);
    nrf_gpio_pin_clear(BUZZER);
}

static void update_packet_app_data()
{
    //memcpy(&(payload[CMD_PAYLOAD_OFFSET]), p_cmd, CMD_PAYLOAD_LENGTH);
    //payload[CMD_PAYLOAD_OFFSET + CMD_PAYLOAD_LENGTH] = '#';

    //uint8_t spk_payload_offset = CMD_PAYLOAD_OFFSET + CMD_PAYLOAD_LENGTH + 1;

    //uint8_t app_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    //ble_advdata_manuf_data_t manuf_adv_data;
    
    //memset(&adv_data, 0, sizeof(adv_data));
    //NRF_LOG_INFO("mp_active_nfc pointing to 999999999999999999999999999999: %X", get_pointer());
    //NRF_LOG_INFO("mp_active_nfc pointing to 77777777777777777777777777: %X", get_pointer());

    uint8_t num_of_characters;
    if (m_state == COUNTING || m_state == CONTINUE)
    {
        //NRF_LOG_INFO("mp_active_nfc pointing to 22222222222222222222222222: %X", get_pointer());
        num_of_characters = snprintf(&payload[CMD_PAYLOAD_OFFSET], 19, "DAT#%d#%d#", m_active_nfc.spk, m_active_nfc.counter);
    }
    else if (m_state == COUNTING_DONE)
    {
        //num_of_characters = snprintf(app_data, BLE_GAP_ADV_SET_DATA_SIZE_MAX, "DON#%d#%d#", spk_id, counter);
        num_of_characters = snprintf(&payload[CMD_PAYLOAD_OFFSET], 19, "DON#%d#%d#", m_active_nfc.spk, m_active_nfc.counter);
    }
    //NRF_LOG_INFO("mp_active_nfc pointing to jjmjmjmjmjmjmjmjmjmjmjmjmjmjmj: %X", get_pointer());
    memset(&payload[CMD_PAYLOAD_OFFSET + num_of_characters], 0, PAYLOAD_LENGTH - CMD_PAYLOAD_OFFSET - num_of_characters);
    //NRF_LOG_INFO("mp_active_nfc pointing to lllllllllllllllllllllllllllllll: %X", get_pointer());
    memcpy(&(payload[CMD_PAYLOAD_OFFSET + num_of_characters]), m_active_nfc.nfc_id, m_active_nfc.nfc_id_len);
    payload[CMD_PAYLOAD_OFFSET + num_of_characters + m_active_nfc.nfc_id_len] = '#';
    //NRF_LOG_HEXDUMP_INFO(payload, PAYLOAD_LENGTH);
    //NRF_LOG_INFO("mp_active_nfc pointing to qwqwqwqwqwqqwqwqwqwqwqwqwqw: %X", get_pointer());
}

static void wdt_event_handler(void)
{
    NRF_LOG_INFO("Reset the MCU");
    NRF_LOG_FINAL_FLUSH();
}

static void start_move_ssd1306(const text_t *p_text, uint8_t pixel_per_move)
{
    ret_code_t err;
    err = app_timer_start(m_timer_ssd1306, APP_TIMER_TICKS(INTERVAL_MOVING_TEXT), NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Timer in function %s failed to started", __func__);
        APP_ERROR_CHECK(err);
    }

    start_moving_text(p_text, pixel_per_move);
}

static void stop_move_ssd1306()
{
    ret_code_t err;
    err = app_timer_stop(m_timer_ssd1306);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Timer in function %s failed to be stopped", __func__);
        APP_ERROR_CHECK(err);
    }

    stop_moving_text();
}

static void startup_nfc_spk()
{
    ret_code_t err;
    //printf("Reset page\n");
    //err = reset_page(START_ADDR_DATA);
    //APP_ERROR_CHECK(err);

#if defined(BRAND_NEW_DEVICE)
    err = reset_page(START_ADDR_DATA);
    APP_ERROR_CHECK(err);
#endif

    NRF_LOG_INFO("Getting NFC and SPK info");
    printf("Getting NFC and SPK info\n");
    err = nfc_spk_retrieve_comprehensive();
    APP_ERROR_CHECK(err);

#ifndef PN532_PRESENT
    NRF_LOG_INFO("In no PN532 mode");
    CHANGE_NFC_TAG_TO_DEADBEEF();

    err = nfc_spk_save();
    APP_ERROR_CHECK(err);
#endif

    
    NRF_LOG_INFO("NFC and SPK info retrieved");
    //NRF_LOG_INFO("mp_active_nfc pointing to zzzzzzzzzzzzzz: %X", get_pointer());

    NRF_LOG_INFO("Fill packet");
    fill_packet();

    //NRF_LOG_INFO("mp_active_nfc pointing to yyyyyyyyyyy: %X", get_pointer());

    
    if (m_active_nfc.active)
    {
        update_display_state(COUNTING);

        err = app_timer_start(m_timer_advertising, APP_TIMER_TICKS(ADVERTISING_INTERVAL), NULL);
        APP_ERROR_CHECK(err);

        printf("start repeated advertising\n");
        timer_repeated_advertising_start();

#if defined(AUTOMATIC_COUNTING_UP) && AUTOMATIC_COUNTING_UP == 1
        start_automatic_counter_up();
#endif
    }
    else
    {
        //NRF_LOG_INFO("mp_active_nfc pointing to pppppppppppp: %X", get_pointer());
        printf("not starting repeated advertising\n");
        if (m_active_nfc.counter != 0) update_display_state(CONTINUE);
        else update_display_state(IDLE);
        //NRF_LOG_INFO("mp_active_nfc pointing to k=jjjjjjjjjjjjjjjjj: %X", get_pointer());
    }
    //NRF_LOG_INFO("mp_active_nfc pointing to xxxxx: %X", get_pointer());
}


/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    clock_initialization();

    utils_setup();

    uint32_t addr1 = NRF_FICR->DEVICEADDR[0];
    uint32_t addr2 = (NRF_FICR->DEVICEADDR[1]);

    uint32_t deviceid = NRF_FICR->DEVICEID[0];
    uint32_t deviceid2 = NRF_FICR->DEVICEID[1];
    uint32_t type = NRF_FICR->DEVICEADDRTYPE;
    NRF_LOG_INFO("0x%X 0x%X %d", addr2, addr1, type);
    printf("0x%X 0x%X %d\n", addr2, addr1, (type & BIT_0));
    printf("0x%X 0x%X\n", deviceid2, deviceid);

    //err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    //APP_ERROR_CHECK(err_code);

    //button_initialization();
    //NRF_LOG_INFO("Button configured");

    // Set radio configuration parameters
    radio_configure();


    NRF_LOG_HEXDUMP_INFO(NRF_RADIO->PACKETPTR, PAYLOAD_LENGTH);
    NRF_LOG_INFO("Access address");
    NRF_LOG_HEXDUMP_INFO(&(NRF_RADIO->BASE0), 4);
    NRF_LOG_HEXDUMP_INFO(&(NRF_RADIO->PREFIX0), 4);

    //NRF_RADIO->DEFMODE = 0x03;

    //err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_OFF);
    NRF_LOG_INFO("Starting in NRF52833");
    NRF_LOG_INFO("Radio transmitter example started.");
    NRF_LOG_INFO("Press Any Button");
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initialization...!");
    printf("Initialization...!");
    NRF_LOG_FLUSH();

    err_code = i2c_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("I2C initialized");
    NRF_LOG_FLUSH();

    //err_code = pn532_init();
    //if (err_code != NRF_SUCCESS)
    //{
    //    display_pn532_err();
    //    APP_ERROR_CHECK(err_code);
    //}
    //    NRF_LOG_INFO("PN532 Initialized");
    //NRF_LOG_FLUSH();

    printf("SSD1306 Initializing!\n");
    NRF_LOG_INFO("SSD1306 Initializing!");
    NRF_LOG_FLUSH();
    nfc_spk_ssd1306_init(&m_twi_master);

    display_init();

    printf("SSD1306 Initialized\n");
    NRF_LOG_INFO("SSD1306 Initialized!");
    NRF_LOG_FLUSH();
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, false);
    //printf("Display Idle SSD1306\n");
    //update_display_state(IDLE);

    NRF_LOG_INFO("Initializing EEPROM");
    printf("Initializing EEPROM");
    err_code = nfc_spk_eeprom_init(&m_twi_master, &m_active_nfc);
    if (err_code != NRF_SUCCESS)
    {
        display_eeprom_err();
        APP_ERROR_CHECK(err_code);
    }
    printf("EEPROM initialized");
    NRF_LOG_INFO("EEPROM initialized");
    NRF_LOG_INFO("%X", get_pointer());
    NRF_LOG_FLUSH();

    startup_nfc_spk();

    //NRF_LOG_INFO("mp_active_nfc pointing to  1 : %X", get_pointer());

#ifdef PN532_PRESENT
    err_code = nfc_spk_pn532_init(&m_twi_master, &m_nfc_tag, after_found);
    if (err_code != NRF_SUCCESS)
    {
        display_pn532_err();
        APP_ERROR_CHECK(err_code);
    }
    
    NRF_LOG_INFO("PN532 Initialized");
    NRF_LOG_FLUSH();
#endif

    //printf("Initializing EEPROM!\n");
    //err_code = eeprom_init(&m_twi_master);
    //APP_ERROR_CHECK(err_code);
    //printf("EEPROM initialization success\n");

    for (;;)
    {
        //NRF_LOG_INFO("mp_active_nfc pointing to : %X", get_pointer());
        nrf_drv_wdt_feed();
        NRF_LOG_FLUSH();
        __SEV();
        __WFE();
        __WFE();
        
    }

    //int i = 0;
    //active_send_packet = true;
    //while (true)
    //{
    //    packet = i;
    //    if (active_send_packet)
    //    {
    //        //send_adv(37);
    //        //send_adv(38);
    //        //send_adv(39);
    //        send_adv();
    //        printf("The contents of the package was %u", (unsigned int)packet);
    //        NRF_LOG_INFO("The contents of the package was %u", (unsigned int)packet);
    //        packet = 0;
    //    }
    //    else 
    //    {
    //        NRF_LOG_INFO("Enter wait for event");
    //        __WFE();
    //        __SEV();
    //        __WFE();
    //    }
    //    //__WFE();
        
    //    i++;
    //    nrf_delay_ms(100);
    //}
}



#ifndef   MAIN_H__
#define   MAIN_H__

#include "eeprom.h"
#include "nfc_spk_eeprom3.h"
#include "ssd1306.h"

//#define BRAND_NEW_DEVICE    
//#define PN532_PRESENT  
#define ACTIVATE_ANDON
#define AUTOMATIC_COUNTING_UP 1        

#define BUTTON_COUNTER_UP                   20
#define BUTTON_COUNTER_DOWN                 26
#define BUTTON_DONE                         15

#define BUTTON_CALLING_1                    32 + 0
#define BUTTON_CALLING_2                    32 + 9
#define BUTTON_CALLING_4                    32 + 5
#define BUTTON_CALLING_8                    4


#define START_ADDR_DATA                        0x0AC0                                        /* The 5 bits LSB represents the address in the page specified for the 12 bits upper the 5 bits LSB.
                                                                                             this 5 bits LSB will be incremented internally by eeprom after writing a byte (the pointer incremented)
                                                                                             . After writing in the last address of the specifed page, the pointer will be rolled over to the first
                                                                                             address in the page
                                                                                          */
#define MAX_NUMBER_OF_SPK                     10
//#define ADDR_INDEXES_ACTIVE_SPK               START_ADDR_DATA
#define   ADDR_NFC_LENGTH_NFC_ID              START_ADDR_DATA


#define ADDR_START_SPK_LIST                   (ADDR_NFC_LENGTH_NFC_ID+BYTES_PER_PAGE)//(ADDR_INDEXES_ACTIVE_SPK+BYTES_PER_PAGE)
//#define ADDR

//#define OFFSET_NFC_ID_LENGTH                  (0)
//#define OFFSET_NFC_ID                         (OFFSET_NFC_ID_LENGTH+1)
//#define OFFSET_COUNTER(nfc_id_len)            (OFFSET_NFC_ID + nfc_id_len)
//#define OFFSET_SPK(nfc_id_len)                (OFFSET_COUNTER(nfc_id_len) + sizeof(counter_t))      

#define OFFSET_COUNTER                                    0
#define OFFSET_SPK                                        (OFFSET_COUNTER + sizeof(counter_t))  
#define OFFSET_NFC_ID_LENGTH                              (OFFSET_SPK + sizeof(spk_t))
#define OFFSET_NFC_ID                                     (OFFSET_NFC_ID_LENGTH + 1)                      

//#define OFFSET_COUNTER_IN_SPK                  (0)
////#define OFFSET_INDEXES_SPK_LIST               (OFFSET_COUNTER_ACTIVE_SPK+1)

#define DELAY_CHANGE_STATE_DISPLAY             2000                                          /// delay state change from START to COUNTING or from COUNTING DONE to IDLE
#define DELAY_ASK_DONE                         2000

#define INTERVAL_RESET_TIMER                   100
#define RESET_HOLD_TIME                        2000
//#define INTERVAL_ADV_COUNTING_DONE             500                                            // define how long in msec interval between counting done advertisement

#define NUM_OF_TICKS_DETECTION_DELAY_BUTTON     20

#define TIMER_TICKS_PER_SHOT                    100
#define NUM_OF_TICKS_CHANGE_NFC                 10

#define APP_BLE_OBSERVER_PRIO 3

#define ADVERTISING_INTERVAL                    100
#define ADVERTISING_REPEATED_INTERVAL           100

#define CMD_PAYLOAD_OFFSET                 20

#define OFFSET_X_DISPLAY                       SSD1306_LCDWIDTH / 2 - 60 
#define OFFSET_Y_DISPLAY                       SSD1306_LCDHEIGHT / 2 - 8                      

#define FONT_SIZE_LARGE_TEXT                    5
#define FONT_SIZE_NORMAL_TEXT                   3
#define FONT_SIZE_SMALL_TEXT                    2
#define FONT_SIZE_EXTRA_SMALL_TEXT              1
//#define FONT_SIZE_ERROR_TEXT                    3   

#define SPACING_SIZE_LARGE                     30  
#define SPACING_SIZE_NORMAL                    18

#define INTERVAL_MOVING_TEXT                   200
#define PIXEL_PER_MOVE(FONT)                   WIDTH_FRAME_FONT(FONT) / 2

#define NRF_WDT_CONFIG_RELOAD                  2000
#define AFTER_PN532_READ_DELAY                 2000

#define INTERVAL_AUTOMATIC_COUNTER_UP          500

#endif  //MAIN_H
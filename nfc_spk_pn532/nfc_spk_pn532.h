#ifndef NFC_SPK_PN532_H_
#define NFC_SPK_PN532_H_

#include <nrf_drv_twi.h>
#include "adafruit_pn532.h"
#include "nfc_t2t_parser.h"
#include "nfc_t4t_cc_file.h"
#include "nfc_t4t_hl_detection_procedures.h"
#include "nfc_ndef_msg_parser.h"

#include "nrf_drv_gpiote.h"

#define SEL_RES_CASCADE_BIT_NUM            3                                              /// Number of Cascade bit within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_MASK          0x60                                           /// Mask of Tag Platform bit group within SEL_RES byte.
#define SEL_RES_TAG_PLATFORM_BIT_OFFSET    5                                              /// Offset of the Tag Platform bit group within SEL_RES byte.

#define TAG_TYPE_2_UID_LENGTH              7                                              /// Length of the Tag's UID.
#define TAG_TYPE_2_DATA_AREA_SIZE_OFFSET   (T2T_CC_BLOCK_OFFSET + 2)                      /// Offset of the byte with Tag's Data size.
#define TAG_TYPE_2_DATA_AREA_MULTIPLICATOR 8                                              /// Multiplicator for a value stored in the Tag's Data size byte.
#define TAG_TYPE_2_FIRST_DATA_BLOCK_NUM    (T2T_FIRST_DATA_BLOCK_OFFSET / T2T_BLOCK_SIZE) /// First block number with Tag's Data.
#define TAG_TYPE_2_BLOCKS_PER_EXCHANGE     (T2T_MAX_DATA_EXCHANGE / T2T_BLOCK_SIZE)       /// Number of blocks fetched in single Tag's Read command.

#define TAG_TYPE_4_NDEF_FILE_SIZE           255                                           /// Size of the buffer for NDEF file.
#define TAG_TYPE_4_NLEN_FIELD_SIZE          2                                             /// Size of NLEN field inside NDEF file.

/**
 * @brief Possible Tag Types.
 */
typedef enum
{
    NFC_T2T = 0x00,      ///< Type 2 Tag Platform.
    NFC_T4T = 0x01,      ///< Type 4A Tag Platform.
    NFC_TT_NOT_SUPPORTED ///< Tag Type not supported
} nfc_tag_type_t;

typedef void (*after_found_event_handler_t)(void);

typedef enum{
    IDLE_PN532,
    COMMAND_WRITE,
    //DATA_READ,
    ACK_READ,
    //WAIT_IRQ_HIGH,
    //INIT_REQUEST,
    TAG_WAIT,
    DELAY_AFTER_READ
}state_pn532_t;

ret_code_t nfc_spk_pn532_init(nrf_drv_twi_t * p_twi_master, 
                              nfc_a_tag_info * p_tag_info, 
                              after_found_event_handler_t after_found);

ret_code_t request_detect_tag();

//ret_code_t adafruit_pn532_nfc_a_target_init_2(uint16_t timeout);

static ret_code_t ack_read();

static void irq_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

static void delay_timeout_handler(void * p_context);

ret_code_t delay(uint16_t timeout);

ret_code_t stop_delay();

nfc_tag_type_t tag_type_identify(uint8_t sel_res);

ret_code_t t2t_data_read_and_analyze(nfc_a_tag_info * p_tag_info);

ret_code_t t4t_data_read_and_analyze(nfc_a_tag_info * p_tag_info);













#endif
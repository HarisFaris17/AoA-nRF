#include "nfc_spk_pn532.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "main.h"
#include "app_timer.h"

#define ACK_PACKET_SIZE                                 6

APP_TIMER_DEF(m_timer_delay);
APP_TIMER_DEF(m_timer_pn532);

typedef struct
{
    uint8_t ss;            // !< Slave select signal for SPI.
    uint8_t clk;           // !< Clock signal for SPI.
    uint8_t mosi;          // !< Master output, slave input signal for SPI.
    uint8_t miso;          // !< Master input, slave output signal for SPI.
    uint8_t irq;           // !< Interrupt pin for Adafruit.
    uint8_t reset;         // !< Reset pin for Adafruit.
    uint8_t in_listed_tag; // !< Tag number of in listed tags.
    bool    using_spi;     // !< True if using SPI, false if using I2C.
    bool    hardware_spi;  // !< True if using hardware SPI, false if using software SPI.
} adafruit_pn532;

static adafruit_pn532 m_pn532_object = {
    .clk          = 0,
    .miso         = 0,
    .mosi         = 0,
    .ss           = 0,
    .irq          = PN532_IRQ,
    .reset        = PN532_RESET,
    .using_spi    = false,
    .hardware_spi = false
};

static bool request_nfc_tag_enabled = false;
static bool m_is_pn532_delay = false;

static after_found_event_handler_t m_after_found_handler = NULL;
static nfc_a_tag_info * mp_tag_info;

static uint8_t m_pn532_packet_buf[PN532_PACKBUFF_SIZE];

static const uint8_t m_pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

static uint8_t m_ack_buf[ACK_PACKET_SIZE];

static state_pn532_t m_state;

#define T4T_ERROR_HANDLE(ERR_CODE, LOG) \
    if (ERR_CODE != NRF_SUCCESS)        \
    {                                   \
        NRF_LOG_INFO(LOG, ERR_CODE);    \
        return NRF_ERROR_INTERNAL;      \
    }

ret_code_t nfc_spk_pn532_init(nrf_drv_twi_t * p_twi_master, 
                              nfc_a_tag_info * p_tag_info, 
                              after_found_event_handler_t after_found)
{
    NRF_LOG_INFO("Initializing %s module", __FILE__);
    NRF_LOG_FLUSH();

    if (p_twi_master == NULL || p_tag_info == NULL || after_found == NULL)
    {
        NRF_LOG_ERROR("The parameters in function %s shouldn't be NULL", __func__);
        return NRF_ERROR_INVALID_PARAM;
    }



    ret_code_t err_code = NRF_SUCCESS;

    nrf_drv_gpiote_in_uninit(PN532_IRQ);

    err_code = adafruit_pn532_init(p_twi_master, false);
    
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("PN532 Failed to be initialized!");
        NRF_LOG_FLUSH();
        return err_code;
    }

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("GPIOTE Failed to be initialized!");
            NRF_LOG_FLUSH();
            return err_code;
        }
    }

    nrf_drv_gpiote_in_config_t config_irq = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

    err_code = nrf_drv_gpiote_in_init(PN532_IRQ, &config_irq, irq_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to initialized PN532 IRQ as GPIOTE");
        NRF_LOG_FLUSH();
        return err_code;
    }

    nrf_drv_gpiote_in_event_enable(PN532_IRQ, true);

    //if (!nrf_drv_gpiote_is_init())
    //{
    //    err_code = nrf_drv_gpiote_init();
    //    if (err_code != NRF_SUCCESS)
    //    {
    //        NRF_LOG_ERROR("GPIOTE Failed to be initialized!");
    //        NRF_LOG_FLUSH();
    //        return err_code;
    //    }
    //}

    //nrf_drv_gpiote_in_config_t config_irq = GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE(false);

    //err_code = nrf_drv_gpiote_in_init(PN532_IRQ, &config_irq, irq_event_handler);
    //if (err_code != NRF_SUCCESS)
    //{
    //    NRF_LOG_ERROR("Failed to initialized PN532 IRQ as GPIOTE");
    //    NRF_LOG_FLUSH();
    //    return err_code;
    //}

    err_code = app_timer_create(&m_timer_delay, APP_TIMER_MODE_SINGLE_SHOT, delay_timeout_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to create m_timer_delay");
        NRF_LOG_FLUSH();
        return err_code;
    }

    m_after_found_handler = after_found;
    mp_tag_info = p_tag_info;

    NRF_LOG_INFO("Request detect the tag");
    err_code = request_detect_tag();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to request detect tag, err code %d", err_code);
        return err_code;
    }

    return err_code;
}

ret_code_t request_detect_tag()
{
    NRF_LOG_INFO("Trying to read passive target ID");
    printf("Trying to read passive target ID\n");

    ret_code_t err_code;

    if (mp_tag_info == NULL)
    {
        printf("NULL pointers passed as arguments to adafruit_pn532_passive_target_init.\n");
        NRF_LOG_INFO("NULL pointers passed as arguments to adafruit_pn532_passive_target_init.");
        return NRF_ERROR_INVALID_PARAM;
    }

    m_pn532_packet_buf[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    m_pn532_packet_buf[1] = 1; // Maximum number of targets.
    m_pn532_packet_buf[2] = PN532_MIFARE_ISO14443A_BAUD;

    NRF_LOG_INFO("send request detect tag");
    err_code = adafruit_pn532_cmd_send_2(m_pn532_packet_buf,
                                        COMMAND_INLISTPASSIVETARGET_BASE_LENGTH,
                                        PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT);
    
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to send command request detect tag, err code %d", err_code);
        return err_code;
    }

    m_state = COMMAND_WRITE;
    NRF_LOG_INFO("Apply delay after command sent");
    err_code = delay(PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT * 2);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Delay failed to be applied");
        return err_code;
    }
    
    err_code = NRF_SUCCESS;
    return err_code;
}

//ret_code_t adafruit_pn532_nfc_a_target_init_2(uint16_t timeout)
//{
    //NRF_LOG_INFO("Trying to read passive target ID");
    //printf("Trying to read passive target ID\n");

    ////if (p_tag_info == NULL)
    ////{
    ////    printf("NULL pointers passed as arguments to adafruit_pn532_passive_target_init.\n");
    ////    NRF_LOG_INFO("NULL pointers passed as arguments to adafruit_pn532_passive_target_init.");
    ////    return NRF_ERROR_INVALID_PARAM;
    ////}

    //m_pn532_packet_buf[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    //m_pn532_packet_buf[1] = 1; // Maximum number of targets.
    //m_pn532_packet_buf[2] = PN532_MIFARE_ISO14443A_BAUD;

    //ret_code_t err_code = adafruit_pn532_cmd_send_2(m_pn532_packet_buf,
    //                                                COMMAND_INLISTPASSIVETARGET_BASE_LENGTH,
    //                                                PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT);
    //if (err_code != NRF_SUCCESS)
    //{
    //    printf("No card(s) read, err_code = %d", err_code);
    //    NRF_LOG_INFO("No card(s) read, err_code = %d\n", err_code);
    //    return err_code;
    //}

    //NRF_LOG_INFO("Waiting for IRQ (indicates card presence)");
    //printf("Waiting for IRQ (indicates card presence)\n");

    //// Give PN532 a little time to scan in case time-out is very small.
    //if (timeout < PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT)
    //{
    //    timeout = PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT;
    //}
//}

static ret_code_t ack_read(void)
{
    NRF_LOG_INFO("Reading ACK");

    ret_code_t err_code = NRF_SUCCESS;

    err_code = adafruit_pn532_data_read_2(m_ack_buf, ACK_PACKET_SIZE);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ACK read failed");
        return err_code;
    }

    NRF_LOG_INFO("Reading ACK Succedd");

    if (memcmp(m_ack_buf, m_pn532_ack, PN532_ACK_PACKET_SIZE) != 0)
    {
        NRF_LOG_INFO("Failed while comparing ACK packet");
        return NRF_ERROR_INTERNAL;
    }

    NRF_LOG_INFO("Compare Succedd");

    return NRF_SUCCESS;
}

static ret_code_t tag_read(void)
{
    NRF_LOG_INFO("Reading tag!");

    ret_code_t err_code = NRF_SUCCESS;
    
    err_code = adafruit_pn532_data_read_2(m_pn532_packet_buf, REPLY_INLISTPASSIVETARGET_106A_TARGET_LENGTH);

    if (err_code != NRF_SUCCESS)
    {
        printf("Failed while reading data! err_code = %d\n", err_code);
        NRF_LOG_INFO("Failed while reading data! err_code = %d", err_code);
        return err_code;
    }

    if (m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET] != 1)
    {
        printf("Failed while checking number of targets, expected 1, got %02x\n",
                     m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET]);
        NRF_LOG_INFO("Failed while checking number of targets, expected 1, got %02x",
                     m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_NBTG_OFFSET]);
        return NRF_ERROR_INVALID_DATA;
    }

    if (MAX_NFC_A_ID_LEN < m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET])
    {
        printf("UID length is invalid\n");
        NRF_LOG_INFO("UID length is invalid.");
        return NRF_ERROR_INVALID_LENGTH;
    }

    mp_tag_info->sens_res[SENS_RES_ANTICOLLISION_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_1_OFFSET];
    mp_tag_info->sens_res[SENS_RES_PLATFORM_INFO_BYTE] =
        m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SENS_RES_BYTE_2_OFFSET];

    mp_tag_info->sel_res    = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_SEL_RES_OFFSET];
    mp_tag_info->nfc_id_len = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_UID_LEN_OFFSET];
    memcpy(mp_tag_info->nfc_id,
           m_pn532_packet_buf + REPLY_INLISTPASSIVETARGET_106A_UID_OFFSET,
           mp_tag_info->nfc_id_len);

    m_pn532_object.in_listed_tag = m_pn532_packet_buf[REPLY_INLISTPASSIVETARGET_106A_TG_OFFSET];

    return NRF_SUCCESS;
}

void irq_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    NRF_LOG_INFO("IRQ event handler called");
    //NRF_LOG_FLUSH();

    if (pin != PN532_IRQ)
    {
        NRF_LOG_ERROR("The pin is not PN532_IRQ pin, but rather %d", pin);
        return;
    }

    NRF_LOG_INFO("Trying to stop delay");
    //NRF_LOG_FLUSH();

    ret_code_t err_code = NRF_SUCCESS;
    //err_code = stop_delay();
    //if (err_code != NRF_SUCCESS)
    //{
    //    NRF_LOG_INFO("Failed to stop delay");
    //    APP_ERROR_CHECK(err_code);
    //}

    NRF_LOG_INFO("Checking polarity");
    //NRF_LOG_FLUSH();

    uint8_t level = nrf_gpio_pin_read(pin);
    
#define LOW_TO_HIGH 1
#define HIGH_TO_LOW 0

    if (level == LOW_TO_HIGH)
    {
        NRF_LOG_INFO("The action is LOWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW to HIGH");
        if (m_state == ACK_READ)
        {
            err_code = stop_delay();
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Failed to stop delay");
                APP_ERROR_CHECK(err_code);
            }
            NRF_LOG_INFO("The IRQ pin goes to idle state (HIGH)");
            m_state = TAG_WAIT;

            NRF_LOG_INFO("Waiting for tag...");
            delay(TAG_DETECT_TIMEOUT);
        }
    }
    else if (level == HIGH_TO_LOW)
    {
        NRF_LOG_INFO("The action is HIGHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH to LOW");
        if (m_state == COMMAND_WRITE)
        {
            NRF_LOG_INFO("The command is COMMAND_WRITE");
            err_code = stop_delay();
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Failed to stop delay");
                APP_ERROR_CHECK(err_code);
            }

            err_code = ack_read();

            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Reading ACK Failed");

            }

            // wait IRQ to be high
            m_state = ACK_READ;
            //delay(PN532_DEFAULT_WAIT_FOR_READY_TIMEOUT * 2);
        }

        else if (m_state == TAG_WAIT)
        {
            NRF_LOG_INFO("The command is TAG_WAIT");

            err_code = stop_delay();
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Failed to stop delay");
                APP_ERROR_CHECK(err_code);
            }

            ret_code_t err_code = tag_read();

            if (err_code != NRF_SUCCESS)
            {
                printf("Error NRF_ERROR_NOT_FOUND\n");
                return NRF_ERROR_NOT_FOUND;
            }
            adafruit_pn532_tag_info_printout(mp_tag_info);
    

            nfc_tag_type_t tag_type = tag_type_identify(mp_tag_info->sel_res);
            switch (tag_type)
            {
                case NFC_T2T:
                    NRF_LOG_INFO("Type 2 Tag Platform detected. ");
                    err_code = t2t_data_read_and_analyze(mp_tag_info);

                case NFC_T4T:
                    NRF_LOG_INFO("Type 4 Tag Platform detected. ");
                    err_code = t4t_data_read_and_analyze(mp_tag_info);

                default:
                    printf("Tag is not type 2 nor type 4, but it is sufficient to be processed\n");
                    err_code = NRF_SUCCESS;
            }

            switch (err_code)
            {
                case NRF_SUCCESS:
                    printf("Found NFC Tag\n");
                    NRF_LOG_INFO("Found NFC Tag");
                    m_after_found_handler();
                    after_read_delay();
                    m_state = DELAY_AFTER_READ;
            
                    break;

                case NRF_ERROR_NO_MEM:
                    NRF_LOG_INFO("Declared buffer for T2T is to small to store tag data.");
                    printf("Declared buffer for T2T is to small to store tag data.\n");
                    after_read_delay();
                    m_state = DELAY_AFTER_READ;

                    break;

                case NRF_ERROR_NOT_FOUND:
                    NRF_LOG_INFO("No Tag found.");
                    printf("No Tag found.\n");
                    m_state = IDLE_PN532;
                    request_detect_tag();
                     //No delay here as we want to search for another tag immediately.
                    break;

                case NRF_ERROR_NOT_SUPPORTED:
                    NRF_LOG_INFO("Tag not supported.");
                    printf("Tag not supported.\n");
                    after_read_delay();
                    m_state = DELAY_AFTER_READ;

                    break;

                default:
                    NRF_LOG_INFO("Error during tag read.");
                    printf("Error during tag read.\n");
                    err_code = adafruit_pn532_field_off();
                    m_state = IDLE_PN532;
                    request_detect_tag();
                    break;
            }
        }
        else if (m_state == DELAY_AFTER_READ)
        {
            // do nothing, just recognize that the IRQ handler is called when the NFC tag detected previously
        }
    }

    if (action == NRF_GPIOTE_POLARITY_TOGGLE)
    {
        NRF_LOG_INFO("TOGGGGLLLLLLLLLLLLLLLLLEEEEEEEEEEEE");
    }

    

    //if (action != NRF_GPIOTE_POLARITY_HITOLO)
    //{
    //    NRF_LOG_ERROR("The polarity is not HIGH to LOW, but rather %d", action);
    //    return;
    //}

    //if (m_after_found_handler == NULL)
    //{
    //    NRF_LOG_ERROR("Initialized the module nfc_spk_pn532.c first");
    //    return NRF_ERROR_INVALID_STATE;
    //}

    //m_after_found_handler();
}

static void delay_timeout_handler(void * p_context)
{
    NRF_LOG_INFO("Delay timeout handler called");
    if (m_state == COMMAND_WRITE)
    {
        NRF_LOG_ERROR("Failed to get ACK");
        return;
    }

    else if (m_state == ACK_READ)
    {
        NRF_LOG_INFO("This is ACK_READ");
        return;
    }
    else if (m_state == TAG_WAIT)
    {
        NRF_LOG_INFO("Tag wait timeout");
        request_detect_tag();
        return;
    }
    else if (m_state == DELAY_AFTER_READ)
    {
        NRF_LOG_INFO("Delay after read timeout");
        request_detect_tag();
        return;
    }
}

ret_code_t delay(uint16_t timeout)
{

    ret_code_t err = NRF_SUCCESS;

    NRF_LOG_INFO("Trying to delay");

    if (m_is_pn532_delay) 
    {
        err = stop_delay();
        VERIFY_SUCCESS(err);
    }

    m_is_pn532_delay = true;

    err = app_timer_start(m_timer_delay, APP_TIMER_TICKS(timeout), NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("m_timer_delay failed to be started");
        return err;
    }

    return err;
}

ret_code_t stop_delay()
{   
    ret_code_t err = NRF_SUCCESS;
    err = app_timer_stop(m_timer_delay);

    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to stop the m_timer_delay")
        return err;
    }

    m_is_pn532_delay = false;
    return err;
}

/**
 * @brief Function for identifying Tag Platform Type.
 */
nfc_tag_type_t tag_type_identify(uint8_t sel_res)
{
    uint8_t platform_config;

    // Check if Cascade bit in SEL_RES response is cleared. Cleared bit indicates that NFCID1 complete.
    if (!IS_SET(sel_res, SEL_RES_CASCADE_BIT_NUM))
    {
        // Extract platform configuration from SEL_RES response.
        platform_config = (sel_res & SEL_RES_TAG_PLATFORM_MASK) >> SEL_RES_TAG_PLATFORM_BIT_OFFSET;
        if (platform_config < NFC_TT_NOT_SUPPORTED)
        {
            return (nfc_tag_type_t) platform_config;
        }
    }

    return NFC_TT_NOT_SUPPORTED;
}

/**
 * @brief Function for analyzing NDEF data coming either from a Type 2 Tag TLV block or
 *        Type 4 Tag NDEF file.
 */
void ndef_data_analyze(uint8_t * p_ndef_msg_buff, uint32_t nfc_data_len)
{
    ret_code_t err_code;

    uint8_t  desc_buf[NFC_NDEF_PARSER_REQIRED_MEMO_SIZE_CALC(MAX_NDEF_RECORDS)];
    uint32_t desc_buf_len = sizeof(desc_buf);

    err_code = ndef_msg_parser(desc_buf,
                               &desc_buf_len,
                               p_ndef_msg_buff,
                               &nfc_data_len);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error during parsing a NDEF message.");
    }

    ndef_msg_printout((nfc_ndef_msg_desc_t *) desc_buf);
}


/**
 * @brief Function for reading data  from a Type 2 Tag Platform.
 */
ret_code_t t2t_data_read(nfc_a_tag_info * p_tag_info, uint8_t * buffer, uint32_t buffer_size)
{
    ret_code_t err_code;
    uint8_t    block_num = 0;

    // Not enough size in the buffer to read a tag header.
    if (buffer_size < T2T_FIRST_DATA_BLOCK_OFFSET)
    {
        return NRF_ERROR_NO_MEM;
    }

    if (p_tag_info->nfc_id_len != TAG_TYPE_2_UID_LENGTH)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }

    // Read blocks 0 - 3 to get the header information.
    err_code = adafruit_pn532_tag2_read(block_num, buffer);
    if (err_code)
    {
        NRF_LOG_INFO("Failed to read blocks: %d-%d", block_num,
                     block_num + T2T_END_PAGE_OFFSET);
        return NRF_ERROR_INTERNAL;
    }

    uint16_t data_bytes_in_tag = TAG_TYPE_2_DATA_AREA_MULTIPLICATOR *
                                 buffer[TAG_TYPE_2_DATA_AREA_SIZE_OFFSET];

    if (data_bytes_in_tag + T2T_FIRST_DATA_BLOCK_OFFSET > buffer_size)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint8_t blocks_to_read = data_bytes_in_tag / T2T_BLOCK_SIZE;

    for (block_num = TAG_TYPE_2_FIRST_DATA_BLOCK_NUM;
         block_num < blocks_to_read;
         block_num += TAG_TYPE_2_BLOCKS_PER_EXCHANGE)
    {
        uint16_t offset_for_block = T2T_BLOCK_SIZE * block_num;
        err_code = adafruit_pn532_tag2_read(block_num, buffer + offset_for_block);
        if (err_code)
        {
            NRF_LOG_INFO("Failed to read blocks: %d-%d",
                         block_num,
                         block_num + T2T_END_PAGE_OFFSET);
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}


/**
 * @brief Function for analyzing data from a Type 2 Tag Platform.
 *
 * This function parses content of a Type 2 Tag Platform and prints it out.
 */
void t2t_data_analyze(uint8_t * buffer)
{
    ret_code_t err_code;

    // Static declaration of Type 2 Tag structure.
    NFC_TYPE_2_TAG_DESC_DEF(test_1, MAX_TLV_BLOCKS);
    type_2_tag_t * test_type_2_tag = &NFC_TYPE_2_TAG_DESC(test_1);

    err_code = type_2_tag_parse(test_type_2_tag, buffer);
    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("Not enough memory to read whole tag. Printing what've been read.");
    }
    else if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error during parsing a tag. Printing what could've been read.");
    }

    type_2_tag_printout(test_type_2_tag);

    tlv_block_t * p_tlv_block = test_type_2_tag->p_tlv_block_array;
    uint32_t      i;

    for (i = 0; i < test_type_2_tag->tlv_count; i++)
    {
        if (p_tlv_block->tag == TLV_NDEF_MESSAGE)
        {
            ndef_data_analyze(p_tlv_block->p_value, p_tlv_block->length);
            p_tlv_block++;
        }
    }
}


/**
 * @brief Function for reading and analyzing data from a Type 2 Tag Platform.
 *
 * This function reads content of a Type 2 Tag Platform, parses it and prints it out.
 */
ret_code_t t2t_data_read_and_analyze(nfc_a_tag_info * p_tag_info)
{
    ret_code_t     err_code;
    static uint8_t t2t_data[TAG_TYPE_2_DATA_BUFFER_SIZE]; // Buffer for tag data.

    err_code = t2t_data_read(p_tag_info, t2t_data, TAG_TYPE_2_DATA_BUFFER_SIZE);
    VERIFY_SUCCESS(err_code);

    t2t_data_analyze(t2t_data);

    return NRF_SUCCESS;
}


/**
 * @brief Function for reading and analyzing data from a Type 4 Tag Platform.
 *
 * This function reads content of a Type 4 Tag Platform, parses it and prints it out.
 */
ret_code_t t4t_data_read_and_analyze(nfc_a_tag_info * p_tag_info)
{
    ret_code_t err_code;

    // Static declaration of Type 4 Tag structure.
    NFC_T4T_CC_DESC_DEF(cc_file, MAX_TLV_BLOCKS);
    static uint8_t ndef_files_buffs[MAX_TLV_BLOCKS][TAG_TYPE_4_NDEF_FILE_SIZE];

    err_code = nfc_t4t_ndef_tag_app_select();
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Tag Application Select Procedure.");

    err_code = nfc_t4t_cc_select();
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during CC Select Procedure.");

    nfc_t4t_capability_container_t * cc_file = &NFC_T4T_CC_DESC(cc_file);
    err_code = nfc_t4t_cc_read(cc_file);
    T4T_ERROR_HANDLE(err_code, "Error (0x%X) during CC Read Procedure.");

    nfc_t4t_tlv_block_t * p_tlv_block = cc_file->p_tlv_block_array;
    uint32_t              i;

    for (i = 0; i < cc_file->tlv_count; i++)
    {
        if ((p_tlv_block->type == NDEF_FILE_CONTROL_TLV) ||
            (p_tlv_block->value.read_access == CONTROL_FILE_READ_ACCESS_GRANTED))
        {
            err_code = nfc_t4t_file_select(p_tlv_block->value.file_id);
            T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Select Procedure.");

            err_code = nfc_t4t_ndef_read(cc_file, ndef_files_buffs[i], TAG_TYPE_4_NDEF_FILE_SIZE);
            T4T_ERROR_HANDLE(err_code, "Error (0x%X) during NDEF Read Procedure.");
        }

        p_tlv_block++;
    }

    nfc_t4t_cc_file_printout(cc_file);

    p_tlv_block = cc_file->p_tlv_block_array;

    for (i = 0; i < cc_file->tlv_count; i++)
    {
        if ((p_tlv_block->type == NDEF_FILE_CONTROL_TLV) ||
            (p_tlv_block->value.file.p_content != NULL))
        {
            ndef_data_analyze(p_tlv_block->value.file.p_content + TAG_TYPE_4_NLEN_FIELD_SIZE,
                              p_tlv_block->value.file.len - TAG_TYPE_4_NLEN_FIELD_SIZE);
        }

        p_tlv_block++;
    }

    return NRF_SUCCESS;
}


/**
 * @brief Function for waiting specified time after a Tag read operation.
 */
void after_read_delay(void)
{
    ret_code_t err_code;

    // Turn off the RF field.
    err_code = adafruit_pn532_field_off();
    APP_ERROR_CHECK(err_code);
    delay(TAG_AFTER_READ_DELAY);
}

/**
 * @brief Function for detecting a Tag, identifying its Type and reading data from it.
 *
 * This function waits for a Tag to appear in the field. When a Tag is detected, Tag Platform
 * Type (2/4) is identified and appropriate read procedure is run.
 */
//ret_code_t tag_detect_and_read()
//{
    //ret_code_t     err_code;
    ////nfc_a_tag_info t_tag;

    //// Detect a NFC-A Tag in the field and initiate a communication. This function activates
    //// the NFC RF field. If a Tag is present, basic information about detected Tag is returned
    //// in tag info structure.
    //err_code = adafruit_pn532_nfc_a_target_init(mp_tag_info, TAG_DETECT_TIMEOUT);
  
    //if (err_code != NRF_SUCCESS)
    //{
    //    printf("Error NRF_ERROR_NOT_FOUND\n");
    //    return NRF_ERROR_NOT_FOUND;
    //}
    //adafruit_pn532_tag_info_printout(&m_nfc_tag);
    

    //nfc_tag_type_t tag_type = tag_type_identify(m_nfc_tag.sel_res);
    //switch (tag_type)
    //{
    //    case NFC_T2T:
    //        NRF_LOG_INFO("Type 2 Tag Platform detected. ");
    //        return t2t_data_read_and_analyze(&m_nfc_tag);

    //    case NFC_T4T:
    //        NRF_LOG_INFO("Type 4 Tag Platform detected. ");
    //        return t4t_data_read_and_analyze(&m_nfc_tag);

    //    default:
    //        printf("Tag is not type 2 nor type 4, but it is sufficient to be processed\n");
    //        return NRF_SUCCESS;
    //        //return NRF_SUCCESS;
    //}
//}








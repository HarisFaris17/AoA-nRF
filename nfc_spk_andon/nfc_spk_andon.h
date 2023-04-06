#ifndef NFC_SPK_ANDON_H_
#define NFC_SPK_ANDON_H_

#include <nrf_drv_twi.h>
#include "adafruit_pn532.h"
#include "nfc_t2t_parser.h"
#include "nfc_t4t_cc_file.h"
#include "nfc_t4t_hl_detection_procedures.h"
#include "nfc_ndef_msg_parser.h"

#include "nrf_drv_gpiote.h"
#include "main.h"

#define CALL_NULL                 255
#define CALL_ID_1                 1
#define CALL_ID_2                 2
#define CALL_ID_4                 4
#define CALL_ID_8                 8

#define CALL_STATE_NULL           0
#define CALL_STATE_CALLING        1
#define CALL_STATE_ARRIVAL        2
#define CALL_STATE_COMPLETED      3

#define BLINK_INTERVAL_LCD        500

#define DELAY_CALL                2000

#define 

ret_code_t nfc_spk_andon_init();

void call(uint8_t call_id);


#endif
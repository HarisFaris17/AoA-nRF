#include "nfc_spk_andon.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "main.h"
#include "app_timer.h"
#include "nfc_spk_ssd1306.h"

#define ACK_PACKET_SIZE                                 6

APP_TIMER_DEF(m_timer_andon);
APP_TIMER_DEF(m_timer_andon_delay);

static bool m_initialized = false;

static uint8_t m_current_call = CALL_NULL;
static uint8_t m_call_state = CALL_STATE_NULL;

static bool m_is_delay = false;

static bool call_displayed = 0;

text_t call_text_object;

static void draw_call(uint16_t color, align_horizontal_t align_horizontal, align_vertical_t align_vertical)
{
    draw_text_2(&call_text_object, color, align_horizontal, align_vertical);
    ssd1306_display();
}

static void timer_handler(void *p_context)
{
    ret_code_t err;
    if (call_displayed == 0) draw_call(BLACK, ALIGN_H_CENTER, ALIGN_V_TOP);
    

    return err;
}

static void timer_handler_delay(void *p_context)
{
    m_is_delay = false;
}

ret_code_t nfc_spk_andon_init()
{
    ret_code_t err;

    if (m_initialized) return;

    err = app_timer_create(&m_timer_andon, APP_TIMER_MODE_SINGLE_SHOT, timer_handler);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to create timer for andon %d", err);
        NRF_LOG_FLUSH();
        return err;
    }

    err = app_timer_create(&m_timer_andon_delay, APP_TIMER_MODE_SINGLE_SHOT, timer_handler_delay);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to create delay timer for andon %d", err);
        NRF_LOG_FLUSH();
        return err;
    }

    m_initialized = true;

    return NRF_SUCCESS;
}

void call(uint8_t call_id)
{
    ret_code_t err;

    if (m_is_delay) return;
    if (m_current_call != CALL_NULL && m_current_call != call_id) return;


    int text_length;
    
    uint8_t call_number;
    if (m_current_call == CALL_NULL) return;
    else if (m_current_call == CALL_ID_1) call_number = 1;
    else if (m_current_call == CALL_ID_2) call_number = 2;
    else if (m_current_call == CALL_ID_4) call_number = 3;
    else if (m_current_call == CALL_ID_8) call_number = 4;

    // get rid previous calling text
    if (m_call_state != 
    draw_call(BLACK, ALIGN_H_CENTER, ALIGN_V_TOP);

    if (m_call_state == CALL_STATE_COMPLETED || m_call_state == CALL_STATE_NULL)
    {
        call_text_object.text_length = snprintf(call_text_object.text, 10, "CALLING %d", call_number);
        m_call_state = CALL_STATE_CALLING;
    }
    
    else if (m_call_state == CALL_STATE_CALLING)
    {
        call_text_object.text_length = snprintf(call_text_object.text, 10, "ARRIVAL %d", call_number);
        m_call_state = CALL_STATE_ARRIVAL;
    }

    else if (m_call_state == CALL_STATE_ARRIVAL)
    {
        call_text_object.text_length = snprintf(call_text_object.text, 10, "COMPLETED %d", call_number);
        m_call_state = CALL_STATE_COMPLETED;
    }

    call_text_object.font = DejaVu_Serif_11;

    call_displayed = 0;

    m_is_delay = true;
    err = app_timer_start(m_timer_andon_delay, APP_TIMER_TICKS(DELAY_CALL), NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to start andon delay");
        NRF_LOG_FLUSH();
    }

    err = app_timer_start(m_timer_andon, APP_TIMER_TICKS(BLINK_INTERVAL_LCD), NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to start andon delay");
        NRF_LOG_FLUSH();
    }
}

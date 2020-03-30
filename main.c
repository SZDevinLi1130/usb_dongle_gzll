/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_power.h"

#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_kbd.h"
#include "app_usbd_dummy.h"
#include "app_error.h"
#include "bsp.h"

#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifdef DL_GZLL_TEST     //add by DL
	#include "nrf_gzll.h"
	#include "nrf_gzp.h"
	#include "nrf_ecb.h"
	#include "nrf_gzll_error.h"
	/*****************************************************************************/
/** @name Configuration  */
/*****************************************************************************/
#define PIPE_NUMBER             0  /**< Pipe 0 is used in this example. */
#define TX_PAYLOAD_LENGTH       1  /**< 1-byte payload length is used when transmitting. */

static uint8_t                  m_data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for data payload received from host. */
static uint8_t                  m_ack_payload[TX_PAYLOAD_LENGTH];                  /**< Payload to attach to ACK sent to device. */

#if GZLL_TX_STATISTICS
static nrf_gzll_tx_statistics_t m_statistics; /**< Struct containing transmission statistics. */
#endif

#define UNENCRYPTED_DATA_PIPE     2   ///< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 504 ///< RXPERIOD/2 on LU1 = timeslot period on nRF5x.
		
static  uint32_t length;
	 // Data and acknowledgement payloads
static  uint8_t payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

#endif  //DL_GZLL_TEST

#if NRF_CLI_ENABLED
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);
#endif

/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief Enable HID mouse class
 */
#define CONFIG_HAS_MOUSE    1

/**
 * @brief Enable HID keyboard class
 */
#define CONFIG_HAS_KBD      1

/**
 * @brief Mouse button count
 */
#define CONFIG_MOUSE_BUTTON_COUNT 2

/**
 * @brief Mouse speed (value sent via HID when board button is pressed).
 */
#define CONFIG_MOUSE_MOVE_STEP (3)

/**
 * @brief Mouse move repeat time in milliseconds
 */
#define CONFIG_MOUSE_MOVE_TIME_MS (5)

/**
 * @brief Letter to be sent on LETTER button
 *
 * @sa BTN_KBD_LETTER
 */
#define CONFIG_KBD_LETTER APP_USBD_HID_KBD_G

/**
 * @brief Propagate SET_PROTOCOL command to other HID instance
 */
#define PROPAGATE_PROTOCOL  0


#define LED_CAPSLOCK       (BSP_BOARD_LED_0) /**< CAPSLOCK */
#define LED_NUMLOCK        (BSP_BOARD_LED_1) /**< NUMLOCK */
#define LED_HID_REP        (BSP_BOARD_LED_2) /**< Changes its state if any HID report was received or transmitted */
#define LED_USB_START      (BSP_BOARD_LED_3) /**< The USBD library has been started and the bus is not in SUSPEND state */

#define BTN_MOUSE_X_POS    0
#define BTN_MOUSE_LEFT     1
#define BTN_KBD_SHIFT      2
#define BTN_KBD_LETTER     3

/**
 * @brief Additional key release events
 *
 * This example needs to process release events of used buttons
 */
enum {
    BSP_USER_EVENT_RELEASE_0 = BSP_EVENT_KEY_LAST + 1, /**< Button 0 released */
    BSP_USER_EVENT_RELEASE_1,                          /**< Button 1 released */
    BSP_USER_EVENT_RELEASE_2,                          /**< Button 2 released */
    BSP_USER_EVENT_RELEASE_3,                          /**< Button 3 released */
    BSP_USER_EVENT_RELEASE_4,                          /**< Button 4 released */
    BSP_USER_EVENT_RELEASE_5,                          /**< Button 5 released */
    BSP_USER_EVENT_RELEASE_6,                          /**< Button 6 released */
    BSP_USER_EVENT_RELEASE_7,                          /**< Button 7 released */
};

/**
 * @brief USB composite interfaces
 */
#define APP_USBD_INTERFACE_MOUSE 0
#define APP_USBD_INTERFACE_KBD   1

/**
 * @brief User event handler, HID mouse
 */
static void hid_mouse_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_hid_user_event_t event);

/**
 * @brief User event handler, HID keyboard
 */
static void hid_kbd_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_hid_user_event_t event);

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Global HID mouse instance
 */
APP_USBD_HID_MOUSE_GLOBAL_DEF(m_app_hid_mouse,
                              APP_USBD_INTERFACE_MOUSE,
                              NRF_DRV_USBD_EPIN1,
                              CONFIG_MOUSE_BUTTON_COUNT,
                              hid_mouse_user_ev_handler,
                              APP_USBD_HID_SUBCLASS_BOOT
);

APP_USBD_DUMMY_GLOBAL_DEF(m_app_mouse_dummy, APP_USBD_INTERFACE_MOUSE);

/**
 * @brief Global HID keyboard instance
 */
APP_USBD_HID_KBD_GLOBAL_DEF(m_app_hid_kbd,
                            APP_USBD_INTERFACE_KBD,
                            NRF_DRV_USBD_EPIN2,
                            hid_kbd_user_ev_handler,
                            APP_USBD_HID_SUBCLASS_BOOT
);
APP_USBD_DUMMY_GLOBAL_DEF(m_app_kbd_dummy, APP_USBD_INTERFACE_KBD);

/*lint -restore*/

/**
 * @brief Timer to repeat mouse move
 */
APP_TIMER_DEF(m_mouse_move_timer);


static void kbd_status(void)
{
    if(app_usbd_hid_kbd_led_state_get(&m_app_hid_kbd, APP_USBD_HID_KBD_LED_NUM_LOCK))
    {
        bsp_board_led_on(LED_NUMLOCK);
    }
    else
    {
        bsp_board_led_off(LED_NUMLOCK);
    }

    if(app_usbd_hid_kbd_led_state_get(&m_app_hid_kbd, APP_USBD_HID_KBD_LED_CAPS_LOCK))
    {
        bsp_board_led_on(LED_CAPSLOCK);
    }
    else
    {
        bsp_board_led_off(LED_CAPSLOCK);
    }
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_mouse_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_hid_user_event_t event)
{
    UNUSED_PARAMETER(p_inst);
    switch (event) {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
            /* No output report defined for HID mouse.*/
            ASSERT(0);
            break;
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
            bsp_board_led_invert(LED_HID_REP);
            break;
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
            UNUSED_RETURN_VALUE(hid_mouse_clear_buffer(p_inst));
#if PROPAGATE_PROTOCOL
            hid_kbd_on_set_protocol(&m_app_hid_kbd, APP_USBD_HID_USER_EVT_SET_BOOT_PROTO);
#endif
            break;
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
            UNUSED_RETURN_VALUE(hid_mouse_clear_buffer(p_inst));
#if PROPAGATE_PROTOCOL
            hid_kbd_on_set_protocol(&m_app_hid_kbd, APP_USBD_HID_USER_EVT_SET_REPORT_PROTO);
#endif
            break;
        default:
            break;
    }
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_kbd_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_hid_user_event_t event)
{
    UNUSED_PARAMETER(p_inst);
    switch (event) {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
            /* Only one output report IS defined for HID keyboard class. Update LEDs state. */
            bsp_board_led_invert(LED_HID_REP);
            kbd_status();
            break;
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
            bsp_board_led_invert(LED_HID_REP);
            break;
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
            UNUSED_RETURN_VALUE(hid_kbd_clear_buffer(p_inst));
#if PROPAGATE_PROTOCOL
            hid_mouse_on_set_protocol(&m_app_hid_mouse, APP_USBD_HID_USER_EVT_SET_BOOT_PROTO);
#endif
            break;
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
            UNUSED_RETURN_VALUE(hid_kbd_clear_buffer(p_inst));
#if PROPAGATE_PROTOCOL
            hid_mouse_on_set_protocol(&m_app_hid_mouse, APP_USBD_HID_USER_EVT_SET_REPORT_PROTO);
#endif
            break;
        default:
            break;
    }
}


/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_START);
            kbd_status(); /* Restore LED state - during SUSPEND all LEDS are turned off */
            break;
        case APP_USBD_EVT_STARTED:
            bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}


static void mouse_move_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    UNUSED_RETURN_VALUE(app_usbd_hid_mouse_x_move(&m_app_hid_mouse, CONFIG_MOUSE_MOVE_STEP));
}

static void bsp_event_callback(bsp_event_t ev)
{
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_X_POS):
            UNUSED_RETURN_VALUE(app_usbd_hid_mouse_x_move(&m_app_hid_mouse, CONFIG_MOUSE_MOVE_STEP));
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_X_POS):
            UNUSED_RETURN_VALUE(app_timer_stop(m_mouse_move_timer));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_LEFT):
            UNUSED_RETURN_VALUE(app_usbd_hid_mouse_button_state(&m_app_hid_mouse, 0, true));
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_LEFT):
            UNUSED_RETURN_VALUE(app_usbd_hid_mouse_button_state(&m_app_hid_mouse, 0, false));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_KBD_SHIFT):
            UNUSED_RETURN_VALUE(app_usbd_hid_kbd_modifier_state_set(&m_app_hid_kbd, APP_USBD_HID_KBD_MODIFIER_LEFT_SHIFT, true));
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_KBD_SHIFT):
            UNUSED_RETURN_VALUE(app_usbd_hid_kbd_modifier_state_set(&m_app_hid_kbd, APP_USBD_HID_KBD_MODIFIER_LEFT_SHIFT, false));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_KBD_LETTER):
            UNUSED_RETURN_VALUE(app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, true));
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_KBD_LETTER):
            UNUSED_RETURN_VALUE(app_usbd_hid_kbd_key_control(&m_app_hid_kbd, CONFIG_KBD_LETTER, false));
            break;

        default:
            return; // no implementation needed
    }
}

/**
 * @brief Auxiliary internal macro
 *
 * Macro used only in @ref init_bsp to simplify the configuration
 */
#define INIT_BSP_ASSIGN_RELEASE_ACTION(btn)                      \
    APP_ERROR_CHECK(                                             \
        bsp_event_to_button_action_assign(                       \
            btn,                                                 \
            BSP_BUTTON_ACTION_RELEASE,                           \
            (bsp_event_t)CONCAT_2(BSP_USER_EVENT_RELEASE_, btn)) \
    )

static void init_bsp(void)
{
    ret_code_t ret;
    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(ret);

    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_X_POS);
    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_LEFT );
    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_KBD_SHIFT  );
    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_KBD_LETTER );

    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}

#if NRF_CLI_ENABLED
static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}
#endif


#ifdef DL_GZLL_TEST

#if 0
#if GZLL_PA_LNA_CONTROL

#define GZLL_PA_LNA_TIMER       CONCAT_2(NRF_TIMER, GZLL_PA_LNA_TIMER_NUM) /**< Convert timer number into timer struct. */

/**< PA/LNA structure configuration. */
static nrf_gzll_pa_lna_cfg_t m_pa_lna_cfg = {
    .lna_enabled        = GZLL_LNA_ENABLED,
    .pa_enabled         = GZLL_PA_ENABLED,
    .lna_active_high    = GZLL_LNA_ACTIVE_HIGH,
    .pa_active_high     = GZLL_PA_ACTIVE_HIGH,
    .lna_gpio_pin       = GZLL_PA_LNA_CRX_PIN,
    .pa_gpio_pin        = GZLL_PA_LNA_CTX_PIN,
    .pa_gpiote_channel  = GZLL_PA_LNA_TX_GPIOTE_CHAN,
    .lna_gpiote_channel = GZLL_PA_LNA_RX_GPIOTE_CHAN,
    .timer              = GZLL_PA_LNA_TIMER,
    .ppi_channels[0]    = GZLL_PA_LNA_PPI_CHAN_1,
    .ppi_channels[1]    = GZLL_PA_LNA_PPI_CHAN_2,
    .ppi_channels[2]    = GZLL_PA_LNA_PPI_CHAN_3,
    .ppi_channels[3]    = GZLL_PA_LNA_PPI_CHAN_4,
    .ramp_up_time       = GZLL_PA_LNA_RAMP_UP_TIME
};

static int32_t m_rssi_sum    = 0; /**< Variable used to calculate average RSSI. */
static int32_t m_packets_cnt = 0; /**< Transmitted packets counter. */
#endif

/**
 * @brief Function to read the button state.
 *
 * @return Returns states of the buttons.
 */
static uint8_t input_get(void)
{
    uint8_t result = 0;
    for (uint32_t i = 0; i < BUTTONS_NUMBER; i++)
    {
        if (bsp_button_is_pressed(i))
        {
            result |= (1 << i);
        }
    }

    return ~(result);
}


/**
 * @brief Function to control the LED outputs.
 *
 * @param[in] val Desirable state of the LEDs.
 */
static void output_present(uint8_t val)
{
    uint32_t i;

    for (i = 0; i < LEDS_NUMBER; i++)
    {
        if (val & (1 << i))
        {
            bsp_board_led_on(i);
        }
        else
        {
            bsp_board_led_off(i);
        }
    }
}


/**
 * @brief Initialize the BSP modules.
 */
/*
static void ui_init(void)
{
    uint32_t err_code;

    // Initialize application timer.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // BSP initialization.
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Gazell ACK payload example. Host mode.");
    NRF_LOG_FLUSH();

    bsp_board_init(BSP_INIT_LEDS);
}
*/

/*****************************************************************************/
/** @name Gazell callback function definitions.  */
/*****************************************************************************/
/**
 * @brief RX data ready callback.
 *
 * @details If a data packet was received, the first byte is written to LEDS.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    // Pop packet and write first byte of the payload to the GPIO port.
    bool result_value = nrf_gzll_fetch_packet_from_rx_fifo(pipe,
                                                           m_data_payload,
                                                           &data_payload_length);

    if (!result_value)
    {
        NRF_LOG_ERROR("RX fifo error ");
    }

    if (data_payload_length > 0)
    {
        output_present(m_data_payload[0]);
    }

    // Read buttons and load ACK payload into TX queue.
    m_ack_payload[0] = input_get(); // Button logic is inverted.

    result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_ack_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
    }

#if GZLL_PA_LNA_CONTROL
    m_rssi_sum += rx_info.rssi;
    m_packets_cnt++;
#endif
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}

#if GZLL_PA_LNA_CONTROL
/**
 * @brief Function for configuring front end control in Gazell.
 */
static bool front_end_control_setup(void)
{
    bool result_value = true;

    // Configure pins controlling SKY66112 module.
    nrf_gpio_cfg_output(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CSD_PIN);

    // Turn on front end module.
    nrf_gpio_pin_clear(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_pin_set(GZLL_PA_LNA_CSD_PIN);

    // PA/LNA configuration must be called after @ref nrf_gzll_init() and before @ref nrf_gzll_enable()
    result_value = nrf_gzll_set_pa_lna_cfg(&m_pa_lna_cfg);

    return result_value;
}
#endif


static void gzll_init(void)
{		
	   // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

#if GZLL_PA_LNA_CONTROL
    // Initialize external PA/LNA control.
    result_value = front_end_control_setup();
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

#if GZLL_TX_STATISTICS
    // Turn on transmission statistics gathering.
    result_value = nrf_gzll_tx_statistics_enable(&m_statistics);
    GAZELLE_ERROR_CODE_CHECK(result_value);
#endif

    // Load data into TX queue.
    m_ack_payload[0] = input_get();

    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        NRF_LOG_ERROR("TX fifo error ");
        NRF_LOG_FLUSH();
    }

    // Enable Gazell to start sending over the air.
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);
    
    NRF_LOG_INFO("Gzll ack payload host example started.");	
}
#endif

static void gzll_pairing_init(void)
{
	 // Debug helper variables
  //  uint32_t length;

    // Data and acknowledgement payloads
   // uint8_t payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

    // Set up the user interface (buttons and LEDs)
   // ui_init();

    // Initialize the Gazell Link Layer
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    result_value = nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2); // Half RX period on an nRF24Lxx device
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Initialize the Gazell Pairing Library
    gzp_init();
    result_value = nrf_gzll_set_rx_pipes_enabled(nrf_gzll_get_rx_pipes_enabled() |
                                                 (1 << UNENCRYPTED_DATA_PIPE));
    GAZELLE_ERROR_CODE_CHECK(result_value);

    gzp_pairing_enable(true);

    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);

    NRF_LOG_INFO("Gazell dynamic pairing example started. Host mode.");
    NRF_LOG_FLUSH();


	
}

static void exe_gzll_pairing(void)
{
			gzp_host_execute();
        // If a Host ID request received
        if (gzp_id_req_received())
        {
            // Always grant a request
            gzp_id_req_grant();
        }

        length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

        if (nrf_gzll_get_rx_fifo_packet_count(UNENCRYPTED_DATA_PIPE))
        {
            if (nrf_gzll_fetch_packet_from_rx_fifo(UNENCRYPTED_DATA_PIPE, payload, &length))
            {
                //output_present(payload[0]);
            }
        }
        else if (gzp_crypt_user_data_received())
        {
            if (gzp_crypt_user_data_read(payload, (uint8_t *)&length))
            {
                //output_present(payload[0]);
            }
        }	
}

#endif  //DL_GZLL_TEST


int main(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler,
    };

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    ret = app_timer_create(&m_mouse_move_timer, APP_TIMER_MODE_REPEATED, mouse_move_timer_handler);
    APP_ERROR_CHECK(ret);

    init_bsp();

#if NRF_CLI_ENABLED
    init_cli();
#endif

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_mouse;
#if CONFIG_HAS_MOUSE
    class_inst_mouse = app_usbd_hid_mouse_class_inst_get(&m_app_hid_mouse);
#else
    class_inst_mouse = app_usbd_dummy_class_inst_get(&m_app_mouse_dummy);
#endif
    ret = app_usbd_class_append(class_inst_mouse);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_kbd;
#if CONFIG_HAS_KBD
    class_inst_kbd = app_usbd_hid_kbd_class_inst_get(&m_app_hid_kbd);
#else
    class_inst_kbd = app_usbd_dummy_class_inst_get(&m_app_kbd_dummy);
#endif
    ret = app_usbd_class_append(class_inst_kbd);
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("USBD HID composite example started.");
    
    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

    gzll_pairing_init();
    while (true)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
#if NRF_CLI_ENABLED
        nrf_cli_process(&m_cli_uart);
#endif
		exe_gzll_pairing();		
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
    }
}

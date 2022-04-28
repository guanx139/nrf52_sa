/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
#include <string.h>
#include "ble_sas.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_saadc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//For this program, the DC/DC convetor is not enable because of missing external circuitry,
//Also, the internal RC is used for the LF clock. To use external crystal, set the following parameter in the sdk_config.h
//#ifndef NRF_SDH_CLOCK_LF_SRC
//#define NRF_SDH_CLOCK_LF_SRC 1 // 0 for RC
//#endif
//
//// <o> NRF_SDH_CLOCK_LF_RC_CTIV - SoftDevice calibration timer interval. 
//#ifndef NRF_SDH_CLOCK_LF_RC_CTIV
//#define NRF_SDH_CLOCK_LF_RC_CTIV 0 //16 for RC
//#endif
//
//// <o> NRF_SDH_CLOCK_LF_RC_TEMP_CTIV - SoftDevice calibration timer interval under constant temperature. 
//// <i> How often (in number of calibration intervals) the RC oscillator shall be calibrated
//// <i>  if the temperature has not changed.
//
//#ifndef NRF_SDH_CLOCK_LF_RC_TEMP_CTIV
//#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 0 //2 for RC
//#endif
//
//// <o> NRF_SDH_CLOCK_LF_ACCURACY  - External clock accuracy used in the LL to compute timing.
// 
//// <0=> NRF_CLOCK_LF_ACCURACY_250_PPM 
//// <1=> NRF_CLOCK_LF_ACCURACY_500_PPM 
//// <2=> NRF_CLOCK_LF_ACCURACY_150_PPM 
//// <3=> NRF_CLOCK_LF_ACCURACY_100_PPM 
//// <4=> NRF_CLOCK_LF_ACCURACY_75_PPM 
//// <5=> NRF_CLOCK_LF_ACCURACY_50_PPM 
//// <6=> NRF_CLOCK_LF_ACCURACY_30_PPM 
//// <7=> NRF_CLOCK_LF_ACCURACY_20_PPM 
//// <8=> NRF_CLOCK_LF_ACCURACY_10_PPM 
//// <9=> NRF_CLOCK_LF_ACCURACY_5_PPM 
//// <10=> NRF_CLOCK_LF_ACCURACY_2_PPM 
//// <11=> NRF_CLOCK_LF_ACCURACY_1_PPM 
//
//#ifndef NRF_SDH_CLOCK_LF_ACCURACY
//#define NRF_SDH_CLOCK_LF_ACCURACY 7 // 1 for RC
//#endif


#define DEVICE_NAME                     "SleepApneaService"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

//Might need to do more testing on this, like current profile
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(1650, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1850, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */
#define TX_POWER                        -16
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



//############### Start ###############//
#define SAS_TIMER_INTERVAL              APP_TIMER_TICKS(1750)                   // 1850 ms intervals
#define POWER_BUTTON_PRESS_TIME         APP_TIMER_TICKS(3000)                   // 3s 
#define LED_OFF_TIME                    APP_TIMER_TICKS(1000)                   // 1s 
#define DEVICE_INACTIVE_TIME          480                                     // 120 second, in the unit of 0.25s
#define POWER_BUTTON                    20                                      /**< Pin number for the reset button for waking up the device from system OFF */
//#define POWER_BUTTON                    28                                      /**< Pin number for the reset button for waking up the device from system OFF, for redBear board, Pin D2 */
//#define SIGNAL_LED                      12                                      /**< Pin number for the signaling LED , pin 11 is for the redBear nano board*/
#define SIGNAL_LED                      11                                      /**< Pin number for the signaling LED, redBear Board only */
#define BUTTON_PRESSED                  0
#define BUTTON_RELEASED                 1

//ADC Macro
#define ADC_BUF_SIZE  7                           //ADC buffer size
#define RTC_FREQUENCY 32                          //Determines the RTC frequency and prescaler, f_rtc = 32768Hz / (31 + 1) = 1024Hz
#define RTC_CC_VALUE  8                          //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency, f_adc = f_rtc / RTC_CC = 1024/256 = 4Hz
#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 0                        //Set to 1 to enable BURST mode, otherwise set to 0.
#define ADC_BUF_FULL 1
#define ADC_BUF_NOT_FULL 0

//SA detection Macro
//The following two macros are determined experimentally
#define RESP_THRESHOLD 100 // The threshold of the presence of respitory signal
#define RESP_AVG 1900 // The average respitory signal when no breath is detected
#define SA_THRESHOLD 40 // The threshold for sa detection, the number here corrsponds ADC sample, each sample = 250 ms

//#define UART_PRINTING_ENABLED 1

//ADC Functions
static void saadc_init();
static uint16_t get(nrf_saadc_value_t ary[][SAADC_SAMPLES_IN_BUFFER]);
static void put(uint16_t value, uint16_t ary[]);
const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint16_t                ADC_BUF[ADC_BUF_SIZE];
static uint16_t ADC_BUF_INDEX           = 0;
static bool buffer_status = 0;
static int counter                      = 0;// for debugging only

//Timer functions
static void timers_init(void);
static void application_timers_start(void);
static void power_button_timer_start(void);
static void signal_led_timer_start(void);
static void sas_char_timer_timeout_handler(void * p_context);
static void power_button_timer_timeout_handler(void* p_context);
static void signal_led_timer_timeout_handler(void * p_contex);

//SA detection variable and function
// counter for the number non breathing samples
static uint16_t sa_counter = 0;
// sa indication flag
static bool is_sa =  false;
// true when the device is ready to monitor sa events
// set to true with first sample is breathing sample
static bool ready = false;
// the messgae sent together with the 7 samples indicate the sa_event
// lower 7 bits are used to indication the presence of sa, 1 -> is_sa = true.
static uint16_t sa_ble_message = 0;

// this function is responsible for checking each sample, and based on the sa_conter to determine if sa occurs
static void check_sa(uint16_t adc_val){
  // set ready when a relatively large breath is detected.
  if(abs(adc_val - RESP_AVG) > 100){
    ready = true;
  }
  // check if the difference between adc_val and RESP_AVG is less that the threshold when device is ready
  if((abs(adc_val - RESP_AVG) <= RESP_THRESHOLD) && ready){
    sa_counter ++;
  }
  // otherwise, reset counter and is_sa flag
  else if (ready){
    sa_counter = 0;
    is_sa = false;
    //sa_ble_message = 0;
    //nrf_drv_gpiote_out_clear(SIGNAL_LED); // turn off led
  }
  // when counter values is larger that SA_THRESHOLD, set is_sa flag
  if(sa_counter >= SA_THRESHOLD){
    is_sa = true;
    //nrf_drv_gpiote_out_set(SIGNAL_LED); //turn on led
    // left shift by 1, and toggle the lsb to 1
    sa_ble_message = (sa_ble_message << 1) ^ 0x0001;
  }
  else{
    // left shift by 1
    sa_ble_message = (sa_ble_message << 1);
  }
  // for light up the led is sa is detected, this can be replaced by external circuit 
  if (is_sa){
    nrf_drv_gpiote_out_set(SIGNAL_LED); //turn on led
  }
  else{
    nrf_drv_gpiote_out_clear(SIGNAL_LED); // turn off led
  }
}


/**@brief Function for setting the radio tx power

 * @detail this will set the radio power for pheripheral mode

 * @param[in] int8_t tx power, supported tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
 */
static void set_radio_power(int8_t tx_power);

/**@brief Function for checking if the device is active, ie, does it detect breath?

 * @param[in] uint16_t adc_val
 */
static void is_device_active(uint16_t val, uint16_t buffer[ADC_BUF_SIZE]);

static void sleep_mode_enter(void);

//############### End ###############//

       
#ifdef UART_PRINTING_ENABLED
static uint32_t                m_adc_evt_counter = 0;
#endif //UART_PRINTING_ENABLED

//############### Start ###############//

static bool                    m_saadc_initialized = false;    

APP_TIMER_DEF(m_sas_char_timer_id);
APP_TIMER_DEF(m_power_button_timer_id);
APP_TIMER_DEF(m_signal_led_timer_id);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

// STEP 1: Declare a ble_os_t service structure for our application

ble_sas_t m_sas;  //sas structure for the application.
static bool current_state = 0;



// STEP 5: Declare variable holding our service UUID
static ble_uuid_t m_adv_uuids[] = 
{
    {
        BLE_UUID_SAS_SERVICE,
        BLE_UUID_TYPE_VENDOR_BEGIN
    }
};

//############### End ###############//

static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//############### Start ###############//

//Timer module
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sas_char_timer_id, APP_TIMER_MODE_REPEATED, sas_char_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_power_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, power_button_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
     
    err_code = app_timer_create(&m_signal_led_timer_id, APP_TIMER_MODE_SINGLE_SHOT, signal_led_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for starting timers for updating the charteristic value.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_sas_char_timer_id, SAS_TIMER_INTERVAL, NULL);

    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}

/**@brief Function for starting timers for power button.
 */
static void power_button_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_power_button_timer_id, POWER_BUTTON_PRESS_TIME, NULL);

    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}

/**@brief Function for starting timers for signaling led.
 */
static void signal_led_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_signal_led_timer_id,LED_OFF_TIME, NULL);
}

/**@brief Function will be called when power_button_timer timeout.
 */
static void power_button_timer_timeout_handler(void* p_context){
    //Sense the button state, if it equals to BUTTON_PRESS, enter sleep mode
    if(nrf_gpio_pin_read(POWER_BUTTON) == BUTTON_PRESSED){
        //sleep_mode_enter();
        nrf_drv_gpiote_out_set(SIGNAL_LED); //turn on led
        nrf_delay_ms(1000);
        nrf_drv_gpiote_out_clear(SIGNAL_LED);
        sleep_mode_enter();
    }
}


/**@brief Function will be called with sas_char_timer timeout and it will update the char value.
 */
static void sas_char_timer_timeout_handler(void * p_context){
    int i = 0;
    uint16_t ble_msg [ADC_BUF_SIZE+1];
    if(buffer_status == ADC_BUF_FULL){
        for(i = 0; i < ADC_BUF_SIZE; i++){
            ble_msg [i] = ADC_BUF[i];
        }
        ble_msg [i] = sa_ble_message;
        sas_data_characteristic_update(&m_sas, ble_msg);
        buffer_status = ADC_BUF_NOT_FULL;
        // sa_ble_message = 0;
    }
    //if(true){
        // sa indication LED, in this case we light up the signal LED, but
        // it can use other gpio to enable the therapy
        //nrf_drv_gpiote_out_set(SIGNAL_LED); //turn on led
        //nrf_delay_ms(300);
        //nrf_drv_gpiote_out_clear(SIGNAL_LED);
        //nrf_delay_ms(300);
        //nrf_drv_gpiote_out_set(SIGNAL_LED); //turn on led
        //nrf_delay_ms(300);
        //nrf_drv_gpiote_out_clear(SIGNAL_LED);
    //}
}

/**@brief Function will be called when signal_led_timer timeout.
 */
static void signal_led_timer_timeout_handler(void * p_contex){
     nrf_drv_gpiote_out_clear(SIGNAL_LED); //turn off led

}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    //prepare wake up button
    nrf_gpio_cfg_sense_input(POWER_BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_delay_ms(3000);
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

//############### End ###############//


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;
        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
	
	// STEP 2: Add code to initialize the services used by the application.
    ble_sas_init(&m_sas);
	
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            set_radio_power(TX_POWER);
            //when the device is connected to cellphone, the rtc is enable
            nrf_drv_rtc_enable(&rtc); 

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt, (void*) &m_sas);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	
	// STEP 6: Declare and instantiate the scan response

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

//############### Start ###############//

/**@brief Function for handling the interupt of the power button, it will start the timer for the power button.
 */
static void power_button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
    if (nrf_gpio_pin_read(POWER_BUTTON) == BUTTON_PRESSED){
        power_button_timer_start();
    }
}

/**@brief Function for configuring the gpio
 */
static void gpio_config(void){

    nrf_drv_gpiote_init(); //Init gpiote, it is for async. event
    //init LED
    nrf_drv_gpiote_out_config_t led_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
    nrf_drv_gpiote_out_init(SIGNAL_LED, &led_config);
    //Turn off the LED
    nrf_drv_gpiote_out_clear(SIGNAL_LED);
    
    //init power button
    nrf_drv_gpiote_in_config_t button_config=GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    button_config.pull = NRF_GPIO_PIN_PULLUP;//enable pull up resistor
    nrf_drv_gpiote_in_init(POWER_BUTTON, &button_config, power_button_handler);
    nrf_drv_gpiote_in_event_enable(POWER_BUTTON, true);

}

//SSADC Thingy - rtc - RTC Method

/**@brief Function will be called rtc overflow, and it will trigger SAADC.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
	
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if(!m_saadc_initialized)
        {
            saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        m_saadc_initialized = true;                                    //Set SAADC as initialize
        if(buffer_status == ADC_BUF_NOT_FULL){
            static uint16_t adc_val = 0;
            nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
            //adc_val++; //debugging only
            adc_val = get(m_buffer_pool);
            //check for sa
            check_sa(adc_val);
            //is_device_active(adc_val, ADC_BUF);
            put(adc_val, ADC_BUF);
        }
			
        err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
    }
}

/**@brief Function for confogurating lfclk.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for confogurating rtc.
 */
static void rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);              //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);           //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
//    nrf_drv_rtc_enable(&rtc);                                          //Enable RTC
}

/**@brief Function will be called after SAADC is triggered.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    //NRF_LOG_INFO("SAADC callback\r\n");
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;
			
        //LEDS_INVERT(BSP_LED_1_MASK);                                                                    //Toggle LED2 to indicate SAADC buffer full		
     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        //err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_BUF_SIZE);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"

        APP_ERROR_CHECK(err_code);
#ifdef UART_PRINTING_ENABLED
        NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);                                      //Print the event number on UART
        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);                                           //Print the SAADC result on UART
        }
        m_adc_evt_counter++;
#endif //UART_PRINTING_ENABLED				
				
        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                              //Set SAADC as uninitialized
    }
}

/**@brief Function for initializing the SAADC.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0;
    //nrf_saadc_channel_config_t channel_config1;

	
    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config0.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config0.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config0.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config0.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config0.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config0.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config0.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config0.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

    //channel_config1.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    //channel_config1.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    //channel_config1.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    //channel_config1.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    //channel_config1.pin_p = NRF_SAADC_INPUT_AIN1;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    //channel_config1.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    //channel_config1.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    //channel_config1.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);                            //Initialize SAADC channel 0 with the channel configuration
//    APP_ERROR_CHECK(err_code);

		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for getting the value from the adc buffer poolm

 * @param[in] nrf_saadc_value_t ary[][SAADC_SAMPLES_IN_BUFFER]

 * @param[out] uint16_t adc value
 */
static uint16_t get(nrf_saadc_value_t ary[][SAADC_SAMPLES_IN_BUFFER]){
    static uint16_t first_index = 0;
    uint16_t adc_value = (uint16_t)ary[first_index][SAADC_SAMPLES_IN_BUFFER - 1];
    return adc_value;
}

/**@brief Function will put a value into the ADC array

 * @param[in] uint16_t value

 * @param[in] uint16_t ary[], this is the ADC array

 */
static void put(uint16_t value, uint16_t ary[]){
    ary[ADC_BUF_INDEX] = value;
    ADC_BUF_INDEX++;
    if(ADC_BUF_INDEX > ADC_BUF_SIZE - 1 && buffer_status == ADC_BUF_NOT_FULL){
        buffer_status = ADC_BUF_FULL;
        ADC_BUF_INDEX = 0;
    }
}


static void set_radio_power(int8_t tx_power){
    ret_code_t err_code;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_power);
    APP_ERROR_CHECK(err_code);
}

static void is_device_active(uint16_t val, uint16_t buffer[ADC_BUF_SIZE]){
    int difference = 0;
    difference = val - buffer[ADC_BUF_INDEX];
    if(difference >= 100 || difference <= -100){
        counter = 0;
    }
    else{
        counter++;
        if(counter >= DEVICE_INACTIVE_TIME){
            sleep_mode_enter();
        }
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    //NRF_POWER->DCDCEN = 1;                           //Enabling the DCDC converter for lower current consumption
    // Initialize.
    log_init();
    timers_init();
    //buttons_leds_init(&erase_bonds);

    lfclk_config();                                  //Configure low frequency 32kHz clock
    rtc_config();                                    //Configure RTC. The RTC will generate periodic interrupts. Requires 32kHz clock to operate.
//
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    gpio_config();

    // Start execution.
    NRF_LOG_INFO("started.");
    application_timers_start();
//    nrf_drv_rtc_enable(&rtc); 
    counter = 0;
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

//############### End ###############//

/**
 * @}
 */
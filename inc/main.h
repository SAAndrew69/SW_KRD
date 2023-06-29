#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

// #include "sdk_config.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_sdh.h"
#include "nrf_delay.h"
#include "nrf_uart.h"

#include "nrfx_spi.h"
#include "nrfx_spim.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_rtc.h"

#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"

#include "bsp_btn_ble.h"

#include "app_timer.h"
#include "app_uart.h"

#include "acquire.h"
#include "datetime.h"

#include "nrf_gpio.h"

#include "ads129x.h"

#define DEVICE_NAME                     "CARDIOGLB_18"                    /* Name of device. Will be included in the advertising data.                                       */

#if defined(__CROSSWORKS_ARM) || defined(__SES_ARM)
  #define DEVICE_SERIAL_NO              "SPG-021-%08X-%08X"
#else
  #define DEVICE_SERIAL_NO              "SPG-020-%08lX-%08lX"
#endif

#define DEVICE_FIRMWARE_VERSION         "0.0.5"

#define APP_BLE_CONN_CFG_TAG            1                                 /* A tag identifying the SoftDevice BLE configuration.                                             */
#define APP_BLE_OBSERVER_PRIO           3                                 /* Application's BLE observer priority. You shouldn't need to modify this value.                   */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN        /* UUID type for the Nordic UART Service (vendor specific).                                        */

#define APP_ADV_INTERVAL                64                                /* The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).               */
#define APP_ADV_DURATION                18000                             /* The advertising duration (180 seconds) in units of 10 milliseconds.                             */

#define DEFAULT_CONN_INTERVAL           (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))

//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)   /* Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units.       */
#define MIN_CONN_INTERVAL               (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))
#define MAX_CONN_INTERVAL               (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))   /* Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units.         */
#define SLAVE_LATENCY                   0                                 /* Slave latency.                                                                                  */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)   /* Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.               */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(15000)            /* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)            /* Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).       */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                 /* Number of attempts before giving up the connection parameter negotiation.                       */

#define UART_TX_BUF_SIZE                256                               /* UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                               /* UART RX buffer size. */

#define DEAD_FACE                       0xDEADFACE                        /* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SPI_INSTANCE                    0                                 /* SPI instance index.                                                                             */

#define SPI_SCK_PIN                     NRF_GPIO_PIN_MAP(0, 10)           /* SCK pin          - P0.10 */
#define SPI_MOSI_PIN                    NRF_GPIO_PIN_MAP(0, 9)            /* MOSI pin         - P0.09 */
#define SPI_MISO_PIN                    NRF_GPIO_PIN_MAP(1, 6)            /* MISO pin         - P1.06 */
#define SPI_SS_PIN                      NRF_GPIO_PIN_MAP(1, 4)            /* CS pin           - P1.04 */
//#define SPI_SS_PIN                      NRFX_SPIM_PIN_NOT_USED

//#define ADS_START_PIN                   NRF_GPIO_PIN_MAP(1, 14)
#define ADS_DATA_READY_PIN              NRF_GPIO_PIN_MAP(1, 10)           /* ADC_nDRDY pin    - P1.10 */
#define ADS_POWER_DOWN_PIN              NRF_GPIO_PIN_MAP(1, 11)           /* ADC_nPWDN pin    - P1.11 */
#define ADS_RESET_PIN                   NRF_GPIO_PIN_MAP(1, 00)           /* ADC_nRST  pin    - P1.00 */

#define PWR_HOLD                        NRF_GPIO_PIN_MAP(0, 1)
#define VDDA_SWITCH                     NRF_GPIO_PIN_MAP(1, 9)            /* 'ANA PWR ON' pin - P1.09 */
#define VIBRATION_MOTOR_SWITCH          NRF_GPIO_PIN_MAP(0, 00)           /* 'MOTOR' pin      - P0.00 */


#define TWI_INSTANCE_ID                 1                                 /* TWI instance ID. */
#if 0
#define NRF_TWI_SCL_PIN                 27
#define NRF_TWI_SDA_PIN                 26
#else
#define NRF_TWI_SCL_PIN                 13
#define NRF_TWI_SDA_PIN                 20
#endif

//#define LED_PIN                         NRF_GPIO_PIN_MAP(0, 25)           /* LED pin - P0.25 */

#define UNIX_TIME_DELTA                 60

#define OFF                             0
#define ON                              (!OFF)

#define YES                             ON
#define NO                              OFF

#if !defined(USE_ADC)
  #define USE_ADC                       YES
#endif

typedef enum {
  ACQUIRING_INACTIVE = 0,
  ACQUIRING_REQUESTED,
  ACQUIRING_STARTED,
  ACQUIRING_ACTIVE,
  ACQUIRING_IN_PROGRESS,
  ACQUIRING_DATA_READY,
  ACQUIRING_COMPLETE
} acquiring_state_t;

#define ADC_CORRECTION_FACTOR           +75

__STATIC_INLINE void init(void);
__STATIC_INLINE void handle_idle_state(void);

static int string_to_binary(uint8_t const *input, uint8_t *output);

__STATIC_INLINE unsigned ads_finish_acquiring(const void *data, ads_convert_t conv_type, unsigned status_word_req);
__STATIC_INLINE void ads_start_acquiring(void);

__STATIC_FORCEINLINE acquiring_state_t get_acquiring_state(void);
__STATIC_FORCEINLINE void set_acquiring_state(acquiring_state_t s);

#ifdef __cplusplus
}
#endif

#endif


#include "main.h"


// <o> NRF_BLE_SCAN_SCAN_PHY  - PHY to scan on.
 
// <0=> BLE_GAP_PHY_AUTO 
// <1=> BLE_GAP_PHY_1MBPS 
// <2=> BLE_GAP_PHY_2MBPS 
// <4=> BLE_GAP_PHY_CODED 
// <255=> BLE_GAP_PHY_NOT_SET 

#ifndef NRF_BLE_SCAN_SCAN_PHY
#define NRF_BLE_SCAN_SCAN_PHY 2
#endif


#define TEST_DATA_LENGTH 244
#define BLOCKS_TO_SEND 1000UL

static volatile uint32_t test_req;
static union {
  uint8_t test_data[TEST_DATA_LENGTH];
  uint32_t id;
} x;

static uint32_t ble_test_output(uint8_t *s);

int main(void) {

  init();

  NRF_TIMER3->BITMODE = 3;                /* set 32-bit mode for TIMER3 */

  for (;;) { /*** Endless loop ****/
    
    handle_idle_state();

    if (test_req != 0) {

      NRF_LOG_INFO("Transmission test started.");
      NRF_LOG_FLUSH();
      NRF_TIMER3->TASKS_CLEAR = 1;
      NRF_TIMER3->TASKS_START = 1;

      for (uint32_t i = 0; i < BLOCKS_TO_SEND; i++) {
        x.id = i;
        ble_test_output(x.test_data);
      };

      NRF_TIMER3->TASKS_CAPTURE[0] = 1;
      NRF_TIMER3->TASKS_STOP = 1;

      NRF_LOG_INFO("Transmission completed. %u bytes sent in %u us.", (BLOCKS_TO_SEND * TEST_DATA_LENGTH), NRF_TIMER3->CC[0]);
      NRF_LOG_INFO("Transmission speed: %u cps.", (BLOCKS_TO_SEND * TEST_DATA_LENGTH) / (NRF_TIMER3->CC[0] / 1000000UL));

      test_req = 0;
    }
  }
}


__STATIC_INLINE void enable_vdd(void) {

  /* Turn ON VDD (digital) */

  #if 0
    nrf_gpio_pin_clear(uint32_t pin_number);
    nrf_gpio_pin_set(uint32_t pin_number);
  #endif
}


__STATIC_INLINE void disable_vdd(void) {

  /* Turn OFF VDD (digital) */

  #if 0
    nrf_gpio_pin_clear(uint32_t pin_number);
    nrf_gpio_pin_set(uint32_t pin_number);
  #endif
}


__STATIC_INLINE void enable_vdda(void) {

  /* Turn ON VDDA (analog) */

  #if 0
    nrf_gpio_pin_clear(uint32_t pin_number);
    nrf_gpio_pin_set(uint32_t pin_number);
  #endif
}


__STATIC_INLINE void disable_vdda(void) {

  /* Turn OFF VDDA (analog) */

  #if 0
    nrf_gpio_pin_clear(uint32_t pin_number);
    nrf_gpio_pin_set(uint32_t pin_number);
  #endif
}


__STATIC_INLINE void enter_reg_mode(void) {

  /* Turn On digital and analog VDDs, wait for incoming commands  */

}


__STATIC_INLINE void enter_power_down_mode(void) {
  /*

    Set pins as follow:

      P0.06 output 0 vibration motor
      P0.08 output 0 VDDA
      P0.09 output 0 VDD
      P0.14 output 0 Segment of LED bar
      P0.15 output 0 Segment of LED bar
      P0.16 output 0 Segment of LED bar
      P0.17 output 0 Segment of LED bar
      P0.19 output 1 I2C_SCL
      P0.20 output 1 I2C_SDA
      P0.22 output 0 R terminal of RGB LED
      P0.23 output 0 G terminal of RGB LED
      P0.24 output 0 B terminal of RGB LED
      P0.30 output 1 UART TX
      P1.01 output 1 ADC_nCS
      P1.02 output 0 SPI_SCLK
      P1.04 output 0 SPI_SDOUT
      P1.05 output 0 ADC_nPWDN
      P1.06 output 0 ADC_nRST
      P1.07 output 0 ADC_START
      P1.10 output 1 QSPI_FLASH_CS
      P1.11 input/output QSPI_FLASH_DQ0
      P1.12 input/output QSPI_FLASH_DQ1
      P1.13 input/output QSPI_FLASH_DQ2
      P1.14 input/output QSPI_FLASH_DQ3
      P1.15 output 0 QSPI_FLASH_CLK

  */
}


const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /* The instance of nrf_drv_rtc for RTC2. */


__STATIC_INLINE void vibrate(uint8_t state) {

  /* Start/Stop vibration motor */

  if (state != 0) {

    NRF_LOG_INFO("vibration started.");

    NRF_RTC2->EVENTS_COMPARE[1] = 0;
    nrfx_err_t err_code = nrfx_rtc_cc_set(&rtc, 1, nrf_rtc_counter_get(rtc.p_reg) + 8, true);
    APP_ERROR_CHECK(err_code);

  } else {

    NRF_LOG_INFO("vibration stopped.");

    nrfx_err_t err_code = nrfx_rtc_cc_disable(&rtc, 1);
    APP_ERROR_CHECK(err_code);

  }
}


__STATIC_INLINE void enter_wait_mode(void) {

  /* Wait for incoming commands  */

  vibrate(ON);
  enable_vdd();
  enable_vdda();

  NRF_LOG_INFO("WAIT mode entered.");
}


static unsigned unix_time;                         /* POSIX time                            */


static void handle_rtc_event(nrf_drv_rtc_int_type_t int_type) {

  /* Process the RTC module event */

  if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
    
    unix_time++;

    nrfx_err_t err_code = nrfx_rtc_cc_set(&rtc, 0, rtc.p_reg->CC[0] + 8, true);
    APP_ERROR_CHECK(err_code);

  }

  if (int_type == NRF_DRV_RTC_INT_COMPARE1) {

    vibrate(OFF);

  }
}


__STATIC_INLINE void init_rtc(void) {

  /* Initialize and configure RTC driver instance */

  uint32_t err_code;

  /* Initialize RTC instance */
  nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
  config.prescaler = UINT16_MAX / 16; /* 4095 */
  
  err_code = nrf_drv_rtc_init(&rtc, &config, handle_rtc_event);
  APP_ERROR_CHECK(err_code);

  /* Set compare channel to trigger interrupt after every second */
  err_code = nrf_drv_rtc_cc_set(&rtc, 0, 8, true);
  APP_ERROR_CHECK(err_code);

  /* Power on RTC instance */
  nrf_drv_rtc_enable(&rtc);
}


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                     /* BLE NUS service instance.              */
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;   /* Handle of the current connection.      */


uint32_t ble_output(uint8_t * s, uint16_t len) {

  /* Send the data out via BLE */

  unsigned err_code;
  do {
    uint16_t length = len;
    err_code = ble_nus_data_send(&m_nus, s, &length, m_conn_handle);
    if ((err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_NOT_FOUND)) {
      APP_ERROR_CHECK(err_code);
    }
  } while (err_code == NRF_ERROR_RESOURCES);
  return err_code;
}

static uint32_t ble_test_output(uint8_t *s) {
  uint16_t length = 244;
  uint32_t err_code;
  do {
    err_code = ble_nus_data_send(&m_nus, s, &length, m_conn_handle);
  } while(err_code == NRF_ERROR_RESOURCES);
  return err_code;
}


void ble_test_throughoutput(void) {

  /*  */

  test_req = !0;
  
  //NRF_LOG_INFO("Transmission test finished. Throughoutput: %u CPS.", (BLOCKS_TO_SEND * TEST_DATA_LENGTH) / meas_counter);

}

#if USE_ADC != 0
  __STATIC_INLINE uint32_t measure_vdd(void);
#endif


__STATIC_INLINE char * get_battery_status(void) {

  #if USE_ADC != 0

    static char buf[25];
    unsigned v1, v2, vdd = measure_vdd() + ADC_CORRECTION_FACTOR;
    v1 = vdd / 1000;
    v2 = vdd % 1000;
    
    char *c;
    
    if (v2 < 10) {
      c = "%u.00%uv";
    } else if (v2 < 100) {
      c = "%u.0%uv";
    } else {
      c = "%u.%uv";
    }
    
    snprintf(buf, sizeof buf, c, v1, v2);
    
    return buf;
  
  #else
  
    return "3.329v";
  
  #endif
}


__STATIC_INLINE uint16_t rssi(void) {

  /* Measure the power present in a received radio signal. */

  if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
    int8_t rssi;
    uint8_t channel;
  
    uint32_t err_code = sd_ble_gap_rssi_get(m_conn_handle, &rssi, &channel);
    APP_ERROR_CHECK(err_code);

    uint16_t res = channel << 8;
    res = res | (uint8_t)rssi;

    return res;

  } else {

    return 0;

  }
}


__STATIC_INLINE void get_info(void) {          /* process 's' command */

  /* Get firmware fersion, s/n, battery voltage and RSSI */

  NRF_LOG_INFO("'s' command: info requested.");

  uint8_t buf[120];
  uint16_t d = rssi();
  int8_t r = (int8_t)(d & 0xFF);
  uint8_t ch = d >> 8;

  int len = snprintf((char*) buf, sizeof(buf), "FIRMWARE: %s\nSerialNO: "DEVICE_SERIAL_NO"\nBattery: %s\nRSSI: %ddBm, Ch: %u\n", 
                     DEVICE_FIRMWARE_VERSION, NRF_FICR->DEVICEADDR[1], NRF_FICR->DEVICEADDR[0], 
                     get_battery_status(), r, ch);
  
  ble_output(buf, len);
}


static unsigned acquiring_status;


__STATIC_INLINE void start_acquiring(void) {   /* process 'b' command */
  
  NRF_LOG_INFO("'b' command: acquiring started.");

  while (NRF_LOG_PROCESS() != false) {
    __NOP();
  }
  acquiring_status = !0;
}


__STATIC_INLINE void stop_acquiring(void) {    /* process 'e' command */

  NRF_LOG_INFO("'e' command: acquiring stopped.");
  acquiring_status = 0;
}


__STATIC_INLINE void help(void) {        /* process 'h' command */

  char *h1 =
    "Command summary:\n"
    "\n"
    "b - start acquiring data\n"
    "c - get the single sample\n"
    "d - add the time offset\n"
    "e - stop acquiring data\n"
    "f - toggle indication\n";

  ble_output((uint8_t*)h1, strlen(h1));

  char *h2 =
    "g - get Gas Gauge info\n"
    "h - print this help\n"
    "i - get receiption level (RSSI)\n"
    "l - toggle led\n"
    "m - get correction factor\n"
    "M - set correction factor\n"
    "o - switch the device off\n";

  ble_output((uint8_t*)h2, strlen(h2));

  char *h3 =
    "p - get current power consumption\n"
    "r - read offset register\n"
    "R - write offset register\n"
    "s - get the device status\n"
    "t - get current date/time\n"
    "v - run vibration motor\n";

  ble_output((uint8_t*)h3, strlen(h3));

  char *h4 =
    "y - get calibration data\n"
    "z - set calibration data\n"
    "========================\n"
    "to set up date/time just send string in the following format:\n"
    "dd.mm.yyyy hh:mm:ss\n";

  ble_output((uint8_t*)h4, strlen(h4));

}

__STATIC_INLINE void toggle_status_led(void) { /* process 'l' command */
  
  NRF_LOG_INFO("'l' command: LED toggled.");

  #if 0
    nrf_gpio_pin_set(LED_PIN);
    nrf_gpio_pin_clear(LED_PIN);
  #endif

  nrf_gpio_pin_toggle(LED_PIN); 
}


__STATIC_INLINE void single_sample(void) {     /* process 'c' command */

  /* Acquire single sample from ADC */

  NRF_LOG_INFO("'c' command: single sample acquired.");

  if (acquiring_status == 0) {
    char buf[80];
    unsigned len = snprintf(buf, sizeof(buf), "%u, %u, %u, ", acq_data[0], acq_data[1], acq_data[2]);
    ble_output((uint8_t *)&buf, len);
  }
}


__STATIC_INLINE void shift_time(void) {     /* process 'd' command */

  /* Shift the time by one step */

  NRF_LOG_INFO("'d' command: the time shifted.");

  unix_time += UNIX_TIME_DELTA;  
}


__STATIC_INLINE void get_power_consumption(void) {

  NRF_LOG_INFO("'p' command: power consumption measured.");

  /* to be implemented soon */

}


__STATIC_INLINE void read_gas_gauge(void) {

  NRF_LOG_INFO("'g' command: gas gauge read.");

  /* to be implemented soon */

}


__STATIC_INLINE void show_status(void) {

  NRF_LOG_INFO("'f' command: status displayed.");
}


__STATIC_INLINE void get_calibration_data(const uint8_t * t, unsigned len) {

  NRF_LOG_INFO("'y' command: calibration data got.");

  /* to be implemented soon */

}

__STATIC_INLINE void set_calibration_data(const uint8_t * t, unsigned len) {

  NRF_LOG_INFO("'z' command: calibration data set.");

  /* to be implemented soon */

}


static volatile short offset_register;


__STATIC_INLINE void read_offset_register(void) {

  /* Read offset register */

  NRF_LOG_INFO("'r' command: offset register read.");

  char buf[80];
  unsigned len = snprintf(buf, sizeof(buf), "Offset: %d\n", offset_register);
  ble_output((uint8_t *)&buf, len);
}


__STATIC_INLINE void write_offset_register(const uint8_t * t, unsigned len) {

  /* Write offset register */

  NRF_LOG_INFO("'R' command: offset register written.");

  char buf[80], c;
  int offs;

  memcpy(buf, t, len);
  
  unsigned cnt = sscanf((const char *)buf, "%d%c", &offs, &c);
  if (cnt != 0) {
    offset_register = offs;
  }
}


static volatile int factor_register;


__STATIC_INLINE void read_factor_register(void) {

  /* Read factor register */

  NRF_LOG_INFO("'m' command: factor register read.");

  char buf[80];
  unsigned len = snprintf(buf, sizeof(buf), "Factor: %d.%u\n", factor_register / 10, factor_register % 10);
  ble_output((uint8_t *)&buf, len);
}


__STATIC_INLINE void write_factor_register(const uint8_t * t, unsigned len) {

  /* Write factor register */

  NRF_LOG_INFO("'M' command: factor register written.");

  char buf[80], c;
  int f1, f2;

  memcpy(buf, t, len);
  
  unsigned cnt = sscanf((const char *)buf, "%d.%u%c", &f1, &f2, &c);
  if (cnt != 0) {
    factor_register = f1 * 10 + f2;
  }
}


__STATIC_INLINE void get_rssi(void) {

  NRF_LOG_INFO("'i' command: rssi processed.");

  char buf[80];
  uint16_t d = rssi();
  int8_t r = (int8_t) d & 0xFF;
  uint8_t ch = d >> 8;

  unsigned len = snprintf(buf, sizeof(buf), "RSSI: %ddBm, Ch: %u\n", r, ch);
  ble_output((uint8_t *)&buf, len);
}


__STATIC_INLINE void get_current_time(void) { /* process 't' command */

  NRF_LOG_INFO("'t' command: current time sent.");

  time_struct_t t;

  unixtime_to_time(unix_time, &t);

  char buf[80];
  unsigned len = snprintf(buf, sizeof(buf), "%02u.%02u.%02u %02u:%02u:%02u\n", t.day, t.month, t.year, t.hour, t.minute, t.second);
  ble_output((uint8_t *)&buf, len);
}


__STATIC_INLINE void set_time(const uint8_t * t, unsigned len) {

  /* Process time/date string and setup the onboard clock */

  int day, month, year, hour, minute, second;

  char buf[25];

  memcpy(buf, t, len);
  buf[len] = 0;

  unsigned cnt = sscanf((const char *)buf, "%u.%u.%u %u:%u:%u", &day, &month, &year, &hour, &minute, &second);
  if ((cnt != 0) && (day > 0) && (day < 32)) {
    if ((month > 0) && (month < 13) && (hour < 24) && (minute < 60) && (second < 60)) {
      time_struct_t t;
      t.year = year;
      t.month = month;
      t.day = day;
      t.hour = hour;
      t.minute = minute;
      t.second = second;
      unsigned tim = time_to_unixtime(&t);
      if (tim != 0) {
        unix_time = tim;
      }
    }
  }
}


__STATIC_INLINE void init_timer(void) {

  /*  Initialize the timer module  */

  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}


__STATIC_INLINE void init_gap_params(void) {

  /* Setup GAP (Generic Access Profile) parameters, permissions and appearance */

  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}


static void enter_sleep_mode(void) {

  /* Put the chip into sleep mode. No return. */

  uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  enter_power_down_mode();

  /* Prepare wakeup buttons. */
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  /* Go to system-off mode (this function will not return; wakeup will cause a reset). */
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}


static unsigned print_help;


static void handle_nus_data(ble_nus_evt_t * p_evt) {

  /* Process the data received from the Nordic UART BLE Service and send it to the UART module */

  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
    uint32_t err_code;

    NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
    NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

    err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
    APP_ERROR_CHECK(err_code);

    for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) {
      do {
        err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY)) {
            NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
            APP_ERROR_CHECK(err_code);
        }
      } while (err_code == NRF_ERROR_BUSY);
    }

    if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r') {
      while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }

    char command = p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1];
    if ('\r' == command) command = p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 2];

    // char buf[50];
    // strcpy(buf, (char*)p_evt->params.rx_data.p_data);
    // if (buf[0] != 0) {
    //   buf[0] = 'x';
    // }

    if (command == 's') {

      get_info();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'b') {

      start_acquiring();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'c') {

      single_sample();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'd') {

      shift_time();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'e') {

      vibrate(ON);
      stop_acquiring();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'f') {

      show_status();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'g') {

      read_gas_gauge();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'h') {

      print_help = !0;
      p_evt->params.rx_data.length = 0;

    } else if (command == 'i') {

      /*  get received signal strength indicator */
      get_rssi();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'l') {

      toggle_status_led();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'o') {

      NRF_LOG_INFO("'o' command: sleep mode entered.");

      vibrate(ON);

      disable_vdd();
      disable_vdda();

      while(NRF_RTC2->EVENTS_COMPARE[1] == 0) {
        __NOP();
      }

      enter_sleep_mode();

    } else if (command == 'p') {

      get_power_consumption();
      p_evt->params.rx_data.length = 0;

    } else if (command == 't') {

      get_current_time();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'T') {

      ble_test_throughoutput();

    } else if ((command >= '0') && (command <= '9') && (p_evt->params.rx_data.length == 19)) {

      NRF_LOG_INFO("Set date/time command processed.");

      set_time(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
      p_evt->params.rx_data.length = 0;

    } else if (command == 'v') {

      vibrate(ON);
      p_evt->params.rx_data.length = 0;

    } else if (command == 'y') {

      get_calibration_data(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
      p_evt->params.rx_data.length = 0;

    } else if (command == 'z') {

      set_calibration_data(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
      p_evt->params.rx_data.length = 0;

    } else if (command == 'r') {

      read_offset_register();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'R') {

      write_offset_register(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length - 1);
      p_evt->params.rx_data.length = 0;

    } else if (command == 'm') {

      read_factor_register();
      p_evt->params.rx_data.length = 0;

    } else if (command == 'M') {

      write_factor_register(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length - 1);
      p_evt->params.rx_data.length = 0;
    }
  }
}


static void handle_qwr_error(uint32_t nrf_error) {

  /* Inform the application about an error */

  APP_ERROR_HANDLER(nrf_error);
}


NRF_BLE_QWR_DEF(m_qwr);                                /* Context for the Queued Write module.   */


__STATIC_INLINE void init_services(void) {

  /* Initialize NUS and Queued Write Module */

  uint32_t           err_code;
  ble_nus_init_t     nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};

  qwr_init.error_handler = handle_qwr_error;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  nus_init.data_handler = handle_nus_data;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);
}


static void handle_conn_params_event(ble_conn_params_evt_t * p_evt) {

  /* Handle an event from the Connection Parameters Module */

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}


static void handle_conn_params_error(uint32_t nrf_error) {

  /* Handle errors from the Connection Parameters Module */

  APP_ERROR_HANDLER(nrf_error);
}


__STATIC_INLINE void init_conn_params(void) {

  /* Initialize the Connection Parameters module */

  uint32_t               err_code;
  ble_conn_params_init_t cp_init;

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = handle_conn_params_event;
  cp_init.error_handler                  = handle_conn_params_error;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);

  //LOG_INF("MTU= %d", bt_nus_get_mtu(conn))    ; 
}


char const * phy_str(ble_gap_phys_t phys)
{
    static char const * str[] =
    {
        "1 Mbps",
        "2 Mbps",
        "Coded",
        "Unknown"
    };

    switch (phys.tx_phys)
    {
        case BLE_GAP_PHY_1MBPS:
            return str[0];

        case BLE_GAP_PHY_2MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
            return str[1];

        case BLE_GAP_PHY_CODED:
            return str[2];

        default:
            return str[3];
    }
}


/*  The ATT_MTU size can be modified freely between BLE_GATT_ATT_MTU_DEFAULT and NRF_SDH_BLE_GATT_MAX_MTU_SIZE. 
    To enable larger values than BLE_GATT_ATT_MTU_DEFAULT to be used, the application must be recompiled with a 
    different NRF_SDH_BLE_GATT_MAX_MTU_SIZE value.
    The GAP event length can be changed by modifying NRF_SDH_BLE_GAP_EVENT_LENGTH.
*/

#if 0
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /* Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#else
static uint16_t m_ble_nus_max_data_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 2; /* Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#endif


static void handle_ble_event(ble_evt_t const * p_ble_evt, void * p_context) {

  /* Handle blutooth stack events */

  uint32_t err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch (p_ble_evt->header.evt_id) {

    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected");
      err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      APP_ERROR_CHECK(err_code);
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_rssi_start(m_conn_handle, 0, 0);		
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      /* LED indication off */

      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
        //m_conn_interval_configured = true;
        NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
    } break;

#if 1
    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        // Accept parameters requested by the peer.
        ble_gap_conn_params_t params;
        params = p_gap_evt->params.conn_param_update_request.conn_params;
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
    } break;
#endif

    case BLE_GAP_EVT_PHY_UPDATE: {
        ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

        if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
        {
            // Ignore LL collisions.
            NRF_LOG_DEBUG("LL transaction collision during PHY update.");
            break;
        }

        //m_phy_updated = true;

        ble_gap_phys_t phys = {0};
        phys.tx_phys = p_phy_evt->tx_phy;
        phys.rx_phys = p_phy_evt->rx_phy;
        NRF_LOG_INFO("PHY update %s. PHY set to %s.",
                     (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
                     "accepted" : "rejected",
                     phy_str(phys));
    } break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys = {
        #if 0
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
        #else
        .rx_phys = BLE_GAP_PHY_2MBPS,
        .tx_phys = BLE_GAP_PHY_2MBPS,
        #endif
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      /* Reject pairing */
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      /* No system attributes have been stored. */
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      /* Disconnect on GATT Client timeout event. */
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_rssi_stop(m_conn_handle);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GATTS_EVT_TIMEOUT:
      /* Disconnect on GATT Server timeout event. */
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_rssi_stop(m_conn_handle);
      APP_ERROR_CHECK(err_code);

      break;

    default:
      break;
  }
}


__STATIC_INLINE void init_ble_stack(void) {

  /* Initialize the SoftDevice and the BLE event interrupt */

  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  /* Configure the BLE stack using the default settings. */
  /* Fetch the start address of the application RAM.     */
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  /* Enable BLE stack.                                   */
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  /* Register a handler for BLE events.                  */
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, handle_ble_event, NULL);
}


NRF_BLE_GATT_DEF(m_gatt);  /* GATT module instance.      */


static void handle_gatt_event(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {

  /* Handle events from the GATT library */

  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }

  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                p_gatt->att_mtu_desired_central,
                p_gatt->att_mtu_desired_periph);
}


__STATIC_INLINE void init_gatt(void) {

  /* Initializie the GATT library */

  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, handle_gatt_event);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

static void handle_uart_event(app_uart_evt_t * p_event) {

  /* Receive and process characters from the app_uart  */

  static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
  static uint8_t index = 0;
  uint32_t       err_code;

  switch (p_event->evt_type) {
    case APP_UART_DATA_READY:
      UNUSED_VARIABLE(app_uart_get(&data_array[index]));
      index++;

      if ((data_array[index - 1] == '\n') ||
          (data_array[index - 1] == '\r') ||
          (index >= m_ble_nus_max_data_len)) {

        err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
        APP_ERROR_CHECK(err_code);


        if (index > 1) {
          NRF_LOG_DEBUG("Ready to send data over BLE NUS");
          NRF_LOG_HEXDUMP_DEBUG(data_array, index);

          ble_output(data_array, index);
        }

        index = 0;

      }
      break;

    case APP_UART_COMMUNICATION_ERROR:
      APP_ERROR_HANDLER(p_event->data.error_communication);
      break;

    case APP_UART_FIFO_ERROR:
      APP_ERROR_HANDLER(p_event->data.error_code);
      break;

    default:
      break;
  }
}

__STATIC_INLINE void init_uart(void) {

  /* Initialize the UART module */

  uint32_t                     err_code;
  app_uart_comm_params_t const comm_params = {
    .rx_pin_no    = RX_PIN_NUMBER,
    .tx_pin_no    = TX_PIN_NUMBER,
    .rts_pin_no   = RTS_PIN_NUMBER,
    .cts_pin_no   = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .use_parity   = false,
    .baud_rate    = NRF_UART_BAUDRATE_115200
  };

  APP_UART_FIFO_INIT(&comm_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     handle_uart_event,
                     APP_IRQ_PRIORITY_LOWEST,
                     err_code);
  APP_ERROR_CHECK(err_code);
}


static void handle_advertising_event(ble_adv_evt_t ble_adv_evt) {

  /* Handle advertising events which are passed to the application */

  uint32_t err_code;

  switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
      err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_ADV_EVT_IDLE:
      enter_sleep_mode();
      break;

    default:
       break;
  }
}


static ble_uuid_t m_adv_uuids[]          = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}}; /* Universally unique service identifier. */
BLE_ADVERTISING_DEF(m_advertising);                                                         /* Advertising module instance.           */


static void handle_bsp_event(bsp_event_t event) {

  /* Handle events from the BSP module */

  uint32_t err_code;
  switch (event) {
    case BSP_EVENT_SLEEP:
      enter_sleep_mode();
      break;

    case BSP_EVENT_DISCONNECT:
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
      }
      break;

    case BSP_EVENT_WHITELIST_OFF:
      if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        if (err_code != NRF_ERROR_INVALID_STATE) {
          APP_ERROR_CHECK(err_code);
        }
      }
      break;

    default:
      break;
  }
}


__STATIC_INLINE void init_advertising(void) {

  /* Initialize the Advertising functionality */

  uint32_t               err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = false;

  #if 1
  init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  #else
  init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  #endif

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled  = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  init.evt_handler = handle_advertising_event;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


__STATIC_INLINE void init_bsp(bool * p_erase_bonds) {

  /* Initialize buttons and leds */

  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, handle_bsp_event);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


__STATIC_INLINE void init_log(void) {

  /* Initialize the nrf log module */

  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
  
}


__STATIC_INLINE void init_power_management(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}


__STATIC_INLINE void start_advertising(void) {

  /* Start advertising in the advertising modes that was enabled during initialization */

  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}


__STATIC_INLINE void handle_idle_state(void) {

  /* Sleep until the next event occurs if there is no pending log operation */

  if (acquiring_status != 0) {
    acquire();
  } else if (print_help != 0) {
    help();
    print_help = 0;
  } else {
    if (NRF_LOG_PROCESS() == false) {
      nrf_pwr_mgmt_run();
    }
  }
}

static const nrfx_spim_t     spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /* SPI instance.    */
static volatile bool         spi_xfer_done;                           /* Flag used to indicate that SPI instance completed the transfer. */
                             
static uint8_t               m_tx_buf[] = SPI_TEST_STRING;            /* TX buffer.       */
static uint8_t               m_rx_buf[sizeof(SPI_TEST_STRING) + 1];   /* RX buffer.       */
static const uint8_t         m_length = sizeof(m_tx_buf);             /* Transfer length. */
static nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, m_length, m_rx_buf, m_length);


void handle_spim_event(nrfx_spim_evt_t const * p_event, void * p_context) {

  /* Handle SPIM user event.            */

  spi_xfer_done = true;
  NRF_LOG_INFO("SPIM transfer (TX/RX) completed.");
  if (m_rx_buf[0] != 0) {
    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
  }
}


__STATIC_INLINE void init_spim(void) {

  /* Init SPIM peripheral */

  nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
  spi_config.frequency      = NRF_SPIM_FREQ_1M;
  spi_config.ss_active_high = false;

  spi_config.ss_pin         = NRFX_SPIM_SS_PIN;
  spi_config.miso_pin       = NRFX_SPIM_MISO_PIN;
  spi_config.mosi_pin       = NRFX_SPIM_MOSI_PIN;
  spi_config.sck_pin        = NRFX_SPIM_SCK_PIN;
  /*
  spi_config.dcx_pin        = NRFX_SPIM_DCX_PIN;
  spi_config.use_hw_ss      = true;
  */

  uint32_t err_code = nrfx_spim_init(&spi, &spi_config, handle_spim_event, NULL);
  APP_ERROR_CHECK(err_code);
  
  NRF_LOG_INFO("SPIM peripheral enabled.");
}


__STATIC_INLINE void test_spim(void) {
  
  /* Test SPI transfer */
  
  memset(m_rx_buf, 0, m_length);
  spi_xfer_done = false;
  
  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer_desc, 0);

  APP_ERROR_CHECK(err_code);
 
  while (!spi_xfer_done) {
    __WFE();
  }
}


static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); /* TWI instance.                                    */
static volatile bool m_xfer_done = false;                                 /* Indicates if operation on TWI has ended.         */
static uint8_t m_sample;                                                  /* Buffer for samples read from TWI device.         */


__STATIC_INLINE void test_twi(void) {

  /* Send to and receive from TWI device. */

  m_xfer_done = false;

  ret_code_t err_code = nrf_drv_twi_tx(&m_twi, 0x68, &m_sample, sizeof(m_sample), 0);
  APP_ERROR_CHECK(err_code);

  nrf_delay_ms(500);

  err_code = nrf_drv_twi_rx(&m_twi, 0x68, &m_sample, sizeof(m_sample)); /* Read 1 byte from the specified address. */
  APP_ERROR_CHECK(err_code);
}

  
__STATIC_INLINE void handle_data(uint8_t data) {

  /* Handle data from TWI device. */

  NRF_LOG_INFO("Data: %d read.", data);
}


static void handle_twi_event(nrf_drv_twi_evt_t const * p_event, void * p_context) {

  /* TWI event handler. */

  switch (p_event->type) {
    case NRF_DRV_TWI_EVT_DONE:
      if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
        handle_data(m_sample);
      }
      m_xfer_done = true;
      NRF_LOG_INFO("TWI transfer completed.");
      break;

    default:
      NRF_LOG_INFO("TWI event occured.");
      break;
  }
}


__STATIC_INLINE void init_twi(void) {

  /* Initialize TWI peripheral */

  const nrf_drv_twi_config_t twi_config = {
    .scl                = NRF_TWI_SCL_PIN,  /*27 */
    .sda                = NRF_TWI_SDA_PIN,  /*26 */
    .frequency          = NRF_DRV_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false
  };

  ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, handle_twi_event, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);

  NRF_LOG_INFO("TWIM peripheral enabled.");
}

#if USE_ADC != 0

__STATIC_INLINE void init_adc(void) {

  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD;
  NRF_SAADC->CH[0].CONFIG = (
    SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos     |
    SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos
  );

  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over64x;

  NRF_SAADC->RESULT.MAXCNT = 1;

  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;  /* Start calibration */
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));
  NRF_LOG_INFO("SAADC initialized.");
}


__STATIC_INLINE uint32_t measure_vdd(void) {

  volatile uint16_t res;

  NRF_SAADC->RESULT.PTR = (unsigned)&res;

  NRF_SAADC->EVENTS_DONE = 0;

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  NRF_SAADC->TASKS_START = 1;              // Start the SAADC
  while (NRF_SAADC->EVENTS_STARTED == 0);  // Wait for STARTED event
  NRF_SAADC->EVENTS_STARTED = 0;           // Reset event flag

  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0);
  NRF_SAADC->EVENTS_END = 0;

  // Disable SAADC
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos;

  return res * 3600UL / 4095;
}
#endif


__STATIC_INLINE void init(void) {

  /* Init all modules and sub-systems then start advertising */
  
  bool erase_bonds;

  init_log();

  NRF_LOG_INFO("Device ID: %08X-%08X", NRF_FICR->DEVICEADDR[1], NRF_FICR->DEVICEADDR[0]);	

  init_uart();
  init_twi();
  test_twi();
  init_spim();
  test_spim();
  init_timer();

  #if USE_ADC
    init_adc();
  #endif
  
  if (0 == nrf_rtc_counter_get(rtc.p_reg)) {
    init_rtc();
  }

  init_bsp(&erase_bonds);
  init_power_management();
  init_ble_stack();
  init_gap_params();
  init_gatt();
  init_services();
  init_advertising();
  init_conn_params();

  nrf_gpio_cfg_output(LED_PIN);  /* Initialize with default config of pin */

  start_advertising();

  enter_wait_mode();

  NRF_LOG_INFO("Debug logging for UART over RTT started.");

  //NRF_LOG_INFO("MTU= %d", ble_nus_get_mtu(m_conn_handle)); 

  printf("All modules are started.\r\n");
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {

  /* Run the error handler if an assert in the SoftDevice is occured. */

  app_error_handler(DEAD_FACE, line_num, p_file_name);
}


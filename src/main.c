#include "main.h"

unsigned tr_count;

int main(void) {

  init(); /* Initialize the system */

  for (;;) { /* Enter an infinite loop to handle events */
    
    handle_idle_state(); /* Do all the work" */

  }

  /*
   
    The program should never exit the infinite loop, but if it does, 
    halt the system. This is a safety measure to prevent undefined behavior.

  */

  __WFE(); /* Reset pending flag      */
  __SEV(); /* Send Event              */
  __WFE(); /* Enter a low-power state */

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

#if 0
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
#else
__STATIC_INLINE void vibrate(uint8_t state) {

  /*
  
    This function starts or stops the vibration motor 
    based on the value passed in the state parameter.

  */

  if (state != 0) { /* If state is not zero, start the vibration motor */

    /* Log a message indicating that the vibration motor has started */
    NRF_LOG_INFO("vibration started.");

    /* Clear the COMPARE[1] event and set a new compare value for CC1, 8 ticks ahead of the current time */
    NRF_RTC2->EVENTS_COMPARE[1] = 0;
    nrfx_err_t err_code = nrfx_rtc_cc_set(&rtc, 1, nrf_rtc_counter_get(rtc.p_reg) + 8, true);
    APP_ERROR_CHECK(err_code);

  } else {  /* Otherwise, stop the vibration motor */ 

    /* Log a message indicating that the vibration motor has stopped */
    NRF_LOG_INFO("vibration stopped.");

    /* Disable CC1 compare match events */
    nrfx_err_t err_code = nrfx_rtc_cc_disable(&rtc, 1);
    APP_ERROR_CHECK(err_code);

  }
}
#endif


__STATIC_INLINE void enter_wait_mode(void) {

  /* Wait for incoming commands  */

  vibrate(ON);
  enable_vdd();
  enable_vdda();

  NRF_LOG_INFO("WAIT mode entered.");
}


static unsigned unix_time;                         /* POSIX time                            */


#if 0
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
#else
static void handle_rtc_event(nrf_drv_rtc_int_type_t int_type) {

  /*
  
    This function is an event handler for RTC module events.
    It is called by the RTC driver when a specified event occurs.
  
  */

  /* Check if the event type is a compare match with CC0 */
  if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
    
    /* Increment the Unix time counter */
    unix_time++;

    /* Set a new compare value for CC0, 8 ticks ahead of the current time */
    nrfx_err_t err_code = nrfx_rtc_cc_set(&rtc, 0, rtc.p_reg->CC[0] + 8, true);
    APP_ERROR_CHECK(err_code);

  }

  /* Check if the event type is a compare match with CC1 */
  if (int_type == NRF_DRV_RTC_INT_COMPARE1) {
    vibrate(OFF); /* Turn off the vibration motor */
  }
}
#endif


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


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);        /* BLE NUS service instance.              */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /* Handle of the current connection.      */


uint32_t ble_output(uint8_t * s, uint16_t len) {

  /* Send data over BLE NUS */

  unsigned err_code;

  do { /* Continue sending data until no more resources are available */

    uint16_t length = len;

    /* Send data using the BLE NUS service instance and the current connection handle */
    err_code = ble_nus_data_send(&m_nus, s, &length, m_conn_handle);

    /* Check for errors that can't be safely ignored */
    if ((err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES)     &&
        (err_code != NRF_ERROR_NOT_FOUND)) {

      /* report it as an application error */
      APP_ERROR_CHECK(err_code);
    }
  } while (err_code == NRF_ERROR_RESOURCES);

  return err_code;
}

static uint32_t ble_test_output(uint8_t *s) {
  uint16_t length = 216;
  uint32_t err_code;
  do {
    err_code = ble_nus_data_send(&m_nus, s, &length, m_conn_handle);
  } while(err_code == NRF_ERROR_RESOURCES);
  return err_code;
}

static uint32_t test_req;

void ble_test_throughoutput(void) {

  /*  */

  test_req = !0;
 
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

#if 0
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
#else
__STATIC_INLINE uint16_t rssi(void) {

  /*
    
    This function measures the power present in a received radio signal.
    It returns a 16-bit value, with the most significant byte being the 
    channel number and the least significant byte being the RSSI value.

  */

  /* Check if a connection handle has been established */
  if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {

    /* Declare variables for RSSI and channel */
    int8_t rssi;
    uint8_t channel;
  
    /* Call the SoftDevice API to get the RSSI and channel of the connected device */
    uint32_t err_code = sd_ble_gap_rssi_get(m_conn_handle, &rssi, &channel);
    APP_ERROR_CHECK(err_code);

    /*
      Combine the channel and RSSI values into a 16-bit value, with the channel 
      in the most significant byte and the RSSI in the least significant byte
    */
    uint16_t res = channel << 8;
    res = res | (uint8_t)rssi;

    return res;

  } else { /* If there is no connection handle, return 0 */
    return 0;
  }
}
#endif


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
  tr_count = 0;
}


__STATIC_INLINE void stop_acquiring(void) {    /* process 'e' command */

  NRF_LOG_INFO("'e' command: acquiring stopped.");
  acquiring_status = 0;
  set_acquiring_state(ACQUIRING_INACTIVE);
  NRF_LOG_INFO("Transmission completed. %u blocks sent.", tr_count);

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
    //unsigned len = snprintf(buf, sizeof(buf), "%u, %u, %u, ", acq_data[0], acq_data[1], acq_data[2]);
    ble_output((uint8_t *)&buf, /* len */ 10);
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

#if 0
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
#else
__STATIC_INLINE void set_time(const uint8_t * t, unsigned len) {

  /*
    
    This function processes a time/date string and sets the onboard clock to the specified time.
    The time/date string is passed in as a uint8_t pointer, with the length of the string passed 
    in as an unsigned integer.

  */

  /* Declare variables for the day, month, year, hour, minute, and second */
  int day, month, year, hour, minute, second;

  /* Create a buffer for the time/date string and copy the string into it */
  char buf[25];
  memcpy(buf, t, len);
  buf[len] = 0;

  /* Use sscanf() to parse the time/date string into day, month, year, hour, minute, and second variables */
  unsigned cnt = sscanf((const char *)buf, "%u.%u.%u %u:%u:%u", &day, &month, &year, &hour, &minute, &second);

  /* Check if the sscanf() call was successful and if the parsed values are valid */
  if ((cnt != 0) && (day > 0) && (day < 32)) {
    if ((month > 0) && (month < 13) && (hour < 24) && (minute < 60) && (second < 60)) {

      /* Create a time_struct_t object and populate it with the parsed values */
      time_struct_t t = {
        .year   = year,
        .month  = month,
        .day    = day,
        .hour   = hour,
        .minute = minute,
        .second = second
      };

      /* Convert the time_struct_t object to Unix time */
      unsigned tim = time_to_unixtime(&t);

      /* If the conversion was successful, set the onboard clock to the Unix time value */
      if (tim != 0) {
        unix_time = tim;
      }
    }
  }
}
#endif


__STATIC_INLINE void init_timer(void) {

  /*  Initialize the timer module  */

  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}


#if 0
__STATIC_INLINE void conn_evt_len_ext_set(bool status) {
  ret_code_t err_code;
  ble_opt_t  opt = {.common_opt.conn_evt_ext.enable = status ? 1 : 0};

  err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
  APP_ERROR_CHECK(err_code);

}
#else
/**
 * Sets a BLE option for extending connection event length.
 *
 * @param status a boolean value that determines whether to enable or disable the feature.
 *               true to enable, false to disable.
 */
__STATIC_INLINE void conn_evt_len_ext_set(bool status) {
  
  /*

    Create a ble_opt_t structure with the common_opt.conn_evt_ext.enable field 
    set to 1 or 0 based on the `status` parameter.

  */

  ble_opt_t  opt = {.common_opt.conn_evt_ext.enable = status ? 1 : 0};

  /* Set the BLE option for extending connection event length using the `sd_ble_opt_set()` function. */
  /* The function returns an error code, which is checked using the `APP_ERROR_CHECK()` macro. */

  ret_code_t err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
  APP_ERROR_CHECK(err_code);

}
#endif

#if 0
__STATIC_INLINE void init_gap_params(void) {

  /* Setup GAP (Generic Access Profile) parameters, permissions and appearance */

  uint32_t                err_code;
  ble_gap_conn_sec_mode_t sec_mode;

  ble_gap_conn_params_t   gap_conn_params = {
    .min_conn_interval = MIN_CONN_INTERVAL,
    .max_conn_interval = MAX_CONN_INTERVAL,
    .slave_latency     = SLAVE_LATENCY,
    .conn_sup_timeout  = CONN_SUP_TIMEOUT
  };

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);

  //conn_evt_len_ext_set(true);

}
#else
__STATIC_INLINE void init_gap_params(void) {

  /*
   
    This function sets up the Generic Access Profile (GAP) parameters, permissions, 
    and appearance of the BLE device.

  */

  /* Declare variables for error code and security mode */
  uint32_t                 err_code;
  ble_gap_conn_sec_mode_t  sec_mode;

  /* Define the connection parameters for the BLE device */
  ble_gap_conn_params_t    gap_conn_params = {
    .min_conn_interval = MIN_CONN_INTERVAL,
    .max_conn_interval = MAX_CONN_INTERVAL,
    .slave_latency     = SLAVE_LATENCY,
    .conn_sup_timeout  = CONN_SUP_TIMEOUT
  };

  /* Set the security mode to open */
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  /* Set the device name using the security mode and device name defined in the macro */
  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  /* Set the preferred connection parameters using the gap_conn_params structure */
  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);

  /* Enable extended connection event length */
  // conn_evt_len_ext_set(true);

}
#endif


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


NRF_BLE_QWR_DEF(m_qwr); /* Context for the Queued Write module.   */


/**
 * @brief Initializes NUS and QWR services.
 *
 * This function initializes the Nordic UART Service (NUS) and the Queued Write Module (QWR).
 * NUS is used to transfer data over Bluetooth Low Energy (BLE) while QWR is used to queue
 * write operations. 
 *
 * @note This function must be called before starting the BLE stack.
 */
__STATIC_INLINE void init_services(void) {

  uint32_t           err_code;         /* Error code returned from BLE API functions */
  ble_nus_init_t     nus_init;         /* NUS initialization structure */
  nrf_ble_qwr_init_t qwr_init = {0};   /* Queued Write Module initialization structure */

  /* Assigns an error handler function for the QWR */
  qwr_init.error_handler = handle_qwr_error;

  /* Initializes the QWR */
  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);           /* Checks and handles any errors */

  /* Assigns a data handler function for the NUS */
  nus_init.data_handler = handle_nus_data;

  /* Initializes the NUS */
  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);           /* Checks and handles any errors */
}


/**
 * @brief Handles BLE connection parameters event.
 * 
 * @param[in] p_evt Pointer to the BLE connection parameters event structure.
 * 
 * @details This function checks if the event type of the provided struct is BLE_CONN_PARAMS_EVT_FAILED. 
 *          If it is, then the function disconnects from the current connection by calling sd_ble_gap_disconnect function 
 *          with the BLE_HCI_CONN_INTERVAL_UNACCEPTABLE error code, which indicates that the connection interval is not acceptable. 
 *          The APP_ERROR_CHECK macro is used to check the return code of the sd_ble_gap_disconnect function.
 * 
 * @note    This function should be called when a BLE connection parameters event occurs.
 *          The function disconnects from the current connection if the event type is BLE_CONN_PARAMS_EVT_FAILED.
 *          Otherwise, the function does nothing.
 * 
 * @warning This function does not handle other BLE connection parameters events except BLE_CONN_PARAMS_EVT_FAILED.
 * 
 * @see     sd_ble_gap_disconnect
 * @see     ble_conn_params_evt_t
 * @see     APP_ERROR_CHECK
 * 
 * @retval  None
 */
static void handle_conn_params_event(ble_conn_params_evt_t * p_evt) {

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    /* 
        BLE_HCI_CONN_INTERVAL_UNACCEPTABLE is an error code for when the connection interval is 
        too short or too long. In this case, we use it to disconnect the BLE connection when 
        the connection parameters event type is failed.
    */
    uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}


static void handle_conn_params_error(uint32_t nrf_error) {

  /* Handle errors from the Connection Parameters Module */

  APP_ERROR_HANDLER(nrf_error);
}


/**
 * @brief Initializes the Connection Parameters module
 *
 * This function sets up the Connection Parameters module for negotiating and updating
 * connection parameters between two BLE devices. The function sets the initial values for
 * the connection parameters module and registers the event and error handlers.
 */
__STATIC_INLINE void init_conn_params(void) {

  /* Define the initialization parameters for the Connection Parameters module */
  ble_conn_params_init_t cp_init    = {

    .p_conn_params                  = NULL,                           /* No connection parameters yet     */
    .first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY, /* Delay before first update        */
    .next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY,  /* Delay between subsequent updates */
    .max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT,   /* Maximum number of updates        */
    .start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID,        /* Invalid ATTRIBUTE handle         */
    .disconnect_on_fail             = false,                          /* Do not disconnect on failure     */
    .evt_handler                    = handle_conn_params_event,       /* Event handler                    */
    .error_handler                  = handle_conn_params_error        /* Error handler                    */

  };

  /* Initialize the Connection Parameters module with the defined parameters */
  uint32_t err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);

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


/**
 * @brief Function for handling Bluetooth stack events.
 *
 * This function is called by the SoftDevice on Bluetooth stack events. It takes a pointer to a
 * ble_evt_t structure and a void pointer to context data.
 *
 * @param[in] p_ble_evt Pointer to the Bluetooth stack event structure.
 * @param[in] p_context Pointer to context data.
 */
#if 0
static void handle_ble_event(ble_evt_t const * p_ble_evt, void * p_context) {

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

        ble_gap_phys_t const phys = {
          .tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
          .rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);

        ble_gap_conn_params_t   gap_conn_params = {
          .min_conn_interval = MIN_CONN_INTERVAL,
          .max_conn_interval = MIN_CONN_INTERVAL,
          .slave_latency     = SLAVE_LATENCY,
          .conn_sup_timeout  = CONN_SUP_TIMEOUT,
        };
        err_code = sd_ble_gap_conn_param_update(m_conn_handle /*p_gap_evt->conn_handle*/, &gap_conn_params);
        APP_ERROR_CHECK(err_code);

      err_code = sd_ble_gap_rssi_start(m_conn_handle, 0, 0);		
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      /* LED indication off */

      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      set_acquiring_state(ACQUIRING_INACTIVE);

      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
        //m_conn_interval_configured = true;
        NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
    } break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        // Accept parameters requested by the peer.
        ble_gap_conn_params_t params;
        params = p_gap_evt->params.conn_param_update_request.conn_params;
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);

        conn_evt_len_ext_set(true);

    } break;

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
        //.rx_phys = BLE_GAP_PHY_2MBPS,
        //.tx_phys = BLE_GAP_PHY_2MBPS,
        .tx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
        .rx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
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
#else
static void handle_ble_event(ble_evt_t const * p_ble_evt, void * p_context) {
	
  uint32_t err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch (p_ble_evt->header.evt_id) {

    case BLE_GAP_EVT_CONNECTED:

      /* Indicate that a connection has been established */

      NRF_LOG_INFO("Connected");
      err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      APP_ERROR_CHECK(err_code);

      /* Remember the handle of the current connection */
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

      /* Assign the connection handle to the QWR module */
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);

      /* Update the PHY to support all available speeds */
      ble_gap_phys_t const phys = {
        .tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
        .rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);

      /* Update the connection parameters */
      ble_gap_conn_params_t gap_conn_params = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MIN_CONN_INTERVAL,
        .slave_latency     = SLAVE_LATENCY,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT,
      };
      err_code = sd_ble_gap_conn_param_update(m_conn_handle, &gap_conn_params);
      APP_ERROR_CHECK(err_code);

      /* Start RSSI measurements */
      err_code = sd_ble_gap_rssi_start(m_conn_handle, 0, 0);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GAP_EVT_DISCONNECTED:
	
      /* Indicate that the connection has been lost */
      NRF_LOG_INFO("Disconnected");

      /* Reset connection handle */
      m_conn_handle = BLE_CONN_HANDLE_INVALID;

      /* Set the acquiring subsystem to an inactive state */
      set_acquiring_state(ACQUIRING_INACTIVE);

      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
	
      /* Connection parameter update event */
	  
      NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
          p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
          p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
	
      /* Connection parameter update request event */
      {
        ble_gap_conn_params_t params;
        params = p_gap_evt->params.conn_param_update_request.conn_params;
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
        APP_ERROR_CHECK(err_code);
        
        NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
        
        /* Enable the use of extended connection event length */
        conn_evt_len_ext_set(true);
      }

      break;

    case BLE_GAP_EVT_PHY_UPDATE:
	
      /* PHY update event */
	
      {  
        ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;
        
        if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION) {
	  	  
          /* Ignore LL collisions */
          NRF_LOG_DEBUG("LL transaction collision during PHY update.");
          break;
        }
        
        ble_gap_phys_t phys = {
          .tx_phys = p_phy_evt->tx_phy,
          .rx_phys = p_phy_evt->rx_phy
        };
        NRF_LOG_INFO("PHY update %s. PHY set to %s.",
            (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
            "accepted" : "rejected",
            phy_str(phys));
      }

      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
	
      /* PHY update request event */
      {
        ble_gap_phys_t const phys = {
          .tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
          .rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
      }

      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
	
      /* Security parameters request event */
      /* Reject pairing */
	  
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle,BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
	
      /* GATT server system attributes missing event */
      /* Set system attributes to default values */
	  
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GATTC_EVT_TIMEOUT:
	
      /* GATT client timeout event */
      /* Disconnect from the peer device */
	  
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);

      /* Stop RSSI measurements */
      err_code = sd_ble_gap_rssi_stop(m_conn_handle);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GATTS_EVT_TIMEOUT:
	
      /* GATT server timeout event */
      /* Disconnect from the peer device */
	  
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);

      /* Stop RSSI measurements */
      err_code = sd_ble_gap_rssi_stop(m_conn_handle);
      APP_ERROR_CHECK(err_code);

      break;

    default:
      // Unknown event, do nothing
      break;
  }
}
#endif

/**
 * Initializes the BLE stack.
 *
 * This function initializes the SoftDevice and the BLE event interrupt.
 * It configures the BLE stack using the default settings, enables the BLE stack,
 * and registers a handler for BLE events.
 */
__STATIC_INLINE void init_ble_stack(void) {

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


#if 0
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

#else
/**
 * Handles GATT events.
 *
 * @param[in] p_gatt Pointer to the GATT instance.
 * @param[in] p_evt  Pointer to the GATT event.
 */
static void handle_gatt_event(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {

    /* Check if the connection handle matches and if the event is an ATT MTU update event */
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {

        /* Calculate the effective maximum data length and update the value of m_ble_nus_max_data_len */
        /* ATT MTU = ATT Protocol Data Unit (PDU) length - opcode length - handle length */

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;

        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }

    /* Log the ATT MTU exchange status */
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}
#endif

//#define CUSTOM_MTU 219
#define CUSTOM_MTU 247

__STATIC_INLINE void init_gatt(void) {

  /* Initializie the GATT library */

  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, handle_gatt_event);
  APP_ERROR_CHECK(err_code);

  #if 0
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  #else
  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, CUSTOM_MTU);
  #endif
  APP_ERROR_CHECK(err_code);

  #if 0
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  #else
  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, CUSTOM_MTU);
  #endif
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


#if 0
__STATIC_INLINE void init_advertising(void) {

  /* Initialize the Advertising functionality */

  uint32_t               err_code;
  ble_advertising_init_t init = {0};

  // memset(&init, 0, sizeof(init));

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
#else
__STATIC_INLINE void init_advertising(void) {

  /* Initialize the Advertising functionality */

  uint32_t err_code;
  ble_advertising_init_t init = {
    .advdata.name_type          = BLE_ADVDATA_FULL_NAME,
    .advdata.include_appearance = false,

    #if 1
    .advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,
    #else
    .advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
    #endif

    .srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]),
    .srdata.uuids_complete.p_uuids  = m_adv_uuids,

    .config.ble_adv_fast_enabled  = true,
    .config.ble_adv_fast_interval = APP_ADV_INTERVAL,
    .config.ble_adv_fast_timeout  = APP_ADV_DURATION,
    .evt_handler = handle_advertising_event
  };

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
#endif


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

static const nrfx_spim_t     spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /* SPI instance.    */
static volatile bool         spi_xfer_done;                           /* Flag used to indicate that SPI instance completed the transfer. */

static uint8_t spi_rx_buf[256];
static uint8_t spi_tx_buf[256];

static nrfx_spim_xfer_desc_t xfer = {.p_tx_buffer = (uint8_t const *)(spi_tx_buf), .p_rx_buffer = spi_rx_buf};

/*
#define NRFX_SPIM_SINGLE_XFER(p_tx, tx_len, p_rx, rx_len) \
    {                                                     \
    .p_tx_buffer = (uint8_t const *)(p_tx),               \
    .tx_length = (tx_len),                                \
    .p_rx_buffer = (p_rx),                                \
    .rx_length = (rx_len),                                \
    }
*/

#define MAX_DATA_LENGTH 244
#define ADS_BLOCK_16_COUNT 15
#define ADS_BLOCK_24_COUNT 10
#define BLOCKS_TO_SEND 10000UL

static union {
  uint8_t buf[MAX_DATA_LENGTH];
  uint32_t id;
} x;

static uint8_t bndx;

__STATIC_INLINE void ads_start_sending_cmd(uint8_t cmd) {

  spi_tx_buf[0] = cmd;
  xfer.tx_length = 1;
  xfer.rx_length = 1;

  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);

  APP_ERROR_CHECK(err_code);

}

__STATIC_INLINE void ads_finish_sending_cmd(void) {

  while (!spi_xfer_done) {
    __WFE();
  }

}

__STATIC_INLINE void ads_send_cmd(uint8_t cmd) {

  spi_xfer_done = false;
  ads_start_sending_cmd(cmd);
  ads_finish_sending_cmd();

}

static int loop_counter;

__STATIC_INLINE void handle_idle_state(void) {

  /* Sleep until the next event occurs if there is no pending log operation */

  if (print_help != 0) {

    help();
    print_help = 0;

  } else if (test_req != 0) {

    NRF_LOG_INFO("Transmission test started.");
    NRF_LOG_FLUSH();
    NRF_TIMER3->BITMODE = 3;                /* set 32-bit mode for TIMER3   */
    NRF_TIMER3->TASKS_CLEAR = 1;            /* clear TIMER3 counter         */
    NRF_TIMER3->TASKS_START = 1;            /* run timer                    */

    for (uint32_t i = 0; i < BLOCKS_TO_SEND; i++) {
      x.id = i;
      for (uint32_t j = 0; j < MAX_DATA_LENGTH / 2; j++) {
        x.buf[j] = '0' + (i & 0x1F);
      }
      for (uint32_t j = MAX_DATA_LENGTH / 2; j < MAX_DATA_LENGTH; j++) {
        x.buf[j] = 'Z' - (i & 0x0F);
      }
      ble_test_output(x.buf);
    };

    NRF_TIMER3->TASKS_CAPTURE[0] = 1;       /* capture TIMER3 counter value */
    NRF_TIMER3->TASKS_STOP = 1;             /* stop TIMER3                  */

    uint64_t cps = BLOCKS_TO_SEND;
    cps *= MAX_DATA_LENGTH;

    NRF_LOG_INFO("Transmission completed. %u bytes sent in %u us.", cps, NRF_TIMER3->CC[0]);

   cps *= 1000000UL;
   cps /= NRF_TIMER3->CC[0];

    NRF_LOG_INFO("Transmission speed: %u cps.", cps);

    test_req = 0;

    //acquire();

  } else if (ACQUIRING_INACTIVE == get_acquiring_state()) {

    /* NOTE: When using the START opcode to begin conversions, hold the START pin low. */

    if (acquiring_status != 0) {
      set_acquiring_state(ACQUIRING_STARTED);
      ads_start_sending_cmd(ADS129X_START); /* START CONVERSION */
      loop_counter++;
    }

  } else if (ACQUIRING_REQUESTED == get_acquiring_state()) {

    // /* NOTE: When using the START opcode to begin conversions, hold the START pin low. */
    // 
    // set_acquiring_state(ACQUIRING_STARTED);
    // ads_start_sending_cmd(ADS129X_START); /* START CONVERSION */

  } else if (ACQUIRING_ACTIVE == get_acquiring_state()) {

    if (0 == nrf_gpio_pin_read(ADS_DATA_READY_PIN)) {
      ads_start_acquiring();             /* START CONVERSION */
      set_acquiring_state(ACQUIRING_IN_PROGRESS);
    }

  } else if (ACQUIRING_DATA_READY == get_acquiring_state()) {

    #if 0
    void *p = x.buf + 24 * bndx++;
    if (ads_finish_acquiring(p, ADS_CONVERT_24_24, ADS_BLOCK_24_COUNT == bndx)) {
      bndx = 0;
      ble_output(x.buf, TEST_DATA_LENGTH);
      memset(x.buf, 0, TEST_DATA_LENGTH);
    }
    #else

    void *p = x.buf + 16 * bndx++;

    if (ads_finish_acquiring(p, ADS_CONVERT_16_16, bndx == ADS_BLOCK_16_COUNT)) {
      bndx = 0;
      tr_count++;
      ble_output(x.buf, MAX_DATA_LENGTH);
      memset(x.buf, 0, MAX_DATA_LENGTH);
    }

    #endif

    set_acquiring_state(ACQUIRING_INACTIVE);

  } else {
    if (NRF_LOG_PROCESS() == false) {
      nrf_pwr_mgmt_run();
    }
  }
}


void handle_spim_event(nrfx_spim_evt_t const * p_event, void * p_context) {

  /* Handle SPIM user event.            */

  spi_xfer_done = true;

  if (ACQUIRING_STARTED == get_acquiring_state()) {

    set_acquiring_state(ACQUIRING_ACTIVE);

  } else if (ACQUIRING_IN_PROGRESS == get_acquiring_state()) {

    set_acquiring_state(ACQUIRING_DATA_READY);

  }

}


#if 0
__STATIC_INLINE void init_spim(void) {

  /* Init SPIM peripheral */

  nrfx_spim_config_t spi_config = {
    .sck_pin        = SPI_SCK_PIN,
    .mosi_pin       = SPI_MOSI_PIN,
    .miso_pin       = SPI_MISO_PIN,
    .ss_pin         = SPI_SS_PIN,
    .ss_active_high = false,
    .irq_priority   = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc            = 0xFF,
    .frequency      = NRF_SPIM_FREQ_1M,
    .mode           = NRF_SPIM_MODE_1,
    .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
  };

  uint32_t err_code = nrfx_spim_init(&spi, &spi_config, handle_spim_event, NULL);
  APP_ERROR_CHECK(err_code);
  
  NRF_LOG_INFO("SPIM peripheral enabled.");
}
#else
/**

    @brief Initializes the SPIM peripheral.
    This function initializes the SPIM peripheral with the following settings:
        SCK pin        : SPI_SCK_PIN
        MOSI pin       : SPI_MOSI_PIN
        MISO pin       : SPI_MISO_PIN
        SS pin         : SPI_SS_PIN
        SS active high : false
        IRQ priority   : NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY
        SPI frequency  : NRF_SPIM_FREQ_4M
        SPI mode       : NRF_SPIM_MODE_1
        Bit order      : NRF_SPIM_BIT_ORDER_MSB_FIRST
        Output resistance calibration (ORC) value: 0xFF

*/
__STATIC_INLINE void init_spim(void) {

  /* Initialize SPIM peripheral configuration */
  nrfx_spim_config_t spi_config = {
    .sck_pin = SPI_SCK_PIN,
    .mosi_pin = SPI_MOSI_PIN,
    .miso_pin = SPI_MISO_PIN,
    .ss_pin = SPI_SS_PIN,
    .ss_active_high = false,
    .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0xFF,
    .frequency = NRF_SPIM_FREQ_1M,
    .mode = NRF_SPIM_MODE_1,
    .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    NRFX_SPIM_DEFAULT_EXTENDED_CONFIG /* Extended configuration */
  };

  /* Initialize SPIM peripheral with the configuration */
  uint32_t err_code = nrfx_spim_init(&spi, &spi_config, handle_spim_event, NULL);
  APP_ERROR_CHECK(err_code);
  
  /* Log that SPIM peripheral has been enabled */
  NRF_LOG_INFO("SPIM peripheral enabled.");
}
#endif


__STATIC_INLINE void ads_finish_reading_register(uint8_t *data) {
  memcpy(data, 2 + spi_rx_buf, xfer.rx_length - 2);
}

#if 0
__STATIC_INLINE void ads_start_reading_register(uint8_t reg_no, uint8_t count) {

  spi_tx_buf[0]  = ADS129X_RREG | (reg_no & 0x1F);
  spi_tx_buf[1]  = count & 0x1F;
  xfer.tx_length = 2;
  xfer.rx_length = count + 2;

  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);

  APP_ERROR_CHECK(err_code);
 
}
#else
__STATIC_INLINE void ads_start_reading_register(uint8_t reg_no, uint8_t count) {
  /*
    Start reading a specified register on the ADS129x device
    reg_no : the register number to read (0-31)
    count  : the number of bytes to read from the register (1-31)

  */

  /* Set the first byte of the SPI transfer to the RREG command and the register number */
  spi_tx_buf[0]  = ADS129X_RREG | (reg_no & 0x1F);
  
  /* Set the second byte of the SPI transfer to the number of bytes to read from the register */
  spi_tx_buf[1]  = count & 0x1F;
  
  /* Set the length of the SPI transfer to be 2 (command and register number) plus the number of bytes to read */
  xfer.tx_length = 2;
  xfer.rx_length = count + 2;

  /* Perform the SPI transfer using the nrfx_spim_xfer function */
  /* The third argument (0) is used to specify transfer options (0 for default settings) */
  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);

  /* Check for errors and halt execution if an error is detected */
  APP_ERROR_CHECK(err_code);
}
#endif

#if 0
__STATIC_INLINE void ads_read_reg(uint8_t reg_no, uint8_t count, uint8_t *data) {

  spi_xfer_done = false;
  ads_start_reading_register(reg_no, count);

  while (!spi_xfer_done) {
    __WFE();
  }

  ads_finish_reading_register(data);

}
#else
/**

    @brief Reads data from a specified register on the ADS129x device
    @param reg_no The register number to read (0-31)
    @param count The number of bytes to read from the register (1-31)
    @param data A pointer to a buffer to store the read data
    */
__STATIC_INLINE void ads_read_reg(uint8_t reg_no, uint8_t count, uint8_t *data) {
    
  spi_xfer_done = false; /* Set the SPI transfer flag to false */
  ads_start_reading_register(reg_no, count); /* Start reading the specified register */

  while (!spi_xfer_done) { /* Wait for the SPI transfer to complete */
    /* Use the Wait For Event (WFE) instruction to enter a low power state until */
    /* an event (in this case, the SPI transfer completion) wakes up the CPU     */
    __WFE();
  }
  
  ads_finish_reading_register(data); /* Finish reading the register and store the data in the provided buffer */

}
#endif


__STATIC_INLINE void ads_start_writting_register(uint8_t reg_no, uint8_t count, uint8_t *data) {

  spi_tx_buf[0] = ADS129X_WREG | (reg_no & 0x1F);
  spi_tx_buf[1] = count & 0x1F;

  xfer.tx_length = count + 2;
  xfer.rx_length = 1;
  
  memcpy(2 + spi_tx_buf, data, count);

  // spi_xfer_done = false;

  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);
  APP_ERROR_CHECK(err_code);

}

__STATIC_INLINE void ads_finish_writting_register(void) { 

  while (!spi_xfer_done) {
    __WFE();
  }

}

#if 0
__STATIC_INLINE void ads_write_reg(uint8_t reg_no, uint8_t count, uint8_t *data) {
  uint8_t temp_data[0x1f + 2] = {ADS129X_WREG | (reg_no & 0x1F), (count & 0x1F)};
  uint8_t tmp;

  memcpy(2 + temp_data, data, count);

  spi_xfer_done = false;

  nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(temp_data, count + 2, &tmp, 1);

  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);

  APP_ERROR_CHECK(err_code);
 
  while (!spi_xfer_done) {
    __WFE();
  }

}
#else
__STATIC_INLINE void ads_write_reg(uint8_t reg_no, uint8_t count, uint8_t *data) {
  
  spi_xfer_done = false;
  ads_start_writting_register(reg_no, count, data);
  ads_finish_writting_register();
}

#endif


#define DATA_LEN (27 + 1)


__STATIC_INLINE void ads_start_acquiring(void) {

  // register_pool_t r = ADS1298_CONFIG;

  spi_tx_buf[0] = ADS129X_RDATA;

  xfer.tx_length = 1;
  xfer.rx_length = 1 + 8 * ((HIGH_RES_32k_SPS == get_acquiring_mode()) ? 2 : 3);
  
  spi_xfer_done = false;

  uint32_t err_code = nrfx_spim_xfer(&spi, &xfer, 0);

  APP_ERROR_CHECK(err_code);

}
 
#if 0
__STATIC_INLINE unsigned ads_finish_acquiring(void *data, ads_convert_t conv_type, unsigned status_word_req) {

  static unsigned status;

  typedef struct {
     uint8_t low;
     uint8_t mid;
     uint8_t high;
  } unsigend_24_t;

  if (ADS_CONVERT_24_24 == conv_type) {

    unsigend_24_t *src = (unsigend_24_t*)&spi_rx_buf[1];
    uint8_t *dst = (uint8_t *) data;

    uint32_t d;

    for (size_t i = 1; i < DATA_LEN / 3; i++) {

      *(unsigend_24_t *)(&d) = src[i];          /* copy 24-bit value to 32-bit var          */
      d = __REV(d) >> 8;                        /* convert from big-endian to little-endian */
      memcpy(dst + (i - 1) * 3, &d, 3);
    
    }

    *(unsigend_24_t *)(&d) = src[0];
    d = __REV(d) >> 8;                          /* convert from big-endian to little-endian */
    d = (d << 4) & 0x00FFFFFF;                  /* alighn 'status register' bits            */
    status |= d;

    if (0 != status_word_req) {
      *(uint32_t*)(data + 10 * 8 * 3) = status | 0xFF000000;
      status = 0;
    }

  
    #if 0
      inbuf->low = 0xc1;
      inbuf->mid = 0x23;
      inbuf->high = 0x45;
    #endif
  
    // for (size_t i = 0; i < DATA_LEN / 3; i++) { 
    //   
    //   *(unsigend_24_t *)&data[i] = inbuf[i];  /* copy 24-bit value to 32-bit var          */
    //   data[i] = __REV(data[i]) >> 8;          /* convert from big-endian to little-endian */
    // 
    //   if ((i > 0) && (data[i] & 0x800000)) {  /* Check for negative number                */
    //     data[i] |= 0xFF000000;                /* Sign extend to 32-bit                    */
    //   }
    // }
    //
    // data[0] = (data[0] << 4) & 0x00FFFFFF;    /* alighn 'status register' bits            */

    // for (size_t i = 0; i < DATA_LEN / 3; i++) { 
    //   
    //   *(unsigend_24_t *)(dst + i) = src[i]; /* copy 24-bit value to 32-bit var          */
    //   dst[i] = __REV(dst[i]) >> 8;          /* convert from big-endian to little-endian */
    // 
    //   if ((i > 0) && (dst[i] & 0x800000)) {  /* Check for negative number                */
    //     dst[i] |= 0xFF000000;                /* Sign extend to 32-bit                    */
    //   }
    // }
    // 
    // dst[0] = (dst[0] << 4) & 0x00FFFFFF;    /* alighn 'status register' bits            */

    // uint8_t buf[8 * 3];


  } else if (ADS_CONVERT_24_16 == conv_type) {

    unsigend_24_t *src = (unsigend_24_t*)&spi_rx_buf[1];
    uint16_t buf[DATA_LEN / 3];

    for (size_t i = 1; i < DATA_LEN / 3; i++) { 
     
      buf[i] = __REV16(*(uint16_t*)&src[i]);               /* copy 24-bit value to 32-bit var          */

    }

    uint32_t d;

    *(unsigend_24_t *)(&d) = src[0];

    d = __REV(d) >> 8;
    d <<= 4;

    status |= d;
    
    memcpy(data, &buf[1], 8 * sizeof(uint16_t));

    if (0 != status_word_req) {
      d = status | 0xFF000000;                   /* Add packet-mark */
      memcpy(data + 8 * sizeof(uint16_t), &status, sizeof(status));
      status = 0;
    }

  } else if (ADS_CONVERT_16_16 == conv_type) {
  } else if (ADS_CONVERT_16_8 == conv_type)  {
  } else {
    APP_ERROR_CHECK((uint32_t)(conv_type));
  }

  return status_word_req;

}
#else

typedef struct {  /* Define a structure to represent a 24-bit unsigned integer */
    uint8_t low;  /* 8 least significant bits                                  */
    uint8_t mid;  /* next 8 bits                                               */
    uint8_t high; /* 8 most significant bits                                   */
} unsigned_24_t;


__STATIC_FORCEINLINE void rev24(void *p24) {
/*
  The function takes a pointer to a 24-bit value (represented as three contiguous bytes in memory) 
  and swaps the first and third bytes. This could be useful in various applications 
  where it is necessary to manipulate the order of bytes in a data stream, 
  such as in networking or file I/O operations.
  
  The function provides two options for performing the swap:
  
      Option 1: use a temporary variable to swap the bytes. This is a simple and 
                efficient method that avoids the need for XOR operations.
      Option 2: use XOR operations to swap the bytes. This method is more complex 
                and less efficient than using a temporary variable, 
                but it can be useful in certain situations where memory usage is a concern.
  
  Note that the function is defined as a static inline function using 
  the __STATIC_FORCEINLINE macro, which suggests that it is intended 
  for use in performance-critical code that requires fast execution.
*/

  #if 1 /* Option 1: use a temporary variable to swap the bytes */
    uint8_t t = *(uint8_t *)p24;             /* Store the first byte in a temporary variable */
    *(uint8_t *)p24 = *(uint8_t *)(p24 + 2); /* Copy the third byte to the first byte's location */
    *(uint8_t *)(p24 + 2) = t;               /* Copy the byte from the temporary variable to the third byte's location */
  
  #else /* Option 2: use XOR operations to swap the bytes (not currently used) */
    *(uint8_t *)p24 = *(uint8_t *)p24 ^ *(uint8_t *)(p24 + 2);
    *(uint8_t *)(p24 + 2) = *(uint8_t *)(p24 + 2) ^ *(uint8_t *)p24;
    *(uint8_t *)p24 = *(uint8_t *)p24 ^ *(uint8_t *)(p24 + 2);
  #endif
}

__STATIC_INLINE unsigned ads_finish_acquiring(const void *data, ads_convert_t conv_type, unsigned status_word_req) {

  /*
      Function to finish acquiring data from an ADS device
      data             : pointer to the buffer where the acquired data should be stored
      conv_type        : the type of conversion to perform on the acquired data
      status_word_req  : flag indicating whether to include a status word in the acquired data
      ========================================================================================
      Returns          : information about whether the package formation has been completed
  */
    
    if (data == NULL) { /* If the data pointer is NULL, return 0 */
        return 0;
    }

    // Initialize a static variable to hold the status word
    static unsigned status;
    uint8_t *dst = (uint8_t *) data;

    switch (conv_type) { /* Switch on the conversion type to determine how to convert the acquired data */
      case ADS_CONVERT_24_24: { /* Convert big-endian 24-bit values to little-endian 24-bit values */

        for (size_t i = 0; i < DATA_LEN; i += 3) {
          rev24(&spi_rx_buf[i + 1]);
        }

        memcpy(dst, &spi_rx_buf[4], 8 * 3); /* Copy the converted data to the output buffer */

        /* Extract the status word from the received data */
        uint32_t u32 = (uint32_t)  spi_rx_buf[1]       | 
                       ((uint32_t) spi_rx_buf[2] << 8) | 
                       ((uint32_t) spi_rx_buf[3] << 16);
        u32 <<= 4;
        u32 &= 0x00FFFFFF;
        status |= u32;

        if (status_word_req != 0) {         /* If a status word is requested, add it to the output buffer */
          u32 = status | 0xFF000000;
          memcpy(dst + 24, &u32, sizeof(status));
          status = 0;
        }

        break;
      }

      case ADS_CONVERT_24_16: {

        unsigned_24_t *src = (unsigned_24_t*)&spi_rx_buf[1];
        uint16_t buf[DATA_LEN / 3 - 1];
        
        for (size_t i = 1; i < DATA_LEN / 3; i++) { 
         
          buf[i - 1] = __REV16(*(uint16_t*)&src[i]);  /* convert big-endian 24-bit value to    */
                                                      /*     little-endian 16-bit one          */
        }
        
        uint32_t d;
        
        *(unsigned_24_t *)(&d) = src[0];
        
        d = __REV(d) >> 8;
        d <<= 4;
        
        status |= d;
        
        memcpy(dst, buf, 8 * sizeof(uint16_t));
        
        if (0 != status_word_req) {
          d = status | 0xFF000000;                   /* Add packet-mark */
          memcpy(dst + 8 * sizeof(uint16_t), &d, sizeof(d));
          status = 0;
        }

        break;
      }

      case ADS_CONVERT_16_16: {

        int16_t *src = (int16_t*)&spi_rx_buf[4];
        
        for (size_t i = 0; i < 8; i++) { 
         
          src[i] = __REV16(src[i]);                   /* convert big-endian 16-bit value to    */
                                                      /*     little-endian 16-bit one          */
        }
        
        uint32_t d;

        d  = (uint32_t) spi_rx_buf[1] << 20;
        d |= (uint32_t) spi_rx_buf[2] << 12;
        d |= (uint32_t) spi_rx_buf[3] << 4;
        
        status |= d;
        
        memcpy(dst, src, 8 * sizeof(uint16_t));
        
        if (0 != status_word_req) {
          d = status | 0xFF000000;                   /* Add packet-mark */
          memcpy(dst + 8 * sizeof(uint16_t), &d, sizeof(d));
          status = 0;
        }

        break;
      }

      default:
        // If an invalid conversion type is specified, return 0
        return 0;
    }

    return status_word_req; /* Return the status */
}
#endif

// void handle_pin_interrupt(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//     // Exception handler
//     __NOP();
// }


// typedef union {
//    status_reg_t status;
//    int channel[9];
// } ads1298_sample_t ;

//static int bkup[30][9];
//static int bk_ndx;

__STATIC_INLINE void init_ads1298(void) {

  /* Configure ADS1298 Chip */
  
  register_pool_t r, reg_pool;
  uint8_t n = sizeof(register_pool_t);

  nrf_gpio_cfg_output(ADS_RESET_PIN);

  nrf_gpio_pin_clear(ADS_RESET_PIN);
  nrf_delay_us(500);
  nrf_gpio_pin_set(ADS_RESET_PIN);
  
    /* From 9.5.2:                                                                 */
    /*   NDS Enable Read Data Continuous mode.                                     */
    /*   RDATAC 0001 0000 (10h) — This mode is the default mode at power up.       */
    /*   When in RDATAC mode, the RREG command is ignored.                         */

  ads_send_cmd(ADS129X_SDATAC);                              /* Disable RDTAC mode */

  /* Detect ADS1298 presence */

  ads_read_reg(ADS129X_ID, n, (uint8_t*)&reg_pool);          /* Read ADS129x ID    */

  if(ID_ADS1298 == reg_pool.ID) { 
    NRF_LOG_INFO("ADS1298 Analog Front-End detected:");
    NRF_LOG_HEXDUMP_INFO((uint8_t *)&reg_pool, ADS129X_REG_POOL_SIZE);
  } else {
    NRF_LOG_ERROR("No ADS129x Analog Front-End detected!");
  }

  //reg_pool = ADS1298_CONFIG;
  reg_pool.CONFIG1 = get_acquiring_mode();

  ads_write_reg(ADS129X_ID, n, (uint8_t*)&reg_pool);
  r = reg_pool;
  memset(&reg_pool, 0, n);
  ads_read_reg(ADS129X_ID, n, (uint8_t*)&reg_pool);

  if (0 == memcmp((void*)&r, (void*)&reg_pool, n)) {
    NRF_LOG_INFO("ADS1298 configuration applied.");
  }

  nrf_gpio_cfg_input(ADS_DATA_READY_PIN, NRF_GPIO_PIN_PULLUP);

  if (0 == nrf_gpio_pin_read(ADS_DATA_READY_PIN)) {
    NRF_LOG_INFO("ADS1298 DATA READY PIN IS LOW.");
  } else {
    NRF_LOG_INFO("ADS1298 DATA READY PIN IS HIGH.");
  }

  //set_acquiring_state(ACQUIRING_REQUESTED);

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


#if 0
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
#else
static void handle_twi_event(nrf_drv_twi_evt_t const * p_event, void * p_context) {

  /* 
    This function is an event handler for TWI events.
    It is called by the TWI driver when an event occurs.
  */

  /* Switch statement to handle different types of TWI events */
  switch (p_event->type) {

    /* This event indicates that a TWI transfer has completed */
    case NRF_DRV_TWI_EVT_DONE:

      /* Check if the transfer was an RX transfer (i.e. receiving data) */
      if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {

        /* Handle the received data using the handle_data() function */
        handle_data(m_sample);
      }

      /* Set the transfer done flag to true */
      m_xfer_done = true;

      /* Log a message indicating that the TWI transfer has completed */
      NRF_LOG_INFO("TWI transfer completed.");
      break;

    /* Default case for any other type of TWI event */
    default:
      /* Log a message indicating that a TWI event has occurred */
      NRF_LOG_INFO("TWI event occurred.");
      break;
  }
}
#endif

#if 0
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
#else
__STATIC_INLINE void init_twi(void) {

  /* This function initializes the TWI (I2C) peripheral on the microcontroller with specific settings. */

  /* Define a TWI configuration struct with the desired settings */
  const nrf_drv_twi_config_t twi_config = {
    .scl                = NRF_TWI_SCL_PIN,       /* Pin number for the SCL pin */
    .sda                = NRF_TWI_SDA_PIN,       /* Pin number for the SDA pin */
    .frequency          = NRF_DRV_TWI_FREQ_100K, /* TWI frequency set to 100kHz */
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH, /* Interrupt priority set to high */
    .clear_bus_init     = false                  /* Do not clear bus during initialization */
  };

  /* Initialize the TWI peripheral with the specified configuration and an event handler function */
  ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, handle_twi_event, NULL);
  APP_ERROR_CHECK(err_code);

  /* Enable the TWI peripheral */
  nrf_drv_twi_enable(&m_twi);

  /* Log a message indicating that the TWI peripheral has been enabled */
  NRF_LOG_INFO("TWIM peripheral enabled.");
}
#endif

#if USE_ADC != 0

#if 0
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
#else
__STATIC_INLINE void init_adc(void) {
  
  /* This function initializes the SAADC on the microcontroller with specific settings. */

  /* Set the ADC resolution to 12 bits */
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  /* Configure channel 0 to measure the VDD voltage and set the acquisition time to 15us */
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD;
  NRF_SAADC->CH[0].CONFIG = (
    SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos     |
    SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos
  );

  /* Set the oversampling rate to 64x */
  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over64x;

  /* Set the maximum number of results to 1 */
  NRF_SAADC->RESULT.MAXCNT = 1;

  /* Set the sampling rate to be triggered by a task */
  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

  /* Enable the SAADC */
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  /* Start the SAADC calibration and wait for it to finish */
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));

  /* Log a message indicating that the SAADC has been initialized */
  NRF_LOG_INFO("SAADC initialized.");
}
#endif

#if 0
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
#else
__STATIC_INLINE uint32_t measure_vdd(void) {

  /* 
    This function measures the voltage of the nRF52 microcontroller using the built-in SAADC
    and returns the voltage in millivolts (mV) as a 32-bit unsigned integer.
  */

  /* Create a volatile uint16_t variable to store the ADC value */
  volatile uint16_t res;

  /* Set the SAADC result pointer to point to the variable 'res' */
  NRF_SAADC->RESULT.PTR = (unsigned)&res;

  /* Clear the DONE event flag */
  NRF_SAADC->EVENTS_DONE = 0;

  /* Enable the SAADC */
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

  /* Start the SAADC and wait for the STARTED event */
  NRF_SAADC->TASKS_START = 1;
  while (NRF_SAADC->EVENTS_STARTED == 0);
  NRF_SAADC->EVENTS_STARTED = 0;

  /* Start the SAADC sampling and wait for the END event */
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0);
  NRF_SAADC->EVENTS_END = 0;

  /* Disable the SAADC */
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos;

  /* Convert the ADC value to millivolts and return it */
  return res * 3600UL / 4095;
}
#endif

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
  init_ads1298();
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

  //printf("All modules are started.\r\n");
  NRF_LOG_FLUSH();
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {

  /* Run the error handler if an assert in the SoftDevice is occured. */

  app_error_handler(DEAD_FACE, line_num, p_file_name);
}


__STATIC_FORCEINLINE void set_acquiring_state(acquiring_state_t s) {
  extern acquiring_state_t volatile acquiring_state;
  acquiring_state = s;
}

__STATIC_FORCEINLINE acquiring_state_t get_acquiring_state(void) {
  extern acquiring_state_t volatile acquiring_state;
  return acquiring_state;
}

__STATIC_FORCEINLINE void set_acquiring_mode(unsigned mode) {
  extern unsigned acquiring_mode;
  acquiring_mode = mode;
}

__STATIC_FORCEINLINE unsigned get_acquiring_mode(void) {
  extern unsigned acquiring_mode;
  return acquiring_mode;
}

/*
  HIGH_RES_32k_SPS
  HIGH_RES_16k_SPS
  HIGH_RES_8k_SPS
  HIGH_RES_4k_SPS
  HIGH_RES_2k_SPS
  HIGH_RES_1k_SPS
  HIGH_RES_500_SPS
  LOW_PWR_250_SPS
*/

unsigned acquiring_mode = HIGH_RES_8k_SPS;
acquiring_state_t volatile acquiring_state = ACQUIRING_INACTIVE;


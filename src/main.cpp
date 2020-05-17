#include <Arduino.h>

#include <WiFi.h>

#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "UlpDebug.h"

// Slow memory variable assignment
enum {
  //SLOW_BLINK_STATE,     // Blink status
  SLOW_PROG_ADDR        // Program start address
};

void ULP_CHARGE_PUMP_DRIVER() {
  // Slow memory initialization
  memset(RTC_SLOW_MEM, 0, 8192);

  // PumpChannel 0 + 1
  const int pump_chan_0 = RTCIO_GPIO32_CHANNEL + 14; // (specify by RTCIO_CHANNEL_9 + 14)
  const int pump_chan_1 = RTCIO_GPIO33_CHANNEL + 14; // (specify by RTCIO_CHANNEL_8 + 14)

  // GPIO2 (LED_BUILTIN) initialization (set to output and initial value is 0)
  rtc_gpio_init(GPIO_NUM_32);
  rtc_gpio_set_direction(GPIO_NUM_32, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_32, 0);

  rtc_gpio_init(GPIO_NUM_33);
  rtc_gpio_set_direction(GPIO_NUM_33, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_33, 0);

  // ULP Program
  const ulp_insn_t  ulp_prog[] = {
    M_LABEL(1),                            // LOOP
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_0, pump_chan_0, 1), // pump_chan_0 = 1
    I_DELAY(64),
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_0, pump_chan_0, 0), // pump_chan_0 = 0
    I_DELAY(4),
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_1, pump_chan_1, 1), // pump_chan_1 = 1
    I_DELAY(64),
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_1, pump_chan_1, 0), // pump_chan_1 = 0
    I_DELAY(4),
    M_BX(1)                                                  // LOOP END
  };

  // Run the program shifted backward by the number of variables
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(SLOW_PROG_ADDR, ulp_prog, &size);
  ulp_run(SLOW_PROG_ADDR);
}

static inline int32_t asm_ccount(void) {
    int32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}

void setup() {
  // For debug output
  Serial.begin(115200);

  ULP_CHARGE_PUMP_DRIVER();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  esp_deep_sleep_start();
  while(!Serial){}
  delay(2000);
  ulpDump();
}

void loop() {
  // For debug output
  //ulpDump();
  
  // Wait
  //Serial.println(RTC_SLOW_MEM[SLOW_BLINK_STATE] & 255,DEC);
  //delay(250);
}

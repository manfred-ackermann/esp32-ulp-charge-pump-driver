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

  // Blink status initialization
  //RTC_SLOW_MEM[SLOW_BLINK_STATE] = 6;

  // PumpChannel0
  const int pump_chan_0 = RTCIO_GPIO32_CHANNEL + 14; // (specify by RTCIO_CHANNEL_9 + 14)

  // PumpChannel1
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
    M_LABEL(97),                            // label_97 => INIT
    I_MOVI(R1, 7),                          // R1 = Sequence Size: 6

    M_LABEL(98),                            // label_98 => START
    I_SUBI(R1, R1, 1),                      // R1 = R1 - 1

                                            // => Sequenz 6
    I_ANDI(R0, R1, 6),                      // R0 = R1 AND 6
    M_BL(1,6),                              // IF R0 < 6 THAN GOTO M_LABEL(1)
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_0, pump_chan_0, 1),// pump_chan_0 = 1
    I_DELAY(60),                            // Compensate Sequenz 4/3/1 additional ticks
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(1),                             // label_1 => Sequenz 4
    I_ANDI(R0, R1, 4),                      // R0 = R1 AND 4
    M_BL(2,4),                              // IF R0 < 6 THAN GOTO M_LABEL(2)
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_0, pump_chan_0, 0),// pump_chan_0 = 0
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(2),                             // label_2 => Sequenz 3
    I_ANDI(R0, R1, 3),                      // R0 = R1 AND 3
    M_BL(3,3),                              // IF R0 < 3 THAN GOTO M_LABEL(3)
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_1, pump_chan_1, 1),// pump_chan_1 = 1
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(3),                             // label_3 => Sequenz 1
    I_ANDI(R0, R1, 1),                      // R0 = R1 AND 1
    M_BL(99,1),                             // IF R0 < 1 THAN GOTO M_LABEL(99)
    I_WR_REG(RTC_GPIO_OUT_REG, pump_chan_1, pump_chan_1, 0),// pump_chan_1 = 0
    I_DELAY(40),                            // Compensate missing M_BX

    M_LABEL(99),                            // label_99 => Finish up
    I_SUBI(R0, R1, 1),                      // R0 = R1 - 1

    M_BGE(98, 1),                           // IF R0 >= 1 THAN GOTO M_LABEL(98) START
    M_BX(97)                                // GOTO M_LABEL(97) INIT
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

  //esp_deep_sleep_start();
  while(!Serial){}
  delay(1000);
  ulpDump();
}

void loop() {
  // For debug output
  //ulpDump();
  
  // Wait
  //Serial.println(RTC_SLOW_MEM[SLOW_BLINK_STATE] & 255,DEC);
  //delay(250);
}

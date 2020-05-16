#include <Arduino.h>

#include <WiFi.h>

#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "UlpDebug.h"

// Slow memory variable assignment
enum {
  SLOW_BLINK_STATE,     // Blink status
  SLOW_PROG_ADDR        // Program start address
};

void ULP_CHARGE_PUMP_DRIVER() {
  // Slow memory initialization
  memset(RTC_SLOW_MEM, 0, 8192);

  // Blink status initialization
  RTC_SLOW_MEM[SLOW_BLINK_STATE] = 6;

  // PumpChannel0 (specify by +14)
  const int pin_blink_bit1 = RTCIO_GPIO32_CHANNEL + 14;
  const gpio_num_t pin_blink1 = GPIO_NUM_32;

  const int pin_blink_bit2 = RTCIO_GPIO33_CHANNEL + 14;
  const gpio_num_t pin_blink2 = GPIO_NUM_33;

  // GPIO2 (LED_BUILTIN) initialization (set to output and initial value is 0)
  rtc_gpio_init(pin_blink1);
  rtc_gpio_set_direction(pin_blink1, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(pin_blink1, 0);

  rtc_gpio_init(pin_blink2);
  rtc_gpio_set_direction(pin_blink2, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(pin_blink2, 0);

  // ULP Program
  const ulp_insn_t  ulp_prog[] = {
    M_LABEL(98),                            // label_98 => START
    I_MOVI(R3, SLOW_BLINK_STATE),           // R3 = SLOW_BLINK_STATE
    I_LD(R1, R3, 0),                        // R1 = RTC_SLOW_MEM[R3(SLOW_BLINK_STATE)]

                                            // => Sequenz 6
    I_ANDI(R0, R1, 6),                      // R0 = R1 AND 6
    M_BL(1,6),                              // IF R0 < 6 THAN GOTO M_LABEL(1)
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit1, pin_blink_bit1, 1),// pin_blink_bit = 1
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(1),                             // label_1 => Sequenz 4
    I_ANDI(R0, R1, 4),                      // R0 = R1 AND 4
    M_BL(2,4),                              // IF R0 < 6 THAN GOTO M_LABEL(2)
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit1, pin_blink_bit1, 0),// pin_blink_bit = 0
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(2),                             // label_2 => Sequenz 3
    I_ANDI(R0, R1, 3),                      // R0 = R1 AND 3
    M_BL(3,3),                              // IF R0 < 3 THAN GOTO M_LABEL(3)
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit2, pin_blink_bit2, 1),// pin_blink_bit = 1
    M_BX(99),                               // GOTO M_LABEL(99)


    M_LABEL(3),                             // label_3 => Sequenz 1
    I_ANDI(R0, R1, 1),                      // R0 = R1 AND 1
    M_BL(99,1),                             // IF R0 < 1 THAN GOTO M_LABEL(99)
    I_WR_REG(RTC_GPIO_OUT_REG, pin_blink_bit2, pin_blink_bit2, 0),// pin_blink_bit = 0


    M_LABEL(99),                            // label_99 => Finish up
    I_SUBI(R0, R1, 1),                      // R0 = R1 - 1

    M_BGE(4, 1),                            // IF R0 >= 1 THAN GOTO M_LABEL(4)
    I_MOVI(R0, 6),                          // R0 = 6

    M_LABEL(4),                             // M_LABEL(4)
    I_ST(R0, R3, 0),                        // RTC_SLOW_MEM[R3(SLOW_BLINK_STATE)] = R0
    M_BX(98)                                // GOTO M_LABEL(98)
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

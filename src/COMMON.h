/**
 * @file COMMON.h
 * @author roneydua (roneyddasilva@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-10-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef COMMON_H
#define COMMON_H
#include <Arduino.h>
/**
 * @brief Pisca o led #times vezes
 *
 * @param times Número de vezes que o led piscará.
 */
inline void blindLED(size_t times) {
  extern gpio_num_t onboard_led;
  int count = 0;

  for (size_t i = 0; i < 2 * times; i++) {
    digitalWrite(onboard_led, count);
    vTaskDelay(500);
    count = 1 - count;
  }
}
/**
 * @brief Liga o led de comunicação.
 *
 */
inline void turnOnLed(void) {
  extern gpio_num_t onboard_led;
  digitalWrite(onboard_led, LOW);
}
/**
 * @brief Desliga o led de comunicação.
 *
 */
inline void turnOffLed(void) {
  extern gpio_num_t onboard_led;
  digitalWrite(onboard_led, HIGH);
}

#endif /*COMMON_H*/
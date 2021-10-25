/**
 * @file: PARAMETROS.h
 * @author: roney
 * @date:   2021-05-12T13:52:07-03:00
 * email:  roneyddasilva@gmail.com
 * @brief [Sobre os conversores digitais
 * analógicos](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html)
 */
#ifndef PARAMETROS_H
#define PARAMETROS_H
#include <Arduino.h>

/**
 * @brief Pinos de conexão dos motores.
 *
 */
struct MOTOR_PINS {
  /*! Pino do motor 1 */
  const gpio_num_t motor_1 = GPIO_NUM_18;
  /*! Pino do motor 2 */
  const gpio_num_t motor_2 = GPIO_NUM_32;
  /*! Pino do motor 3 */
  const gpio_num_t motor_3 = GPIO_NUM_33;
  /*! Pino do motor 4 */
  const gpio_num_t motor_4 = GPIO_NUM_14;
};

/**
 * @brief Pinos de contadores de RPM.
 *
 */
struct RPM_PIN {
  /*! Pino de leitura do RPM do  motor 1 */
  const gpio_num_t pin_rpm_motor_1 = GPIO_NUM_18;
  /*! Pino de leitura do RPM do motor 2 */
  const gpio_num_t pin_rpm_motor_2 = GPIO_NUM_34;
  /*! Pino de leitura do RPM do motor 3 */
  const gpio_num_t pin_rpm_motor_3 = GPIO_NUM_25;
  /*! Pino de leitura do RPM do 4 */
  const gpio_num_t pin_rpm_motor_4 = GPIO_NUM_12;
};
/*! Led onboard para sinalização.*/
gpio_num_t onboard_led = GPIO_NUM_5;
/*! Porta Rx do controle */
const int8_t pin_rx_controle = 13;
/*! Período de temetria em milissegundos*/
const uint16_t dtTelemetry = 12;
/*! Periodo de telemetria em Ticks*/
const TickType_t xdtTelemetry = pdMS_TO_TICKS(dtTelemetry);
/*! Período de cotrole em milissegundos*/
const uint16_t dtControle = 15;
/*! Periodo de telemetria em Ticks*/
const TickType_t xdtControle = pdMS_TO_TICKS(dtControle);

#endif /* PARAMETROS_H */

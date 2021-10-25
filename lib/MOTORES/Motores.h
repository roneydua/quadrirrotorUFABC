/**
 * @author: Roney Silva (roneydua)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Motores.h
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */
#ifndef Motores_H
#define Motores_H
#include "Arduino.h"
class Motores {
private:
  /*! pino do motor */
  gpio_num_t motor_pin;
  /*! Canal utilizado pelo motor. */
  uint8_t canal = 0;
  /*! Tempo para calibracao entre atribuicao de minimo e maximo. milli segundos]
   */
  const uint16_t tempo_calibracao = 8000;
  //! Resolucao do controle PWM
  const uint16_t resolucao = 14;
  //! Frequencia do PWM de controle
  const double freq = 470.0;

public:
  Motores(gpio_num_t motor_pin, uint8_t canal = 0);
  /*!Minimo valor do duty cycle*/
  uint16_t min_vel;
  /*!Maximo valor do duty cycle*/
  uint16_t max_vel;
  // constantes de forcao dos motores
  // static void static_update_rpm(void *pvParameter);
  void inicializa();
  void set_portas();
  void calibra();
  void set_vel_mot(uint16_t vel);
};
#endif

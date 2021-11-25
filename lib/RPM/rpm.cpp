/**
 * @author: Roney Silva (roneydua)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: rpm.cpp
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */
#include "rpm.h"
#include <Arduino.h>

// Fora da classe eh declarado um ponteiro para a classe
Rpm *pointer_Rpm;
// defini-se um manipulador global
static void
outside_interrupt_handler(void *arg) { // define o manipulador global
  pointer_Rpm->update_rpm();           // chama membro da classe
}

Rpm::Rpm(gpio_num_t sensor, pcnt_unit_t pcnt_unit, uint8_t ciclos_por_volta) {

  pointer_Rpm = this;
  this->sensor = sensor;
  this->pcnt_unit = pcnt_unit;
  this->ciclos_por_volta = ciclos_por_volta;
  this->rpm_factor = 60 * this->MAX_H_LIM * 1000000 / this->ciclos_por_volta;
  init();
}

void Rpm::init() {
  gpio_set_pull_mode(this->sensor, GPIO_PULLUP_ONLY);
  pcnt_config_t pcnt_config = {
      .pulse_gpio_num = this->sensor, // configura a gpio de leitura
      .ctrl_gpio_num = -1,            // define o controle
      .lctrl_mode = PCNT_MODE_KEEP,   // when control signal is low, keep the
                                      // primary counter mode
      .hctrl_mode = PCNT_MODE_KEEP,   // when control signal is high, keep the
                                      // primary counter mode
      .pos_mode = PCNT_COUNT_INC,     // incrementa com a subida do sinal
      .neg_mode = PCNT_COUNT_DIS,     // ignora a contagem com a decida
      .counter_h_lim = MAX_H_LIM,     // limita o numero maximo de contagem
      .counter_l_lim = 0,             // limite inferior
      .unit = this->pcnt_unit,        // PCNT unit number
      .channel = PCNT_CHANNEL_0};
  pcnt_unit_config(&pcnt_config);
  time_array[index_time_array] = esp_timer_get_time();
  restart_conter();
  pcnt_get_counter_value(this->pcnt_unit, &pulses);
  update_rpm();
  /* set max value. Important! this interruption reset counter!*/
  pcnt_event_enable(this->pcnt_unit, PCNT_EVT_H_LIM);
  /* set a value different of maximum or minumum.*/
  pcnt_isr_service_install(0);
  // pcnt_isr_handler_add(this->pcnt_unit, outside_interrupt_handler,
  //                      (void *)pcnt_unit);
}
void Rpm::restart_conter() {
  pcnt_counter_pause(this->pcnt_unit);
  pcnt_counter_clear(this->pcnt_unit);
  pcnt_counter_resume(this->pcnt_unit);
  // change between 0 and 1 for update time_array
}

void Rpm::update_rpm() {
  index_time_array = 1 - index_time_array;
  time_array[index_time_array] = esp_timer_get_time();
  dt = time_array[index_time_array] - time_array[1 - index_time_array];
  rpm = this->rpm_factor / dt;
}

void Rpm::get_pulses() { pcnt_get_counter_value(this->pcnt_unit, &pulses); }

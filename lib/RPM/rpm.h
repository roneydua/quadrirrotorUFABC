/**
 * @author: Roney Silva (roneydua)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: rpm.h
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */
#include "driver/pcnt.h"
#include <Arduino.h>

class Rpm {

public:
  // Rpm(gpio_num_t sensor, pcnt_unit_t pcnt_unit = PCNT_UNIT_0,
  //     uint32_t sample_period_millisecond = 1000, uint8_t ciclos_por_volta =
  //     1);
  Rpm(gpio_num_t sensor, pcnt_unit_t pcnt_unit = PCNT_UNIT_0,
      uint8_t ciclos_por_volta = 1);

  int16_t rpm = 2;
  void update_rpm();
  // uint64_t get_pulses();
  int8_t ciclos_por_volta;
  const int16_t MAX_H_LIM =
      28 + 1;          // Numero maximo de pulsos para chamar a interrupcao
  uint32_t rpm_factor; // 1000000 * (MAX_H_LIM / ciclos_por_volta)
  void restart_conter();
  pcnt_unit_t pcnt_unit; // numero da contadora de pulso

  int16_t pulses = 0;
  void get_pulses();

private:
  gpio_num_t sensor;
  void init();
  // float sample_periode_second_normalized_per_cycle;
  // this array stores the last two get_time
  uint64_t time_array[2] = {};
  uint8_t index_time_array = 0;
  uint32_t dt = 1000000;

protected:
};

/**
* @file TIME.cpp
* @author Roney D da Silva
* @date 5 May 2021
* @copyright 2021 Roney D da Silva
         Email: roneyddasilva@gmail.com
* @brief Classe para medir Intervalo de tempo entre duas chamadas.
*/

#include "TIME.h"
#include <Arduino.h>
/**
 * @brief Instância da classe Time.
 * @details Utilizando o temporizador de alta resolução. [timer
 * esp_timer_get_time()](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html#_CPPv418esp_timer_get_timev)
 */
TIME::TIME() { begin(); }
/**
 * Inicialização da classe Time.
 */
void TIME::begin() {
  timeVector[0] = esp_timer_get_time();
  timeVectorIndex = 0;
}
/**
 * @brief Retorna o valor do tempo decorrido desde a última requisição.
 * @return float de tempo decorrido em segundos.
 * @note
 * [esp_timer_get_time()](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html#_CPPv418esp_timer_get_timev)
 * Computa o tempo decorrido em ms desde a Inicialização.
 */
float TIME::computeElapsedTime() {
  timeVectorIndex = 1 - timeVectorIndex;
  timeVector[timeVectorIndex] = esp_timer_get_time();
  return 1e-6 *
         float((timeVector[timeVectorIndex] - timeVector[1 - timeVectorIndex]));
}

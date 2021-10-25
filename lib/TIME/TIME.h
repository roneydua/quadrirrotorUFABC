/**
* @file TIME.h
* @author Roney D da Silva
* @date 5 May 2021
* @copyright 2021 Roney D da Silva
         Email: roneyddasilva@gmail.com
* @brief Classe para medir Intervalo de tempo entre duas chamadas.
*/

#ifndef TIME_h
#define TIME_h

#include <Arduino.h>
class TIME {
public:
  // construtor
  TIME();
  float computeElapsedTime();
  void begin();

private:
  /*! Indice do vetor. */
  uint8_t timeVectorIndex = 0;
  /*! Vetor com as ultimas duas medidas. */
  uint64_t timeVector[2] = {};
};

#endif
/* TIME_H_ */

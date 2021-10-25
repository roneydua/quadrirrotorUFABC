/**
 * @author: roney
 * @date:   2021-05-07T10:25:15-03:00
 * email:  roneyddasilva@gmail.com
 * @File: TELEMETRIA.h
 * Last modified by:   roney
 * Last modified time: 2021-07-19T18:33:13-03:00
 */

#ifndef TELEMETRIA_h
#define TELEMETRIA_h
#include <eigen3/Eigen/Dense>

#include "Arduino.h"
#include "WiFi.h"
extern "C" {
#include "esp_now.h"
}
#define DEBUG_BY_SERIAL
#define DATA_SIZE 26

using namespace Eigen;

class TELEMETRIA {

public:
  TELEMETRIA();
  void begin();
  void envia();
  void enviaTunningGains();
  void enviaMensagem(char *aviso);
  uint8_t _broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  void pisca(uint8_t porta);
  /**
   * @brief Estrutura de mensagem.
   * @details Tamanho 72
   *
   */
  struct IMU_DATA {
    /*! Tipo de mensagem. */
    // uint16_t quantidadeValida;
    union {
      float dados[DATA_SIZE];
      Vector<float, DATA_SIZE> dadosEigen = VectorXf::Zero(DATA_SIZE);
    };
  } imu;

  float gainsTunning[3];

  /**
   * @brief Mensagem de texto para envio.
   * @details Tamanho 35 Bytes
   */
  struct MENSAGEM {
    char info[35]; // dados para informacao;
  };
  typedef struct __attribute__((packed)) controle {
    uint8_t dados;
  } controle;

  static uint8_t comAceito;
  uint8_t _quantDados;

private:
  static void OnDataRecv(const uint8_t *_broadcast, const uint8_t *data,
                         int data_len);
};
#endif

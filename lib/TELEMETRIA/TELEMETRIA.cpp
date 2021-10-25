/**
 * @author: roney
 * @date:   2021-05-07T10:25:15-03:00
 * email:  roneyddasilva@gmail.com
 * @file: TELEMETRIA.cpp
 * Last modified by:   roney
 * Last modified time: 2021-05-07T10:27:01-03:00
 */

#include "TELEMETRIA.h"
#include "Arduino.h"

/**
 * Construtor
 */
TELEMETRIA::TELEMETRIA() { begin(); }
/**
 * Inicialização da telemetria
 */
void TELEMETRIA::begin() {
  // disconeta WIfi para evitar instabilidades
  WiFi.disconnect();
  // inicializa wifi em modo station
  WiFi.mode(WIFI_STA);
  // tenta estabelecer coneccao
  while (esp_now_init() != ESP_OK) {
    ESP.restart();
  };
  Serial.println("ESP Inicializado");
  // Criamos uma variável que irá guardar as informações do slave
  esp_now_peer_info_t slave;
  slave.channel = 1; // Informamos o canal
  slave.encrypt = 0; // 0 para não usar criptografia ou 1 para usar
  // Copia o endereço do array para a estrutura
  memcpy(slave.peer_addr, _broadcast, sizeof(_broadcast));

  esp_now_add_peer(&slave); // Adiciona o slave
  esp_now_register_recv_cb(OnDataRecv);
}
/**
 * @brief Envia vetor Eigen de float.
 *
 * @param quant Dimensão maxima de 20
 * @param _a Vetor com dados.
 */
void TELEMETRIA::envia() {
// Array que irá armazenar os valores lidos
// IMU_DATA imu;
// float temp[_a.size()];
// imu.quantidadeValida = quant;
#ifdef DEBUG_BY_SERIAL
  for (uint16_t i = 0; i < imu.dadosEigen.size(); i++) {
    printf("%f  ", imu.dados[i]);
  }
  printf("\n");
#endif

  uint8_t bs[DATA_SIZE * 4];

  memcpy(&bs, &imu, sizeof(bs));
  esp_now_send(_broadcast, bs, sizeof(bs));
}
void TELEMETRIA::enviaTunningGains() {

  uint8_t bs[sizeof(gainsTunning)];
  memcpy(&bs, &gainsTunning, sizeof(gainsTunning));
  esp_now_send(_broadcast, bs, sizeof(bs));
}
void TELEMETRIA::enviaMensagem(char *aviso) {
  MENSAGEM msg;

  strcpy(msg.info, aviso);
#ifdef DEBUG_BY_SERIAL
  printf(msg.info);
  printf("\n");
#endif
  // Serial.println(msg.info);
  uint8_t bs[sizeof(msg)];
  memcpy(&bs, &msg, sizeof(msg));
  esp_now_send(_broadcast, bs, sizeof(bs));
}
// funcao responsavel por receber dados
void TELEMETRIA::OnDataRecv(const uint8_t *_broadcast, const uint8_t *data,
                            int data_len) {
  TELEMETRIA TELEMETRIA;
  // Para cada pino
  controle ctrl;
  memcpy(&ctrl, data, sizeof(controle));
  // atualizacontrole(ctrl.dados);
  TELEMETRIA::comAceito = ctrl.dados;
}
uint8_t TELEMETRIA::comAceito;
void TELEMETRIA::pisca(uint8_t porta) {
  digitalWrite(porta, HIGH);
  delay(1000);
  digitalWrite(porta, LOW);
  delay(1000);
}

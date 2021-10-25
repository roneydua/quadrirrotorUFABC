/**
 * @author: Roney Silva (roneydua)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Motores.cpp
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */
#include "Motores.h"
#include "Arduino.h"

/**
 * @brief Construtor da classe motor.
 * @param [in] motor_pin GPIO que ESC esta conectado
 * @param [in] canal que o PWM sera conectado
 */
Motores::Motores(gpio_num_t motor_pin, uint8_t canal) {
  this->motor_pin = motor_pin;
  this->canal = canal;
  min_vel = int((pow(2, resolucao) - 1) * 0.001 * freq);
  max_vel = int((pow(2, resolucao) - 1) * 0.002 * freq);
  inicializa();
}
/**
 * @brief configuracao das frequencias PWM que o esc's utilizam.
 */
void Motores::inicializa() {
#ifdef DEBUG_MOTOR
  Serial.println("Configurando os ESc'");
  delay(100);
#endif
  ledcSetup(this->canal, this->freq, resolucao);
  set_portas();
  set_vel_mot(min_vel);
}
/**
 * @brief funcao que muda a velocidade.
 * @param vel inteiro que possui valor entre min_vel e max_vel
 */

void Motores::set_vel_mot(uint16_t vel) { ledcWrite(this->canal, vel); }
void Motores::set_portas() {
// define como saida cada pino ligado aos motores
#ifdef DEBUG_MOTOR
  delay(1000);
  Serial.println("Inicializando");
#endif
  pinMode(this->motor_pin, OUTPUT);
  ledcAttachPin(this->motor_pin, this->canal);
}
/**
 * @brief Rotina de calibração dos motores.
 *
 */
void Motores::calibra() {

  // escrevendo o valor inicial
  Serial.println("Ligue Os esc's na bateria");
  Serial.println("Atribuindo maximo");
  ledcWrite(this->canal, max_vel);
  while (Serial.parseInt() != 1) {
    Serial.println("Conecte a bateria no ESC e digite 1 para proseguir");
    delay(1000);
  }
  delay(tempo_calibracao);
  Serial.println("Atribuindo minimo");
  ledcWrite(this->canal, min_vel);
  delay(tempo_calibracao);
  Serial.println("Calibrado.");
  Serial.println("Desligue seu ESC da bateira e tambem da placa de comando");

  delay(10000);
};

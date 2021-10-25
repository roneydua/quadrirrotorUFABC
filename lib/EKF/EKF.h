/**
 * @author: Roney Silva (roney)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: EKF.h
 * Last modified by:   roney
 * Last modified time: 01-Sep-2021
 * @brief Classe que implementa o filtro de Kalman estendido
 */

#ifndef EKF_H
#define EKF_H
#include "eigen3/Eigen/Dense"
#ifdef __XTENSA__
#include "../../src/COMMON.h"
#include "../AQUA/AQUA.h"
#include "../GRUPO_QUAT/GRUPO_QUAT.h"
#include <Arduino.h>
#else
// #include "GPS.h"
#include "../../include/READWRITEEIGEN/readWriteEigen.h"
#include "../AQUA/AQUA.h"
#include "../GRUPO_QUAT/GRUPO_QUAT.h"
#include <iostream>
#define PM(X) cout << #X << ":\n" << X << "\n"
#endif
// #define DEBUG_EKF
struct BASE {
  /*! Latitude em Radianos*/
  float lat;
  /*! Longitude em Radianos*/
  float lon;
  /*! Altitude ao nível do mar*/
  float hMSL;
  /*! Matriz de transformação */
  Eigen::Matrix3f RotEcef2ned;
  /*! Coordenadas da base no sistema ECEF*/
  long rECEF[3];
};

namespace ekf {
/*! Enum Status Filter*/
enum _status { BEGIN, READY, FAIL, STOPED };
class EKF {
private:
  const float DEG_TO_RAD_BY_10000000 =
      1e-7 / 57.295779513082320876798154814105f;

public:
  /*! Status do filtro*/
  int status = BEGIN;
  /*! Número de estados do sistema completo.*/
  static const int nFull = 15;
  /*! Número total de observações*/
  static const int pFull = 7;
  /*! Número de estados de atitude.*/
  const int nAtitude = 6;
  /*! Número de observações da atitude*/
  const int pAtitude = 4;
  EKF();
  BASE base;
  void begin(float dt);
  void loopEKF();
  /*! Intervalo de tempo do Loop.*/
  // float dtLoop = 0.0f;
  /*! Intervalo de tempo para integração da atitude*/
  float dt = 0.0f;
  /*! Vetor com as medidas do acelerometro*/
  Eigen::Vector3f accel = Eigen::Vector3f::Zero();
  /*! Gravidade no sistema de navegação*/
  const Eigen::Vector3f navegationGravity{0.0f, 0.0f, 9.786171951281709f};
  /*! Vetor com as medidas do giroscopios */
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  /*! Vetor com as medidas do magnetometros*/
  Eigen::Vector3f mag = Eigen::Vector3f::Zero();
  /*! Vetor com as medias do magnetometro sem as projeções na direção do vetor
   * gravitacional */
  /*! Quaternion de observação */
  Eigen::Vector4f qObs{1.0f, 0.0f, 0.0f, 0.0f};
  /*! Quaternion de atitude */
  Eigen::Vector4f q{1.0f, 0.0f, 0.0f, 0.0f};
  /*! Ângulos de Euler na convenção 3-2-1 Tait-Bryan */
  Eigen::Vector3f euler{0.0f, 0.0f, 0.0f};
  /*! Vetor de estados estimados do EKF. O erros são: as três componetes da
 parte vetorial do quaternion, os três bias do giroscópio e as compoenentes da
 velocidade no sistema NED. [ &delta;**q** &delta;**b**
 * &delta;**r**] */
  /*! Posicoes no sistema NED */
  Eigen::Vector3f rNED = Eigen::Vector3f::Zero();
  /*! Posicao no sistema NED de observação.*/
  Eigen::Vector3f rNEDgps = Eigen::Vector3f::Zero();
  /*! Velocidade linear m/s*/
  Eigen::Vector3f drNED = Eigen::Vector3f::Zero();
  /*! Velocidade linear sistema NED de observação.*/
  Eigen::Vector3f drNEDgps = Eigen::Vector3f::Zero();
  /*! Aceleração linear m/s^2*/
  Eigen::Vector3f ddr = Eigen::Vector3f::Zero();
  /*! Matrix de cossenos diretores */
  Eigen::Matrix3f mcd = Eigen::Matrix3f::Identity();
  uint8_t messageGPS = 0;
  /*! Número de linhas da matriz H. Assume 3 quando apenas a MARG está
   * disponível e 6 quando MARG/GPS estão disponíveis. */
  int p = pFull;
  /*! Números de estados. q, b_giro, vel, b_accel, posicao*/
  int n = 15;
  /*! Quadrado do desvio padrão do giroscópio.*/
  const float sigma_giro_squared = 1e-3;
  /*! Quadrado do desvio padrão do bias do giroscópio.*/
  const float sigma_bias_giro_squared = 1e-6;
  /*! Quadrado do desvio padrão do acelerômetro.*/
  const float sigma_accel_squared = 1e-3;
  /*! Quadrado do desvio padrão do bias do acelerômetro.*/
  const float sigma_bias_accel_squared = 1e-6;
  /*! Quadrado do desvio padrão da observação do quatérnion.*/
  const float sigma_aqua_squared = 1e-3;
  /*! Quadrado do desvio padrão da velocidade no gps.*/
  const float sigma_vel_squared = 0.02f * 0.02f;
  /*! Quadrado do desvio padrão da posição no gps.*/
  const float sigma_pos_squared = 5.0f * 5.0f;
  /*! Matriz de covariancia de processo discretizada. */
  Eigen::Matrix<float, nFull, nFull> Q_k =
      Eigen::MatrixXf::Identity(nFull, nFull);
  /*! Matriz de covariancia de medidas.*/
  Eigen::Matrix<float, pFull, pFull> R =
      Eigen::MatrixXf::Identity(pFull, pFull);
  /*! Matrix de estados linearizada. @note mF é utilizado como simbolo de F pq
   * F é um simbolo reservado.*/
  Eigen::Matrix<float, nFull, nFull> mF = Eigen::MatrixXf::Zero(nFull, nFull);
  /*! Matriz Jacobiana dos estados. */
  Eigen::Matrix<float, nFull, nFull> fi = Eigen::MatrixXf::Zero(nFull, nFull);
  /*! Matriz G de ruido do processo. */
  Eigen::Matrix<float, nFull, 12> G = Eigen::MatrixXf::Zero(nFull, 12);
  /*! Matriz Covariancia */
  Eigen::Matrix<float, nFull, nFull> P =
      ((Eigen::VectorXf(n) << .5, .5, .5, .1, .10, .10, 1, 1, 1, .1, .10, .10,
        1e8, 1e8, 1e8)
           .finished())
          .asDiagonal();
  /*! Matriz sensibilidade */
  Eigen::Matrix<float, pFull, nFull> H = Eigen::MatrixXf::Zero(pFull, nFull);
  /*! Ganho de Kalman */
  Eigen::Matrix<float, nFull, pFull> K =
      Eigen::MatrixXf::Identity(nFull, pFull);
  /*! Vetor de estados perturbados do estados estimados do EKF.
 @warning deltaChi possui dimensão 15+1, o primeiro elemento não é calculado
 pelo
 * filtro mas computado na #updateStates().
 */
  Eigen::Vector<float, nFull + 1> deltaChi = Eigen::VectorXf::Zero(nFull + 1);
  /*! Erro das observações */
  Eigen::Vector<float, pFull> deltaYObs = Eigen::VectorXf::Zero(pFull);
  // /*! Observações preditas */
  // Eigen::VectorXf yPreditas = Eigen::VectorXf::Zero(15);
  void integrationOfStates();
  void updateStates();
  void predictionStage();
  void updateStage();
  void updateF();
  void updateFi();
  void updateHq();
  void updateHv();
  void updateHp();

#ifdef __XTENSA__
  void calibrationMethods();
#endif
  void updateOfMeasurements();
  void positionIntegration();
  void updateBaseRef();
  void updateNedPos();
  void updateNedVel();

  /*! Vetor ponteiro do bias do giroscópio */
  Eigen::Vector3f *biasGyro;
  /*! Vetor ponteiro do bias do acelerômetro */
  Eigen::Vector3f *biasAccel;
};
} // namespace ekf

#endif
/* EKF_H */
/**
 * @author: Roney Silva (roney)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: EKF.cpp
 * Last modified by:   roney
 * Last modified time: 01-Sep-2021
 * @brief Classe que implementa o filtro de Kalman estendido
 */

#include "EKF.h"
#ifdef __XTENSA__
#include "GPS.h"

#include "IMU.h"
#include "TIME.h"
#include <Arduino.h>
#endif
// #include "../AQUA/AQUA.h"
// #include "../GRUPO_QUAT/GRUPO_QUAT.h"

namespace ekf {
/**
 * Classe do filtro de Kalman.
 */
#ifdef __XTENSA__
IMU imu(Wire, 0x68);
TIME timeClass;
TIME timeClassLoop;
#endif
AQUA aqua;
EKF::EKF() {}
/**
 * @brief Inicializacao do filtro de Kalman
 * @details Inicializa e configura:o GPS, a classe timeClass, instancia os
 * sensores #accel, #gyro e #mag; e Configura os valores padrões da matrizes #F,
 * #G e #H.
 */
void EKF::begin(float _dt) {
#ifdef __XTENSA__
  printf("Nucleo do Filtro: %d\n", xPortGetCoreID());
#endif
  status = BEGIN;
  this->dt = _dt;
#ifdef __XTENSA__
  gpsSetup();

  timeClass.begin();
  // timeClassLoop.begin();

#endif

  // Instancia o vetor de medidas na biblioteca da IMU.
  /*!HACK: @hack #accel, #gyro e #mag são ponteiros. Quando as medidas são
   * atualizadas na biblioteca IMU, o vetor é modificado na EKF. */
#ifdef __XTENSA__
  imu.begin(accel, gyro, mag);
  // referencia o bias da classe IMU.
  biasGyro = &imu._biasGyro;
  biasAccel = &imu._biasAccel;
#endif
  // Atualiza valor do bias do giroscópio com uma média de amostras
  // valores constantes das matriz F
  mF.block<3, 3>(0, 3) = -0.5f * Eigen::Matrix3f::Identity();
  mF.block<3, 3>(12, 6) = Eigen::Matrix3f::Identity();
  // Valores constantes da matriz G
  G.topLeftCorner(3, 3) = -0.5f * Eigen::Matrix3f::Identity();
  G.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();
  G.block(6, 6, 3, 3) = -Eigen::Matrix3f::Identity();
  G.block<3, 3>(9, 9) = Eigen::Matrix3f::Identity();
  // matrix de covariâncias do processo continuo
  Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(12, 12);
  Eigen::Matrix3f _I = Eigen::Matrix3f::Identity();
  Q.block<3, 3>(0, 0) = _I * _dt * _dt * sigma_giro_squared;
  Q.block<3, 3>(3, 3) = _I * _dt * sigma_bias_giro_squared;
  Q.block<3, 3>(6, 6) = _I * _dt * _dt * sigma_accel_squared;
  Q.block<3, 3>(9, 9) = _I * _dt * sigma_bias_accel_squared;
  Q_k = G * Q * G.transpose();
  R.block<4, 4>(0, 0) = Eigen::Matrix4f::Identity() * sigma_aqua_squared;

#ifndef __XTENSA__
  // Q_k = csvLeia<Eigen::MatrixXf>("Q");
  // R = csvLeia<Eigen::MatrixXf>("R");
#endif
  // condições iniciais da posicao
  aqua.begin(qObs, accel, mag);
#ifdef __XTENSA__
  updateBaseRef();
  imu.readSensor();
#endif
  // valor inicial a partir da medida
  aqua.computeAQUAQuaternion();
  q = qObs;
  // aqua.qAlignment = qObs;
  status = READY;
}
/*!
 *  Loop do Filtro de Kalman
 */
void EKF::loopEKF() {
  // le os sensores
  // timeClassLoop.computeElapsedTime();
  updateOfMeasurements();
  predictionStage();
  updateStage();
  updateStates();
  // dtLoop = timeClassLoop.computeElapsedTime();
}

/**
 *  Integra os estados e a matriz de covariancia a priori do filtro.
 */
void EKF::predictionStage() {
  // Integracao dos estados
  integrationOfStates();
  updateF();
  updateFi();
  // atualiza a matriz de covariâncias
  P = fi * P * fi.transpose() + Q_k;
}
/**
 * Atualiza a medicao a partir dos sensores da unidade Margin e GPS.
 * @details Faz a leitura dos sensores da MARG atualizando os vetores #accel,
 * #gyro e #mag e ElapsedTime #dt.
 * Atualiza as medidas do GPS quando estiverem disponíveis, o valor da variável
 * #p e bloco da matriz H correspondente.
 */
void EKF::updateOfMeasurements() {
#ifdef __XTENSA__
  // Faz a leitura dos sensores da MARG atualizando os vetores #accel, #gyro e
  imu.readSensor();
  // tempo decorrido entre medidas a medida anterior e a atual.
  // dt = timeClass.computeElapsedTime();
  // Etapa de predição. Verifica se há medidas novas a partir do GPS. A função
  // processoGPS() retorna a MT_NONE, MT_NAV_POSLLH, MT_NAV_VELNED ou MT_NAV_PVT
  // dependendo da configuração no cabeçalho GPS.h.
  messageGPS = processGPS();
  /*! @note MT_NONE é avalidado primeiro uma vez que a frequência do GPS é
menor que a da MARG.*/
  /*! @todo É possível que essas contas sejam realizadas no
   * cabeçalho, eliminando-se alguns condicionais abaixo.*/
  // p e n corresponde aos numeros de medidas e estados.
  if (messageGPS == MT_NONE) {
    // não ha observações de posição ou velocidade
    p = pAtitude;
  } else if (messageGPS == MT_NAV_POSECEF) {
    // há observação de posicao
    p = pFull;
    // Atualiza o erro de medida na velocidade linear
    updateNedPos();
    // Atualiza matriz covariancia da matriz de observação da posição
    R.block<3, 3>(4, 4) = sigma_pos_squared * Eigen::Matrix3f::Identity();
    deltaYObs.tail(3) = rNEDgps - rNED;
  } else if (messageGPS == MT_NAV_PVT) {
    p = pFull;
    // há observações de velocidade
    updateNedVel();
    // Atualiza matriz covariancia da matriz de observação da velocidade
    R.block<3, 3>(4, 4) = sigma_vel_squared * Eigen::Matrix3f::Identity();
    deltaYObs.tail(3) = drNEDgps - drNED;
  }
#endif
  // computa o quaternion de observação com o algoritmo #AQUA.
  aqua.computeAQUAQuaternion();
}
/**
 * @brief Atualiza matrix lineariza de estados #F e a matrix #G.
 */
void EKF::updateF() {
  /*! @hack Apenas parte da matrix é atualizada*/
  mF.topLeftCorner(3, 3) = -skew(gyro);
  // TEST: Matrix mF atualizada em todos os loops
  mF.block<3, 3>(6, 0) = -2.0 * mcd * skew(accel);
  mF.block<3, 3>(6, 9) = -mcd;
}
/**
 * @brief
 * @todo Oportunidade de otimização. Talves, na programação, a matrix F
 * não precise existir e seja absorvida pela matrix fi.
 */
void EKF::updateFi() {
  // fi = I+dt * F
  fi = Eigen::MatrixXf::Identity(nFull, nFull) + (mF * (0.5f * dt) + mF) * dt;
}

/**
 * Integra os estado #q e #r;.
 */
void EKF::integrationOfStates() {
  // Quatérnio com Euler de primeira ordem
  integrationQuaternion(q, gyro, dt);
  // atualiza matriz de rotação com o quaternio predito
  computeMcdFromQuaternion(q, mcd);
  Eigen::Vector3f _accel_g = dt * (mcd * accel - navegationGravity);
  // integracao da Posicao NED
  rNED += (drNED + 0.5f * _accel_g) * dt;
  // integracao da Velocidade
  drNED += _accel_g;
}
/**
 * @brief Etapa de atualização.
 */
void EKF::updateStage() {
  // Ganho de Kalman
  deltaYObs.head(4) = qObs - q;
  updateHq();
  K.leftCols(p) =
      P * H.topRows(p).transpose() *
      (H.topRows(p) * P * H.topRows(p).transpose() + R.topLeftCorner(p, p))
          .inverse();
  // Atualiza a matriz de covariância.
  P -= K.leftCols(p) * H.topRows(p) * P;
  // Computa o erro de estimação #deltaChi
  deltaChi.segment(1, n) = K.leftCols(p) * deltaYObs.head(p);
}
/**
 * Atualiza os estados predidos com os erros estimados.
 * @note Quando há medidas do GPS o bias do acelerômetro, a velocidade NED e a
 * posição Geodésica são atualizadas.
 */
void EKF::updateStates() {
  // calcula o escalar do erro de estimação.
  deltaChi(0) = sqrtf(1.0f - deltaChi.segment<3>(1).squaredNorm());
  // Atualiza a orientação.
  q = multiplyQuaternions(q, deltaChi.head(4));
  // Atualiza o bias do giroscópio.
  *biasGyro += deltaChi.segment<3>(4);
  /*! @attention dcm é uma referência. */
  // Corrige a velocidade de translacao.
  drNED += deltaChi.segment<3>(7);
  // Corrige o bias do acelerometro.
  *biasAccel += deltaChi.segment<3>(10);
  // Corrige a posição Geodésica
  rNED += deltaChi.segment<3>(13);
}
/**
 * @brief Atuliza o bloco pertinente á atitude na matriz #H
 */
void EKF::updateHq() { H.block<4, 3>(0, 0) = Q_l(q); }
/**
 * @brief Atualiza parte relativa á posição da matriz de sensitividade.
 */
void EKF::updateHp() {
  H.block<3, 3>(4, 6) = Eigen::Matrix3f::Zero();
  H.block<3, 3>(4, 12) = Eigen::Matrix3f::Identity();
}
/**
 * @brief Atualiza parte relatica á velocidade da matriz de sensitividade.
 */
void EKF::updateHv() {
  H.block<3, 3>(4, 6) = Eigen::Matrix3f::Identity();
  H.block<3, 3>(4, 12) = Eigen::Matrix3f::Zero();
}

#ifdef __XTENSA__
/**
 * @brief Função que armazena as coordenas da base.
 * @details É atualizada a estrutura #base.
 */
void EKF::updateBaseRef() {
  blindLED(2);
  // raio Meridional
  base.RotEcef2ned = Eigen::Matrix3f::Identity();
  base.rECEF[0] = 0;
  base.rECEF[1] = 0;
  base.rECEF[2] = 0;
  int _mensagem = processGPS();
  ubxMessage.navPosecef.pAcc = 1000;
  while (ubxMessage.navPosecef.pAcc > 300) {
    while (true) {
      _mensagem = processGPS();
      if (_mensagem == MT_NAV_POSECEF)
        break;
    }
#ifdef DEBUG_EKF
    printf("mensagem: %d\t%lu\n", _mensagem, ubxMessage.navPosecef.pAcc);
#endif
    vTaskDelay(100);
    // mensagem em cm/s e em long para subtrair na transformacao ECEF->NED
    base.rECEF[0] = ubxMessage.navPosecef.ecefX;
    base.rECEF[1] = ubxMessage.navPosecef.ecefY;
    base.rECEF[2] = ubxMessage.navPosecef.ecefZ;
  }
  while (ubxMessage.navPvt.fixType != 3) {
    while (true) {
      _mensagem = processGPS();
      if (_mensagem == 2)
        break;
    }
  }
#ifdef DEBUG_EKF
  printf("%d\n", ubxMessage.navPvt.fixType);
#endif
  base.lat = (float)ubxMessage.navPvt.lat * DEG_TO_RAD_BY_10000000;
  base.lon = (float)ubxMessage.navPvt.lon * DEG_TO_RAD_BY_10000000;
  base.hMSL = (float)ubxMessage.navPvt.hMSL * 1e-3;
  // updade Matriz ECEF -> NED
  base.RotEcef2ned(0, 0) = -sinf(base.lat) * cosf(base.lon);
  base.RotEcef2ned(1, 0) = -sinf(base.lon);
  base.RotEcef2ned(2, 0) = -cosf(base.lat) * cosf(base.lon);
  base.RotEcef2ned(0, 1) = -sinf(base.lat) * sinf(base.lon);
  base.RotEcef2ned(1, 1) = cosf(base.lon);
  base.RotEcef2ned(2, 1) = -cosf(base.lat) * sinf(base.lon);
  base.RotEcef2ned(0, 2) = cosf(base.lat);
  base.RotEcef2ned(1, 2) = 0.0f;
  base.RotEcef2ned(2, 2) = -sinf(base.lat);
  while (processGPS() != MT_NAV_POSECEF) {
    vTaskDelay(1000);
  }
  updateNedPos();
#ifdef DEBUG_EKF
  printf("lon: %f\tlat: %f\thMSL: %f\t\n", base.lon, base.lat, base.hMSL);
  printf("%s\n", "Matrix de transformacao ECEF to NED");
  printEigen(base.RotEcef2ned);
  printf("%s\n", "Coordenadas ECEF");
  printf("%ld\t,%ld\t,%ld\t\n", base.rECEF[0] / 100, base.rECEF[1] / 100,
         base.rECEF[2] / 100);
  printf("%s\n", "Coordenadas NED em m");
  printEigen(rNEDgps);
#endif
  blindLED(5);
  turnOnLed();
}

/**
 * @brief Atualiza a posição ned de observação dada as informações contidas na
 * estrutura #base.
 */
void EKF::updateNedPos() {
  // NOTE: a mensagem ubxMessage.navPosecef fornece as medidas em cm.
  rNEDgps = base.RotEcef2ned.col(0) *
                (0.01 * (float)(ubxMessage.navPosecef.ecefX - base.rECEF[0])) +
            base.RotEcef2ned.col(1) *
                (0.01 * (float)(ubxMessage.navPosecef.ecefY - base.rECEF[1])) +
            base.RotEcef2ned.col(2) *
                (0.01 * (float)(ubxMessage.navPosecef.ecefZ - base.rECEF[2]));
  // atualiza a matriz H correspondente á posição.
  updateHp();
}
/**
 * @brief  Atualiza o vetor de velocidade no sistema NED #drNEDgps.
 */
void EKF::updateNedVel() {
  // NOTE: (float)ubxMessage.navPvt.velX está em mm/s.
  drNEDgps(0) = 0.001f * (float)ubxMessage.navPvt.velN;
  drNEDgps(1) = 0.001f * (float)ubxMessage.navPvt.velE;
  drNEDgps(2) = 0.001f * (float)ubxMessage.navPvt.velD;
  // atualiza a matriz H correspondente á velocidade
  updateHv();
}

/**
 * Metodos de calibracao
 * @param _porta_led [description]
 */
void EKF::calibrationMethods() {
  blindLED(3);
  imu.calibraGyro(200);
  blindLED(5);
  while (imu.calibracaoMagnetometro(22.8908f) < 0) {
    blindLED(2);
  }
  timeClass.computeElapsedTime();
  timeClass.computeElapsedTime();
  blindLED(3);
}
#endif

} // namespace ekf
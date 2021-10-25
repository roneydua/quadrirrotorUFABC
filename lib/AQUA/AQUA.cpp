/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: AQUA.cpp
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */
#include "AQUA.h"
#ifdef __XTENSA__
#include "Arduino.h"
#endif
/**
 * Instância da classe AQUA.
 */
AQUA::AQUA() {}
void AQUA::begin(Eigen::Vector4f &_qObs, Eigen::Vector3f &accel,
                 Eigen::Vector3f &mag) {
  qAQUA = &_qObs;
  this->accel = &accel;
  this->mag = &mag;
}

/**
 * @brief Calcula o quaternion de inclinacao.
 */
void AQUA::computeQuaternionAccel() {

  if (normalizedGravity(2) < 0) {
    qAcc(1) = INVERSE_SQUARE_2 * sqrtf(1.0 - normalizedGravity(2));
    float _k = 0.5f / qAcc(1);
    qAcc(0) = -_k * normalizedGravity(1);
    qAcc(2) = 0.0f;
    qAcc(3) = _k * normalizedGravity(0);
  } else {
    qAcc(0) = INVERSE_SQUARE_2 * sqrtf(1.0 + normalizedGravity(2));
    float _k = 0.5f / qAcc(0);
    qAcc(1) = -_k * normalizedGravity(1);
    qAcc(2) = _k * normalizedGravity(0);
    qAcc(3) = 0.0f;
  }
}
/**
 * Computa o quaternion de guinada.
 */
void AQUA::computeQuaternionMag() {
  float gama = l(0) * l(0) + l(1) * l(1);
  if (l(0) < 0) {
    float k3 = sqrtf(gama - l(0) * sqrtf(gama));
    qMag(3) = INVERSE_SQUARE_2 * k3 * invSqrt(gama);
    qMag(0) = INVERSE_SQUARE_2 * l(1) / k3;
  } else {
    float k3 = sqrtf(gama + l(0) * sqrtf(gama));
    qMag(0) = INVERSE_SQUARE_2 * k3 * invSqrt(gama);
    qMag(3) = INVERSE_SQUARE_2 * l(1) / k3;
  }
}

// void AQUA::getInitialAlignment(Eigen::Vector4f &q) {
//   float phiAlignment;
//   calc_phi(phiAlignment, q);
//   float thetaAlignment;
//   calc_theta(thetaAlignment, q);
//   qAlignment = multiplyQuaternions(q_phi(phiAlignment),
//   q_theta(thetaAlignment));
// }

/**
 * Calcula o quaternion de atitude com o algoritimo AQUA.
 * @return      0 se falhar ou 1 se sucesso.
 */
int AQUA::computeAQUAQuaternion() {
  Eigen::Vector4f _qLast = *qAQUA;
  normalizedGravity = (*accel).normalized();
  magNormalized = (*mag).normalized();
  computeQuaternionAccel();
  l = rotateVectorWithQuaternion_Conjugate(qAcc, magNormalized);
  // leva as medidas do magnetometro para o sistema intermediario;
  computeQuaternionMag();
  *qAQUA = multiplyQuaternions(qAcc, qMag);
  (*qAQUA).tail(3) *= -1.0f;
  if (_qLast.dot((*qAQUA)) < 0.0f) {
    (*qAQUA) *= -1.0f;
  }
  // ajusta o desalinhamento
  // (*qAQUA) = multiplyQuaternions(qAlignment, (*qAQUA));
  return 1;
}

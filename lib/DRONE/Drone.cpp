/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Drone.cpp
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */

#include "Drone.h"

Drone::Drone(float _dt) {

  constructJ();
  dt = _dt;
  half_dt = 0.5f * dt;
  // matAT = (1.0f - dt * dx / massa) * Eigen::Matrix3f::Identity();
  matAT.bottomRightCorner(3, 3) =
      (1.0f - dt * dx / massa) * Eigen::Matrix3f::Identity();
  // bloco integral
  matAT.topRightCorner(3, 3) = (1.0f + dt) * Eigen::Matrix3f::Identity();

  matBT.bottomRightCorner(3, 3) = dt * Eigen::Matrix3f::Identity();
  matBR.block<3, 3>(3, 0) = dt * inverseMatrizInercia;
}
/**
 * Atualiza as matrizes dependentes do estado.
 */
void Drone::updateStateMatrices(bool &negative_q0) {

  float _q0 = 0.0f;
// HACK
#if 0
  if (negative_q0) {
    _q0 = -abs(q(0));
  } else {
    _q0 = abs(q(0));
    // _q0 = q(0);
  }
#endif
  _q0 = q(0);
  if (abs(q(0)) > 0.01) {
    flagSDC = 1;
    matAR(0, 0) = 1.0f;
    matAR(1, 1) = 1.0f;
    matAR(2, 2) = 1.0f;
    matAR.topRightCorner(3, 3) =
        half_dt * (_q0 * Eigen::Matrix3f::Identity() + ekf::skew(q.tail(3)));
  } else {
    flagSDC = 2;
    matAR(0, 0) = -half_dt * w(0) * q(1) + 1.0f;
    matAR(1, 1) = -half_dt * w(1) * q(2) + 1.0f;
    matAR(2, 2) = -half_dt * w(2) * q(3) + 1.0f;
    // _q0 = q(0);
    matAR.topRightCorner(3, 3) = half_dt * ekf::skew(q.tail(3));
    matAR(0, 3) = half_dt * (q(1) * q(1));
    matAR(1, 4) = half_dt * (q(2) * q(2));
    matAR(2, 5) = half_dt * (q(3) * q(3));
  }
  matAR.block<3, 3>(3, 3) =
      -dt * inverseMatrizInercia * ekf::skew(w) * matrizInercia;
  matAR(3, 3) = 1.0f;
  matAR(4, 4) = 1.0f;
  matAR(5, 5) = 1.0f;
}
void Drone::constructJ() {
#ifndef MOTOR_MODEL_SIMPLE
  /* Constantes de força*/
  float kf1 = 1.2457e-7;
  float kf2 = 1.4793e-7;
  float kf3 = 1.4969e-7;
  float kf4 = 1.3940e-7;
  /* Constantes de momento*/
  float km1 = 2.4656e-9;
  float km2 = 3.1518e-9;
  float km3 = 2.7691e-9;
  float km4 = 3.1051e-9;

  J << -kf1, -kf2, -kf3, -kf4, -L * kf1, 0, L * kf3, 0, 0, L * kf2, 0, -L * kf4,
      km1, -km2, km3, -km4;
#else
  float kf = 1.469e-7;
  float km = 3.1518e-9;
  J << -kf, -kf, -kf, -kf, -L * kf, 0, L * kf, 0, 0, L * kf, 0, -L * kf, -km,
      km, -km, km;
#endif
  JInverse = J.inverse();
}

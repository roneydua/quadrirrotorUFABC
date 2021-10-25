/**
 * @author: Roney Silva (roneydua)
 * @date:   19-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Controle.h
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */

#ifndef CONTROLE_H
#define CONTROLE_H
#include "../DRONE/Drone.h"
#include "../SDRE/Sdre.h"
#ifndef __XTENSA__
#define DEBUG_CONTROL
/*! Precompilador para leitrua dos ganhos*/
#define LOAD_GAINS
#include "../../../quadricopterFinal/lib/GRUPO_QUAT/GRUPO_QUAT.h"
#include <iostream>
#define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"
#include "../READWRITEEIGEN/readWriteEigen.h"

#else
#include "Arduino.h"
#include "GRUPO_QUAT.h"
#endif
#include <eigen3/Eigen/Dense>
#define INTEGRAL_CONTROL
class Controle {

private:
public:
  // enum _control_mode { TRANSLATIONAL_CONTROL, ACROBATIC_CONTROL };
  // uint8_t control_mode = TRANSLATIONAL_CONTROL;
  Eigen::Vector3f eulerTarget;
  int minimalDigitalCommand = 7700;
  int maximalDigitalCommand = 15400;
  float gravidade = 9.80f;
  uint8_t index_alt = 1;
  Drone *drone;
  Sdre *sdre;
  /*! Referência de velocidade translacional */
  Eigen::Vector3f r = Eigen::Vector3f::Zero();
  /*! Quaternion alvo de atitude */
  Eigen::Matrix<float, 4, 2> qa = Eigen::Matrix<float, 4, 2>::Zero(4, 2);
  /*! Quaternio de erro de atitude*/
  Eigen::Vector4f qe{1, 0, 0, 0};
  /*! Velocidade angular alvo */
  Eigen::Vector3f wa = Eigen::Vector3f::Zero();
  /*! Ponteiros de velocidade angular */
  /*! Psi alvo */
  float psi = 0.0f;

  /*! Velocidade Psi alvo */
  float diffPsi = 0.0f;
#ifdef INTEGRAL_CONTROL
  /*! Integral da referência de velocidade translacional */
  Eigen::Vector3f rI = Eigen::Vector3f::Zero();
  /*! Matriz Ganho de Kalman. u = -L * x da dinâmica translacional*/
  Eigen::Matrix<float, 3, 6> Lt = Eigen::MatrixXf::Zero(3, 6);
  /*! Matriz de ponderação do estado translacional. */
  Eigen::MatrixXf Qt =
      (Eigen::VectorXf(6) << 5, 5, 5, 3, 3, 3).finished().asDiagonal();
  // {
#else
  Eigen::Matrix3f Lt = Eigen::Matrix3f::Zero();
  /*! Matriz de ponderação do estado translacional. */
  Eigen::MatrixXf Qt =
      (Eigen::VectorXf(3) << 1e0, 1e0, 1e0).finished().asDiagonal();
  // {
#endif
  // Eigen::Matrix3f Ls_G = Eigen::MatrixXf::Identity(3, 3);
  /*! Matriz de ponderação do controle translacional. */
  Eigen::MatrixXf Rt =
      (Eigen::Vector3f() << 1e1, 1e1, 1e1).finished().asDiagonal();

  /*! Matriz de ponderação do controle rotacional. */
  Eigen::MatrixXf Rr =
      (Eigen::Vector3f() << 1e1, 1e1, 1e1).finished().asDiagonal();
  /*! Matriz de ponderação do estado rotacional. */
  Eigen::MatrixXf Qr = (Eigen::VectorXf(6) << 1e1, 1e1, 1e1, 1e1, 1e1, 1e1)
                           .finished()
                           .asDiagonal();
  /* Contoles */
  /*! Vetor de controle da dinâmica translacional */
  Eigen::Vector3f ut = Eigen::Vector3f::Zero();
  /*! Vetor de tração específica e momentos no corpo.
    @note que a tração já esta considerando a massa.
   */
  Eigen::Vector4f u = Eigen::Vector4f::Zero();
  /*! Comandos digitais */
  Eigen::Vector4i motorCommands = Eigen::Vector4i::Zero();
  /*! Vetor de rotação dos motores */
  Eigen::Vector4f rotationsOfMotors = Eigen::Vector4f::Zero();

  float oldPsi = 0;
  Controle(float discreteTime);
  void computeTranslationalControl();
  void computeRotationalControl();
  void computeRotationalTarget();
  void virtualCommandToMotorCommand();
  void virtualCommandToMotorRpm();
  void controlLoop();
  bool negative_q0 = false;
  float TSum = 10.0f;
};
#endif
/* CONTROLE_H */

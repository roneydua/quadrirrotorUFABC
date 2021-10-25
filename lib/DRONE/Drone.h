/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Drone.h
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */
#ifndef DRONE_H
#define DRONE_H
#include "eigen3/Eigen/Dense"
#ifndef __XTENSA__
#include "../../../quadricopterFinal/lib/GRUPO_QUAT/GRUPO_QUAT.h"
#include <iostream>
using namespace std;
#define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"
#else
#include "Arduino.h"
#include "GRUPO_QUAT.h"
#endif
// #define MOTOR_MODEL_SIMPLE
class Drone {

private:
  /*! Braço do quadro. */
  const float L = 0.225f;
  float half_dt;

public:
  /*! Mapa quadrado das rotações para u=[T, mx, my, mz]*/
  Eigen::Matrix4f J;
  /*! Mapa u=[T, mx, my, mz] para quadrado das rotações*/
  Eigen::Matrix4f JInverse;
  /* Período do controle discreto*/
  float dt;
  /*! Coeficiente de Arrasto*/
  float dx = 0.25f;
  /*! "Massa do quadrirrotor em kg. Sem os apoios e as proteções massa=1.135,
   * coso contrário, massa = 1.363" */
  float massa = 1.136f;

  // float massa = 1.0f;
  /**
   * @brief Construct a new Drone object
   *
   * @param _dt Tempo do discretização da planta para loop do controle.
   */
  Drone(float _dt);
  /*! Matriz de momentos de inércia
  @note É utilizada a classe DiagonalMatrix para economia de memoria (n
  vezes menos). Todavia deve-se ter cuidado com a limitação de operações desta
  classe.
  */
  // const Eigen::DiagonalMatrix<float, 3> matrizInercia{10e-3f, 11e-3f,
  // 18e-3f};
  const Eigen::DiagonalMatrix<float, 3> matrizInercia{10e-3f, 10e-3f, 18e-3f};
  /*! Inversa da matriz de momentos de inércia.
  @note São utilizadas as classes DiagonalMatrix para economia de memoria (n
  vezes menos). Todavia deve-se ter cuidado com a limitação de operações desta
  classe.
  */
  const Eigen::DiagonalMatrix<float, 3> inverseMatrizInercia =
      matrizInercia.inverse();
  /*! Posicao translacional */
  Eigen::Vector3f p{0, 0, 0};
  /*! Velocidade translacional */
  Eigen::Vector3f v{0, 0, 0};
  /*! Quaternion de atitude. */
  Eigen::Vector4f q{1.0f, 0.0f, 0.0f, 0.0f};
  /*! Vetor de velocidade angular. */
  Eigen::Vector3f w{0.0f, 0.0f, 0.0f};
  /*! Matrix de estados translacional discreta. */
  // Eigen::MatrixXf matAT = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf matAT = Eigen::MatrixXf::Identity(6, 6);
  /*! Matriz de controle translacional discreta. */
  // Eigen::MatrixXf matBT = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf matBT = Eigen::MatrixXf::Zero(6, 3);
  // TEST: INTEGRAL CONTROL
  /*! Matriz de estados rotacional discreta. */
  Eigen::MatrixXf matAR = Eigen::MatrixXf::Identity(6, 6);
  /*! Matriz de controle rotacional discreta. */
  Eigen::MatrixXf matBR = Eigen::MatrixXf::Zero(6, 3);
  /*! Matrix H conversão u to rotations */

  // const Eigen::DiagonalMatrix<float, 4> H =
  //     (Eigen::Vector4f(4) << 0.724048f, 0.724048f, 0.724048f, 0.724048f)
  //         .finished()
  //         .asDiagonal();

  const Eigen::DiagonalMatrix<float, 4> H =
      (Eigen::Vector4f(4) << 0.724048f, 0.73493f, 0.707656f, 0.727027f)
          .finished()
          .asDiagonal();

  // const Eigen::DiagonalMatrix<float, 4> HInverse =
  //     (Eigen::Vector4f(4) << 1.0f / 0.724048f, 1.0f / 0.724048f,
  //      1.0f / 0.724048f, 1.0f / 0.724048f)
  //         .finished()
  //         .asDiagonal();

  const Eigen::DiagonalMatrix<float, 4> HInverse =
      (Eigen::Vector4f(4) << 1.0f / 0.724048f, 1.0f / 0.73493f,
       1.0f / 0.707656f, 1.0f / 0.727027f)
          .finished()
          .asDiagonal();

  /*! Vector h da conversão u to rotations */
  const Eigen::Vector4f h{-3911.13f, -4180.05f, -3620.13f, -4279.45f};
  // const Eigen::Vector4f h{-3911.13f, -3911.13f, -3911.13f, -3911.13f};
  void updateStateMatrices(bool &negativeq0);
  void constructJ();
  // Eigen::VectorXf continuosModel(Eigen::VectorXf x, Eigen::VectorXf u);
  int flagSDC = 0;
};
#endif
/* DRONE_H */

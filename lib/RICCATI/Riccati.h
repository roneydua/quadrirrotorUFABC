/**
 * @file Riccati.h
 * @author roneydua (roneyddasilva@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-09-19
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef RICCATI_H
#define RICCATI_H
#include "eigen3/Eigen/Dense"
#ifndef __XTENSA__
#define DEBUG
#include <iostream>
using namespace std;
#define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"
#else
#include "Arduino.h"
#endif

class Riccati {

private:
public:
  /*! Matriz de Riccati */
  Eigen::MatrixXf K, K_new;
  /*! Ponteiro da matriz de estado. */
  Eigen::MatrixXf *phi;
  /*! Ponteiro da matriz de controle. */
  Eigen::MatrixXf *gamma;
  /*! Ponteiro da matriz de ponderação do controle */
  Eigen::MatrixXf *R;
  /*! Ponteiro da matriz de ponderação dos estados. */
  Eigen::MatrixXf *Q;
  Eigen::MatrixXf I;
  /*! Matriz La = (B'PB+R)^-1 * B' */
  Eigen::MatrixXf Ls;
  int num_iterations = 0;

  Riccati(Eigen::MatrixXf &A, Eigen::MatrixXf &B, Eigen::MatrixXf &Q,
          Eigen::MatrixXf &R);

  bool dareInteration(const float &tolerance = 1.0E-4,
                      const uint16_t iter_max = 50);
};
#endif
/* RICCATI_H */

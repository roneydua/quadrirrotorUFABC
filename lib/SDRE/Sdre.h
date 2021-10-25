/**
 * @author: roney
 * @date:   2021-08-03T17:16:43-03:00
 * email:  roneyddasilva@gmail.com
 * @File: Sdre.h
 * Last modified by:   roney
 * Last modified time: 2021-08-03T17:16:43-03:00
 */
#ifndef SDRE_H
#define SDRE_H
#include "../RICCATI/Riccati.h"
#ifndef __XTENSA__
#include <iostream>
using namespace std;

#define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"
#endif
#include <eigen3/Eigen/Dense>
// #define COMPUTE_MAT_REALIMENTACAO

class Sdre {

private:
public:
  /*! Ponteiro da matriz de estado. */
  Eigen::MatrixXf *phi;
  /*! Ponteiro da matriz de controle. */
  Eigen::MatrixXf *gamma;
  /*! Ponteiro para o solver da equação de Riccati */
  Riccati *ricObj;
  /*! Matrix ponderação dos controle. */
  Eigen::MatrixXf *R;
  /*! Matrix ponderação dos estados. */
  Eigen::MatrixXf *Q;
  /*! Matrix do ganho de Kalman. */
  Eigen::MatrixXf L;
  /*! Matrix E = B R^-1 B' */
  Eigen::MatrixXf E;
#ifdef COMPUTE_MAT_REALIMENTACAO
  /*! Vetor de realimentacao; */
  Eigen::MatrixXf G;
#endif
  Sdre(Eigen::MatrixXf &A, Eigen::MatrixXf &B, Eigen::MatrixXf &Q,
       Eigen::MatrixXf &R);
  ~Sdre();
  bool updateControl();
  Eigen::MatrixXcf closeLoopEig();
};
#endif
/* SDRE_H */

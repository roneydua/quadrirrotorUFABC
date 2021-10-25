/**
 * @author: roney
 * @date:   2021-08-02T14:13:06-03:00
 * email:  roneyddasilva@gmail.com
 * @File: Riccati.cpp
 * Last modified by:   roney
 * Last modified time: 2021-08-02T14:13:06-03:00
 */
#include "Riccati.h"

Riccati::Riccati(Eigen::MatrixXf &A, Eigen::MatrixXf &B, Eigen::MatrixXf &_Q,
                 Eigen::MatrixXf &_R) {
  phi = &A;
  int n = A.cols();
  int r = B.cols();
  gamma = &B;
  R = &_R;
  Q = &_Q;
  Ls = Eigen::MatrixXf::Identity(r, r);
  K = *Q;
  K_new = K;
  I = Eigen::MatrixXf::Identity(n, n);
}
/**
 * Função de cálculo da solução de Riccati discreta.
 * @param  tolerance máxima erro permitido.
 * @param  iter_max  Número máximo de iterações
 * @return           true se ok, false se número máximo de iterações for
 * atingido.
 * @todo Testar a influencia em armazenar a matrix A.transpose().
 */
bool Riccati::dareInteration(const float &tolerance,
                             const uint16_t iter_max) { //
  /*! Diferença entre #K_new e #K */
  float diff = 0;
  for (num_iterations = 1; num_iterations < iter_max; num_iterations++) {

    Ls = (*R + (*gamma).transpose() * K * (*gamma)).inverse() *
         (*gamma).transpose();
    K_new = (*phi).transpose() * K * (I - (*gamma) * Ls * K) * (*phi);
    K_new += *Q;
    // avalia o erro para parar as iterações.
    // diff = (K_new - K).cwiseAbs().maxCoeff();
    Eigen::MatrixXf erro = K_new - K;
    diff = (erro).norm() / K.norm();
    K = K_new;
    if (diff < tolerance) {
      return true;
    }
  }
#ifndef __XTENSA__
  printf("Falha. Erro %f\n", diff);
#endif
  return false;
};

/**
 * @author: roney
 * @date:   2021-08-03T17:16:37-03:00
 * email:  roneyddasilva@gmail.com
 * @File: Sdre.cpp
 * Last modified by:   roney
 * Last modified time: 2021-08-03T17:16:37-03:00
 */
#include "Sdre.h"
Sdre::Sdre(Eigen::MatrixXf &A, Eigen::MatrixXf &B, Eigen::MatrixXf &_Q,
           Eigen::MatrixXf &_R) {
  int n = A.cols();
  int r = B.cols();
  L = Eigen::MatrixXf::Identity(r, n);
  Q = &_Q;
  R = &_R;
  /* instancia o Solver de Riccati */
  ricObj = new Riccati(A, B, *Q, *R);
  /* Salvas os endereços das matrizes de estado e controle*/
  phi = &A;
  gamma = &B;
  E = B * _R.inverse() * B.transpose();
#ifdef COMPUTE_MAT_REALIMENTACAO
  G = Eigen::MatrixXf::Identity(n, n);
#endif
}
Sdre::~Sdre() {}
/**
 * Atualiza a solução de Riccati e o controle ótimo L.
 * @return true para sucesso ou false para fracasso.
 */
bool Sdre::updateControl() {
  // Verifica o se o algoritmo converge com o numero de iterações e tolerância
  // desejado.
  if ((*ricObj).dareInteration()) { // Obtem a matriz de Riccati.
    // atualiza a matriz de ganho de Kalman
    L = (*ricObj).Ls * (*ricObj).K * (*phi);
#ifdef COMPUTE_MAT_REALIMENTACAO
    G = ((*ricObj).I -
         (*phi).transpose() *
             ((*ricObj).I -
              (*ricObj).K * ((*ricObj).I + E * (*ricObj).K).inverse() * E))
            .inverse();
#endif
    return true;
  } else {
    // Retorna falso mantendo o ultimo ganho de Riccati.
    return false;
  }
}
/**
 * [Sdre::closeLoopEig description]
 */

Eigen::MatrixXcf Sdre::closeLoopEig() {
  // Eigen::MatrixXf closeLoop =
  //     *phi - *gamma * (*R).inverse() * (*gamma).transpose() * (*ricObj).K;
  Eigen::MatrixXf closeLoop = *phi - *gamma * L;
  Eigen::EigenSolver<Eigen::MatrixXf> Eigs(closeLoop);
  return Eigs.eigenvalues();
};

;

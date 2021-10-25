/**
 * @author: Roney Silva (roneydua)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Controle.cpp
 * Last modified by:   roneydua
 * Last modified time: 25-Aug-2021
 */
#include "Controle.h"

/**
 * @brief Construct a new Controle:: Controle object
 *
 * @param discreteTime intervalo de tempo de discretização em seconds
 */
Controle::Controle(float discreteTime) {
#ifdef __XTENSA__
  printf("Nucleo do Controle: %d\n", xPortGetCoreID());
  vTaskDelay(2000);
#endif
#if defined(DEBUG_CONTROL) && defined(LOAD_GAINS)
  // Para debugar diversos valores de ganho este trecho pode ser descomentado
  // mas
  //     as variáveis Qt,
  //     Rt,
  //     Qr e Rr devem deixar de ser constantes.
  string _folderPlace = "include/CONTROLE/";
#ifdef INTEGRAL_CONTROL
  Qt = csvLeia<Eigen::MatrixXf>(_folderPlace + "Qt_INTEGRAL.csv");
  Rt = csvLeia<Eigen::MatrixXf>(_folderPlace + "Rt_INTEGRAL.csv");
#else
  Qt = csvLeia<Eigen::MatrixXf>(_folderPlace + "Qt.csv");
  Rt = csvLeia<Eigen::MatrixXf>(_folderPlace + "Rt.csv");

#endif
  Qr = csvLeia<Eigen::MatrixXf>(_folderPlace + "Qr.csv");
  Rr = csvLeia<Eigen::MatrixXf>(_folderPlace + "Rr.csv");
#endif
  drone = new Drone(discreteTime);
  u(0) = drone->massa * gravidade;
  ut(2) = drone->massa * gravidade;
  qa(0, 1) = 1.0f;
/*Calcula o ganho para controle da dinâmica translacional*/
#ifdef INTEGRAL_CONTROL
  sdre = new Sdre(drone->matAT, drone->matBT, Qt, Rt);
#else
  Eigen::MatrixXf matAt_temp = drone->matAT.bottomRightCorner(3, 3);
  Eigen::MatrixXf matBt_temp = drone->matBT.bottomRightCorner(3, 3);
  sdre = new Sdre(matAt_temp, matBt_temp, Qt, Rt);
#endif
  // força a convergência inicial
  sdre->ricObj->dareInteration(1e-5, 10000);
  sdre->updateControl();
  Lt = sdre->L;
  Eigen::MatrixXcf eigenValuesTranslacional = sdre->closeLoopEig();
  PRINT_MAT(drone->matAT);
  PRINT_MAT(drone->matBT);
  PRINT_MAT(eigenValuesTranslacional);
  sdre->~Sdre();
  /* Instância a classe SDRE para computo da dinâmica rotacional.*/
  sdre = new Sdre(drone->matAR, drone->matBR, Qr, Rr);
  ekf::calc_psi(psi, drone->q);
#ifdef DEBUG_CONTROL
  PRINT_MAT(Qt);
  PRINT_MAT(Rt);
  PRINT_MAT(Qr);
  PRINT_MAT(Rr);
#endif
}

/**
 * Calcula o controle de translação.
 */
void Controle::computeTranslationalControl() { /* */
  index_alt = 1 - index_alt;
#ifdef INTEGRAL_CONTROL
  Eigen::VectorXf _x(6);
  rI += r * drone->dt;
  _x << drone->p - rI, drone->v - r;
  ut = -Lt * _x; // + Ls_G * r;
#else
  ut = -Lt * (drone->v - r); // + Ls_G * r;
#endif
  ut(2) -= gravidade;
  // Tração local

  u(0) = -ut.norm(); // * drone->massa;
}
void Controle::computeRotationalTarget() {
  /*!attention  ut(2)  já contém a gravidade adicionada no método
   * &computeTranslationalControl*/
  psi += diffPsi * drone->dt;
  float qc0 = sqrtf(0.5f * (ut(2) / u(0) + 1.0f));
  float cosPhi_2 = cosf(0.5f * psi);
  float sinPhi_2 = sinf(0.5f * psi);
  /* Calculo do quaternion de atitude alvo.*/

  qa(0, index_alt) = qc0 * cosPhi_2;
  qa(1, index_alt) =
      (ut(1) * cosPhi_2 - ut(0) * sinPhi_2) * .5f / (-u(0)) / qc0;
  qa(2, index_alt) = (ut(0) * cosPhi_2 + ut(1) * sinPhi_2) * .5f / u(0) / qc0;
  qa(3, index_alt) = qc0 * sinPhi_2;
  // Atualiza a flag da parte escalar do quaternion alvo.
  (qa(0, index_alt)) < 0.0f ? negative_q0 = true : negative_q0 = false;
  // Calcula a velocidade angular.
  wa = 2.0f / drone->dt * (ekf::Q_l(qa.col(1 - index_alt))).transpose() *
       qa.col(index_alt);

  ekf::quaternion2Euler(eulerTarget, qa.col(index_alt));
  u(0) *= drone->massa;
}
/**
 * Computa do controle da dinâmica rotacional.
 */
void Controle::computeRotationalControl() {
  sdre->updateControl();
  //  Eigen::VectorXf _r(6), _x(6);
  Eigen::VectorXf _x(6);
  qe = (ekf::S_l(drone->q)).transpose() * qa.col(index_alt);
  // _qI = ;
  if (negative_q0) {
    _x << qe.tail(3), drone->w - wa;
  } else {
    _x << -qe.tail(3), drone->w - wa;
  }
  u.tail(3) = -sdre->L * _x;
}
/**
 * Converte a tração e os momentos em valores digitais de entrada do ESC na
 * classe #Motores.h
 * @note: Esta funcao limita os
 */
void Controle::virtualCommandToMotorCommand() {
  motorCommands =
      (drone->HInverse * ((drone->JInverse * u).cwiseSqrt() - drone->h))
          .cast<int>();
  motorCommands = motorCommands.array()
                      .min(maximalDigitalCommand)
                      .max(minimalDigitalCommand)
                      .matrix();
}
void Controle::virtualCommandToMotorRpm() { /* */
  // rotationsOfMotors << (drone->JInvese * u).array().sqrt().matrix();
  rotationsOfMotors << drone->H * (motorCommands.cast<float>()) + drone->h;
  u = drone->J * rotationsOfMotors.array().square().matrix();
}
/**
 * @brief Loop de controle das duas dinâmicas.
 *
 */
void Controle::controlLoop() {
  // atualiza as matriz com os valores de q e wb
  drone->updateStateMatrices(negative_q0);
  computeTranslationalControl();
  computeRotationalTarget();
  computeRotationalControl();
  virtualCommandToMotorCommand();
#ifndef __XTENSA__
  //  Na aplicação embarcada esta etapa não eh necessária.
  virtualCommandToMotorRpm();
#endif
}

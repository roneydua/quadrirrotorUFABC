/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: AQUA.h
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */

#ifndef AQUA_H
#define AQUA_H
#include <eigen3/Eigen/Dense>
#ifdef __XTENSA__
#include <Arduino.h>
#endif
#include "../GRUPO_QUAT/GRUPO_QUAT.h"

using namespace ekf;

class AQUA {

public:
  AQUA();
  /*! Quaternion de inclinacao. */
  Eigen::Vector4f qAcc = Eigen::Vector4f::Zero();
  /*! Quaternion de Guinada. */
  Eigen::Vector4f qMag = Eigen::Vector4f::Zero();
  /*!Campo gravitacional normalizado no sistema do corpo*/
  Eigen::Vector3f normalizedGravity = Eigen::Vector3f::Zero();
  /*! Campo magnetico normalizado no sistema do corpo */
  Eigen::Vector3f magNormalized = Eigen::Vector3f::Zero();
  /*! Campo magnetico expresso no sistema intermediario */
  Eigen::Vector3f l = Eigen::Vector3f::Zero();
  /*! Ponteiro do quaternion de atitude. */
  Eigen::Vector4f *qAQUA;
  /*! Ponteiros do acelerometro e do magnetometro */
  Eigen::Vector3f *accel, *mag;
  /*! Quaternion de desalinhamento.*/
  Eigen::Vector4f qAlignment{1.0f, 0.0f, 0.0f, 0.0f};
  void getInitialAlignment(Eigen::Vector4f &);
  void begin(Eigen::Vector4f &_qObs, Eigen::Vector3f &accel,
             Eigen::Vector3f &mag);
  int computeAQUAQuaternion();
  // void getInitialAlignment(Eigen::Vector4f &q);
  void computeQuaternionAccel();
  void computeQuaternionMag();

private:
};
#endif
/* AQUA_H */

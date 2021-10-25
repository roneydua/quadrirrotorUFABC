/**
 * @Author: roney
 * @Date:   2021-08-16T10:17:42-03:00
 * @Last modified by:   roney
 * @Last modified time: 2021-08-18T14:32:31-03:00
 */

/**
* @file GRUPO_QUAT.h
* @author Roney D da Silva
* @date 7 Apr 2021
* @copyright 2021 Roney D da Silva
         Email: roneyddasilva@gmail.com
* @brief Cabecalho com funcoes de quaternion
*/

#ifndef GRUPO_QUAT_H
#define GRUPO_QUAT_H

#ifdef __XTENSA__
#include "eigen3/Eigen/Dense"
#include <Arduino.h>
#else
#define RAD_TO_DEG 57.295779513082320876798154814105
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace std;
#define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"
#endif
/*! Inverso da raiz quadrada de dois 1/sqrtf{2} */
#define INVERSE_SQUARE_2 0.707106781f

/**
 * @brief Funções genéricas com álgebra de quatérnions e demais.
 *
 */
namespace ekf {
Eigen::Matrix3f skew(const Eigen::Vector3f &v);
Eigen::Matrix4f S_r(const Eigen::Ref<Eigen::Vector4f> &q);
Eigen::Matrix4f S_l(const Eigen::Ref<Eigen::Vector4f> &q);
void printEigen(const Eigen::MatrixXf &m);
void quaternion2Euler(Eigen::Ref<Eigen::Vector3f> _euler,
                      const Eigen::Ref<const Eigen::Vector4f> &_q);
void computeMcdFromQuaternion(const Eigen::Vector4f &q, Eigen::Matrix3f &mcd);
Eigen::Vector3f computeVectorProjection(const Eigen::Ref<Eigen::Vector3f> &u,
                                        const Eigen::Ref<Eigen::Vector3f> &v);

// Eigen::Vector4f multiplyQuaternions(Eigen::VectorXf &p, Eigen::VectorXf &q);

// Eigen::Vector4f multiplyQuaternions(const Eigen::Vector4f &p,
//                                     const Eigen::VectorXf &q);

void integrationQuaternion(Eigen::Vector4f &_quat,
                           const Eigen::Ref<const Eigen::VectorXf> &_gyroscopio,
                           float &_dt);
float invSqrt(float _x);
Eigen::Vector3f rotateVectorWithQuaternion(Eigen::Vector4f &q,
                                           Eigen::Vector3f &v_old);
Eigen::Vector3f rotateVectorWithQuaternion_Conjugate(Eigen::Vector4f &q,
                                                     Eigen::Vector3f &v_old);
Eigen::Vector4f multiplyQuaternions(const Eigen::Ref<Eigen::VectorXf> &p,
                                    const Eigen::Ref<Eigen::VectorXf> &q);
void computeQuaternionFromMCDShepperd(Eigen::Matrix3f &m, Eigen::Vector4f &q);
void computeQuaternionFromMCDMarkley(Eigen::Matrix3f &m,
                                     Eigen::Ref<Eigen::Vector4f> q);
Eigen::VectorXf RK(float h, const Eigen ::Ref<Eigen::VectorXf> &xk,
                   const Eigen ::Ref<Eigen::VectorXf> &u,
                   Eigen::VectorXf fn(Eigen::VectorXf x, Eigen::VectorXf u));
Eigen::MatrixXf Q_l(const Eigen::Ref<Eigen::Vector4f> &q);
Eigen::MatrixXf Q_r(const Eigen::Ref<Eigen::Vector4f> &q);
void conjugate(Eigen::Ref<Eigen::Vector4f> _q);
void calc_phi(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q);
void calc_theta(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q);
void calc_psi(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q);
Eigen::Vector4f q_theta(float &t);
Eigen::Vector4f q_phi(float &t);
Eigen::Vector4f q_psi(float &t);
} // namespace ekf
#endif
/*GRUPO_QUAT_H*/

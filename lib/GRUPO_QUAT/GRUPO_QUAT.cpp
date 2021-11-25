/**
 * @Author: roney
 * @Date:   2021-08-16T10:17:42-03:00
 * @Last modified by:   roney
 * @Last modified time: 2021-08-18T14:32:09-03:00
 */

/**
 * @file GRUPO_QUAT.cpp
 * @author Roney D da Silva
 * @date 7 Apr 2021
 * @copyright 2021 Roney D da Silva
          Email: roneyddasilva@gmail.com
 * @brief Cabecalho com funcoes de quaternion
 */

#include "GRUPO_QUAT.h"
#ifdef __XTENSA__
// #include <Arduino.h>
#endif

namespace ekf {

const float TOLERANCE = 1e-6;

/**
 * Matrix Right-Quaternion Q
 * @param  q quaternion de atitude
 * @return Matrix Right Quarternio Q
 */
Eigen::MatrixXf Q_r(const Eigen::Ref<Eigen::Vector4f> &q) {

  Eigen::MatrixXf M = Eigen::MatrixXf::Zero(4, 3);
  M.row(0) = -q.tail(3);
  M.bottomLeftCorner(3, 3) =
      q(0) * Eigen::Matrix3f::Identity() - skew(q.tail(3));
  return M;
}

/**
 * Matrix Left-Quaternion Q
 * @param  q quaternion de atitude
 * @return Matrix Left Quarternio Q
 */
Eigen::MatrixXf Q_l(const Eigen::Ref<Eigen::Vector4f> &q) {

  Eigen::MatrixXf M = Eigen::MatrixXf::Zero(4, 3);
  M.row(0) = -q.tail(3);
  M.bottomLeftCorner(3, 3) =
      q(0) * Eigen::Matrix3f::Identity() + skew(q.tail(3));
  return M;
}

/**
 * @brief Matrix antissimetrica
 * @param [v] Vetor tridimensional
 * @return [M] Matrix antissimetrica
 * @details [A matrix
 * antissimetrica](https://mathworld.wolfram.com/AntisymmetricMatrix.html)
 * torna possivel converter um produto vetorial em produto de matrizes.
 */
Eigen::Matrix3f skew(const Eigen::Vector3f &v) {

  Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
  M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return M;
}
/**
 * @brief Calcula a Matriz _Right-Quaternion_ a partir do quaternion q
 * @param  q quaternion de atitude
 * @return mS_r Matrix _Right-Quaternion_
 * @brief A matrix _Right-Quaternion_ torna possível realizar o produto de
 * quaternions como um produto matricial comum.
 */
Eigen::Matrix4f S_r(const Eigen::Ref<Eigen::Vector4f> q) {

  Eigen::Matrix4f mS_r = Eigen::Matrix4f::Identity() * q(0);
  mS_r.block<1, 3>(0, 1) = -q.segment<3>(1);
  mS_r.block<3, 1>(1, 0) = q.segment<3>(1);
  mS_r.block<3, 3>(1, 1) -= skew(q.segment<3>(1));
  return mS_r;
}
/**
 * @brief Calcula a Matriz _Left-Quaternion_ a partir do quaternion q
 * @param  q quaternion de atitude
 * @return mS_l Matrix _Right-Quaternion_
 * @brief A matrix _Left-Quaternion_ torna possível realizar o produto de
 * quaternions como um produto matricial comum.
 */
Eigen::Matrix4f S_l(const Eigen::Ref<Eigen::Vector4f> &q) {
  Eigen::Matrix4f mS_r = Eigen::Matrix4f::Identity() * q(0);
  mS_r.block<1, 3>(0, 1) = -q.segment<3>(1);
  mS_r.block<3, 1>(1, 0) = q.segment<3>(1);
  mS_r.block<3, 3>(1, 1) += skew(q.segment<3>(1));
  return mS_r;
}
/**
 * Calcula a matriz de cossenos ditores a parir do quaternion de atitude
 * @param q   quaternion de atitude
 * @param mcd Matriz de cossenos diretores a ser atualizada (Referencia)
 */
void computeMcdFromQuaternion(const Eigen::Vector4f &q, Eigen::Matrix3f &mcd) {
  mcd = Eigen::Matrix3f::Identity() * (2.0f * q(0) * q(0) - 1.0f) +
        2.0f * (q.tail(3) * q.tail(3).transpose()) +
        2.0f * q(0) * skew(q.tail(3));
}
/**
 * Imprime um dado Eigen. Matriz ou vetor.
 * @param m dado a ser impresso.
 */
void printEigen(const Eigen::MatrixXf &m) {
  int c = m.cols();
  int l = m.rows();
  for (int i = 0; i < l; i++) {
    for (int j = 0; j < c; j++) {
      // printf("%f\t", m(i, j));
      printf(",%f", m(i, j));
    }
    printf("%s\n", ",");
  }
}
/**
 * Rejeita a projecao do vetor v sobre u .
 * @param  u Vetor tridimensional
 * @param  v Vetor tridimensional
 * @return v sem a projeção de v sobre u. * [Vector
 * Rejection](https://en.wikipedia.org/wiki/Vector_projection#Vector_rejection_2)
 * [Vector Rejection ]
 */
Eigen::Vector3f computeVectorProjection(const Eigen::Ref<Eigen::Vector3f> &v,
                                        const Eigen::Ref<Eigen::Vector3f> &u) {

  return (v - v.dot(u) * u);
}

/**
 * @brief Produto de quaternions aplicado.
 * @param  p Quaternion
 * @param  q Vetor tridimensional
 * @return Quaternion product p&otimes;q
 */
Eigen::Vector4f multiplyQuaternions(const Eigen::Ref<Eigen::VectorXf> &p,
                                    const Eigen::Ref<Eigen::VectorXf> &q) {
  Eigen::Vector4f pq;
  pq(0) = p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3);
  pq(1) = p(0) * q(1) + p(1) * q(0) + p(2) * q(3) - p(3) * q(2);
  pq(2) = p(0) * q(2) - p(1) * q(3) + p(2) * q(0) + p(3) * q(1);
  pq(3) = p(0) * q(3) + p(1) * q(2) - p(2) * q(1) + p(3) * q(0);
  return pq;
}

void quaternion2Euler(Eigen::Ref<Eigen::Vector3f> _euler,
                      const Eigen::Ref<const Eigen::Vector4f> &_q) {
  /**
   * Extrai os ângulos de Euler do quatérnion.
   * @param _euler Vetor de ângulos de Euler.
   * @param _q Quatérnion de atitude.
   */
  // calculo de fi
  calc_phi(_euler(0), _q);
  // calculo de theta
  calc_theta(_euler(1), _q);
  // calculo de psi
  calc_psi(_euler(2), _q);
}
/**
 * @brief Calcula, a partir de um quatérnio de atitude, o valor do ângulo Phi em
 * Radianos.
 *
 * @param x Referencia a ser sobrescrita. -pi < x < pi
 * @param _q Quaternion de atitude
 */
void calc_phi(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q) {
  x = -atan2f(_q(2) * _q(3) - _q(0) * _q(1),
              _q(0) * _q(0) + _q(3) * _q(3) - 0.5f);
}
/**
 * @brief Calcula, a partir de um quatérnio de atitude, o valor do ângulo Theta
 * em Radianos.
 *
 * @param x Referencia a ser sobrescrita. -pi/2 < x < pi/2
 * @param _q Quaternion de atitude
 */
void calc_theta(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q) {
  x = -asinf(-2.0f * (_q(0) * _q(2) + _q(1) * _q(3)));
}
/**
 * @brief Calcula, a partir de um quatérnio de atitude, o valor do ângulo Psi em
 * Radianos.
 *
 * @param x Referencia a ser sobrescrita. -pi < x < pi
 * @param _q Quaternion de atitude
 */
void calc_psi(float &x, const Eigen::Ref<const Eigen::Vector4f> &_q) {
  x = -atan2f(_q(1) * _q(2) - _q(0) * _q(3),
              _q(0) * _q(0) + _q(1) * _q(1) - 0.5f);
}
void integrationQuaternion(Eigen::Vector4f &_quat,
                           const Eigen::Ref<const Eigen::VectorXf> &_gyroscope,
                           float &_dt) {
  /**
   * Integracao de quaternion com mapa exponencial.
   * @param _quat       Quaternion q_k
   * @param _giroscopio medidas do giroscópio [Radianos]
   * @param _dt         Passo de integração.
   */
  /*! @note computa norma para evitar instabilidade */
  float _gyroscopeNorm = _gyroscope.norm();

  if (_gyroscopeNorm > TOLERANCE) {
    Eigen::Vector4f expMap;
    float _dtGyroscopeNormHalf = 0.5f * _dt * _gyroscopeNorm;
    expMap(0) = cosf(_dtGyroscopeNormHalf);
    expMap.tail(3) = _gyroscope / _gyroscopeNorm * sinf(_dtGyroscopeNormHalf);
    _quat = multiplyQuaternions(_quat, expMap);
  }
}
/**
 * @brief Rotaciona um vetor tridimensional com um quaternion
 * @param  q quaternion de atitude
 * @param  v_old vetor a ser rotacionado
 * @return  v_new vetor rotacionado.
 * @note A transformação desta forma possui 30 operações (15 produtos e 15
 * multiplicações.)(Rotacao de vetor tridimensional com formalismo de
 * eixo/angulo)[https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons]
 */
Eigen::Vector3f rotateVectorWithQuaternion(Eigen::Vector4f &q,
                                           Eigen::Vector3f &v_old) {
  return v_old +
         2.0f * q.tail<3>().cross(q.tail<3>().cross(v_old) + q(0) * v_old);
}

/**
 * @brief Rotaciona um vetor tridimensional com um quaternion conjugado.
 * @param  q quaternion de atitude
 * @param  v_old vetor a ser rotacionado
 * @return  v_new vetor rotacionado.
 * @note A transformação desta forma possui 30 operações (15 produtos e 15
 * multiplicações.)(Rotacao de vetor tridimensional com formalismo de
 * eixo/angulo)[https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons]
 */
Eigen::Vector3f rotateVectorWithQuaternion_Conjugate(Eigen::Vector4f &q,
                                                     Eigen::Vector3f &v_old) {
  return v_old +
         2.0f *
             (-q.tail<3>()).cross((-q.tail<3>()).cross(v_old) + q(0) * v_old);
}

/**
 * @brief Calcula o inverso do raiz de um float.
 * @note Para mais detalhes consulte [Fast inverse
 * square-root](http://en.wikipedia.org/wiki/Fast_inverse_square_root)
 * @param  _x
 * @return  1/sqrt(_x)
 */
float invSqrt(float _x) {
  float half_x = 0.5f * _x;
  union {
    float _y;
    uint32_t _i;
  } conv = {._y = _x};
  // float _y = _x;
  // long i = *(long *)&_y;
  conv._i = 0x5f3759df - (conv._i >> 1);
  // _y = *(float *)&i;
  conv._y *= (1.5f - (half_x * conv._y * conv._y));
  // segunda iteracao.
  // _y = _y * (1.5f - (half_x * _y * _y));
  return conv._y;
}
/**
 * @brief Calcula o quaternion de atitude de rotação phi sobre o eixo x
 *
 * @param phi
 * @return Eigen::Vector4f Quaternion de rotacao phi
 */
Eigen::Vector4f q_theta(float &phi) {
  return Eigen::Vector4f{cosf(0.5f * phi), sinf(0.5f * phi), 0, 0};
}
/**
 * @brief Calcula o quaternion de atitude de rotação theta sobre o eixo y
 *
 * @param theta
 * @return Eigen::Vector4f Quaternion de rotacao theta
 */
Eigen::Vector4f q_phi(float &theta) {
  return Eigen::Vector4f{cosf(0.5f * theta), 0, sinf(0.5f * theta), 0};
}
/**
 * @brief Calcula o quaternion de atitude de rotação psi sobre o eixo z
 *
 * @param psi
 * @return Eigen::Vector4f Quaternion de rotacao psi
 */
Eigen::Vector4f q_psi(float &psi) {
  return Eigen::Vector4f{cosf(0.5f * psi), 0, 0, sinf(0.5f * psi)};
}

/**
 * Extrai o quatérnio a partir da matrix de cossenos diretores.
 * @param m Matrix de cossenos diretores.
 * @param q Quaternion de atitude.
 * @note Utiliza o algoritmo BUG
 */
/*
void computeQuaternionFromMCDShepperd(Eigen::Matrix3f &m, Eigen::Vector4f &q) {
  Eigen::Vector4f indexVector;
  indexVector << m.trace(), m.diagonal();
  Eigen::Vector4f::Index max_index;
  indexVector.maxCoeff(&max_index);
  printf("indice %d\n", max_index);
  float inv_q;
  switch (max_index) {
  case 0:
    q(0) = sqrtf(1.0f + m.trace());
    inv_q = 0.5 / q(0);
    q(0) *= 0.5;
    q(1) = inv_q * (m(2, 1) - m(1, 2));
    q(2) = inv_q * (m(0, 2) - m(2, 0));
    q(3) = inv_q * (m(1, 0) - m(0, 1));
    break;
  case 1:
    q(1) = sqrtf(1.0f + m(0, 0) - m(1, 1) - m(2, 2));
    inv_q = 0.5f / q(1);
    q(1) *= 0.5f;
    q(0) = inv_q * (m(2, 1) - m(1, 2));
    q(2) = inv_q * (m(0, 1) + m(1, 0));
    q(3) = inv_q * (m(2, 0) + m(0, 2));
    break;
  case 2:
    q(2) = sqrtf(1.0f - m(0, 0) + m(1, 1) - m(2, 2));
    inv_q = 0.5f / q(2);
    q(2) *= 0.5f;
    q(0) = inv_q * (m(0, 2) - m(2, 0));
    q(1) = inv_q * (m(0, 1) + m(1, 0));
    q(3) = inv_q * (m(1, 2) + m(2, 1));
    break;
  default:
    q(3) = sqrtf(1.0f - m(0, 0) - m(1, 1) + m(2, 2));
    inv_q = 0.5f / q(3);
    q(3) *= 0.5f;
    q(0) = inv_q * (m(1, 0) - m(0, 1));
    q(1) = inv_q * (m(2, 0) + m(0, 2));
    q(2) = inv_q * (m(2, 1) + m(1, 2));
    break;
  }
}

*/
void computeQuaternionFromMCDMarkley(Eigen::Matrix3f &m,
                                     Eigen::Ref<Eigen::Vector4f> q) {
  Eigen::Vector4f indexVector;
  float _trace = m.trace();
  indexVector << _trace, m.diagonal();
  Eigen::Vector4f::Index max_index;
  indexVector.maxCoeff(&max_index);
  switch (max_index) {
  case 0:
    q(0) = 1.0f + _trace;
    q(1) = m(2, 1) - m(1, 2);
    q(2) = m(0, 2) - m(2, 0);
    q(3) = m(1, 0) - m(0, 1);
    break;
  case 1:
    q(0) = m(2, 1) - m(1, 2);
    q(1) = 1.0f + m(0, 0) - m(1, 1) - m(2, 2);
    q(2) = m(0, 1) + m(1, 0);
    q(3) = m(2, 0) + m(0, 2);
    break;
  case 2:
    q(0) = m(2, 0) - m(0, 2);
    q(1) = m(0, 1) + m(1, 0);
    q(2) = 1.0f - m(0, 0) + m(1, 1) - m(2, 2);
    q(3) = m(1, 2) + m(2, 1);
    break;
  default:
    q(0) = m(1, 0) - m(0, 1);
    q(1) = m(2, 0) + m(0, 2);
    q(2) = m(2, 1) + m(1, 2);
    q(3) = 1.0f - m(0, 0) - m(1, 1) + m(2, 2);
    break;
  }
  q.normalize();
}
Eigen::VectorXf RK(float h, const Eigen ::Ref<Eigen::VectorXf> &xk,
                   const Eigen ::Ref<Eigen::VectorXf> &u,
                   Eigen::VectorXf(fn)(Eigen::VectorXf x, Eigen::VectorXf u)) {
  Eigen::VectorXf k1(xk.rows()), k2(xk.rows()), k3(xk.rows()), k4(xk.rows());
  k1 = h * (*fn)(xk, u);
  k2 = h * (*fn)(xk + 0.5 * k1, u);
  k3 = h * (*fn)(xk + 0.5 * k2, u);
  k4 = h * (*fn)(xk + k3, u);
  return xk + (k1 + 2.0f * (k2 + k3) + k4) / 6.0;
}
/**
 * @brief Conjuga o quaternion _q
 *
 * @param _q
 */
void conjugate(Eigen::Ref<Eigen::Vector4f> _q) { _q.tail(3) *= -1.0f; }
} // namespace ekf

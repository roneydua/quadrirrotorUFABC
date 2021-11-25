/**
 * @author: roney
 * @date:   2021-05-03T16:29:30-03:00
 * email:  roneyddasilva@gmail.com
 * @file: IMU.h
 * @details biblioteca criada com base na biblioteca disponível no GitHub :
 https://github.com/bolderflight/MPU9250
 */

#ifndef IMU_h
#define IMU_h
#include "METODOSCALIBRACAO.h"
#include <Arduino.h>
#include <Wire.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class IMU {
public:
  enum GyroRange {
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
  };
  enum AccelRange {
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
  };
  enum DlpfBandwidth {
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
  };
  enum LpAccelOdr {
    LP_ACCEL_ODR_0_24HZ = 0,
    LP_ACCEL_ODR_0_49HZ = 1,
    LP_ACCEL_ODR_0_98HZ = 2,
    LP_ACCEL_ODR_1_95HZ = 3,
    LP_ACCEL_ODR_3_91HZ = 4,
    LP_ACCEL_ODR_7_81HZ = 5,
    LP_ACCEL_ODR_15_63HZ = 6,
    LP_ACCEL_ODR_31_25HZ = 7,
    LP_ACCEL_ODR_62_50HZ = 8,
    LP_ACCEL_ODR_125HZ = 9,
    LP_ACCEL_ODR_250HZ = 10,
    LP_ACCEL_ODR_500HZ = 11
  };
  /*! Ponteiro das medidas do acelerômetro */
  Vector3f *_accel;
  /*! Ponteiro das medidas do giroscópio */
  Vector3f *_gyro;
  /*! Ponteiro das medidas do Magnetômetro*/
  Vector3f *_mag;
  Vector3f magTemp;
  float _t;

  /*! Matriz fator de escala do acelerometro. */
  const Matrix3f sFAccel =
      (Matrix3f() << 0.00477306f, -2.94365e-05, 3.90831e-05, 4.92044e-05,
       0.00477824f, 5.7648e-05, 1.55692e-05, -7.4314e-05, 0.00474654f)
          .finished();
  //
  /*! Vetor bias do acelerometro.
  @todo A MARG eh capaz de registrar os bias/Offset nos resgistradores. Feito
  isso as medidas  */
  Vector3f _biasAccel{-0.37238f, -0.109074f, 0.121389f};
  // const int16_t _biasAccel[3]{50, 17, -19};
  /*! Bias do Giroscópio. */
  Vector3f _biasGyro = Vector3f::Zero();
  /*! Bias do Magnetômetro. */
  Vector3f _biasMag = Vector3f::Zero();
  /*! Matriz fator de escala. */
  Matrix3f _sM = Matrix3f::Identity();
  IMU(TwoWire &bus, uint8_t address);
  int begin(Vector3f &accel, Vector3f &gyro, Vector3f &mag);
  int readSensor();
  int setDlpfBandwidth(DlpfBandwidth bandwidth);
  int setGyroRange(GyroRange range);
  int setAccelRange(AccelRange range);

  void calibraGyro(int N = 100);
  int calibracaoMagnetometro(float moduloCampo);
  TwoWire *_i2c;

protected:
  /*! buffer de leitura do sensor */
  uint8_t _buffer[21];
  //@{
  /*! Contadores para leitura das medidas. */
  int16_t _ax_counts, _ay_counts, _az_counts;
  int16_t _gx_counts, _gy_counts, _gz_counts;
  int16_t _hx_counts, _hy_counts, _hz_counts;
  int16_t _t_counts;
  //@}
  // vetores de leitura
  uint8_t _address;
  //! numero de bytes para leitura funcao readRegisters
  size_t _numBytes;

  // frequencia do i2c;
  const uint32_t _i2cRate = 400000;
  // verificacao de sucesso na comunicacao com sensor
  int _status;
  // constantes
  const float gG = 9.786171951281709f;
  // escalas e faixa de trabalho
  float _accelScale;
  float _gyroScale;
  float _magScaleX, _magScaleY, _magScaleZ;
  AccelRange _accelRange;
  GyroRange _gyroRange;
  // configuracoes das faixas de trabalho
  // AccelRange  ACCEL_FS_SEL_16G
  // GyroRange GYRO_FS_SEL_2000DPS;
  DlpfBandwidth _bandwidth;
  uint8_t _srd;
  // char _buf[32];
  // MPU9250 registers
  const uint8_t ACCEL_OUT = 0x3B;
  const uint8_t GYRO_OUT = 0x43;
  const uint8_t TEMP_OUT = 0x41;
  const uint8_t EXT_SENS_DATA_00 = 0x49;
  const uint8_t ACCEL_CONFIG = 0x1C;
  /* Quanto menor a faixa que o acelerometro trabalho maior sua
sensibilidade. Com o intuido de reduzir a quantidade de ruidos
usaremos as maiores escalas */
  const uint8_t ACCEL_FS_SEL_2G = 0x00;
  const uint8_t ACCEL_FS_SEL_4G = 0x08;
  const uint8_t ACCEL_FS_SEL_8G = 0x10;
  const uint8_t ACCEL_FS_SEL_16G = 0x18;
  const uint8_t GYRO_CONFIG = 0x1B;
  const uint8_t GYRO_FS_SEL_250DPS = 0x00;
  const uint8_t GYRO_FS_SEL_500DPS = 0x08;
  const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
  const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
  const uint8_t ACCEL_CONFIG2 = 0x1D;
  const uint8_t ACCEL_DLPF_184 = 0x01;
  const uint8_t ACCEL_DLPF_92 = 0x02;
  const uint8_t ACCEL_DLPF_41 = 0x03;
  const uint8_t ACCEL_DLPF_20 = 0x04;
  const uint8_t ACCEL_DLPF_10 = 0x05;
  const uint8_t ACCEL_DLPF_5 = 0x06;
  const uint8_t CONFIG = 0x1A;
  const uint8_t GYRO_DLPF_184 = 0x01;
  const uint8_t GYRO_DLPF_92 = 0x02;
  const uint8_t GYRO_DLPF_41 = 0x03;
  const uint8_t GYRO_DLPF_20 = 0x04;
  const uint8_t GYRO_DLPF_10 = 0x05;
  const uint8_t GYRO_DLPF_5 = 0x06;
  const uint8_t SMPDIV = 0x19;
  const uint8_t INT_PIN_CFG = 0x37;
  const uint8_t INT_ENABLE = 0x38;
  const uint8_t INT_DISABLE = 0x00;
  const uint8_t INT_PULSE_50US = 0x00;
  const uint8_t INT_WOM_EN = 0x40;
  const uint8_t INT_RAW_RDY_EN = 0x01;
  const uint8_t PWR_MGMNT_1 = 0x6B;
  const uint8_t PWR_CYCLE = 0x20;
  const uint8_t PWR_RESET = 0x80;
  const uint8_t CLOCK_SEL_PLL = 0x01;
  const uint8_t PWR_MGMNT_2 = 0x6C;
  const uint8_t SEN_ENABLE = 0x00;
  const uint8_t DIS_GYRO = 0x07;
  const uint8_t USER_CTRL = 0x6A;
  const uint8_t I2C_MST_EN = 0x20;
  const uint8_t I2C_MST_CLK = 0x0D;
  const uint8_t I2C_MST_CTRL = 0x24;
  const uint8_t I2C_SLV0_ADDR = 0x25;
  const uint8_t I2C_SLV0_REG = 0x26;
  const uint8_t I2C_SLV0_DO = 0x63;
  const uint8_t I2C_SLV0_CTRL = 0x27;
  const uint8_t I2C_SLV0_EN = 0x80;
  const uint8_t I2C_READ_FLAG = 0x80;
  const uint8_t MOT_DETECT_CTRL = 0x69;
  const uint8_t ACCEL_INTEL_EN = 0x80;
  const uint8_t ACCEL_INTEL_MODE = 0x40;
  const uint8_t LP_ACCEL_ODR = 0x1E;
  const uint8_t WOM_THR = 0x1F;
  const uint8_t WHO_AM_I = 0x75;
  // const uint8_t FIFO_EN = 0x23;
  // const uint8_t FIFO_TEMP = 0x80;
  // const uint8_t FIFO_GYRO = 0x70;
  // const uint8_t FIFO_ACCEL = 0x08;
  // const uint8_t FIFO_MAG = 0x01;
  // const uint8_t FIFO_COUNT = 0x72;
  // const uint8_t FIFO_READ = 0x74;
  // registro do magnetometro AK8963
  const uint8_t AK8963_I2C_ADDR = 0x0C;
  const uint8_t AK8963_HXL = 0x03;
  const uint8_t AK8963_CNTL1 = 0x0A;
  const uint8_t AK8963_PWR_DOWN = 0x00;
  const uint8_t AK8963_CNT_MEAS1 = 0x12;
  const uint8_t AK8963_CNT_MEAS2 = 0x16;
  const uint8_t AK8963_FUSE_ROM = 0x0F;
  const uint8_t AK8963_CNTL2 = 0x0B;
  const uint8_t AK8963_RESET = 0x01;
  const uint8_t AK8963_ASA = 0x10;
  const uint8_t AK8963_WHO_AM_I = 0x00;
  // funcoes privadas
  int writeRegister(uint8_t subAddress, uint8_t data);
  int readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
  int writeAK8963Register(uint8_t subAddress, uint8_t data);
  int whoAmI();
  int whoAmIAK8963();
  int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest);
};

#endif
/* IMU_H_ */

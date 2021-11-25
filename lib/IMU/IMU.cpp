/**
 * @author: roney
 * @date:   2021-05-03T16:29:30-03:00
 * email:  roneyddasilva@gmail.com
 * @file: IMU.cpp
 * Last modified by:   roney
 * Last modified time: 2021-07-19T18:32:20-03:00
 */

#include "IMU.h"
#include "Arduino.h"
/**
 * Construtor da classe IMU.
 * @param bus     classe de comunicacao Wire.
 * @param address Endereço do imu.
 */
IMU::IMU(TwoWire &bus, uint8_t address) {
  _i2c = &bus;
  _address = address;
  Serial.println("Imu ok");
}
/**
 * Inicialização e configuração da unidade MARG.
 * @param  accel Vetor de medidas do acelerômetro.
 * @param  gyro Vetor de medidas do giroscópio.
 * @param  mag  Vetor de medidas do Magnetômetro.
 * @return      inteiro positivo para ok ou negativo para erros.
 */
int IMU::begin(Vector3f &accel, Vector3f &gyro, Vector3f &mag) {
  _accel = &accel;
  _gyro = &gyro;
  _mag = &mag;
  // para analise da calibracao
  // inicializa a biblioteca Wire para comunicacao i2c
  _i2c->begin(21, 22);

  // seta a frequencia de comunicacao I2c
  // _i2c->setClock(_i2cRate);
  // seleciona o "clock source" do giroscopio

  writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
  // habilita o modo master i2c
  writeRegister(USER_CTRL, I2C_MST_EN);
  // configura a frequencia do I2c
  writeRegister(I2C_MST_CTRL, I2C_MST_CLK);
  // desliga o magnetometro
  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
  // reseta a MPU9250
  writeRegister(PWR_MGMNT_1, PWR_RESET);
  delay(1);
  // reset o magnetometro
  writeAK8963Register(AK8963_CNTL2, AK8963_RESET);
  // seleciona o "clock source" do giroscopio
  writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
  // habilita o acelerometro o giroscopio
  writeRegister(PWR_MGMNT_2, SEN_ENABLE);
  // selecionas as escalas de 16G e 2000DPS para accel e Giro como padrao
  setAccelRange(ACCEL_RANGE_16G);
  setGyroRange(GYRO_RANGE_1000DPS);
  // atribui define a largura de banda como padrao
  _bandwidth = DLPF_BANDWIDTH_5HZ;
  setDlpfBandwidth(_bandwidth);
  writeRegister(SMPDIV, 0x00);
  _srd = 0;
  // enable I2C master mode
  if (writeRegister(USER_CTRL, I2C_MST_EN) < 0) {
    return -12;
  }
  // set the I2C bus speed to 400 kHz
  if (writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0) {
    return -13;
  }
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if (whoAmIAK8963() != 72) {
    return -14;
  }
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if (writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM) < 0) {
    return -16;
  }
  delay(100);
  // para detalhes ver manual AK8963 pag. 32  e pg. 29
  readAK8963Registers(AK8963_ASA, 3, _buffer);
  _magScaleX = ((((float)_buffer[0]) - 128.0) / (256.0) + 1.0) * 4912.0 /
               32760.0; // micro Tesla
  _magScaleY = ((((float)_buffer[1]) - 128.0) / (256.0) + 1.0) * 4912.0 /
               32760.0; // micro Tesla
  _magScaleZ = ((((float)_buffer[2]) - 128.0) / (256.0) + 1.0) * 4912.0 /
               32760.0; // micro Tesla
  // set AK8963 desliga o magnetometro
  if (writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
    return -17;
  }
  delay(100); // delay para aplicacao das configuracoes
  // inicializa o magnetometro com resolucao de 16bit e 100Hz
  if (writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2) < 0) {
    return -18;
  }
  delay(100); // delay para aplicacao das configuracoes
  // select clock source to gyro
  if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
    return -19;
  }
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample
  // rate
  readAK8963Registers(AK8963_HXL, 7, _buffer);

  return 1;
}
/**
 * Configura a sensibilidade de operação do acelerômetro.
 * @param  range de operação
 * @return positivo se ok, negativo se falhar.
 */
int IMU::setAccelRange(AccelRange range) {
  switch (range) {
  case ACCEL_RANGE_2G: {
    // setting the accel range to 2G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G) < 0) {
      return -1;
    }
    _accelScale = (gG * 2.0) / 32768.0; // setting the accel scale to 2G
    break;
  }
  case ACCEL_RANGE_4G: {
    // setting the accel range to 4G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G) < 0) {
      return -1;
    }
    _accelScale = (gG * 4.0f) / 32768.0f; // setting the accel scale to 4G
    break;
  }
  case ACCEL_RANGE_8G: {
    // setting the accel range to 8G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G) < 0) {
      return -1;
    }
    _accelScale = (gG * 8.0f) / 32768.0f; // setting the accel scale to 8G
    break;
  }
  case ACCEL_RANGE_16G: {
    // setting the accel range to 16G
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0) {
      return -1;
    }
    _accelScale = (gG * 16.0f) / 32768.0f; // setting the accel scale to 16G
    break;
  }
  }
  _accelRange = range;
  return 1;
}

/**
 * Configura a sensibilidade de operação do giroscópio.
 * @param  range de operação
 * @return positivo se ok, negativo se falhar.
 */
int IMU::setGyroRange(GyroRange range) {
  switch (range) {
  case GYRO_RANGE_250DPS: {
    // setting the gyro range to 250DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS) < 0) {
      return -1;
    }
    _gyroScale =
        250.0 / 32767.5 * DEG_TO_RAD; // setting the gyro scale to 250DPS
    break;
  }
  case GYRO_RANGE_500DPS: {
    // setting the gyro range to 500DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS) < 0) {
      return -1;
    }
    _gyroScale =
        500.0 / 32767.5 * DEG_TO_RAD; // setting the gyro scale to 500DPS
    break;
  }
  case GYRO_RANGE_1000DPS: {
    // setting the gyro range to 1000DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS) < 0) {
      return -1;
    }
    _gyroScale =
        1000.0 / 32767.5 * DEG_TO_RAD; // setting the gyro scale to 1000DPS
    break;
  }
  case GYRO_RANGE_2000DPS: {
    // setting the gyro range to 2000DPS
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0) {
      return -1;
    }
    _gyroScale =
        2000.0 / 32767.5 * DEG_TO_RAD; // setting the gyro scale to 2000DPS
    break;
  }
  }
  _gyroRange = range;
  return 1;
}

/**
 * Configura a o filtro passa baixa do acelerômetro e do giroscópio.
 * @param  bandwidth de operação
 * @return positivo se ok, negativo se falhar.
 */
int IMU::setDlpfBandwidth(DlpfBandwidth bandwidth) {
  switch (bandwidth) {
  case DLPF_BANDWIDTH_184HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) <
        0) { // setting accel bandwidth to 184Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_184) <
        0) { // setting gyro bandwidth to 184Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_92HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92) <
        0) { // setting accel bandwidth to 92Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_92) <
        0) { // setting gyro bandwidth to 92Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_41HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41) <
        0) { // setting accel bandwidth to 41Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_41) <
        0) { // setting gyro bandwidth to 41Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_20HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20) <
        0) { // setting accel bandwidth to 20Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_20) <
        0) { // setting gyro bandwidth to 20Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_10HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10) <
        0) { // setting accel bandwidth to 10Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_10) <
        0) { // setting gyro bandwidth to 10Hz
      return -2;
    }
    break;
  }
  case DLPF_BANDWIDTH_5HZ: {
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5) <
        0) { // setting accel bandwidth to 5Hz
      return -1;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_5) <
        0) { // setting gyro bandwidth to 5Hz
      return -2;
    }
    break;
  }
  }
  _bandwidth = bandwidth;
  return 1;
}
/**
 * Faz leitura da MARG.
 * @return Positivo se ok, negativo para ok contrário.
 * @details As variáveis #_accel, #_gyro e #_mag são modificadas.
 */
int IMU::readSensor() {
  //_use SPIHS = true; // use the high speed SPI for data readout
  // grab the data from the MPU9250
  if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _ax_counts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  _ay_counts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _az_counts = (((int16_t)_buffer[4]) << 8) | _buffer[5];

  // _t_counts = (((int16_t)_buffer[6]) << 8) | _buffer[7];

  _gx_counts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gy_counts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gz_counts = (((int16_t)_buffer[12]) << 8) | _buffer[13];

  _hx_counts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  _hy_counts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  _hz_counts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  /* corrige a direcao do eixo z  para -z e comuta o x com o y
esse procedimento eh fundamental para que os eixos do acelerometro e gyro seja
compartilhados como o magnetometro */
  *_accel << (float)(_ay_counts), (float)(_ax_counts), (float)(_az_counts);
  *_accel = sFAccel * (*_accel) - _biasAccel;
  *_gyro << -(float)_gy_counts, -(float)_gx_counts, -(float)_gz_counts;
  *_gyro *= _gyroScale;
  *_gyro -= _biasGyro;
  // para analise da calibracao
  magTemp << -(float)_hx_counts * _magScaleX, -(float)_hy_counts * _magScaleX,
      -(float)_hz_counts * _magScaleX;
  *_mag = _sM * (magTemp - _biasMag);
  // *_mag = (magTemp - _biasMag);
  /* @annotation retirado do manual de registros da mpu9250 pag.33*/
  // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity)+ 21degC
  // RoomTemp_Offset = 21; Temp_Sensitivity = 333.87
  // _t = (float)_t_counts / 333.87f - 20.937101267f;
  return 1;
}
/**
 * Método de calibração do giroscópio.
 * @param N número de dados coletados para computodo da média
 * @details Compensa-se o erro sistemático eliminando calculando a medida de #N
 * medidas. O valor compensando é dado por gyro_c = _gyroScale*(gyro_m -
 * _biasGyro)
 */
void IMU::calibraGyro(int N /*=100*/) {
  Eigen::Vector3f _biasGyroTemp = Eigen::Vector3f::Zero();
  for (int i = 0; i < N; i++) {
    readSensor();
    _biasGyroTemp += *_gyro;
    delay(20);
  }
  _biasGyro = _biasGyroTemp / float(N);
}

int IMU::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest) {
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
  writeRegister(I2C_SLV0_REG, subAddress);
  writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);
  // delay(1);
  _status = readRegisters(EXT_SENS_DATA_00, count, _buffer);
  return _status;
}
int IMU::writeAK8963Register(uint8_t subAddress, uint8_t data) {
  // set slave 0 to the AK8963 and set for write
  if (writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(I2C_SLV0_REG, subAddress) < 0) {
    return -2;
  }
  // store the data for write
  if (writeRegister(I2C_SLV0_DO, data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
  if (writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
  // read the register and confirm
  if (readAK8963Registers(subAddress, 1, _buffer) < 0) {
    return -5;
  }
  if (_buffer[0] == data) {
    return 1;
  } else {
    return -6;
  }
}
/**
 * Escreve nos registros da Marg
 * @param  subAddress
 * @param  data
 * @return
 * @details Usada para ler os dados e configurar a MARG.
 */
int IMU::writeRegister(uint8_t subAddress, uint8_t data) {
  _i2c->beginTransmission(_address); // open the device
  _i2c->write(subAddress);           // write the register address
  _i2c->write(data);                 // write the data
  _i2c->endTransmission();
  // delay(10);
  /* read back the register */
  readRegisters(subAddress, 1, _buffer);
  /* check the read back register against the written register */
  if (_buffer[0] == data) {
    return 1;
  } else {
    return -1;
  }
}
/**
 * Leitura dos registros da MARG
 * @param  subAddress
 * @param  count
 * @param  dest
 * @return            positivo se ok, negativo para o contrário.
 */
int IMU::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest) {
  _i2c->beginTransmission(_address);
  _i2c->write(subAddress);
  _i2c->endTransmission(false);
  _numBytes = _i2c->requestFrom(_address, count);
  if (_numBytes == count) {
    for (uint8_t i = 0; i < count; i++) {
      dest[i] = _i2c->read();
    }
    return 1;
  } else {
    return -1;
  }
}
/**
 * Checa o registro de um dispositivo.
 * @details usado para o acelerômetro e giroscópio.
 */
int IMU::whoAmI() {
  readRegisters(WHO_AM_I, 1, _buffer);
  return _buffer[0];
}
/**
 * Verifica se o Magnetômetro está ok.
 */
int IMU::whoAmIAK8963() {
  if (readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer) < 0) {
    Serial.println("Nao sei quem sou");
    return -1;
  }
  // Serial.println("Endereco do magnetometro");
  // Serial.print(_buffer[0]);
  return _buffer[0];
}
int IMU::calibracaoMagnetometro(float moduloCampo) {

  // numero de medidas para armazenar dados
  METODOSCALIBRACAO CAL; // instancia funcoes de calibracao
  int N = 500;
  MatrixXf medidas = MatrixXf::Zero(3, N);
  // coleta N medidas
  VectorXf temp(17);
  for (int i = 0; i < N; i++) {
    readSensor();
    medidas.col(i) = magTemp;
    delay(20);
  }
  return CAL.calibracaoGeometrica(medidas, _sM, _biasMag, moduloCampo);
}

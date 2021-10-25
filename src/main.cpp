/**
 * @author: Roney Silva (roney)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: main.cpp
 * Last modified by:   roney
 * Last modified time: 31-Aug-2021
 */
#include "Controle.h"
#include "EKF.h"
#include "GRUPO_QUAT.h"
#include "IBusBM.h"
#include "Motores.h"
#include "PARAMETROS.h"
#include "TELEMETRIA.h"
#include <Arduino.h>
#include <eigen3/Eigen/Dense>
// #include <esp_now.h>
#define DEBUG_MAIN
#include "COMMON.h"
#ifdef DEBUG_MAIN
#define PM(X) std::cout << #X << ":\n" << X << "\n"
#include <iostream>
#endif
#define CALIBRATION_OF_MAGNETOMETER_GYROSCOPE

IBusBM iBus;
// ponteiros para telemetria
Vector3f *eulerTarget, *ra, *wa, *rI, qa;

Vector4f *moments;
Vector4i *comandosDigitais;

TELEMETRIA tel;
#ifdef EKF_H
using namespace ekf;
ekf::EKF filtro;
struct STATUS_CLASS {
  int ctrlStatus = STOPED;
  int filtroStatus = STOPED;
};
STATUS_CLASS statusClass;
#endif
// identificadores de tasks
TaskHandle_t xHandleSetupFilter = NULL;
TaskHandle_t xHandleLoopFilter = NULL;
TaskHandle_t xHandleControl = NULL;
TaskHandle_t xHandleEmergency = NULL;

void xTaskLoopFilter(void *) {
  tel.enviaMensagem((char *)"xTaskLoopFilter Criada");
  TickType_t xLastDtTelemetry = 0;
  const uint8_t size_data_to_send = 2 * (3 + 3 + 3 + 3);
  Vector<float, size_data_to_send + 1> dataSerial;
  statusClass.filtroStatus = filtro.status;
  tel.enviaMensagem((char *)"Esperando a classe controle...");
  while (statusClass.ctrlStatus != READY) {
    vTaskDelay(1000);
  }
  tel.enviaMensagem((char *)"Contr.OK. estimador Rodando");

  for (;;) {
    filtro.loopEKF();
    quaternion2Euler(filtro.euler, filtro.q);
// 4 + 3+3+1+6
#if 0
    tel.imu.dadosEigen <<         //
                                  // /*0:3*/ RAD_TO_DEG * filtro.euler,   //
        /*0:3*/ filtro.q.tail(3), //
        /*3:6*/ filtro.gyro,      //
        /*6:9*/ filtro.drNED,     //
        /*9:12  filtro.rNED, */   //
                                  // /*9:12*/ RAD_TO_DEG * (*eulerTarget),   //
        /*9:12*/ qa,              //
        /*12:15*/ *wa,            //
        /*15:18*/ *ra,            //
        /*  *rI*/                 //;
        /*18:21*/ (*moments),
        /*22:25*/ (*comandosDigitais).cast<float>();
    // tel.envia(size_data_to_send, dataSerial);
    tel.envia();
#endif
    vTaskDelayUntil(&xLastDtTelemetry, xdtTelemetry);
  }
}
void xTaskSetupFiltro(void *pvParameters) {
  tel.enviaMensagem((char *)"Dentro da xTaskSetupFiltro");
  filtro.begin(float(dtTelemetry) * 1e-3);
  tel.enviaMensagem((char *)"filtro.begin() ok");
#ifdef CALIBRATION_OF_MAGNETOMETER_GYROSCOPE
  tel.enviaMensagem((char *)"Dentro da Calibracao");
  iBus.loop();
  printf(" valor da calibracao %d\n", iBus.remoteControl.calibration);
  if (iBus.remoteControl.calibration > 1500) {
    tel.enviaMensagem((char *)"Ent. no método de Calibracao");
    filtro.calibrationMethods();
    while (iBus.remoteControl.calibration > 1500) {
      iBus.loop();
      tel.enviaMensagem((char *)"Cal. OK. Turn of swb");
      vTaskDelay(500);
    }
  }
#endif
  xTaskCreate(xTaskLoopFilter, "FilterLoop", 15000, NULL, 0,
              &xHandleLoopFilter);
  tel.enviaMensagem((char *)"xTaskLoopFiltro");
  vTaskDelete(NULL);
}
/**
 * @brief Task de Procedimentos emergenciais.
 *
 */
void emergencyMethods(void *pv) {
  tel.enviaMensagem((char *)"emergencyMethods() Ativada");
  vTaskSuspend(xHandleControl);
  turnOnLed();
  // pausa a task de controle
  while (iBus.remoteControl.emergency > 1500) {
    iBus.loop();
    vTaskDelay(100);
  }
  turnOffLed();
  vTaskResume(xHandleControl);
  tel.enviaMensagem((char *)"emergencyMethods() Desativada");
  vTaskDelete(NULL);
}
void xTaskContol(void *pv) {
  Controle ctrl(dtControle * 1e-3);
  MOTOR_PINS mp;
  eulerTarget = &ctrl.eulerTarget;
  wa = &ctrl.wa;
  ra = &ctrl.r;
  moments = &ctrl.u;
  comandosDigitais = &ctrl.motorCommands;
#ifdef INTEGRAL_CONTROL
  rI = &ctrl.rI;
#else
  rI = &ctrl.drone->p;
#endif
  TickType_t xLastDtControl = 0;
  // Instancia dos motores
  Motores motor1(mp.motor_1, 0);
  Motores motor2(mp.motor_2, 1);
  Motores motor3(mp.motor_3, 2);
  Motores motor4(mp.motor_4, 3);
  tel.enviaMensagem((char *)"TaskControl .. esperando filtro");
  while (filtro.status != READY) {
    iBus.loop();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  tel.enviaMensagem((char *)"Filtro ok");
  iBus.loop();
  statusClass.ctrlStatus = READY;
  ctrl.r = Vector3f::Zero();

  for (;;) {
    // atualiza estados da classe Drone dada os valores estimado no EKF
    iBus.loop();
    // if (ibus.readChannel(4) > 1500) {
    if (iBus.remoteControl.emergency < 1500) {
      /*Para tentar obter os corretos valores de */

      ctrl.r(0) = ((float)(iBus.remoteControl.dX - 1500)) / 500.f;
      ctrl.r(1) = ((float)(iBus.remoteControl.dY - 1500)) / 500.f;
      ctrl.r(2) = ((float)(iBus.remoteControl.dZ - 1500)) / 500.f;
      // Limita o máximo que o commando envia para o cotrole.
      // ctrl.maximalDigitalCommand = 7.7 *
      // ibus.remoteControl.throttle_limitation;

      tel.gainsTunning[1] =
          9.999999 * ((float)iBus.remoteControl.adjustQuat - 1000.0f) + 0.001f;
      tel.gainsTunning[2] =
          9.999999 * ((float)iBus.remoteControl.adjustQrAngular - 1000.0f) +
          0.001f;

      ctrl.Qr.topLeftCorner(3, 3) = Matrix3f::Identity() * tel.gainsTunning[1];
      ctrl.Qr.bottomRightCorner(3, 3) =
          Matrix3f::Identity() * tel.gainsTunning[2];
      tel.gainsTunning[0] = ctrl.ut[2];
      tel.enviaTunningGains();
      ctrl.diffPsi = ((float)(iBus.remoteControl.dPsi - 1500)) / 500.f;
      /* Atualiza estados da Classe Drone. Estes mesmos estados são utilizadas
       * pela classe Controle para calcular as rotações dos motores.*/
      ctrl.drone->p = filtro.rNED;
      ctrl.drone->v = filtro.drNED;
      ctrl.drone->q = filtro.q;
      ctrl.drone->w = filtro.gyro;
      qa = ctrl.qa.block(1, ctrl.index_alt, 3, 1);

      // adiciona as rotacoes
      ctrl.controlLoop();
      motor1.set_vel_mot(ctrl.motorCommands(0));
      motor2.set_vel_mot(ctrl.motorCommands(1));
      motor3.set_vel_mot(ctrl.motorCommands(2));
      motor4.set_vel_mot(ctrl.motorCommands(3));

    } else {
      // para os motores
      motor1.set_vel_mot(ctrl.minimalDigitalCommand);
      motor2.set_vel_mot(ctrl.minimalDigitalCommand);
      motor3.set_vel_mot(ctrl.minimalDigitalCommand);
      motor4.set_vel_mot(ctrl.minimalDigitalCommand);
      tel.enviaMensagem((char *)"Condicao de paradada emergencyMethods()");
      xTaskCreatePinnedToCore(emergencyMethods, "emergencyMethods", 2000, NULL,
                              5, NULL, 1);
    }

    vTaskDelayUntil(&xLastDtControl, xdtControle);
  }
}

// #define TEST_IMU
void setup() {
  pinMode(onboard_led, OUTPUT);

#ifndef TEST_IMU
  tel.begin();
  iBus.begin(Serial2, IBUSBM_NOTIMER, pin_rx_controle);
  iBus.loop();
  while (iBus.remoteControl.emergency < 1000) {
    tel.enviaMensagem((char *)"Cont Remoto com problema");
    vTaskDelay(250);
    iBus.loop();
  }
  tel.enviaMensagem((char *)"Controle ok");
  tel.enviaMensagem((char *)"Criando Task xTaskSetupFiltro ");
  xTaskCreatePinnedToCore(xTaskSetupFiltro, "SetupFiltro", 25000, NULL, 0,
                          &xHandleSetupFilter, 0);
  xTaskCreatePinnedToCore(xTaskContol, "Control", 20000, NULL, 2,
                          &xHandleControl, 1);
#else
  filtro.begin(float(dtTelemetry) * 1e-3);
#endif
}

void loop() {
#ifdef TEST_IMU
  filtro.updateOfMeasurements();
  PM(filtro.gyro.transpose());
  PM(filtro.accel.transpose());
  PM(filtro.mag.transpose());
  delay(500);
#endif
}

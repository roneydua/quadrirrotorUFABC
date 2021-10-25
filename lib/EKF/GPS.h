/**
 * @author: Roney Silva (roney)
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: GPS.h
 * @brief Simples Cabecalho de funcoes para leituras do GPS
 * Last modified by:   roney
 * Last modified time: 31-Aug-2021
 */

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
HardwareSerial GPSSERIAL(1);

// #define serialHW GPSSERIAL;
// Definas quais mensagens deseja que o sistema envie
// #define GxGGA
// #define GxGLL
// #define GxGSA
// #define GxGSV
// #define GxRMC
// #define GxVTG
#define NAV_PVT
// #define NAV_VELNED
// #define NAV_POSLLH
#define NAV_POSECEF
// Defina a frequencia desejada
#define RATE_10
// #define RATE_5
// #define RATE_1
#define NAV_MODE_1G
// #define NAV_MODE_2G
const unsigned char UBX_HEADER[] = {0xB5, 0x62};

#ifdef NAV_POSLLH
const unsigned char NAV_POSLLH_HEADER[] = {0x01, 0x02};
struct NAV_POSLLH_STRUCT {
  unsigned char cls;  // 0x01
  unsigned char id;   // 0x02
  unsigned short len; // 28

  unsigned long iTOW; // millisecond time of week
  // long lon;           // longitude
  long lon;           // longitude
  long lat;           // latitude
  long height;        // height above ellipsoid
  long hMSL;          // height above mean sea level
  unsigned long hAcc; // horizontal accuracy
  unsigned long vAcc; // vertical accuracy
};

#endif
#ifdef NAV_POSECEF
const unsigned char NAV_POSECEF_HEADER[] = {0x01, 0x01};
struct NAV_POSECEF_STRUCT {
  unsigned char cls;  // 0x01
  unsigned char id;   // 0x02
  unsigned short len; // 20
  unsigned long iTOW; // millisecond time of week
  /*! ECEF X coordinate (cm)*/
  long ecefX;
  /*! ECEF Y coordinate (cm)*/
  long ecefY;
  /*! ECEF Z coordinate (cm)*/
  long ecefZ;
  long pAcc; // Position Accuracy Estimate
};
#endif
#ifdef NAV_VELNED
const unsigned char NAV_VELNED_HEADER[] = {0x01, 0x12};
struct NAV_VELNED_STRUCT {
  unsigned char cls;  // 0x01
  unsigned char id;   // 0x12
  unsigned short len; // 36
  unsigned long iTOW; // millisecond time of week
  /*! Velocidade na direção norte @warning (cm/s) */
  long velN;
  /*! Velocidade na direção leste @warning (cm/s) */
  long velE;
  /*! Velocidade na direção para baixo @warning (cm/s) */
  long velD;
  unsigned long speed;  // 3d (cm/s)
  unsigned long gSpeed; // ground speed (2d)
  long heading;         // heading of motion (2d)
  unsigned long sAcc;   // speed accuracy
  unsigned long cAcc;   // course/heading accuracy
};

#endif
#ifdef NAV_PVT
const unsigned char NAV_PVT_HEADER[] = {0x01, 0x07};
struct NAV_PVT_STRUCT {
  unsigned char cls;  // 0x01
  unsigned char id;   // 0x12
  unsigned short len; // 92
  /* 0 */ unsigned long iTOW;
  /* 4 */ unsigned short year;
  /* 6 */ unsigned char month;
  /* 7 */ unsigned char day;
  /* 8 */ unsigned char hour;
  /* 9 */ unsigned char min;
  /* 10 */ unsigned char sec;
  /* 11 */ unsigned char valid;
  /* 12 */ unsigned long tAcc;
  /* 16 */ long nano;
  /* 20 */ unsigned char fixType;
  /* 21 */ unsigned char flags;
  /* 22 */ unsigned char flags2;
  /* 23 */ unsigned char numSV;
  /* 24 */ long lon;
  /* 28 */ long lat;
  /* 32 */ long height;
  /* 36 */ long hMSL;
  /* 40 */ unsigned long hAcc;
  /* 44 */ unsigned long vAcc;
  /*! Velocidade na direção norte (cm/s) */
  /* 48 */ long velN;
  /*! Velocidade na direção leste  (cm/s) */
  /* 52 */ long velE;
  /*! Velocidade na direção para baixo (cm/s) */
  /* 56 */ long velD;
  /* 60 */ long gSpeed;
  /* 64 */ long headMot;
  /* 68 */ unsigned long sAcc;
  /* 72 */ unsigned long headAcc;
  /* 76 */ unsigned short pDOP;
  /* 78 */ unsigned char reserved1[6];
  /* 84 */ long headVeh;
  /* 85 */ unsigned long reserved2;
};

#endif
struct PacketData {
  long lon;
  long lat;
  int32_t altitude;
  long velN;
  long velE;
  long velD;
};
//
PacketData data;

union UBXMessage {
#ifdef NAV_POSLLH
  NAV_POSLLH_STRUCT navPosllh;
#endif
#ifdef NAV_VELNED
  NAV_VELNED_STRUCT navVelned;
#endif
#ifdef NAV_POSECEF
  NAV_POSECEF_STRUCT navPosecef;
#endif
#ifdef NAV_PVT
  NAV_PVT_STRUCT navPvt;
#endif
};

enum _ubxMsgType {
  MT_NONE,
#ifdef NAV_POSLLH
  MT_NAV_POSLLH,
#endif
#ifdef NAV_VELNED
  MT_NAV_VELNED,
#endif
#ifdef NAV_POSECEF
  MT_NAV_POSECEF,
#endif
#ifdef NAV_PVT
  MT_NAV_PVT,
#endif
};

UBXMessage ubxMessage;

void gpsSetup() {
  // GPSSERIAL.begin(baud, config = SERIAL_8N1, rxPin,txPin)
  GPSSERIAL.begin(9600, SERIAL_8N1, 16, 17);

  unsigned char VALORES_PADROES[] PROGMEM = {
      // valores padroes
      0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A,
  };
  // Desabilitar NMEA
  unsigned char UBLOX_CONFIGURACAO[] PROGMEM = {
  // Desabilitar NMEA

#ifndef GxGGA
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xFF, 0x23,
#endif
#ifndef GxGLL
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x2A,
#endif
#ifndef GxGSA
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x31,
#endif
#ifndef GxGSV
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x02, 0x38,
#endif
#ifndef GxRMC
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x03, 0x3F,
#endif
#ifndef GxVTG
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x04, 0x46,
#endif
// Configuracao de Navegacao
#ifdef NAV_VELNED
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x23, 0x2E,
#endif

#ifdef NAV_POSLLH
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x13, 0xBE, //
#endif

#ifdef NAV_POSECEF
      // 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x29, 0x01, 0x00, 0x01, 0x00, 0x00,
      // 0x00, 0x00, 0x3A, 0xF7, //
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x12, 0xB7, //

#endif

#ifdef NAV_PVT
      0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x18, 0xE1, //

#endif
// frequencia de amostragem
#ifdef RATE_10
      0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00,
      0x7A, 0x12, //
#endif
#ifdef RATE_5
      0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00,
      0xDE, 0x6A, //
#endif
#ifdef RATE_1
      0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
      0x01, 0x39,
#endif
#ifdef NAV_MODE_2G
      // Configuracao do NAV MODE  opcao 7- Airbone < 2g
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00,
      0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
      0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A,
#endif
#ifdef NAV_MODE_1G
      // NAV MODE  opcao 7- Airbone < 1g
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00,
      0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
      0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85,
      0x2A, //
#endif
  };

  for (int i = 0; i < (int)sizeof(VALORES_PADROES); i++) {
    GPSSERIAL.write(pgm_read_byte(VALORES_PADROES + i));
    delay(5); // Simula 38400 baudrate.
  }
  delay(1000);
  // Envia os dados
  for (int i = 0; i < (int)sizeof(UBLOX_CONFIGURACAO); i++) {
    GPSSERIAL.write(pgm_read_byte(UBLOX_CONFIGURACAO + i));
    delay(5); // Simula 38400 baudrate.
  }
}
// The last two bytes of the message is a checksum value, used to confirm that
// the received payload is valid. The procedure used to calculate this is given
// as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char *CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char *)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}

// Compares the first two bytes of the ubxMessage struct with a specific message
// header. Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char *msgHeader) {
  unsigned char *ptr = (unsigned char *)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];

  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);
  // trocado while por if para naum travar o processo.
  while (GPSSERIAL.available()) {
    byte c = GPSSERIAL.read();
    if (fpos < 2) {
      /* Procura se os primeiro dois bytes sao os cabecalhos(0xB5,0x62)*/
      if (c == UBX_HEADER[fpos]) {
        fpos++;
      } else {
        fpos = 0;
      }
    } else {
      /* Este trecho é acessado apenas se a mensagem coincide com UBX_HEADER.*/
      // Place the incoming byte into the ubxMessage struct. The position is
      // fpos-2 because the struct does not include the initial two-byte header
      // (UBX_HEADER).
      if ((fpos - 2) < payloadSize)
        ((unsigned char *)(&ubxMessage))[fpos - 2] = c;

      fpos++;

      if (fpos == 4) {
        // We have just received the second byte of the message type header,
        // so now we can check to see what kind of message it is.
#ifdef NAV_POSLLH
        if (compareMsgHeader(NAV_POSLLH_HEADER)) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH_STRUCT);
        } else
#endif
#ifdef NAV_VELNED
            if (compareMsgHeader(NAV_VELNED_HEADER)) {
          currentMsgType = MT_NAV_VELNED;
          payloadSize = sizeof(NAV_VELNED_STRUCT);
        } else
#endif
#ifdef NAV_POSECEF
            if (compareMsgHeader(NAV_POSECEF_HEADER)) {
          currentMsgType = MT_NAV_POSECEF;
          payloadSize = sizeof(NAV_POSECEF_STRUCT);
        } else
#endif
#ifdef NAV_PVT
            if (compareMsgHeader(NAV_PVT_HEADER)) {
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(NAV_PVT_STRUCT);

        } else
#endif
        {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if (fpos == (payloadSize + 2)) {
        // All payload bytes have now been received, so we can calculate the
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      } else if (fpos == (payloadSize + 3)) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if (c != checksum[0]) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0;
        }
      } else if (fpos == (payloadSize + 4)) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum
                  // matches.
        if (c == checksum[1]) {
          // Checksum matches, we have a valid message.
          return currentMsgType;
        }
      } else if (fpos > (payloadSize + 4)) {
        // We have now read more bytes than both the expected payload and
        // checksum together, so something went wrong. Reset to beginning state
        // and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}
void gps_imprime() {

  while (GPSSERIAL.available()) {
    // Serial.write(GPSSERIAL.read());
    int msgType = processGPS();
#ifdef NAV_VELNED
    if (msgType == MT_NAV_VELNED) {
      printf("MT_NAV_VELNED: %lu\t %f\t %f\t %f\t \n",
             ubxMessage.navVelned.iTOW,
             float(ubxMessage.navVelned.velD) * 0.01f,
             float(ubxMessage.navVelned.velD) * 0.01f,
             float(ubxMessage.navVelned.velD) * 0.01f);
    } // else
#endif
#ifdef NAV_POSLLH
    if (msgType == MT_NAV_POSLLH) {
      printf("MT_NAV_POSLLH: %lu\t %ld\t \t%ld \t\n", ubxMessage.navPosllh.iTOW,
             ubxMessage.navPosllh.lat, ubxMessage.navPosllh.lon);
    } // else
#endif
#ifdef NAV_POSECEF
    if (msgType == MT_NAV_POSECEF) {
      printf("NAV_POSECEF : %lu \t %ld\t \t%ld \t%ld \t\n",
             ubxMessage.navPosecef.iTOW, ubxMessage.navPosecef.ecefX,
             ubxMessage.navPosecef.ecefY, ubxMessage.navPosecef.ecefZ);
    } // else
#endif

#ifdef NAV_PVT
    if (msgType == MT_NAV_PVT) {
      printf("MT_NAV_PVT:\t %lu\t %ul\t  %f\t %f\t %f\t %f\t %f\t %f \n",
             ubxMessage.navPvt.iTOW, ubxMessage.navPvt.fixType,
             0.001 * (float)ubxMessage.navPvt.velN,
             0.001 * (float)ubxMessage.navPvt.velE,
             0.001 * (float)ubxMessage.navPvt.velD,
             1e-7 * DEG_TO_RAD * (float)ubxMessage.navPvt.lat,
             1e-7 * DEG_TO_RAD * (float)ubxMessage.navPvt.lon,
             0.001f * (float)ubxMessage.navPvt.hMSL);
    }
#endif
  }
}

#endif /* GPS_H */

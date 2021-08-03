#ifndef NTRIP_H
#define NTRIP_H

#include <string>
#include "serial.h"

enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

struct Args
{
  const char *server;
  const char *port;
  const char *user;
  const char *proxyhost;
  const char *proxyport;
  const char *password;
  const char *nmea;
  const char *data;
  int         bitrate;
  int         mode;

  int         udpport;
  int         initudp;
  enum SerialBaud baud;
  enum SerialDatabits databits;
  enum SerialStopbits stopbits;
  enum SerialParity parity;
  enum SerialProtocol protocol;
  const char *serdevice;
  const char *serlogfile;
  bool stop;
};


struct Location
{

    std::string nmea;
    std::string lat;
    std::string lon;
    std::string alt;
    std::string hdop;
    std::string vdop;
    std::string fix;
};

void ntrip_client(Args*  const);
Location getGNGGA();
#endif



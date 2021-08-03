#ifndef NSERIAL_H
#define NSERIAL_H

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#define SERIALDEFAULTDEVICE "/dev/ttyACM0"
enum SerialBaud {
  SPABAUD_50 = B50, SPABAUD_110 = B110, SPABAUD_300 = B300, SPABAUD_600 = B600,
  SPABAUD_1200 = B1200, SPABAUD_2400 = B2400, SPABAUD_4800 = B4800,
  SPABAUD_9600 = B9600, SPABAUD_19200 = B19200,
  SPABAUD_38400 = B38400, SPABAUD_57600 = B57600, SPABAUD_115200 = B115200 };
enum SerialDatabits {
  SPADATABITS_5 = CS5, SPADATABITS_6 = CS6, SPADATABITS_7 = CS7, SPADATABITS_8 = CS8 };
enum SerialStopbits {
  SPASTOPBITS_1 = 0, SPASTOPBITS_2 = CSTOPB };
enum SerialParity {
  SPAPARITY_NONE = 0, SPAPARITY_ODD = PARODD | PARENB, SPAPARITY_EVEN = PARENB };
enum SerialProtocol {
  SPAPROTOCOL_NONE = 0, SPAPROTOCOL_RTS_CTS = 9999,
  SPAPROTOCOL_XON_XOFF = IXOFF | IXON };

struct serial
{
  struct termios Termios;
  int            Stream;
};


static enum SerialParity SerialGetParity(const char *buf, int *ressize)
{
  int r = 0;
  enum SerialParity p = SPAPARITY_NONE;
  if(!strncasecmp(buf, "none", 4))
  { r = 4; p = SPAPARITY_NONE; }
  else if(!strncasecmp(buf, "no", 2))
  { r = 2; p = SPAPARITY_NONE; }
  else if(!strncasecmp(buf, "odd", 3))
  { r = 3; p = SPAPARITY_ODD; }
  else if(!strncasecmp(buf, "even", 4))
  { r = 4; p = SPAPARITY_EVEN; }
  else if(*buf == 'N' || *buf == 'n')
  { r = 1; p = SPAPARITY_NONE; }
  else if(*buf == 'O' || *buf == 'o')
  { r = 1; p = SPAPARITY_ODD; }
  else if(*buf == 'E' || *buf == 'e')
  { r = 1; p = SPAPARITY_EVEN; }
  if(ressize) *ressize = r;
  return p;
}

static enum SerialProtocol SerialGetProtocol(const char *buf, int *ressize)
{
  int r = 0;
  enum SerialProtocol Protocol = SPAPROTOCOL_NONE;
  /* try some possible forms for input, be as gentle as possible */
  if(!strncasecmp("xonxoff",buf,7)){r = 7; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("xon_xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("xon-xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("xon xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("xoff",buf,4)){r = 4; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("xon",buf,3)){r = 3; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(*buf == 'x' || *buf == 'X'){r = 1; Protocol=SPAPROTOCOL_XON_XOFF;}
  else if(!strncasecmp("rtscts",buf,6)){r = 6; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("rts_cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("rts-cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("rts cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("rts",buf,3)){r = 3; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("cts",buf,3)){r = 3; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(*buf == 'r' || *buf == 'R' || *buf == 'c' || *buf == 'C')
  {r = 1; Protocol=SPAPROTOCOL_RTS_CTS;}
  else if(!strncasecmp("none",buf,4)){r = 4; Protocol=SPAPROTOCOL_NONE;}
  else if(!strncasecmp("no",buf,2)){r = 2; Protocol=SPAPROTOCOL_NONE;}
  else if(*buf == 'n' || *buf == 'N'){r = 1; Protocol=SPAPROTOCOL_NONE;}
  if(ressize) *ressize = r;
  return Protocol;
}


void SerialFree(struct serial *sn);
int SerialWrite(struct serial *sn, const char *buffer, size_t size);
int SerialRead(struct serial *sn, char *buffer, size_t size);
const char * SerialInit(struct serial *sn,
const char *Device, enum SerialBaud Baud, enum SerialStopbits StopBits,
	enum SerialProtocol Protocol, enum SerialParity Parity,
	enum SerialDatabits DataBits, int dowrite
	#ifdef __GNUC__
	__attribute__((__unused__))
	#endif /* __GNUC__ */
);

#endif


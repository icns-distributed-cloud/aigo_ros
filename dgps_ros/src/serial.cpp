/*
  Serial port access for NTRIP client for POSIX.
  $Id$
  Copyright (C) 2008 by Dirk Stöcker <soft@dstoecker.de>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  or read http://www.gnu.org/licenses/gpl.txt
*/

/* system includes */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "serial.h"

void SerialFree(struct serial *sn)
{
  if(sn->Stream)
  {
    /* reset old settings */
    tcsetattr(sn->Stream, TCSANOW, &sn->Termios);
    close(sn->Stream);
    sn->Stream = 0;
  }
}

const char * SerialInit(struct serial *sn,
const char *Device, enum SerialBaud Baud, enum SerialStopbits StopBits,
enum SerialProtocol Protocol, enum SerialParity Parity,
enum SerialDatabits DataBits, int dowrite
#ifdef __GNUC__
__attribute__((__unused__))
#endif /* __GNUC__ */
)
{
  struct termios newtermios;

  if((sn->Stream = open(Device, O_RDWR | O_NOCTTY | O_NONBLOCK)) <= 0)
    return "could not open serial port";
  tcgetattr(sn->Stream, &sn->Termios);

  memset(&newtermios, 0, sizeof(struct termios));
  newtermios.c_cflag = Baud | StopBits | Parity | DataBits
  | CLOCAL | CREAD;
  if(Protocol == SPAPROTOCOL_RTS_CTS)
    newtermios.c_cflag |= CRTSCTS;
  else
    newtermios.c_cflag |= Protocol;
  newtermios.c_cc[VMIN] = 1;
  tcflush(sn->Stream, TCIOFLUSH);
  tcsetattr(sn->Stream, TCSANOW, &newtermios);
  tcflush(sn->Stream, TCIOFLUSH);
  fcntl(sn->Stream, F_SETFL, O_NONBLOCK);
  return 0;
}

int SerialRead(struct serial *sn, char *buffer, size_t size)
{
  int j = read(sn->Stream, buffer, size);
  if(j < 0)
  {
    if(errno == EAGAIN)
      return 0;
    else
      return j;
  }
  return j;
}

int SerialWrite(struct serial *sn, const char *buffer, size_t size)
{
  int j = write(sn->Stream, buffer, size);
  if(j < 0)
  {
    if(errno == EAGAIN)
      return 0;
    else
      return j;
  }
  return j;
}



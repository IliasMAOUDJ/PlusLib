/*=Plus=header=begin======================================================
   Program: Plus
   Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
   See License.txt for details.
 =========================================================Plus=header=end*/

#include "PlusSerialLineLinux.h"

#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions

#include "PlusConfigure.h"
/* open(2) */
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

/* close(2) */
#include <unistd.h>

/* ioctl(2) */
#include <sys/ioctl.h>

/* getchar(3) */
#include <stdio.h>
#include <sys/socket.h>

//----------------------------------------------------------------------------
SerialLineLinux::SerialLineLinux()
  : CommHandle(INVALID_HANDLE_VALUE),
    SerialPortSpeed(9600),
    MaxReplyTime(1000) {}

//----------------------------------------------------------------------------
SerialLineLinux::~SerialLineLinux()
{
  if (CommHandle < 0)
  {
    Close();
  }
}

//----------------------------------------------------------------------------
void SerialLineLinux::Close()
{
  if (CommHandle < 0)
  {
    close(CommHandle);
  }
  CommHandle = INVALID_HANDLE_VALUE;
}

//----------------------------------------------------------------------------
bool SerialLineLinux::Open()
{
  if (CommHandle < 0)
  {
    Close();
  }

  // Open serial port
  CommHandle = open(PortName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (CommHandle < 0)
  {
    return false;
  }

  struct termios options;
  tcgetattr(CommHandle, &options);

  cfsetispeed(&options, B115200); // dcb.BaudRate = SerialPortSpeed;
  cfsetospeed(&options, B115200);

  options.c_cflag &= ~PARENB; // dcb.Parity = NOPARITY;
  options.c_cflag &= ~CSTOPB; // dcb.StopBits = ONESTOPBIT;
  options.c_cflag &= ~CSIZE;  // dcb.ByteSize = 8;
  options.c_cflag |= CS8;

  options.c_cc[VTIME] = 2; // MaxReplyTime/100?;  // tenth of seconds
  options.c_cc[VMIN] = 0;

  /*
  if (!SetupComm(CommHandle, 16384, 16384))
  {
    Close();
    return false;
  }
  DCB dcb;
  GetCommState(CommHandle, &dcb);
  if (!SetCommState(CommHandle, &dcb))
  {
    Close();
    return false;
  }

  COMMTIMEOUTS timeouts;
  GetCommTimeouts(CommHandle, &timeouts);
  timeouts.ReadIntervalTimeout = 200;
  timeouts.ReadTotalTimeoutConstant = MaxReplyTime;
  timeouts.ReadTotalTimeoutMultiplier = 100;
  timeouts.WriteTotalTimeoutConstant = MaxReplyTime;
  timeouts.WriteTotalTimeoutMultiplier = 100;
  if (!SetCommTimeouts(CommHandle, &timeouts))
  {
    Close();
    return false;
  }
   */
  return true;
}

//----------------------------------------------------------------------------
int SerialLineLinux::Write(const BYTE* data, int numberOfBytesToWrite)
{
  int numberOfBytesWritten;
  int numberOfBytesWrittenTotal = 0;
  if ((numberOfBytesWritten =
         write(CommHandle, (char*)data, numberOfBytesToWrite)) < 0)
  {
    if (errno == ECONNABORTED)
    {
      // system error: clear error and retry
      // unsigned int errors = 0;
      // ClearCommError(CommHandle, &errors, NULL);
      LOG_INFO("WRITE ERROR (ECONNABORTED)");
    }
    else if (errno == EAGAIN)
    {
      LOG_INFO("WRITE ERROR (EAGAIN)");
    }
    else
    {
      // other error
      LOG_INFO("WRITE ERROR (" << errno << ") with data " << std::hex
               << std::setw(2) << std::setfill('0')
               << (int)*data)
    }
  }
  else if (numberOfBytesWritten == 0)
  {
    // no characters written, must have timed out
    LOG_INFO("nbytesTotal(timeout):" << numberOfBytesWrittenTotal);
  }
  else
  {
    numberOfBytesWrittenTotal += numberOfBytesWritten;
  }

  return numberOfBytesWrittenTotal;
}

//----------------------------------------------------------------------------
bool SerialLineLinux::Write(const BYTE data)
{
  return Write(&data, 1) == 1;
}

//----------------------------------------------------------------------------
int SerialLineLinux::Read(BYTE* data, int maxNumberOfBytesToRead)
{
  int nbytes; /* Number of bytes read */
  int nbytesTotal = 0;
  if ((nbytes = read(CommHandle, data, maxNumberOfBytesToRead)) < 0)
  {
    if (errno == ECONNABORTED)
    {
      LOG_INFO("READ ERROR (ECONNABORTED)");
    }
    else if (errno == EAGAIN)
    {
      LOG_INFO("READ ERROR (EAGAIN)");
    }
    else
    {
      // other error
      LOG_INFO("READ ERROR (" << errno << ") with data " << (int)*data);
    }
  }
  else if (nbytes == 0)
  {
    LOG_INFO("nbytesTotal(timeout):" << nbytesTotal);
  }
  else
  {
    nbytesTotal += nbytes;
  }
  return nbytesTotal;
}

//----------------------------------------------------------------------------
bool SerialLineLinux::Read(BYTE& data)
{
  return Read(&data, 1) == 1;
}

//----------------------------------------------------------------------------
unsigned int SerialLineLinux::ClearError()
{
  unsigned int dwErrors = 0;
  // COMSTAT comStat;
  // ClearCommError(CommHandle, &dwErrors, &comStat);
  return dwErrors;
}

//----------------------------------------------------------------------------
void SerialLineLinux::SetSerialPortSpeed(unsigned int speed)
{
  SerialPortSpeed = speed;
}

//----------------------------------------------------------------------------
void SerialLineLinux::SetMaxReplyTime(int maxreply)
{
  MaxReplyTime = maxreply;
}

//----------------------------------------------------------------------------
int SerialLineLinux::GetMaxReplyTime() const
{
  return MaxReplyTime;
}

//----------------------------------------------------------------------------
bool SerialLineLinux::IsHandleAlive() const
{
  int flags;
  ioctl(CommHandle, TIOCMGET, &flags);
  return (CommHandle != INVALID_HANDLE_VALUE);
}

//----------------------------------------------------------------------------
unsigned int SerialLineLinux::GetNumberOfBytesAvailableForReading() const
{
  unsigned int bytes = 0;
  if (ioctl(CommHandle, FIONREAD, &bytes) < 0)
  {
    LOG_INFO("GetNumberOfBytesAvailableForReading ERROR");
  }
  return bytes;
}

//----------------------------------------------------------------------------
PlusStatus SerialLineLinux::SetDTR(bool onOff)
{
  int bit = TIOCM_DTR;
  if (onOff)
  {
    if (ioctl(CommHandle, TIOCMBIS, &bit) == 0) // set
    {
      return PLUS_SUCCESS;
    }
  }
  else
  {
    if (ioctl(CommHandle, TIOCMBIC, &bit) == 0)
    {
      // clear
      return PLUS_SUCCESS;
    }
  }
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus SerialLineLinux::SetRTS(bool onOff)
{
  int bit = TIOCM_RTS;
  if (onOff)
  {
    if (ioctl(CommHandle, TIOCMBIS, &bit) == 0) // set
    {
      return PLUS_SUCCESS;
    }
  }
  else
  {
    if (ioctl(CommHandle, TIOCMBIC, &bit) == 0)
    {
      // clear
      return PLUS_SUCCESS;
    }
  }
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus SerialLineLinux::GetDSR(bool& onOff)
{
  unsigned int dwStatus = 0;
  if (ioctl(CommHandle, TIOCMGET, &dwStatus) != 0)
  {
    return PLUS_FAIL;
  }
  onOff = TIOCM_DSR & dwStatus;
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus SerialLineLinux::GetCTS(bool& onOff)
{
  unsigned int dwStatus = 0;
  if (ioctl(CommHandle, TIOCMGET, &dwStatus) != 0)
  {
    return PLUS_FAIL;
  }
  onOff = TIOCM_CTS & dwStatus;
  return PLUS_SUCCESS;
}
//----------------------------------------------------------------------------
PlusStatus SerialLineLinux::Flush()
{
  if (ioctl(CommHandle, TCFLSH, 2) == 0)
  {
    return PLUS_SUCCESS;
  }
  return PLUS_FAIL;
}
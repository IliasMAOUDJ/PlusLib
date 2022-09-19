/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef _RAWSerialLineLinux_H_
#define _RAWSerialLineLinux_H_

#define INVALID_HANDLE_VALUE (-1)

#include <PlusCommon.h>

#include <string>

class SerialLineLinux
{
public:
  typedef unsigned int UINT;
  typedef unsigned char BYTE;

  typedef int HANDLE;

  SerialLineLinux();
  virtual ~SerialLineLinux();

  bool Open();

  void Close();

  int Write(const BYTE* data, int numberOfBytesToWrite);

  bool Write(const BYTE data);

  int Read(BYTE* data, int maxNumberOfBytesToRead);

  bool Read(BYTE& data);

  // SetStdStringMacro(PortName);
  // GetStdStringMacro(PortName);

  std::string GetPortName() { return this->PortName; };
  void SetPortName(std::string name) { this->PortName = name; };

  void SetSerialPortSpeed(unsigned int speed);

  void SetMaxReplyTime(int maxreply);

  int GetMaxReplyTime() const;

  bool IsHandleAlive() const;

  unsigned int GetNumberOfBytesAvailableForReading() const;

  PlusStatus SetDTR(bool onOff);

  PlusStatus SetRTS(bool onOff);

  PlusStatus GetDSR(bool& onOff);

  PlusStatus GetCTS(bool& onOff);

  PlusStatus Flush();

  unsigned int ClearError();

private:
  HANDLE CommHandle;
  std::string PortName;
  unsigned int SerialPortSpeed;
  int MaxReplyTime;
};

#endif
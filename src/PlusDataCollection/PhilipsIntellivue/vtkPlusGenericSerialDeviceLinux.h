/*=Plus=header=begin======================================================
 Program: Plus
 Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
 See License.txt for details.
 =========================================================Plus=header=end*/

#ifndef __vtkPlusGenericSerialDeviceLinux_h
#define __vtkPlusGenericSerialDeviceLinux_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

class SerialLineLinux;

class vtkPlusDataCollectionExport vtkPlusGenericSerialDeviceLinux
  : public vtkPlusDevice
{
public:
  static vtkPlusGenericSerialDeviceLinux* New();
  vtkTypeMacro(vtkPlusGenericSerialDeviceLinux, vtkPlusDevice);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  PlusStatus InternalConnect() override;

  PlusStatus InternalDisconnect() override;

  PlusStatus InternalUpdate() override;

  PlusStatus ReadConfiguration(vtkXMLDataElement* config) override;

  PlusStatus WriteConfiguration(vtkXMLDataElement* config) override;

  bool IsTracker() const override { return false; }

  vtkSetMacro(SerialPort, std::string);
  vtkSetMacro(BaudRate, unsigned long);
  vtkSetMacro(MaximumReplyDelaySec, double);
  vtkSetMacro(MaximumReplyDurationSec, double);

  void SetLineEnding(const char* lineEndingHex);
  vtkGetMacro(LineEnding, std::string);

  enum ReplyTermination
  {
    REQUIRE_LINE_ENDING =
      0,  // only proper LineEnding termination will result in success
    REQUIRE_NOT_EMPTY =
      1,   // if respnse is not empty, results in success even on timeout
    ANY = 2  // always results in success, but waits for a timeout or
          // LineEnding termination
  };

  virtual PlusStatus SendText(const std::string& textToSend,
                              std::string* textReceived = NULL) VTK_OVERRIDE
  {
    return this->SendText(textToSend, textReceived, REQUIRE_LINE_ENDING);
  };
  virtual PlusStatus SendText(const std::string& textToSend,
                              std::string* textReceived,
                              ReplyTermination acceptReply);

  virtual PlusStatus ReceiveResponse(
    std::string& textReceived,
    ReplyTermination acceptReply = REQUIRE_LINE_ENDING);

  PlusStatus NotifyConfigured() override;

  PlusStatus SetDTR(bool onOff);

  vtkGetMacro(DTR, bool);

  PlusStatus SetRTS(bool onOff);

  vtkGetMacro(RTS, bool);

  PlusStatus GetDSR(bool& onOff);

  PlusStatus GetCTS(bool& onOff);

protected:
  vtkPlusGenericSerialDeviceLinux();
  ~vtkPlusGenericSerialDeviceLinux();

  virtual bool WaitForResponse();

private:
  vtkPlusGenericSerialDeviceLinux(const vtkPlusGenericSerialDeviceLinux&);
  void operator=(const vtkPlusGenericSerialDeviceLinux&);

protected:
  SerialLineLinux* Serial;

  std::string SerialPort;

  unsigned long BaudRate;

  bool DTR;

  bool RTS;

  std::string LineEnding;

  std::string LineEndingBin;

  double MaximumReplyDelaySec;

  double MaximumReplyDurationSec;

  long FrameNumber;
  vtkPlusDataSource* FieldDataSource;

  vtkSmartPointer<vtkIGSIORecursiveCriticalSection> Mutex;
};

#endif

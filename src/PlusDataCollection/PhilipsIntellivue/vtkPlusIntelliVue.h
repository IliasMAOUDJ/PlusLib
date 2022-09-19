/*=Plus=header=begin======================================================
  Progra  : Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusIntelliVue_h
#define __vtkPlusIntelliVue_h

#include "time.h"
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusGenericSerialDeviceLinux.h"
//
// TODO: NEW DEVICE TO BE TESTED
// Replace by your device to be tested here
// Note that you can take the config file in data directory as
// startup point for your device configuration
//

/*!
  \class vtkPlusIntelliVue
  \brief Interface class to Kinect Azure cameras
  \ingroup PlusLibDataCollection
*/
struct Ava
{
  uint16_t attributeID;
  uint16_t length;
  uint16_t attributeVal;  // placeholder
};
struct AttributeList
{
  uint16_t count;
  uint16_t length;
  Ava value1;  // null placeholder
  std::vector<uint8_t> avaobjectsarray;
};
struct ObservationPoll
{
  uint16_t objHandle;
  AttributeList attributes;
  std::vector<uint8_t> avaobjectsarray;
};
struct SingleContextPoll
{
  uint16_t contextID;
  uint16_t count;
  uint16_t length;
  ObservationPoll value1;  // null placeholder
  std::vector<uint8_t> obpollobjectsarray;
};
struct PollInfoList
{
  uint16_t count;
  uint16_t length;
  SingleContextPoll value1;  // null placeholder
  std::vector<uint8_t> scpollarray;
};
struct NuObsValue
{
  uint16_t physioID;
  uint16_t state;
  uint16_t unitCode;
  uint32_t value;
};
struct NuObsValueCmp
{
  uint16_t count;
  uint16_t length;
  NuObsValue value1;
};
struct SaObsValue
{
  uint16_t physioID;
  uint16_t state;
  uint16_t length;
  // public byte [] value = new byte[1];
  uint8_t value1;
};
struct StringMP
{
  uint16_t length;
  uint16_t value1;
};
struct NumericValResult
{
  std::string Timestamp;
  std::string PhysioID;
  std::string UnitCode;
  std::string Value;
};
struct SaSpec
{
  uint16_t arraySize;
  uint8_t sampleSize;
  uint8_t significantBits;
  uint16_t SaFlags;
  uint16_t obpollHandle;
};
struct SaCalibData16
{
  double lowerAbsoluteValue;
  double upperAbsoluteValue = -99999;
  uint16_t lowerScaledValue;
  uint16_t upperScaledValue;
  uint16_t increment;
  uint16_t calType;
  uint16_t obpollHandle;
  uint16_t physioID;
};
struct WaveValResult
{
  std::string Timestamp;
  std::string PhysioID;
  std::vector<uint8_t> Value;
  uint16_t obpollHandle;
  SaSpec saSpecData;
  SaCalibData16 saCalibData;
};

class vtkPlusDataCollectionExport vtkPlusIntelliVue
  : public vtkPlusGenericSerialDeviceLinux
{
public:
  static vtkPlusIntelliVue* New();
  vtkTypeMacro(vtkPlusIntelliVue, vtkPlusGenericSerialDeviceLinux);

  void PrintSelf(ostream& os, vtkIndent indent) override;

  PlusStatus InternalConnect() override;

  PlusStatus InternalDisconnect() override;

  PlusStatus InternalUpdate() override;

  PlusStatus ReadConfiguration(vtkXMLDataElement* config) override;

  PlusStatus WriteConfiguration(vtkXMLDataElement* config) override;

  PlusStatus sendWaveAssociationRequest();
  PlusStatus sendCycledExtendedPollWaveDataRequest();
  PlusStatus sendCycledExtendedPollDataRequest();
  void setRTSAPriorityList(const std::string& p_signsList);

  PlusStatus sendRTSAPriorityMessage(
    const std::vector<uint8_t>& p_waveTrType);
  PlusStatus getRTSAPriorityListRequest();
  PlusStatus sendMDSPollDataRequest();
  PlusStatus sendMDSCreateEventResult();
  void pollPacketDecoder(std::vector<uint8_t> p_packetbuffer,
                         int p_headersize);
  void checkPollPacketActionType(std::vector<uint8_t> p_packetbuffer);
  void checkLinkedPollPacketActionType(std::vector<uint8_t> p_packetbuffer);
  void processPacket(std::vector<uint8_t> p_packetbuffer);
  void parseMDSCreateEventReport(std::vector<uint8_t> p_readmdsconnectbuffer);
  void decodeMDSAttribObjects(Ava& p_avaobject,
                              std::vector<uint8_t>& p_packetbuffer);
  int decodeObservationPollObjects(ObservationPoll& p_obpollobject,
                                   std::vector<uint8_t>& p_packetbuffer);
  void decodeAvaObjects(Ava& avaobject,
                        std::vector<uint8_t>& p_avaobjectsarray);
  double getAbsoluteTimeFromRelativeTimestamp(
    uint32_t p_currentRelativeTime) const;
  std::string getPacketTimestamp(std::vector<uint8_t> p_header);
  void getBaselineRelativeTimestamp(std::vector<uint8_t> p_timebuffer);
  void readIDLabel(std::vector<uint8_t> p_avaattribobjects);
  static void readIDLabelString(std::vector<uint8_t> p_avaattribobjects);
  void readNumericObservationValue(std::vector<uint8_t> p_avaattribobjects);
  void readCompoundNumericObsValue(std::vector<uint8_t> p_avaattribobjects);
  void readWaveSaObservationValue(std::vector<uint8_t>& p_avaattribobjects);
  void readSaSpecifications(std::vector<uint8_t> p_avaattribobjects);
  void readSaCalibrationSpecifications(
    std::vector<uint8_t> p_avaattribobjects);
  void readCompoundWaveSaObservationValue(
    std::vector<uint8_t> p_avaattribobjects);
  static void exportValListToCSVFile(
    const std::string& p_filename,
    const std::string& p_strbuildNumVal = "");
  void sendNumValues();
  void sendWaveValues();
  void writeNumericHeadersListConsolidatedCSV();
  void createFrameListFromByte(uint8_t b);
  void readPacketFromFrame();
  void clearReadBuffer();

  PlusStatus writeBuffer(const std::vector<uint8_t>& p_txbuf);
  PlusStatus readBuffer();
  bool IsTracker() const override { return true; }

  vtkPlusIntelliVue();
  ~vtkPlusIntelliVue();

private:  // Functions.
  vtkPlusIntelliVue(const vtkPlusIntelliVue&);
  void operator=(const vtkPlusIntelliVue&);
  void SetSigns(std::string Signs) { this->Signs = Signs; }

private:  // Variables.
  std::string Signs;
  vtkPlusDataSource* NumSource{nullptr};
  vtkPlusDataSource* WaveSource{nullptr};

  double m_baseDateTime;
  uint32_t m_baseRelativeTime = 0;
  std::string m_strTimestamp;

  std::string m_strbuildheaders;
  std::string m_strbuildvalues;
  int m_elementcount = 0;
  uint16_t m_actionType = 0;
  uint16_t m_obpollhandle = 0;
  uint32_t m_idlabelhandle = 0;
  std::vector<SaSpec> m_SaSpecVec;
  std::vector<SaCalibData16> m_SaCalibDataSpecVec;
  bool m_transmissionstart = true;

  std::vector<std::string> m_NumValHeaders;
  std::vector<NumericValResult> m_NumericValVec;
  std::vector<std::string> m_WaveValHeaders;
  std::vector<WaveValResult> m_WaveValVec;

  std::vector<uint8_t> m_rxbuf;
  std::vector<std::vector<uint8_t>> m_frameVec;
  std::vector<uint8_t> m_bVec;
  bool m_end = false;
  bool m_start = false;
  bool m_bitshiftnext = false;
};

#endif
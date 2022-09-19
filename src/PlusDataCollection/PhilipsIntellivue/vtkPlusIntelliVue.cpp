/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "vtkPlusIntelliVue.h"

#include "DataConstants.h"
#include "PlusConfigure.h"
#include "PlusSerialLineLinux.h"

// Local includes
#include "vtkPlusChannel.h"
#include "vtkPlusDataSource.h"

// Linux headers
#include <fcntl.h>  // File control definitions
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>   // UNIX standard function definitions

#include <cerrno>  // Error number definitions
#include <chrono>
#include <cmath>
#include <cstdio>  // standard input / output functions
#include <cstdlib>
#include <ctime>

// VTK includes
#include <vtkImageData.h>
#include <vtkObjectFactory.h>
#include <vtkXMLDataElement.h>
#include <vtkXMLUtilities.h>
// Stl includes
#include <algorithm>
#include <memory>
#include <regex>
#include <string>
#include <vector>
//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPlusIntelliVue);  // NOLINT
using namespace std;
using namespace std::chrono;
// Utils ---------------------------------------------------------------------
namespace
{
  // Little endian --> reverse vector
  vector<uint8_t> u32to8(uint32_t p_data)  // NOLINT
  {
    vector<uint8_t> rawData(
      reinterpret_cast<uint8_t*>(&p_data),
      reinterpret_cast<uint8_t*>(&(p_data)) + sizeof(uint32_t));
    reverse(rawData.begin(), rawData.end());  // Correcting endian
    return rawData;
  }

  vector<uint8_t> u16to8(uint16_t p_data, bool p_reverseVec = true)  // NOLINT
  {
    vector<uint8_t> rawData(
      reinterpret_cast<uint8_t*>(&p_data),
      reinterpret_cast<uint8_t*>(&(p_data)) + sizeof(uint16_t));
    if (p_reverseVec)
    {
      reverse(rawData.begin(), rawData.end());
    }
    return rawData;
  }

  uint16_t u8to16(vector<uint8_t> p_data, bool p_reverseVec = true)  // NOLINT
  {
    if (p_reverseVec)
    {
      reverse(p_data.begin(), p_data.end());
    }
    return ((p_data[1]) << 8) | p_data[0];  // NOLINT
  }

  uint32_t u8to32(vector<uint8_t> p_data, bool p_reverseVec = true)  // NOLINT
  {
    int sz = static_cast<int>(p_data.size());
    if (p_reverseVec)
    {
      reverse(p_data.begin(), p_data.end());
    }
    if (sz < 4)
    {
      p_data.resize(4, 0);
    }

    return ((p_data[3]) << 24)  // NOLINT
           | ((p_data[2]) << 16)  // NOLINT
           | ((p_data[1]) << 8)   // NOLINT
           | p_data[0];
  }

  double floattypeToValue(uint32_t fValue)  // NOLINT
  {
    double value = 0;
    if (fValue != DataConstants::FLOATTYPE_NAN)
    {
      auto exponentbits = static_cast<int>(fValue >> 24);  // NOLINT

      auto mantissabits = static_cast<int>(fValue << 8);  // NOLINT
      if (mantissabits == 0x7fffff00)
      {
        LOG_INFO("NaN");
      }
      else if ((unsigned int)mantissabits == 0x80000000)      // NOLINT
      {
        LOG_INFO("NRes");
      }
      else if (mantissabits == 0x7ffffe00 ||
               (unsigned int)mantissabits == 0x80000200)  // NOLINT
      {
        LOG_INFO("+/- INFINITY");
      }
      mantissabits = (mantissabits >> 8);  // NOLINT

      auto signedexponentbits = static_cast<signed char>(
                                  exponentbits);  // Get Two's complement signed byte
      auto exponent = static_cast<int>(signedexponentbits);
      auto mantissa = mantissabits;
      value = mantissa * pow(static_cast<double>(10), exponent);

      return value;
    }
    return static_cast<double>(fValue);
  }

  uint16_t get16bitLSBfromUInt(uint32_t sourcevalue)  // NOLINT
  {
    uint32_t lsb = (sourcevalue & 0xFFFF);  // NOLINT

    return static_cast<uint16_t>(lsb);
  }

  string strFromPacket(const vector<uint8_t>& p_packet)
  {
    stringstream ss;
    for (uint8_t b : p_packet)
    {
      ss << b;
    }
    return ss.str();
  }

  string u32toString(uint32_t p_time)
  {
    stringstream ss;
    ss << p_time;
    string str;
    ss >> str;
    return str;
  }

  uint32_t stringTo32(const string& p_strTime)
  {
    stringstream ss(p_strTime);
    uint32_t time = 0;
    ss >> time;
    return time;
  }
  /*
  string millisecondsSinceEpoch(){
    auto ms = duration_cast< milliseconds >(
    system_clock::now().time_since_epoch()).count();
    return to_string(ms);
  }*/

  uint16_t getFCS(const std::vector<uint8_t>& p_buffer)  // NOLINT
  {
    uint16_t fcs = 0;
    const uint16_t kInitialFCS = 0xFFFF;
    fcs = kInitialFCS;
    for (unsigned char b : p_buffer)
    {
      fcs = (fcs >> 8) ^ DataConstants::FCSTable[(fcs ^ b) & 0xFF];  // NOLINT
    }
    return fcs;
  }

  vector<uint8_t> onesComplement(uint16_t p_numberToComplement)
  {
    vector<uint8_t> complementVal = u16to8(p_numberToComplement);

    for (int i = 0; i <= 1; i++)
    {
      complementVal[i] = (complementVal[i] ^ 0xff);  // NOLINT
    }
    complementVal.resize(2);
    reverse(complementVal.begin(), complementVal.end());
    return complementVal;
  }

  double getTimestamp()
  {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  }

  void createWaveFormSet(const std::string& p_signsList,
                         vector<uint8_t>& p_waveTrType)
  {
    vector<string> list;
    stringstream ss(p_signsList);
    while (ss.good())
    {
      string substr;
      getline(ss, substr, ',');
      list.push_back(substr);
    }
    auto u16count = static_cast<uint16_t>(list.size());
    auto u16length = static_cast<uint16_t>(list.size() * 4);
    auto count = u16to8(u16count);
    auto length = u16to8(u16length);
    p_waveTrType.insert(p_waveTrType.end(), count.begin(),
                        count.end());  // count
    p_waveTrType.insert(p_waveTrType.end(), length.begin(),
                        length.end());  // length (usually count*4)
    for (const auto& signString : list)
    {
      auto signValue =
        u32to8(DataConstants::LabelResolver.find(signString)->second);
      p_waveTrType.insert(p_waveTrType.end(), signValue.begin(),
                          signValue.end());
    }
  }

  int decodePollObjects(PollInfoList& p_pollobjects,
                        vector<uint8_t>& p_packetbuffer)
  {
    vector<uint8_t> vecCount{p_packetbuffer.begin(),
                             p_packetbuffer.begin() + 2};
    p_pollobjects.count = u8to16(vecCount);
    if (p_pollobjects.count > 0)
    {
      vector<uint8_t> vecLength{p_packetbuffer.begin() + 2,
                                p_packetbuffer.begin() + 4};
      p_pollobjects.length = u8to16(vecLength);
    }

    int scpollobjectscount = u8to32(vecCount);
    if (p_pollobjects.length > 0)
    {
      vector<uint8_t> vecArray
      {
        p_packetbuffer.begin() + 4,
        p_packetbuffer.begin() + 4 + p_pollobjects.length};
      p_pollobjects.scpollarray = vecArray;
    }
    p_packetbuffer.erase(p_packetbuffer.begin(),
                         p_packetbuffer.begin() + 4 + p_pollobjects.length);
    return scpollobjectscount;
  }

  int decodeSingleContextPollObjects(SingleContextPoll& p_scpoll,
                                     vector<uint8_t>& p_packetbuffer)
  {
    vector<uint8_t> vecContextID{p_packetbuffer.begin(),
                                 p_packetbuffer.begin() + 2};
    p_scpoll.contextID = u8to16(vecContextID);
    vector<uint8_t> vecCount{p_packetbuffer.begin() + 2,
                             p_packetbuffer.begin() + 4};
    p_scpoll.count = u8to16(vecCount);
    // There can be empty singlecontextpollobjects
    vector<uint8_t> vecLength{p_packetbuffer.begin() + 4,
                              p_packetbuffer.begin() + 6};
    p_scpoll.length = u8to16(vecLength);

    int obpollobjectscount = u8to32(vecCount);
    if (p_scpoll.length > 0)
    {
      vector<uint8_t> vecArray{p_packetbuffer.begin() + 6,
                               p_packetbuffer.begin() + 6 + p_scpoll.length};
      p_scpoll.obpollobjectsarray = vecArray;
    }
    p_packetbuffer.erase(p_packetbuffer.begin(),
                         p_packetbuffer.begin() + 6 + p_scpoll.length);
    return obpollobjectscount;
  }

  int createMask(int p_significantbits)
  {
    int mask = 0;

    for (int i = 0; i < p_significantbits; i++)
    {
      mask |= (1 << i);  // NOLINT
    }
    return mask;
  }

  /*
  double CalibrateSaValue(double Waveval, SaCalibData16 sacalibdata)
  {
    if (!isnan(Waveval))
    {
    if (sacalibdata.upperAbsoluteValue != -99999)
    {
      double prop = 0;
      double value = 0;
      double Wavevalue = Waveval;

      //Check if value is out of range
      if (Waveval > sacalibdata.upperScaledValue) Waveval =
  sacalibdata.upperScaledValue; if (Waveval < sacalibdata.lowerScaledValue)
  Waveval = sacalibdata.lowerScaledValue;

      //Get proportion from scaled values
      if (sacalibdata.upperScaledValue != sacalibdata.lowerScaledValue)
      {
      prop = (Waveval - sacalibdata.lowerScaledValue) /
  (sacalibdata.upperScaledValue - sacalibdata.lowerScaledValue);
      }
      if (sacalibdata.upperAbsoluteValue != sacalibdata.lowerAbsoluteValue)
      {
      value = sacalibdata.lowerAbsoluteValue + (prop *
  (sacalibdata.upperAbsoluteValue - sacalibdata.lowerAbsoluteValue)); value =
  ceil(value * 100.0) / 100.0;

      }
      Wavevalue = value;
      return Wavevalue;
    }
    }
    return Waveval;
  }
  */
}  // namespace
// PlusLib -------------------------------------------------------------------
vtkPlusIntelliVue::vtkPlusIntelliVue() {}  // NOLINT

vtkPlusIntelliVue::~vtkPlusIntelliVue()
{
  this->Serial->Close();
}

void vtkPlusIntelliVue::PrintSelf(ostream& os, vtkIndent indent)  // NOLINT
{
  os << indent << "IntelliVue Configuration" << endl;
  os << indent << indent << "Port:" << this->Serial->GetPortName() << endl;
  os << indent << indent << "Signs to retrieve: " << this->Signs.c_str() << endl;
}

PlusStatus vtkPlusIntelliVue::InternalConnect()
{
  this->FrameNumber = 0;
  if (this->Superclass::InternalConnect() != PLUS_SUCCESS)
  {
    LOG_ERROR("Unable to connect to the device");
    return PLUS_FAIL;
  }
  if (!this->Serial->IsHandleAlive())
  {
    LOG_ERROR("Error opening port");
    return PLUS_FAIL;
  }

  sendWaveAssociationRequest();
  getRTSAPriorityListRequest();
  setRTSAPriorityList(this->Signs);

  if (GetFieldDataSource("NumData", this->NumSource) == PLUS_FAIL)
  {
    LOG_ERROR("CAN'T CONNECT TO NUM SOURCE");
    return PLUS_FAIL;
  }

  if (GetFieldDataSource("WaveData", this->WaveSource) == PLUS_FAIL)
  {
    LOG_ERROR("CAN'T CONNECT TO WAVE SOURCE");
    return PLUS_FAIL;
  }
  return PLUS_SUCCESS;
}

PlusStatus vtkPlusIntelliVue::InternalDisconnect()
{
  if (this->Superclass::InternalConnect() != PLUS_SUCCESS)
  {
    LOG_ERROR("Unable to disconnect");
    return PLUS_FAIL;
  }
  return PLUS_SUCCESS;
}

PlusStatus vtkPlusIntelliVue::InternalUpdate()
{
  // Either update or send commands - but not simultaneously
  igsioLockGuard<vtkIGSIORecursiveCriticalSection> updateMutexGuardedLock(
    this->Mutex);
  sendCycledExtendedPollDataRequest();
  sendCycledExtendedPollWaveDataRequest();
  do
  {
    clearReadBuffer();
    auto lenread = this->Serial->Read(
                     reinterpret_cast<unsigned char*>(&m_rxbuf[0]), 4096);
    vector<uint8_t> copyarray;

    for (int i = 0; i < lenread; i++)
    {
      copyarray.push_back(m_rxbuf[i]);
      createFrameListFromByte(copyarray[i]);
    }/*
    ofstream trace;
    trace.open("MPRawoutput2.txt", ofstream::app);
    trace << "R: ";
    for(auto b : copyarray){
        trace << hex << setfill('0') << setw(2) << (int)b << " ";
    }
    trace << endl;
    trace.close();
    */
  }
  while (this->Serial->GetNumberOfBytesAvailableForReading() > 0);
  if (!m_frameVec.empty())
  {
    readPacketFromFrame();
    m_frameVec.erase(m_frameVec.begin(),
                     m_frameVec.begin() + m_frameVec.size());
  }
  return PLUS_SUCCESS;
}

PlusStatus vtkPlusIntelliVue::ReadConfiguration(
  vtkXMLDataElement* config)  // NOLINT
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, config);
  XML_READ_CSTRING_ATTRIBUTE_REQUIRED(SerialPort, deviceConfig);
  XML_READ_CSTRING_ATTRIBUTE_REQUIRED(Signs, deviceConfig);

  XML_READ_SCALAR_ATTRIBUTE_OPTIONAL(unsigned long, BaudRate, deviceConfig);
  return PLUS_SUCCESS;
}

PlusStatus vtkPlusIntelliVue::WriteConfiguration(
  vtkXMLDataElement* config)  // NOLINT
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, config);
  deviceConfig->SetAttribute("SerialPort", this->SerialPort.c_str());
  deviceConfig->SetAttribute("Signs", this->Signs.c_str());
  return PLUS_SUCCESS;
}

//-----------------------------------------------------------------------------
// Check "Transport Protocols for the MIB/RS232 Interface" section of the Data
// Export Guide for more information.
PlusStatus vtkPlusIntelliVue::writeBuffer(const vector<uint8_t>& p_txbuf)
{
  uint8_t bofframebyte[] =
  {
    DataConstants::ESCAPECHAR,
    (DataConstants::BOFCHAR ^ DataConstants::BIT5COMPL)
  };
  uint8_t eofframebyte[] =
  {
    DataConstants::ESCAPECHAR,
    (DataConstants::EOFCHAR ^ DataConstants::BIT5COMPL)
  };
  uint8_t ctrlbyte[] =
  {
    DataConstants::ESCAPECHAR,
    (DataConstants::ESCAPECHAR ^ DataConstants::BIT5COMPL)
  };

  vector<uint8_t> temptxbufflist;
  for (unsigned char b : p_txbuf)
  {
    switch (b)
    {
      case DataConstants::BOFCHAR:
        temptxbufflist.push_back(bofframebyte[0]);
        temptxbufflist.push_back(bofframebyte[1]);
        break;
      case DataConstants::EOFCHAR:
        temptxbufflist.push_back(eofframebyte[0]);
        temptxbufflist.push_back(eofframebyte[1]);
        break;
      case DataConstants::ESCAPECHAR:
        temptxbufflist.push_back(ctrlbyte[0]);
        temptxbufflist.push_back(ctrlbyte[1]);
        break;
      default:
        temptxbufflist.push_back(b);
        break;
    };
  }

  vector<uint8_t> serialframeheader = {0x11, 0x01, 0x00, 0x00};
  auto serialuserdatalength = static_cast<uint16_t>(p_txbuf.size());
  vector<uint8_t> seriallength = u16to8(serialuserdatalength);
  for (int i = 0; i < 2; i++)
  {
    serialframeheader[i + 2] = seriallength[i];
  }

  vector<uint8_t> seriallist = serialframeheader;

  seriallist.insert(seriallist.end(), temptxbufflist.begin(),
                    temptxbufflist.end());
  vector<uint8_t> inputbuff;
  inputbuff.reserve(4);
  for (int i = 0; i < 4; i++)
  {
    inputbuff.push_back(serialframeheader[i]);
  }
  for (unsigned char c : p_txbuf)
  {
    inputbuff.push_back(c);
  }
  int checksumcalc = getFCS(inputbuff);
  auto checksumbytes = onesComplement(checksumcalc);

  for (unsigned char& checksumbyte : checksumbytes)
  {
    seriallist.push_back(checksumbyte);
  }
  seriallist.push_back(DataConstants::EOFCHAR);
  seriallist.insert(seriallist.begin(), DataConstants::BOFCHAR);
  auto nbytes = static_cast<int>(seriallist.size() * sizeof(uint8_t));
  if (this->Serial->Write(
        reinterpret_cast<const unsigned char*>(&seriallist[0]), nbytes) ==
      nbytes)
  {
    return PLUS_SUCCESS;
  }
  LOG_ERROR("Write Buffer to Intellivue device failed.");
  return PLUS_FAIL;
}

void vtkPlusIntelliVue::clearReadBuffer()
{
  m_rxbuf.clear();
  m_rxbuf.resize(4096, 0);
}

// Create the frame list corresponding to the message sent by the device
void vtkPlusIntelliVue::createFrameListFromByte(uint8_t p_b)
{
  switch (p_b)
  {
    case DataConstants::BOFCHAR:
      m_start = true;
      m_end = false;
      break;
    case DataConstants::EOFCHAR:
      m_start = false;
      m_end = true;
      break;
    case DataConstants::ESCAPECHAR:
      m_bitshiftnext = true;
      break;
    default:
      if (m_bitshiftnext)
      {
        p_b ^= (DataConstants::BIT5COMPL);
        m_bitshiftnext = false;
        m_bVec.push_back(p_b);
      }
      else if (m_start && !m_end)
      {
        m_bVec.push_back(p_b);
      }
      break;
  }
  if (!m_start && m_end)
  {
    int framelen = static_cast<int>(m_bVec.size());
    if (framelen != 0)
    {
      vector<uint8_t> bArray{m_bVec.begin(), m_bVec.end()};
      // serial header is 4 bytes and checksum 2 bytes
      int serialheaderwithuserdatalen = (framelen - 2);
      int serialuserdataframelen = (framelen - 6);
      vector<uint8_t> dataArray;
      vector<uint8_t> userdataArray;
      dataArray.reserve(serialheaderwithuserdatalen);
      for (int i = 0; i < serialheaderwithuserdatalen; i++)
      {
        dataArray.push_back(bArray[i]);
      }
      userdataArray.reserve(serialuserdataframelen);
      for (int i = 0; i < serialuserdataframelen; i++)
      {
        userdataArray.push_back(bArray[i + 4]);
      }
      // Read checksum
      vector<uint8_t> checksumbytes;
      checksumbytes.reserve(2);
      for (int i = 0; i < 2; i++)
      {
        checksumbytes.push_back(bArray[i + framelen - 2]);
      }

      auto checksum = u8to16(checksumbytes, false);

      // Calculate checksum
      int checksumcalc = getFCS(dataArray);
      vector<uint8_t> checksumbytevalue = onesComplement(checksumcalc);

      auto checksumcomputed = u8to16(checksumbytevalue, false);

      if (checksumcomputed == checksum)
      {
        m_frameVec.push_back(userdataArray);
      }
      else
      {
        LOG_INFO("Checksum Error");
      }
      m_bVec.clear();
      m_end = false;
    }
  }
}

//Read the messages in the buffer sent by the device
void vtkPlusIntelliVue::readPacketFromFrame()
{
  if (!m_frameVec.empty())
  {
    for (const auto& fArray : m_frameVec)
    {
      processPacket(fArray);
    }
  }
}

//Process the message sent by the device
void vtkPlusIntelliVue::processPacket(vector<uint8_t> p_packetbuffer)
{
  vector<uint8_t> sessionheader{p_packetbuffer.begin(),
                                p_packetbuffer.begin() + 4};
  vector<uint8_t> roap{p_packetbuffer.begin() + 4,
                       p_packetbuffer.begin() + 6};
  auto rOapduType = u8to16(roap, true);
  switch (rOapduType)
  {
    case DataConstants::ROIV_APDU:
      // This is an MDS create event, answer with create response
      parseMDSCreateEventReport(p_packetbuffer);
      sendMDSCreateEventResult();
      break;

    case DataConstants::RORS_APDU:
      checkPollPacketActionType(p_packetbuffer);
      break;

    case DataConstants::RORLS_APDU:
      checkLinkedPollPacketActionType(p_packetbuffer);
      break;
    default:
      break;
  }
}

//
void vtkPlusIntelliVue::parseMDSCreateEventReport(
  vector<uint8_t> p_readmdsconnectbuffer)
{
  vector<uint8_t> header{p_readmdsconnectbuffer.begin(),
                         p_readmdsconnectbuffer.begin() + 34};
  vector<uint8_t> vecCount{p_readmdsconnectbuffer.begin() + 34,
                           p_readmdsconnectbuffer.begin() + 36};
  vector<uint8_t> vecLength{p_readmdsconnectbuffer.begin() + 36,
                            p_readmdsconnectbuffer.begin() + 38};
  auto attriblistLength = static_cast<int>(u8to16(vecLength));
  auto avaobjectscount = static_cast<int>(u8to32(vecCount));
  if (avaobjectscount > 0)
  {
    vector<uint8_t> attriblistobjects
    {
      p_readmdsconnectbuffer.begin() + 38,
      p_readmdsconnectbuffer.begin() + 38 + attriblistLength};
    for (int i = 0; i < avaobjectscount; i++)
    {
      Ava avaobjects = {};
      decodeMDSAttribObjects(avaobjects, attriblistobjects);
    }
  }
}

void vtkPlusIntelliVue::checkPollPacketActionType(
  vector<uint8_t> p_packetbuffer)
{
  vector<uint8_t> header{p_packetbuffer.begin(), p_packetbuffer.begin() + 20};
  vector<uint8_t> tmp{p_packetbuffer.begin() + 20,
                      p_packetbuffer.begin() + 22};
  uint16_t actionType = u8to16(tmp);
  m_actionType = actionType;

  switch (actionType)
  {
    case DataConstants::NOM_ACT_POLL_MDIB_DATA:
      pollPacketDecoder(p_packetbuffer, 44);
      break;
    case DataConstants::NOM_ACT_POLL_MDIB_DATA_EXT:
      pollPacketDecoder(p_packetbuffer, 46);
      break;
    default:
      break;
  }
}
void vtkPlusIntelliVue::checkLinkedPollPacketActionType(
  vector<uint8_t> p_packetbuffer)
{
  vector<uint8_t> header{p_packetbuffer.begin(), p_packetbuffer.begin() + 22};
  vector<uint8_t> tmp{p_packetbuffer.begin() + 22,
                      p_packetbuffer.begin() + 24};
  uint16_t actionType = u8to16(tmp);
  m_actionType = actionType;
  switch (actionType)
  {
    case DataConstants::NOM_ACT_POLL_MDIB_DATA:
      pollPacketDecoder(p_packetbuffer, 46);
      break;
    case DataConstants::NOM_ACT_POLL_MDIB_DATA_EXT:
      pollPacketDecoder(p_packetbuffer, 48);
      break;
    default:
      break;
  }
}
void vtkPlusIntelliVue::pollPacketDecoder(vector<uint8_t> p_packetbuffer,
    int p_headersize)
{
  int packetsize = p_packetbuffer.size();

  vector<uint8_t> header{p_packetbuffer.begin(),
                         p_packetbuffer.begin() + p_headersize};
  vector<uint8_t> packetdata;
  packetdata.reserve(packetsize - p_headersize);
  for (int i = 0; i < (packetsize - p_headersize); i++)
  {
    packetdata.push_back(p_packetbuffer[p_headersize + i]);
  }
  m_strTimestamp = getPacketTimestamp(header);

  // ParsePacketType
  PollInfoList pollobjects = {};
  int scpollobjectscount = decodePollObjects(pollobjects, packetdata);
  for (int i = 0; i < scpollobjectscount; i++)
  {
    SingleContextPoll scpoll = {};
    int obpollobjectscount =
      decodeSingleContextPollObjects(scpoll, pollobjects.scpollarray);
    for (int j = 0; j < obpollobjectscount; j++)
    {
      ObservationPoll obpollobject = {};
      int avaobjectscount = decodeObservationPollObjects(
                              obpollobject, scpoll.obpollobjectsarray);
      for (int k = 0; k < avaobjectscount; k++)
      {
        Ava avaobject = {};
        decodeAvaObjects(avaobject,
                         obpollobject.avaobjectsarray);
      }
    }

    sendWaveValues();
    sendNumValues();

    this->FrameNumber++;
    // clear memory
    m_WaveValVec.clear();
  }
}

void vtkPlusIntelliVue::sendNumValues()
{
  if (!m_NumericValVec.empty())
  {
    long long firstelementreltimestamp = stoll(m_NumericValVec.at(0).Timestamp);
    int listcount = static_cast<int>(m_NumericValVec.size());
    int elementCount = 0;
    for (int i = elementCount; i < listcount; i++)
    {
      long long elementreltime = stoll(m_NumericValVec.at(i).Timestamp);
      if (elementreltime == firstelementreltimestamp)
      {
        if (m_strbuildvalues.find(m_NumValHeaders.at(i)) == string::npos)
        {
          m_strbuildvalues.append(m_NumValHeaders.at(i));
          m_strbuildvalues.append(",");
          m_strbuildvalues.append(m_NumericValVec.at(i).Value);
          m_strbuildvalues.append(",");
        }
        elementCount++;
      }
      else
      {
        m_strbuildvalues.insert(0, ",");
        m_strbuildvalues.insert(0, m_NumericValVec.at(0).Timestamp);
        m_strbuildvalues.erase(m_strbuildvalues.size() - 1, 1);

        m_strbuildvalues =
          regex_replace(m_strbuildvalues, regex(",,"), ",");
        m_strbuildvalues.append("\n");
        if (this->NumSource != nullptr)
        {
          igsioFieldMapType fieldMap;
          fieldMap[this->NumSource->GetId()].first = FRAMEFIELD_NONE;
          fieldMap[this->NumSource->GetId()].second =
            m_strbuildvalues;
          if (this->NumSource->AddItem(
                fieldMap, this->FrameNumber,
                vtkIGSIOAccurateTimer::GetSystemTime(),
                vtkIGSIOAccurateTimer::GetSystemTime()) ==
              PLUS_FAIL)
          {
            LOG_ERROR("Unable to send Num Data. Skipping frame.");
          }

          m_NumericValVec.erase(m_NumericValVec.begin(),
                                m_NumericValVec.begin() + elementCount);
          m_NumValHeaders.erase(m_NumValHeaders.begin(),
                                m_NumValHeaders.begin() + elementCount);
          listcount = static_cast<int>(m_NumericValVec.size());
        }
        m_strbuildvalues.clear();
      }
    }
  }
}

void vtkPlusIntelliVue::sendWaveValues()
{
  int wavevallistcount = static_cast<int>(m_WaveValVec.size());
  string ts = "";
  int timediff = 2;
  std::string message;
  if (wavevallistcount != 0)
  {
    for (WaveValResult wavValResult : m_WaveValVec)
    {
      int wavvalarraylength = static_cast<int>(wavValResult.Value.size());
      int nbValues = wavvalarraylength / 2;
      if (nbValues == 16)
      {
        timediff = 16;
      }
      else if (nbValues == 32)
      {
        timediff = 8;
      }
      else if (nbValues == 64)
      {
        timediff = 4;
      }  // Compound ECG
      else if (nbValues == 128)
      {
        timediff = 2;
      }  // ECG*/
      ts = wavValResult.Timestamp;
      for (int index = 0; index < wavvalarraylength; index++)
      {
        // Data sample size is 16 bits, but the significant bits
        // represent actual sample value Read every 2 bytes
        uint8_t msb = wavValResult.Value.at(index);
        uint8_t lsb = wavValResult.Value.at(index + 1);

        int msbval = static_cast<int>(msb);
        int mask = createMask(wavValResult.saSpecData.significantBits);
        int msbshift = (msb << 8);  // NOLINT

        if (wavValResult.saSpecData.SaFlags < 0x4000)
        {
          msbval = (msbshift & mask);  // NOLINT
          msbval = (msbval >> 8);    // NOLINT
        }
        else
        {
          msbval = msb;
        }
        msb = static_cast<uint8_t>(msbval);
        vector<uint8_t> data = {msb, lsb};
        double value = floattypeToValue((msb << 8) + lsb);  // NOLINT
        // double Waveval = floattypeToValue(u8to32(data));
        /*
        double v = 9999.9999;
        if (WavValResult.saSpecData.SaFlags != 0x2000 &&
        m_calibratewavevalues == true)
        {
          v = CalibrateSaValue(Waveval, WavValResult.saCalibData);
        }*/
        message.append(to_string(stoll(wavValResult.Timestamp)));
        wavValResult.Timestamp =
          to_string(stoll(wavValResult.Timestamp) + timediff);
        message.append(",");
        message.append(wavValResult.PhysioID);
        message.append(",");
        message.append(to_string(value));
        message.append(",");
        message.append("\n");

        index = index + 1;
      }
    }
    m_WaveValVec.erase(m_WaveValVec.begin(),
                       m_WaveValVec.begin() + wavevallistcount);
  }
  if (this->WaveSource != nullptr)
  {
    igsioFieldMapType fieldMap;
    fieldMap[this->WaveSource->GetId()].first = FRAMEFIELD_NONE;
    fieldMap[this->WaveSource->GetId()].second = message;
    if (this->WaveSource->AddItem(
          fieldMap, this->FrameNumber,
          vtkIGSIOAccurateTimer::GetSystemTime(),
          vtkIGSIOAccurateTimer::GetSystemTime()) == PLUS_FAIL)
    {
      LOG_ERROR("Unable to send Wave Data. Skipping frame.");
    }
  }
}

int vtkPlusIntelliVue::decodeObservationPollObjects(
  ObservationPoll& p_obpollobject, vector<uint8_t>& p_packetbuffer)
{
  vector<uint8_t> vecObjHandle{p_packetbuffer.begin(),
                               p_packetbuffer.begin() + 2};
  p_obpollobject.objHandle = u8to16(vecObjHandle);

  m_obpollhandle = p_obpollobject.objHandle;

  AttributeList attributeliststruct = {};
  vector<uint8_t> vecCount{p_packetbuffer.begin() + 2,
                           p_packetbuffer.begin() + 4};
  attributeliststruct.count = u8to16(vecCount);
  if (attributeliststruct.count > 0)
  {
    vector<uint8_t> vecLength{p_packetbuffer.begin() + 4,
                              p_packetbuffer.begin() + 6};
    attributeliststruct.length = u8to16(vecLength);
  }

  int avaobjectscount = u8to32(vecCount);
  if (attributeliststruct.length > 0)
  {
    vector<uint8_t> vecArray
    {
      p_packetbuffer.begin() + 6,
      p_packetbuffer.begin() + 6 + attributeliststruct.length};
    p_obpollobject.avaobjectsarray = vecArray;
  }
  p_packetbuffer.erase(
    p_packetbuffer.begin(),
    p_packetbuffer.begin() + 6 + attributeliststruct.length);
  return avaobjectscount;
}
void vtkPlusIntelliVue::decodeAvaObjects(Ava& p_avaobject,
    vector<uint8_t>& p_avaobjectsarray)
{
  vector<uint8_t> attrID{p_avaobjectsarray.begin(),
                         p_avaobjectsarray.begin() + 2};
  p_avaobject.attributeID = u8to16(attrID);
  vector<uint8_t> len{p_avaobjectsarray.begin() + 2,
                      p_avaobjectsarray.begin() + 4};
  p_avaobject.length = u8to16(len);
  if (p_avaobject.length > 0)
  {
    vector<uint8_t> avaattribobjects
    {
      p_avaobjectsarray.begin() + 4,
      p_avaobjectsarray.begin() + 4 + p_avaobject.length};
    switch (p_avaobject.attributeID)
    {
      case DataConstants::NOM_ATTR_ID_HANDLE:
        // ReadIDHandle(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_ID_LABEL:
        readIDLabel(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_NU_VAL_OBS:
        readNumericObservationValue(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_NU_CMPD_VAL_OBS:
        readCompoundNumericObsValue(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_METRIC_SPECN:
        break;
      case DataConstants::NOM_ATTR_ID_LABEL_STRING:
        readIDLabelString(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_SA_VAL_OBS:
        readWaveSaObservationValue(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_SA_CMPD_VAL_OBS:
        readCompoundWaveSaObservationValue(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_SA_SPECN:
        readSaSpecifications(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_SCALE_SPECN_I16:
        // ReadSaScaleSpecifications(avaattribobjects);
        break;
      case DataConstants::NOM_ATTR_SA_CALIB_I16:
        readSaCalibrationSpecifications(avaattribobjects);
        break;
      default:
        // unknown attribute -> do nothing
        break;
    }
  }
  p_avaobjectsarray.erase(p_avaobjectsarray.begin(),
                          p_avaobjectsarray.begin() + 4 + p_avaobject.length);
}
void vtkPlusIntelliVue::readIDLabel(vector<uint8_t> p_avaattribobjects)
{
  vector<uint8_t> vecIDlabel{p_avaattribobjects.begin(),
                             p_avaattribobjects.begin() + 4};
  uint32_t iDlabel = u8to32(vecIDlabel);
  m_idlabelhandle = iDlabel;
}

void vtkPlusIntelliVue::readIDLabelString(vector<uint8_t> p_avaattribobjects)
{
  StringMP strmp = {};
  vector<uint8_t> vecLength{p_avaattribobjects.begin(),
                            p_avaattribobjects.begin() + 2};
  strmp.length = u8to16(vecLength);
  vector<uint8_t> stringval{p_avaattribobjects.begin() + 2,
                            p_avaattribobjects.begin() + 2 + strmp.length};

  string label = strFromPacket(stringval);
  // LOG_INFO("Label String: "+ label);
}

void vtkPlusIntelliVue::readNumericObservationValue(
  vector<uint8_t> p_avaattribobjects)
{
  NuObsValue numObjectValue = {};
  vector<uint8_t> vecPhysioID{p_avaattribobjects.begin(),
                              p_avaattribobjects.begin() + 2};
  vector<uint8_t> vecState{p_avaattribobjects.begin() + 2,
                           p_avaattribobjects.begin() + 4};
  vector<uint8_t> vecUnitCode{p_avaattribobjects.begin() + 4,
                              p_avaattribobjects.begin() + 6};
  vector<uint8_t> vecValue{p_avaattribobjects.begin() + 6,
                           p_avaattribobjects.begin() + 10};
  numObjectValue.physioID = u8to16(vecPhysioID);
  numObjectValue.state = u8to16(vecState);
  if (numObjectValue.state == 0x8000)
  {
    //INVALID STATE, No reason to interpret value
    return;
  }
  numObjectValue.unitCode = u8to16(vecUnitCode);
  numObjectValue.value = u8to32(vecValue);

  double value = floattypeToValue(numObjectValue.value);

  string physioID = DataConstants::SourceResolver.find(numObjectValue.physioID)->second;
  string state = to_string(numObjectValue.state);

  string valuestr;
  if (value != DataConstants::FLOATTYPE_NAN)
  {
    valuestr = to_string(value);
  }
  else
  {
    valuestr = "-";
  }

  NumericValResult numVal = {};
  uint currentRelativeTime = stringTo32(m_strTimestamp);

  auto dtDateTime =
    getAbsoluteTimeFromRelativeTimestamp(currentRelativeTime);
  numVal.Timestamp = to_string(dtDateTime);
  numVal.PhysioID = physioID;
  numVal.Value = valuestr;

  m_NumericValVec.push_back(numVal);
  m_NumValHeaders.push_back(numVal.PhysioID + " (" + DataConstants::UnitResolver.find(numObjectValue.unitCode)->second + ")");
}

void vtkPlusIntelliVue::readCompoundNumericObsValue(
  vector<uint8_t> p_avaattribobjects)
{
  vector<uint8_t> countVec{p_avaattribobjects.begin(),
                           p_avaattribobjects.begin() + 2};

  int cmpnumericobjectscount = static_cast<int>(u8to32(countVec));

  if (cmpnumericobjectscount > 0)
  {
    for (int j = 0; j < cmpnumericobjectscount; j++)
    {
      vector<uint8_t> cmpnumericarrayobject
      {
        p_avaattribobjects.begin() + 4,
        p_avaattribobjects.begin() + 14};
      readNumericObservationValue(cmpnumericarrayobject);
    }
  }
}

void vtkPlusIntelliVue::readWaveSaObservationValue(
  vector<uint8_t>& p_avaattribobjects)
{
  SaObsValue waveSaObjectValue = {};
  vector<uint8_t> vecPhysioID{p_avaattribobjects.begin(),
                              p_avaattribobjects.begin() + 2};
  vector<uint8_t> vecState{p_avaattribobjects.begin() + 2,
                           p_avaattribobjects.begin() + 4};
  vector<uint8_t> vecLength{p_avaattribobjects.begin() + 4,
                            p_avaattribobjects.begin() + 6};
  waveSaObjectValue.physioID = u8to16(vecPhysioID);
  waveSaObjectValue.state = u8to16(vecState);
  waveSaObjectValue.length = u8to16(vecLength);

  int wavevalobjectslength = static_cast<int>(u8to32(vecLength));
  vector<uint8_t> waveValObjects
  {
    p_avaattribobjects.begin() + 6,
    p_avaattribobjects.begin() + 6 + wavevalobjectslength};
  p_avaattribobjects.erase(
    p_avaattribobjects.begin(),
    p_avaattribobjects.begin() + 6 + wavevalobjectslength);
  string physioID = DataConstants::SourceResolver.find(waveSaObjectValue.physioID)->second;
  WaveValResult waveVal = {};

  uint currentRelativeTime = stringTo32(m_strTimestamp);
  auto dtDateTime =
    getAbsoluteTimeFromRelativeTimestamp(currentRelativeTime);
  waveVal.Timestamp = to_string(dtDateTime);
  waveVal.PhysioID = physioID;

  waveVal.obpollHandle = m_obpollhandle;
  uint16_t physioIDHandle = waveSaObjectValue.physioID;
  auto indexCalibData =
    find_if(m_SaCalibDataSpecVec.begin(), m_SaCalibDataSpecVec.end(),
            [&pH = physioIDHandle](const SaCalibData16 & p_s) -> bool
  {
    return pH == p_s.physioID;
  });
  if (indexCalibData != m_SaCalibDataSpecVec.end())
  {
    waveVal.saCalibData = *indexCalibData;
  }
  else
  {
    if (physioIDHandle == 0x107)
    {
      // use default values for ecg II
      waveVal.saCalibData.lowerAbsoluteValue = 0;
      waveVal.saCalibData.upperAbsoluteValue = 1;
      waveVal.saCalibData.lowerScaledValue = 0x1fe7;
      waveVal.saCalibData.upperScaledValue = 0x20af;
    }
    else if (physioIDHandle == 0x102)
    {
      // use default values for ecg V5
      waveVal.saCalibData.lowerAbsoluteValue = 0;
      waveVal.saCalibData.upperAbsoluteValue = 1;
      waveVal.saCalibData.lowerScaledValue = 0x1fd4;
      waveVal.saCalibData.upperScaledValue = 0x209c;
    }
    else if (physioIDHandle == 0x4A10)
    {
      // use default values for art ibp
      waveVal.saCalibData.lowerAbsoluteValue = -40;
      waveVal.saCalibData.upperAbsoluteValue = 520;
      waveVal.saCalibData.lowerScaledValue = 0x00a0;
      waveVal.saCalibData.upperScaledValue = 0x23a0;
    }
    else if (physioIDHandle == 0x5000)
    {
      // use default values for resp
      waveVal.saCalibData.lowerAbsoluteValue = -0.6;
      waveVal.saCalibData.upperAbsoluteValue = 1.9;
      waveVal.saCalibData.lowerScaledValue = 0x0000;
      waveVal.saCalibData.upperScaledValue = 0x0fff;
    }
    else
    {
      waveVal.saCalibData = {};
    }
  }
  waveVal.Value = waveValObjects;
  // Find the Sample array specification definition that matches the
  // observation sample array size
  auto indexSpecData =
    find_if(m_SaSpecVec.begin(), m_SaSpecVec.end(),
            [&oH = waveVal.obpollHandle](const SaSpec & p_s) -> bool
  {
    return oH == p_s.obpollHandle;
  });
  if (indexSpecData != m_SaSpecVec.end())
  {
    waveVal.saSpecData = *indexSpecData;
  }
  else
  {
    if (wavevalobjectslength % 128 == 0)
    {
      // use default values for ecg
      waveVal.saSpecData.significantBits = 0x0E;
      waveVal.saSpecData.SaFlags = 0x3000;
      waveVal.saSpecData.sampleSize = 0x10;
      waveVal.saSpecData.arraySize = 0x80;
    }
    else if (wavevalobjectslength % 64 == 0)
    {
      // use default values for art ibp
      waveVal.saSpecData.significantBits = 0x0E;
      waveVal.saSpecData.SaFlags = 0x3000;
      waveVal.saSpecData.sampleSize = 0x10;
      waveVal.saSpecData.arraySize = 0x40;

    }
    else if (wavevalobjectslength % 32 == 0)
    {
      // use default values for resp
      waveVal.saSpecData.significantBits = 0x0C;
      waveVal.saSpecData.SaFlags = 0x8000;
      waveVal.saSpecData.sampleSize = 0x10;
      waveVal.saSpecData.arraySize = 0x20;
    }
    else if (wavevalobjectslength % 16 == 0)
    {
      // use default values for pleth
      waveVal.saSpecData.significantBits = 0x0C;
      waveVal.saSpecData.SaFlags = 0x8000;
      waveVal.saSpecData.sampleSize = 0x10;
      waveVal.saSpecData.arraySize = 0x10;
    }
  }
  m_WaveValVec.push_back(waveVal);
}

void vtkPlusIntelliVue::readCompoundWaveSaObservationValue(
  vector<uint8_t> p_avaattribobjects)
{
  vector<uint8_t> vecCount{p_avaattribobjects.begin(),
                           p_avaattribobjects.begin() + 2};
  vector<uint8_t> vecLength{p_avaattribobjects.begin() + 2,
                            p_avaattribobjects.begin() + 4};
  int cmpwaveobjectscount = static_cast<int>(u8to32(vecCount));
  int cmpwaveobjectslength = static_cast<int>(u8to32(vecLength));

  vector<uint8_t> cmpwavearrayobject
  {
    p_avaattribobjects.begin() + 4,
    p_avaattribobjects.begin() + 4 + cmpwaveobjectslength};

  if (cmpwaveobjectscount > 0)
  {
    for (int k = 0; k < cmpwaveobjectscount; k++)
    {
      readWaveSaObservationValue(cmpwavearrayobject);
    }
  }
}

void vtkPlusIntelliVue::readSaCalibrationSpecifications(
  vector<uint8_t> p_avaattribobjects)
{
  SaCalibData16 saCalibData = {};
  vector<uint8_t> vecLowerAbsValue{p_avaattribobjects.begin(),
                                   p_avaattribobjects.begin() + 4};
  vector<uint8_t> vecUpperAbsValue{p_avaattribobjects.begin() + 4,
                                   p_avaattribobjects.begin() + 8};
  vector<uint8_t> vecLowerScaledValue{p_avaattribobjects.begin() + 8,
                                      p_avaattribobjects.begin() + 10};
  vector<uint8_t> vecUpperScaledValue{p_avaattribobjects.begin() + 10,
                                      p_avaattribobjects.begin() + 12};

  saCalibData.lowerAbsoluteValue = floattypeToValue(u8to32(vecLowerAbsValue));
  saCalibData.upperAbsoluteValue = floattypeToValue(u8to32(vecUpperAbsValue));
  saCalibData.lowerScaledValue = u8to16(vecLowerScaledValue);
  saCalibData.upperScaledValue = u8to16(vecUpperScaledValue);

  saCalibData.obpollHandle = m_obpollhandle;

  // Get 16 bit physiological id from 32 bit wave id label
  saCalibData.physioID = get16bitLSBfromUInt(m_idlabelhandle);
  // Add to a list of Sample array calibration specification definitions if
  // it's not already present
  auto salistindex = find_if(
                       m_SaCalibDataSpecVec.begin(), m_SaCalibDataSpecVec.end(),
                       [&pID = saCalibData.physioID](const SaCalibData16 & p_s) -> bool
  {
    return pID == p_s.physioID;
  });

  if (salistindex == m_SaCalibDataSpecVec.end())
  {
    m_SaCalibDataSpecVec.push_back(saCalibData);
  }
  else
  {
    m_SaCalibDataSpecVec.erase(salistindex);
    m_SaCalibDataSpecVec.push_back(saCalibData);
  }
}

void vtkPlusIntelliVue::readSaSpecifications(
  vector<uint8_t> p_avaattribobjects)
{
  SaSpec saspecobj = {};
  vector<uint8_t> vecArraySize{p_avaattribobjects.begin(),
                               p_avaattribobjects.begin() + 2};
  saspecobj.arraySize = u8to16(vecArraySize);
  saspecobj.sampleSize = p_avaattribobjects.at(2);
  saspecobj.significantBits = p_avaattribobjects.at(3);
  vector<uint8_t> vecSaFlags{p_avaattribobjects.begin() + 4,
                             p_avaattribobjects.begin() + 6};
  saspecobj.SaFlags = u8to16(vecSaFlags);

  saspecobj.obpollHandle = m_obpollhandle;

  // Add to a list of Sample array specification definitions if it's not
  // already present
  auto salistindex =
    find_if(m_SaSpecVec.begin(), m_SaSpecVec.end(),
            [&oH = saspecobj.obpollHandle](const SaSpec & p_s) -> bool
  {
    return oH == p_s.obpollHandle;
  });
  if (salistindex == m_SaSpecVec.end())
  {
    m_SaSpecVec.push_back(saspecobj);
  }
  else
  {
    m_SaSpecVec.erase(salistindex);
    m_SaSpecVec.push_back(saspecobj);
  }
}
double vtkPlusIntelliVue::getAbsoluteTimeFromRelativeTimestamp(
  uint32_t p_currentRelativeTime) const
{
  double elapsedTimeMilliseconds =
    abs((static_cast<double>(p_currentRelativeTime) -
         static_cast<double>(m_baseRelativeTime)) *
        125 / 1000);
  auto dtDateTime = m_baseDateTime + elapsedTimeMilliseconds;
  return dtDateTime;
}
string vtkPlusIntelliVue::getPacketTimestamp(vector<uint8_t> p_header)
{
  int pollmdibdatareplysize = 20;
  if (m_actionType == DataConstants::NOM_ACT_POLL_MDIB_DATA)
  {
    pollmdibdatareplysize = 20;
  }
  else if (m_actionType == DataConstants::NOM_ACT_POLL_MDIB_DATA_EXT)
  {
    pollmdibdatareplysize = 22;
  }
  unsigned long firstpartheaderlength = (p_header.size() - pollmdibdatareplysize);
  vector<uint8_t> pollmdibdatareplyarray
  {
    p_header.begin() + firstpartheaderlength,
    p_header.begin() + firstpartheaderlength + pollmdibdatareplysize};

  vector<uint8_t> vecRelTimeStamp
  {
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 18),
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 14)};
  uint32_t relTimeStamp = u8to32(vecRelTimeStamp);
  vector<uint8_t> absolutetimearray
  {
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 14),
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 6)};
  vector<uint8_t> code
  {
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 4),
    pollmdibdatareplyarray.begin() + (pollmdibdatareplysize - 2)};

  auto pollresultcode = u8to16(code);
  string strRelativeTime = u32toString(relTimeStamp);
  if (pollresultcode == DataConstants::NOM_MOC_VMS_MDS)
  {
    // Get baseline timestamps if packet type is MDS attributes
    m_baseRelativeTime = relTimeStamp;
    m_baseDateTime = getTimestamp();
  }
  return strRelativeTime;
}

void vtkPlusIntelliVue::decodeMDSAttribObjects(
  Ava& p_avaobject, vector<uint8_t>& p_packetbuffer)
{
  p_avaobject.attributeID =
    u8to16({p_packetbuffer.begin(), p_packetbuffer.begin() + 2});
  p_avaobject.length =
    u8to16({p_packetbuffer.begin() + 2, p_packetbuffer.begin() + 4});
  if (p_avaobject.length > 0)
  {
    vector<uint8_t> avaattribobjects
    {
      p_packetbuffer.begin() + 4,
      p_packetbuffer.begin() + 4 + p_avaobject.length};
    switch (p_avaobject.attributeID)
    {
      // Get Date and Time
      case DataConstants::NOM_ATTR_TIME_ABS:
        m_baseDateTime = getTimestamp();// getAbsoluteTimeFromBCDFormat(avaattribobjects);
        break;
      // Get Relative Time attribute
      case DataConstants::NOM_ATTR_TIME_REL:
        getBaselineRelativeTimestamp(avaattribobjects);
        break;
      // Get Patient demographics (to do)
      default:
        break;
    }
  }
  p_packetbuffer.erase(p_packetbuffer.begin(),
                       p_packetbuffer.begin() + 4 + p_avaobject.length);
}

void vtkPlusIntelliVue::getBaselineRelativeTimestamp(
  vector<uint8_t> p_timebuffer)
{
  vector<uint8_t> time{p_timebuffer.begin(), p_timebuffer.begin() + 4};
  m_baseRelativeTime = u8to32(time);
}

// IntelliVue Messages -------------------------------------------------------
void vtkPlusIntelliVue::setRTSAPriorityList(const std::string& p_signsList)
{
  vector<uint8_t> waveTrType;
  createWaveFormSet(p_signsList, waveTrType);
  sendRTSAPriorityMessage(waveTrType);
}

PlusStatus vtkPlusIntelliVue::sendRTSAPriorityMessage(
  const vector<uint8_t>& p_waveTrType)
{
  auto tempbufflist = p_waveTrType;
  // AvaType
  auto avaAttribID =
    u16to8(DataConstants::AttributeIDs::NOM_ATTR_POLL_RTSA_PRIO_LIST);
  auto avaLength = u16to8(p_waveTrType.size());
  tempbufflist.insert(tempbufflist.begin(), avaLength.begin(),
                      avaLength.end());
  tempbufflist.insert(tempbufflist.begin(), avaAttribID.begin(),
                      avaAttribID.end());

  // AttributeModEntry
  vector<uint8_t> attributeModEntry = {0x00, 0x00};
  tempbufflist.insert(tempbufflist.begin(), attributeModEntry.begin(),
                      attributeModEntry.end());

  // ModificationList
  uint16_t modListSize = tempbufflist.size();
  auto vecSize = u16to8(modListSize);
  vector<uint8_t> modListCount = {0x00, 0x01};
  tempbufflist.insert(tempbufflist.begin(), vecSize.begin(), vecSize.end());
  tempbufflist.insert(tempbufflist.begin(), modListCount.begin(),
                      modListCount.end());

  // ManageObjectId
  vector<uint8_t> managedObjectID = {0x00, 0x21, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00
                                    };
  tempbufflist.insert(tempbufflist.begin(), managedObjectID.begin(),
                      managedObjectID.end());

  // ROIVApdu
  auto roivLength = u16to8(tempbufflist.size());
  auto roivCommandType = u16to8(DataConstants::Commands::CMD_CONFIRMED_SET);
  auto roivInvokeId = u16to8(0x0000);
  tempbufflist.insert(tempbufflist.begin(), roivLength.begin(),
                      roivLength.end());
  tempbufflist.insert(tempbufflist.begin(), roivCommandType.begin(),
                      roivCommandType.end());
  tempbufflist.insert(tempbufflist.begin(), roivInvokeId.begin(),
                      roivInvokeId.end());

  // ROapdus
  auto roapLength = u16to8(tempbufflist.size());
  auto roapRoType = u16to8(DataConstants::ROIV_APDU);
  tempbufflist.insert(tempbufflist.begin(), roapLength.begin(),
                      roapLength.end());
  tempbufflist.insert(tempbufflist.begin(), roapRoType.begin(),
                      roapRoType.end());

  // SPpdu
  vector<uint8_t> sPpdu = {0xE1, 0x00, 0x00, 0x02};
  tempbufflist.insert(tempbufflist.begin(), sPpdu.begin(), sPpdu.end());
  const vector<uint8_t> kFinaltxbuff = tempbufflist;
  return writeBuffer(kFinaltxbuff);
}
PlusStatus vtkPlusIntelliVue::getRTSAPriorityListRequest()
{
  return writeBuffer(DataConstants::get_rtsa_prio_msg);
}
PlusStatus vtkPlusIntelliVue::sendWaveAssociationRequest()
{
  return writeBuffer(DataConstants::aarq_msg_wave_ext_poll);
}
PlusStatus vtkPlusIntelliVue::sendCycledExtendedPollDataRequest()
{
  return writeBuffer(DataConstants::ext_poll_request_msg);
}
PlusStatus vtkPlusIntelliVue::sendCycledExtendedPollWaveDataRequest()
{
  return writeBuffer(DataConstants::ext_poll_request_wave_msg);
}
PlusStatus vtkPlusIntelliVue::sendMDSCreateEventResult()
{
  return writeBuffer(DataConstants::mds_create_resp_msg);
}
PlusStatus vtkPlusIntelliVue::sendMDSPollDataRequest()
{
  return writeBuffer(DataConstants::poll_mds_request_msg);
}

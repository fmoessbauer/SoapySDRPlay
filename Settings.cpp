/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapySDRPlay.hpp"

extern bool deviceSelected;    // global declared in Registration.cpp

#define MAX_RSP_DEVICES  (6)

static sdrplay_api_DeviceT rspDevs[MAX_RSP_DEVICES];

SoapySDRPlay::SoapySDRPlay(const SoapySDR::Kwargs &args)
{
  using RxChannelT = sdrplay_api_RxChannelParamsT;
  std::vector<RxChannelT*> channels;

    std::string label = args.at("label");
    std::string baseLabel = "SDRplay Dev";

    size_t posidx = label.find(baseLabel);

    if (posidx == std::string::npos)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Can't find Dev string in args");
        return;
    }
    sdrplay_api_Open();

	//retreive device index
    unsigned int devIdx = label.at(posidx + baseLabel.length()) - 0x30;

	// retreive hwVer and serNo by API
	unsigned int nDevs = 0;

	sdrplay_api_GetDevices(&rspDevs[0], &nDevs, MAX_RSP_DEVICES);

	if (devIdx < nDevs) {

		hwVer = rspDevs[devIdx].hwVer;
		serNo = rspDevs[devIdx].SerNo;

		size_t poscom = serNo.find(",");

		if (poscom != std::string::npos)
		{
			serNo = serNo.substr(0, poscom);
		}
	}
	else {
		SoapySDR_logf(SOAPY_SDR_WARNING, "Can't determine hwVer/serNo");
		return;
	}

    sdrplay_api_ApiVersion(&ver);
    if (ver != SDRPLAY_API_VERSION)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "sdrplay_api version: '%.3f' does not equal build version: '%.3f'", ver, SDRPLAY_API_VERSION);
    }

    sdrplay_api_LockDeviceApi();
    sdrplay_api_SelectDevice(&rspDevs[devIdx]);
    dev = rspDevs[devIdx].dev;
    sdrplay_api_UnlockDeviceApi();
    deviceSelected = true;

    // Set Dual-Tuner mode
    rspDevs[devIdx].rspDuoMode = sdrplay_api_RspDuoMode_Dual_Tuner;

    auto err = sdrplay_api_GetDeviceParams(dev, &deviceParams);
    if(err != sdrplay_api_Success)
      SoapySDR_logf(SOAPY_SDR_WARNING, "failed to get device parameters");

    deviceParams->devParams->fsFreq.fsHz = 2000000;
    reqSampleRate                        = sampleRate;
    deviceParams->devParams->ppm         = 0.0;

    // per channel settings
    channels.push_back(deviceParams->rxChannelA);
    if(hwVer == SDRPLAY_RSPduo_ID)
      channels.push_back(deviceParams->rxChannelB);

    for(const auto & chan : channels){
      chan->ctrlParams.decimation.enable = false;
      chan->ctrlParams.agc.enable        = sdrplay_api_AGC_100HZ;
      chan->ctrlParams.agc.setPoint_dBfs = -30;
      chan->ctrlParams.dcOffset.DCenable = 1;
      chan->ctrlParams.dcOffset.IQenable = 1;

      chan->tunerParams.rfFreq.rfHz = 100;
      chan->tunerParams.ifType = sdrplay_api_IF_Zero;
      chan->tunerParams.bwType = sdrplay_api_BW_1_536;
      chan->tunerParams.gain.gRdB = 40;
      chan->tunerParams.gain.LNAstate = (hwVer == SDRPLAY_RSP2_ID || hwVer == SDRPLAY_RSPduo_ID || hwVer > 253)? 4: 1;

      if(hwVer == SDRPLAY_RSPduo_ID){
        chan->rspDuoTunerParams.biasTEnable      = false;
        chan->rspDuoTunerParams.rfNotchEnable    = false;
        chan->rspDuoTunerParams.rfDabNotchEnable = false;
      }
    }

    if(hwVer == SDRPLAY_RSPduo_ID){
      deviceParams->rxChannelA->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
      deviceParams->devParams->rspDuoParams.extRefOutputEn = 0;
    }
    // TODO: other receivers

    //bufferedElems = 0;
    //_currentBuff = 0;
    resetBuffer = false;
    useShort = true;
    
    streamActive = false;
}

SoapySDRPlay::~SoapySDRPlay(void)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    if (streamActive)
    {
        sdrplay_api_Uninit(dev);
    }
    streamActive = false;
    sdrplay_api_Close();
    deviceSelected = false;
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapySDRPlay::getDriverKey(void) const
{
    return "SDRplay";
}

std::string SoapySDRPlay::getHardwareKey(void) const
{
    return serNo;
}

SoapySDR::Kwargs SoapySDRPlay::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs hwArgs;

    hwArgs["sdrplay_api_api_version"] = std::to_string(ver);
    hwArgs["sdrplay_api_hw_version"] = std::to_string(hwVer);

    return hwArgs;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapySDRPlay::getNumChannels(const int dir) const
{
  if(dir == SOAPY_SDR_TX)
    return 0;
  if(hwVer == SDRPLAY_RSPduo_ID)
    return 2;
  else
    return 1;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapySDRPlay::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;

    if (direction == SOAPY_SDR_TX) {
        return antennas;
    }

    if (hwVer == 1 || hwVer > 253) {
        antennas.push_back("RX");
    }
    else if (hwVer == SDRPLAY_RSP2_ID) {
        antennas.push_back("Antenna A");
        antennas.push_back("Antenna B");
        antennas.push_back("Hi-Z");
    }
    else if (hwVer == SDRPLAY_RSPduo_ID) {
      if(channel == 0){
        antennas.push_back("Tuner 1 50 ohm");
        antennas.push_back("Tuner 1 Hi-Z");
      } else {
        antennas.push_back("Tuner 2 50 ohm");
      }
    }
    return antennas;
}

void SoapySDRPlay::setAntenna(const int direction, const size_t channel, const std::string &name)
{
  std::lock_guard <std::mutex> lock(_general_state_mutex);
  auto tuner = sdrplay_api_Tuner_A;

  if ((direction != SOAPY_SDR_RX) || (hwVer == SDRPLAY_RSP1_ID) || (hwVer > 253)) {
    return;
  } else if(hwVer == SDRPLAY_RSP2_ID){
    if(name == "Antenna A"){
      deviceParams->rxChannelA->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
      deviceParams->rxChannelA->rsp2TunerParams.amPortSel  = sdrplay_api_Rsp2_AMPORT_1;
    } else if(name == "Antenna B"){
      deviceParams->rxChannelA->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
      deviceParams->rxChannelA->rsp2TunerParams.amPortSel  = sdrplay_api_Rsp2_AMPORT_1;
    } else if(name == "Hi-Z"){
      deviceParams->rxChannelA->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
      deviceParams->rxChannelA->rsp2TunerParams.amPortSel  = sdrplay_api_Rsp2_AMPORT_2;
    }
  } else if(hwVer == SDRPLAY_RSPduo_ID){
    if(channel == 0){
      if(name == "Tuner 1 Hi-Z"){
        deviceParams->rxChannelA->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
      }
      else if(name == "Tuner 1 50 ohm"){
        deviceParams->rxChannelA->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
      }
    } else if(channel == 1){
        tuner = sdrplay_api_Tuner_B;
        if(name == "Tuner 2 50 ohm"){
          // just one antenna on this tuner
        }
      }
  }
  sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Rsp2_AntennaControl);
}

std::string SoapySDRPlay::getAntenna(const int direction, const size_t channel) const
{
  if(direction != SOAPY_SDR_RX) return "";

  if(hwVer == SDRPLAY_RSP1_ID)
    return "RX";

  if(hwVer == SDRPLAY_RSP2_ID){
    if(deviceParams->rxChannelA->rsp2TunerParams.amPortSel == sdrplay_api_Rsp2_AMPORT_2)
      return "Hi-Z";
    else if(deviceParams->rxChannelA->rsp2TunerParams.antennaSel == sdrplay_api_Rsp2_ANTENNA_A)
      return "Anteanna A";
    else
      return "Antenna B";
  }

  if(hwVer == SDRPLAY_RSPduo_ID){
    if(channel == 1)
      return "Tuner 2 50 ohm";
    else {
      if(deviceParams->rxChannelA->rspDuoTunerParams.tuner1AmPortSel == sdrplay_api_RspDuo_AMPORT_1)
        return "Tuner 1 Hi-Z";
      else
        return "Tuner 1 50 ohm";
    }
  }
  return "";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapySDRPlay::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapySDRPlay::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
    auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;

    rxChannel->ctrlParams.dcOffset.DCenable = automatic;
    // TODO: possibly add IQenable as well
    sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Ctrl_DCoffsetIQimbalance);
}

bool SoapySDRPlay::getDCOffsetMode(const int direction, const size_t channel) const
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
    return rxChannel->ctrlParams.dcOffset.DCenable;
}

bool SoapySDRPlay::hasDCOffset(const int direction, const size_t channel) const
{
    //is a specific DC removal value configurable?
    return false;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapySDRPlay::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;

    results.push_back("IFGR");
    results.push_back("RFGR");

    return results;
}

bool SoapySDRPlay::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapySDRPlay::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
    auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;
    auto agcMode = sdrplay_api_AGC_DISABLE;

    if (automatic == true) {
        agcMode = sdrplay_api_AGC_100HZ;
        //align known agc values with current value before starting AGC.
        current_gRdB[channel] = gRdB[channel];
    }
    auto & agc = rxChannel->ctrlParams.agc;
    agc.enable = agcMode;
    //agc.setPoint = setPoint;

    sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Ctrl_Agc);
}

bool SoapySDRPlay::getGainMode(const int direction, const size_t channel) const
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;

    return (rxChannel->ctrlParams.agc.enable == sdrplay_api_AGC_DISABLE)? false: true;
}

void SoapySDRPlay::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
   std::lock_guard <std::mutex> lock(_general_state_mutex);

   bool doUpdate = false;
    auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;

   if (name == "IFGR")
   {
      // Depending of the previously used AGC context, the real applied
      // gain may be either gRdB or current_gRdB, so apply the change if required value is different
      // from one of them.
      if ((gRdB[channel] != (int)value) || (current_gRdB[channel] != (int)value))
      {
         gRdB[channel] = (int)value;
         current_gRdB[channel] = (int)value;
         doUpdate = true;
      }
   }
   else if (name == "RFGR")
   {
      if (lnaState[channel] != (int)value) {

          lnaState[channel] = (int)value;
          doUpdate = true;
      }
   }
  if ((doUpdate == true) && (streamActive))
  {
    sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Tuner_Gr);
  }
}

double SoapySDRPlay::getGain(const int direction, const size_t channel, const std::string &name) const
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

   if (name == "IFGR")
   {
       return current_gRdB[channel];
   }
   else if (name == "RFGR")
   {
      return lnaState[channel];
   }

   return 0;
}

SoapySDR::Range SoapySDRPlay::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
   if (name == "IFGR")
   {
      return SoapySDR::Range(20, 59);
   }
   else if ((name == "RFGR") && (hwVer == 1))
   {
      return SoapySDR::Range(0, 3);
   }
   else if ((name == "RFGR") && (hwVer == 2))
   {
      return SoapySDR::Range(0, 8);
   }
   else if ((name == "RFGR") && (hwVer == 3))
   {
      return SoapySDR::Range(0, 9);
   }
   else if ((name == "RFGR") && (hwVer > 253))
   {
      return SoapySDR::Range(0, 9);
   }
    return SoapySDR::Range(20, 59);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapySDRPlay::setFrequency(const int direction,
                                const size_t channel,
                                const std::string &name,
                                const double frequency,
                                const SoapySDR::Kwargs &args)
{
  std::lock_guard <std::mutex> lock(_general_state_mutex);

  auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
  auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;

  if (direction == SOAPY_SDR_RX)
  {
    uint32_t centerFrequency = (uint32_t) rxChannel->tunerParams.rfFreq.rfHz;
      if ((name == "RF") && (centerFrequency != (uint32_t)frequency))
      {
        rxChannel->tunerParams.rfFreq.rfHz = frequency;
        if (streamActive)
        {
          sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Tuner_Frf);
        }
      }
      else if ((name == "CORR") && (deviceParams->devParams->ppm != frequency))
      {
         deviceParams->devParams->ppm = frequency;
         sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Dev_Ppm);
      }
   }
}

double SoapySDRPlay::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;

    if (name == "RF")
    {
        return rxChannel->tunerParams.rfFreq.rfHz;
    }
    else if (name == "CORR")
    {
        return deviceParams->devParams->ppm;
    }

    return 0;
}

std::vector<std::string> SoapySDRPlay::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    names.push_back("CORR");
    return names;
}

SoapySDR::RangeList SoapySDRPlay::getFrequencyRange(const int direction, const size_t channel,  const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
       results.push_back(SoapySDR::Range(10000, 2000000000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapySDRPlay::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapySDRPlay::setSampleRate(const int direction, const size_t channel, const double rate)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d", sampleRate);

    if (direction == SOAPY_SDR_RX)
    {
      unsigned int decMp = decM;
      reqSampleRate = (uint32_t)rate;
      uint32_t currSampleRate = (uint32_t) deviceParams->devParams->fsFreq.fsHz;
      auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
      auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;
      auto ifMode = rxChannel->tunerParams.ifType;

      sampleRate = getInputSampleRateAndDecimation(reqSampleRate, &decM, &decEnable, ifMode);

       if ((sampleRate != currSampleRate) || (decM != decMp) || (reqSampleRate != sampleRate))
       {
          resetBuffer = true;
          if (streamActive)
          {
            uint32_t reason = (uint32_t) sdrplay_api_Update_Dev_Fs;
            deviceParams->devParams->fsFreq.fsHz = (double) sampleRate;

            if (ifMode == sdrplay_api_IF_Zero)
            {
              reason |= (uint32_t) sdrplay_api_Update_Ctrl_Decimation;
              rxChannel->ctrlParams.decimation.enable = (bool)decEnable;
            }
            sdrplay_api_Update(dev, tuner, (sdrplay_api_ReasonForUpdateT) reason);
          }
       }
    }
}

double SoapySDRPlay::getSampleRate(const int direction, const size_t channel) const
{
   return deviceParams->devParams->fsFreq.fsHz;
}

std::vector<double> SoapySDRPlay::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> rates;

    rates.push_back(250000);
    rates.push_back(500000);
    rates.push_back(1000000);
    rates.push_back(2000000);
    rates.push_back(2048000);
    rates.push_back(3000000);
    rates.push_back(4000000);
    rates.push_back(5000000);
    rates.push_back(6000000);
    rates.push_back(7000000);
    rates.push_back(8000000);
    rates.push_back(9000000);
    rates.push_back(10000000);

    return rates;
}

uint32_t SoapySDRPlay::getInputSampleRateAndDecimation(uint32_t rate, unsigned int *decM, unsigned int *decEnable, sdrplay_api_If_kHzT ifMode)
{
   if (ifMode == sdrplay_api_IF_2_048)
   {
      if      (rate == 2048000) { *decM = 4; *decEnable = 1; return 8192000; }
   }
   else if (ifMode == sdrplay_api_IF_0_450)
   {
      if      (rate == 1000000) { *decM = 2; *decEnable = 1; return 2000000; }
      else if (rate == 500000)  { *decM = 4; *decEnable = 1; return 2000000; }
   }
   else if (ifMode == sdrplay_api_IF_Zero)
   {

      if      ((rate >= 200000)  && (rate < 500000))  { *decM = 8; *decEnable = 1; return 2000000; }
      else if ((rate >= 500000)  && (rate < 1000000)) { *decM = 4; *decEnable = 1; return 2000000; }
      else if ((rate >= 1000000) && (rate < 2000000)) { *decM = 2; *decEnable = 1; return 2000000; }
      else                                            { *decM = 1; *decEnable = 0; return rate; }
   }

   // this is invalid, but return something
   *decM = 1; *decEnable = 0; return rate;
}

/*******************************************************************
* Bandwidth API
******************************************************************/

void SoapySDRPlay::setBandwidth(const int direction, const size_t channel, const double bw_in)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
    auto tuner     = (channel == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;

   if (direction == SOAPY_SDR_RX) 
   {
      auto bwMode = rxChannel->tunerParams.bwType;
      if (getBwValueFromEnum(bwMode) != bw_in)
      {
         bwMode = mirGetBwMhzEnum(bw_in);
         rxChannel->tunerParams.bwType = bwMode;
         if (streamActive)
         {
           sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Tuner_BwType);
         }
      }
   }
}

double SoapySDRPlay::getBandwidth(const int direction, const size_t channel) const
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);
    auto rxChannel = (channel == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;

   if (direction == SOAPY_SDR_RX)
   {
    return getBwValueFromEnum(rxChannel->tunerParams.bwType);
   }
   return 0;
}

std::vector<double> SoapySDRPlay::listBandwidths(const int direction, const size_t channel) const
{
   std::vector<double> bandwidths;
   bandwidths.push_back(200000);
   bandwidths.push_back(300000);
   bandwidths.push_back(600000);
   bandwidths.push_back(1536000);
   bandwidths.push_back(5000000);
   bandwidths.push_back(6000000);
   bandwidths.push_back(7000000);
   bandwidths.push_back(8000000);
   return bandwidths;
}

SoapySDR::RangeList SoapySDRPlay::getBandwidthRange(const int direction, const size_t channel) const
{
   SoapySDR::RangeList results;
   //call into the older deprecated listBandwidths() call
   for (auto &bw : this->listBandwidths(direction, channel))
   {
     results.push_back(SoapySDR::Range(bw, bw));
   }
   return results;
}

double SoapySDRPlay::getRateForBwEnum(sdrplay_api_Bw_MHzT bwEnum)
{
   if (bwEnum == sdrplay_api_BW_0_200) return 250000;
   else if (bwEnum == sdrplay_api_BW_0_300) return 500000;
   else if (bwEnum == sdrplay_api_BW_0_600) return 1000000;
   else if (bwEnum == sdrplay_api_BW_1_536) return 2000000;
   else if (bwEnum == sdrplay_api_BW_5_000) return 5000000;
   else if (bwEnum == sdrplay_api_BW_6_000) return 6000000;
   else if (bwEnum == sdrplay_api_BW_7_000) return 7000000;
   else if (bwEnum == sdrplay_api_BW_8_000) return 8000000;
   else return 0;
}


sdrplay_api_Bw_MHzT SoapySDRPlay::getBwEnumForRate(double rate, sdrplay_api_If_kHzT ifMode)
{
   if (ifMode == sdrplay_api_IF_Zero)
   {
      if      ((rate >= 200000)  && (rate < 300000))  return sdrplay_api_BW_0_200;
      else if ((rate >= 300000)  && (rate < 600000))  return sdrplay_api_BW_0_300;
      else if ((rate >= 600000)  && (rate < 1536000)) return sdrplay_api_BW_0_600;
      else if ((rate >= 1536000) && (rate < 5000000)) return sdrplay_api_BW_1_536;
      else if ((rate >= 5000000) && (rate < 6000000)) return sdrplay_api_BW_5_000;
      else if ((rate >= 6000000) && (rate < 7000000)) return sdrplay_api_BW_6_000;
      else if ((rate >= 7000000) && (rate < 8000000)) return sdrplay_api_BW_7_000;
      else                                            return sdrplay_api_BW_8_000;
   }
   else if ((ifMode == sdrplay_api_IF_0_450) || (ifMode == sdrplay_api_IF_1_620))
   {
      if      ((rate >= 200000)  && (rate < 500000))  return sdrplay_api_BW_0_200;
      else if ((rate >= 500000)  && (rate < 1000000)) return sdrplay_api_BW_0_300;
      else                                            return sdrplay_api_BW_0_600;
   }
   else
   {
      if      ((rate >= 200000)  && (rate < 500000))  return sdrplay_api_BW_0_200;
      else if ((rate >= 500000)  && (rate < 1000000)) return sdrplay_api_BW_0_300;
      else if ((rate >= 1000000) && (rate < 1536000)) return sdrplay_api_BW_0_600;
      else                                            return sdrplay_api_BW_1_536;
   }
}


double SoapySDRPlay::getBwValueFromEnum(sdrplay_api_Bw_MHzT bwEnum)
{
   if      (bwEnum == sdrplay_api_BW_0_200) return 200000;
   else if (bwEnum == sdrplay_api_BW_0_300) return 300000;
   else if (bwEnum == sdrplay_api_BW_0_600) return 600000;
   else if (bwEnum == sdrplay_api_BW_1_536) return 1536000;
   else if (bwEnum == sdrplay_api_BW_5_000) return 5000000;
   else if (bwEnum == sdrplay_api_BW_6_000) return 6000000;
   else if (bwEnum == sdrplay_api_BW_7_000) return 7000000;
   else if (bwEnum == sdrplay_api_BW_8_000) return 8000000;
   else return 0;
}


sdrplay_api_Bw_MHzT SoapySDRPlay::mirGetBwMhzEnum(double bw)
{
   if      (bw == 200000) return sdrplay_api_BW_0_200;
   else if (bw == 300000) return sdrplay_api_BW_0_300;
   else if (bw == 600000) return sdrplay_api_BW_0_600;
   else if (bw == 1536000) return sdrplay_api_BW_1_536;
   else if (bw == 5000000) return sdrplay_api_BW_5_000;
   else if (bw == 6000000) return sdrplay_api_BW_6_000;
   else if (bw == 7000000) return sdrplay_api_BW_7_000;
   else if (bw == 8000000) return sdrplay_api_BW_8_000;
   else return sdrplay_api_BW_0_200;
}

/*******************************************************************
* Settings API
******************************************************************/

sdrplay_api_If_kHzT SoapySDRPlay::stringToIF(std::string ifMode)
{
   if (ifMode == "Zero-IF")
   {
      return sdrplay_api_IF_Zero;
   }
   else if (ifMode == "450kHz")
   {
      return sdrplay_api_IF_0_450;
   }
   else if (ifMode == "1620kHz")
   {
      return sdrplay_api_IF_1_620;
   }
   else if (ifMode == "2048kHz")
   {
      return sdrplay_api_IF_2_048;
   }
   return sdrplay_api_IF_Zero;
}

std::string SoapySDRPlay::IFtoString(sdrplay_api_If_kHzT ifkHzT)
{
   switch (ifkHzT)
   {
   case sdrplay_api_IF_Zero:
      return "Zero-IF";
      break;
   case sdrplay_api_IF_0_450:
      return "450kHz";
      break;
   case sdrplay_api_IF_1_620:
      return "1620kHz";
      break;
   case sdrplay_api_IF_2_048:
      return "2048kHz";
      break;
   case sdrplay_api_IF_Undefined:
      return "";
      break;
   }
   return "";
}

SoapySDR::ArgInfoList SoapySDRPlay::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;
 
#ifdef RF_GAIN_IN_MENU
    if (hwVer == 2)
    {
       SoapySDR::ArgInfo RfGainArg;
       RfGainArg.key = "rfgain_sel";
       RfGainArg.value = "4";
       RfGainArg.name = "RF Gain Select";
       RfGainArg.description = "RF Gain Select";
       RfGainArg.type = SoapySDR::ArgInfo::STRING;
       RfGainArg.options.push_back("0");
       RfGainArg.options.push_back("1");
       RfGainArg.options.push_back("2");
       RfGainArg.options.push_back("3");
       RfGainArg.options.push_back("4");
       RfGainArg.options.push_back("5");
       RfGainArg.options.push_back("6");
       RfGainArg.options.push_back("7");
       RfGainArg.options.push_back("8");
       setArgs.push_back(RfGainArg);
    }
    else if (hwVer == 3)
    {
       SoapySDR::ArgInfo RfGainArg;
       RfGainArg.key = "rfgain_sel";
       RfGainArg.value = "4";
       RfGainArg.name = "RF Gain Select";
       RfGainArg.description = "RF Gain Select";
       RfGainArg.type = SoapySDR::ArgInfo::STRING;
       RfGainArg.options.push_back("0");
       RfGainArg.options.push_back("1");
       RfGainArg.options.push_back("2");
       RfGainArg.options.push_back("3");
       RfGainArg.options.push_back("4");
       RfGainArg.options.push_back("5");
       RfGainArg.options.push_back("6");
       RfGainArg.options.push_back("7");
       RfGainArg.options.push_back("8");
       RfGainArg.options.push_back("9");
       setArgs.push_back(RfGainArg);
    }
    else if (hwVer > 253)
    {
       SoapySDR::ArgInfo RfGainArg;
       RfGainArg.key = "rfgain_sel";
       RfGainArg.value = "4";
       RfGainArg.name = "RF Gain Select";
       RfGainArg.description = "RF Gain Select";
       RfGainArg.type = SoapySDR::ArgInfo::STRING;
       RfGainArg.options.push_back("0");
       RfGainArg.options.push_back("1");
       RfGainArg.options.push_back("2");
       RfGainArg.options.push_back("3");
       RfGainArg.options.push_back("4");
       RfGainArg.options.push_back("5");
       RfGainArg.options.push_back("6");
       RfGainArg.options.push_back("7");
       RfGainArg.options.push_back("8");
       RfGainArg.options.push_back("9");
       setArgs.push_back(RfGainArg);
    }
    else
    {
       SoapySDR::ArgInfo RfGainArg;
       RfGainArg.key = "rfgain_sel";
       RfGainArg.value = "1";
       RfGainArg.name = "RF Gain Select";
       RfGainArg.description = "RF Gain Select";
       RfGainArg.type = SoapySDR::ArgInfo::STRING;
       RfGainArg.options.push_back("0");
       RfGainArg.options.push_back("1");
       RfGainArg.options.push_back("2");
       RfGainArg.options.push_back("3");
       setArgs.push_back(RfGainArg);
    }
#endif
    
    SoapySDR::ArgInfo AIFArg;
    //AIFArg.key = "if_mode"; // TODO
    //AIFArg.value = IFtoString(ifMode);
    AIFArg.name = "IF Mode";
    AIFArg.description = "IF frequency in kHz";
    AIFArg.type = SoapySDR::ArgInfo::STRING;
    AIFArg.options.push_back(IFtoString(sdrplay_api_IF_Zero));
    AIFArg.options.push_back(IFtoString(sdrplay_api_IF_0_450));
    AIFArg.options.push_back(IFtoString(sdrplay_api_IF_1_620));
    AIFArg.options.push_back(IFtoString(sdrplay_api_IF_2_048));
    setArgs.push_back(AIFArg);

    SoapySDR::ArgInfo IQcorrArg;
    IQcorrArg.key = "iqcorr_ctrl";
    IQcorrArg.value = "true";
    IQcorrArg.name = "IQ Correction";
    IQcorrArg.description = "IQ Correction Control";
    IQcorrArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(IQcorrArg);

    SoapySDR::ArgInfo SetPointArg;
    SetPointArg.key = "agc_setpoint";
    SetPointArg.value = "-30";
    SetPointArg.name = "AGC Setpoint";
    SetPointArg.description = "AGC Setpoint (dBfs)";
    SetPointArg.type = SoapySDR::ArgInfo::INT;
    SetPointArg.range = SoapySDR::Range(-60, 0);
    setArgs.push_back(SetPointArg);

    if (hwVer == 2) // RSP2/RSP2pro
    {
       SoapySDR::ArgInfo ExtRefArg;
       ExtRefArg.key = "extref_ctrl";
       ExtRefArg.value = "true";
       ExtRefArg.name = "ExtRef Enable";
       ExtRefArg.description = "External Reference Control";
       ExtRefArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(ExtRefArg);

       SoapySDR::ArgInfo BiasTArg;
       BiasTArg.key = "biasT_ctrl";
       BiasTArg.value = "true";
       BiasTArg.name = "BiasT Enable";
       BiasTArg.description = "BiasT Control";
       BiasTArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(BiasTArg);

       SoapySDR::ArgInfo RfNotchArg;
       RfNotchArg.key = "rfnotch_ctrl";
       RfNotchArg.value = "true";
       RfNotchArg.name = "RfNotch Enable";
       RfNotchArg.description = "RF Notch Filter Control";
       RfNotchArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(RfNotchArg);
    }
    else if (hwVer == 3) // RSPduo
    {
       SoapySDR::ArgInfo ExtRefArg;
       ExtRefArg.key = "extref_ctrl";
       ExtRefArg.value = "true";
       ExtRefArg.name = "ExtRef Enable";
       ExtRefArg.description = "External Reference Control";
       ExtRefArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(ExtRefArg);

       SoapySDR::ArgInfo BiasTArg;
       BiasTArg.key = "biasT_ctrl";
       BiasTArg.value = "true";
       BiasTArg.name = "BiasT Enable";
       BiasTArg.description = "BiasT Control";
       BiasTArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(BiasTArg);

       SoapySDR::ArgInfo RfNotchArg;
       RfNotchArg.key = "rfnotch_ctrl";
       RfNotchArg.value = "true";
       RfNotchArg.name = "RfNotch Enable";
       RfNotchArg.description = "RF Notch Filter Control";
       RfNotchArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(RfNotchArg);

       SoapySDR::ArgInfo DabNotchArg;
       DabNotchArg.key = "dabnotch_ctrl";
       DabNotchArg.value = "true";
       DabNotchArg.name = "DabNotch Enable";
       DabNotchArg.description = "DAB Notch Filter Control";
       DabNotchArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(DabNotchArg);
    }
    else if (hwVer > 253) // RSP1A
    {
       SoapySDR::ArgInfo BiasTArg;
       BiasTArg.key = "biasT_ctrl";
       BiasTArg.value = "true";
       BiasTArg.name = "BiasT Enable";
       BiasTArg.description = "BiasT Control";
       BiasTArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(BiasTArg);

       SoapySDR::ArgInfo RfNotchArg;
       RfNotchArg.key = "rfnotch_ctrl";
       RfNotchArg.value = "true";
       RfNotchArg.name = "RfNotch Enable";
       RfNotchArg.description = "RF Notch Filter Control";
       RfNotchArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(RfNotchArg);

       SoapySDR::ArgInfo DabNotchArg;
       DabNotchArg.key = "dabnotch_ctrl";
       DabNotchArg.value = "true";
       DabNotchArg.name = "DabNotch Enable";
       DabNotchArg.description = "DAB Notch Filter Control";
       DabNotchArg.type = SoapySDR::ArgInfo::BOOL;
       setArgs.push_back(DabNotchArg);
    }

    return setArgs;
}

void SoapySDRPlay::writeSetting(const std::string &key, const std::string &value)
{
#if 0
    std::lock_guard <std::mutex> lock(_general_state_mutex);

#ifdef RF_GAIN_IN_MENU
   if (key == "rfgain_sel")
   {
      if      (value == "0") lnaState = 0;
      else if (value == "1") lnaState = 1;
      else if (value == "2") lnaState = 2;
      else if (value == "3") lnaState = 3;
      else if (value == "4") lnaState = 4;
      else if (value == "5") lnaState = 5;
      else if (value == "6") lnaState = 6;
      else if (value == "7") lnaState = 7;
      else if (value == "8") lnaState = 8;
      else                   lnaState = 9;
      if (agcMode != sdrplay_api_AGC_DISABLE)
      {
         sdrplay_api_AgcControl(agcMode, setPoint, 0, 0, 0, 0, lnaState);
      }
      else
      {
         sdrplay_api_Reinit(&gRdB, 0.0, 0.0, sdrplay_api_BW_Undefined, sdrplay_api_IF_Undefined, sdrplay_api_LO_Undefined, lnaState, &gRdBsystem, sdrplay_api_USE_RSP_SET_GR, &sps, sdrplay_api_CHANGE_GR);
      }
   }
   else
#endif
   if (key == "if_mode")
   {
      if (ifMode != stringToIF(value))
      {
         ifMode = stringToIF(value);
         sampleRate = getInputSampleRateAndDecimation(reqSampleRate, &decM, &decEnable, ifMode);
         bwMode = getBwEnumForRate(reqSampleRate, ifMode);
         if (streamActive)
         {
            sdrplay_api_DecimateControl(0, 1, 1);
            sdrplay_api_Reinit(&gRdB, sampleRate / 1e6, 0.0, bwMode, ifMode, sdrplay_api_LO_Undefined, lnaState, &gRdBsystem, sdrplay_api_USE_RSP_SET_GR, &sps, (sdrplay_api_ReasonForReinitT)(sdrplay_api_CHANGE_FS_FREQ | sdrplay_api_CHANGE_BW_TYPE | sdrplay_api_CHANGE_IF_TYPE));
         }
      }
   }
   else if (key == "iqcorr_ctrl")
   {
      if (value == "false") IQcorr = 0;
      else                  IQcorr = 1;
      sdrplay_api_DCoffsetIQimbalanceControl(1, IQcorr);
      //sdrplay_api_DCoffsetIQimbalanceControl(IQcorr, IQcorr);
   }
   else if (key == "agc_setpoint")
   {
      setPoint = stoi(value);
      sdrplay_api_AgcControl(agcMode, setPoint, 0, 0, 0, 0, lnaState);
   }
   else if (key == "extref_ctrl")
   {
      if (value == "false") extRef = 0;
      else                  extRef = 1;
      if (hwVer == 2) sdrplay_api_RSPII_ExternalReferenceControl(extRef);
      if (hwVer == 3) sdrplay_api_rspDuo_ExtRef(extRef);
   }
   else if (key == "biasT_ctrl")
   {
      if (value == "false") biasTen = 0;
      else                  biasTen = 1;
      if (hwVer == 2) sdrplay_api_RSPII_BiasTControl(biasTen);
      if (hwVer == 3) sdrplay_api_rspDuo_BiasT(biasTen);
      if (hwVer > 253) sdrplay_api_rsp1a_BiasT(biasTen);
   }
   else if (key == "rfnotch_ctrl")
   {
      if (value == "false") notchEn = 0;
      else                  notchEn = 1;
      if (hwVer == 2) sdrplay_api_RSPII_RfNotchEnable(notchEn);
      if (hwVer == 3)
      {
        if (tunSel == sdrplay_api_rspDuo_Tuner_1 && amPort == 1) sdrplay_api_rspDuo_Tuner1AmNotch(notchEn);
        if (amPort == 0) sdrplay_api_rspDuo_BroadcastNotch(notchEn);
      }
      if (hwVer > 253) sdrplay_api_rsp1a_BroadcastNotch(notchEn);
   }
   else if (key == "dabnotch_ctrl")
   {
      if (value == "false") dabNotchEn = 0;
      else                  dabNotchEn = 1;
      if (hwVer == 3) sdrplay_api_rspDuo_DabNotch(dabNotchEn);
      if (hwVer > 253) sdrplay_api_rsp1a_DabNotch(dabNotchEn);
   }
#endif
}

std::string SoapySDRPlay::readSetting(const std::string &key) const
{
#if 0
    std::lock_guard <std::mutex> lock(_general_state_mutex);

#ifdef RF_GAIN_IN_MENU
    if (key == "rfgain_sel")
    {
       if      (lnaState == 0) return "0";
       else if (lnaState == 1) return "1";
       else if (lnaState == 2) return "2";
       else if (lnaState == 3) return "3";
       else if (lnaState == 4) return "4";
       else if (lnaState == 5) return "5";
       else if (lnaState == 6) return "6";
       else if (lnaState == 7) return "7";
       else if (lnaState == 8) return "8";
       else                    return "9";
    }
    else
#endif
    if (key == "if_mode")
    {
        return IFtoString(ifMode);
    }
    else if (key == "iqcorr_ctrl")
    {
       if (IQcorr == 0) return "false";
       else             return "true";
    }
    else if (key == "agc_setpoint")
    {
       return std::to_string(setPoint);
    }
    else if (key == "extref_ctrl")
    {
       if (extRef == 0) return "false";
       else             return "true";
    }
    else if (key == "biasT_ctrl")
    {
       if (biasTen == 0) return "false";
       else              return "true";
    }
    else if (key == "rfnotch_ctrl")
    {
       if (notchEn == 0) return "false";
       else              return "true";
    }
    else if (key == "dabnotch_ctrl")
    {
       if (dabNotchEn == 0) return "false";
       else                 return "true";
    }

    // SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
#endif
}

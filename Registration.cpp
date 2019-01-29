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
#include <SoapySDR/Registry.hpp>

#if !defined(_M_X64) && !defined(_M_IX86)
#define sprintf_s(buffer, buffer_size, stringbuffer, ...) (sprintf(buffer, stringbuffer, __VA_ARGS__))
#endif

#define MAX_RSP_DEVICES  (4)

static sdrplay_api_DeviceT rspDevs[MAX_RSP_DEVICES];
bool deviceSelected = false;

static std::vector<SoapySDR::Kwargs> findSDRPlay(const SoapySDR::Kwargs &args)
{
   std::vector<SoapySDR::Kwargs> results;
   std::string labelHint;
   if (args.count("label") != 0) labelHint = args.at("label");
   unsigned int nDevs = 0;
   char lblstr[128];

   if (deviceSelected == true)
   {
     sdrplay_api_Close();
     deviceSelected = false;
   }

   sdrplay_api_Open();
   //Enable (= 1) API calls tracing,
   //but only for debug purposes due to its performance impact. 
   // TODO: Replace constant with undocumented sdrplay_api_DbgLvl_t
   sdrplay_api_DebugEnable(NULL, (sdrplay_api_DbgLvl_t)0u);

   std::string baseLabel = "SDRplay Dev";

	// list devices by API
   sdrplay_api_GetDevices(&rspDevs[0], &nDevs, MAX_RSP_DEVICES);


   size_t posidx = labelHint.find(baseLabel);

   if (posidx != std::string::npos)
   {
      unsigned int devIdx = labelHint.at(posidx + baseLabel.length()) - 0x30;

      if (devIdx < nDevs)
      {
         SoapySDR::Kwargs dev;
         dev["driver"] = "sdrplay3";
         if (rspDevs[devIdx].hwVer > 253)
         {
             sprintf_s(lblstr, 128, "SDRplay Dev%d RSP1A %s", devIdx, rspDevs[devIdx].SerNo);
         }
         else if (rspDevs[devIdx].hwVer == SDRPLAY_RSPduo_ID)
         {
             sprintf_s(lblstr, 128, "SDRplay Dev%d RSPduo %s", devIdx, rspDevs[devIdx].SerNo);
         }
         else
         {
             sprintf_s(lblstr, 128, "SDRplay Dev%d RSP%d %s", devIdx, rspDevs[devIdx].hwVer, rspDevs[devIdx].SerNo);
         }
         dev["label"] = lblstr;
         results.push_back(dev);
      }
   }
   else
   {
      for (unsigned int i = 0; i < nDevs; i++)
      {
        SoapySDR::Kwargs dev;
        dev["driver"] = "sdrplay3";
        if (rspDevs[i].hwVer > 253)
        {
           sprintf_s(lblstr, 128, "SDRplay Dev%d RSP1A %s", i, rspDevs[i].SerNo);
        }
        else if (rspDevs[i].hwVer == 3)
        {
           sprintf_s(lblstr, 128, "SDRplay Dev%d RSPduo %s", i, rspDevs[i].SerNo);
        }
        else
        {
           sprintf_s(lblstr, 128, "SDRplay Dev%d RSP%d %s", i, rspDevs[i].hwVer, rspDevs[i].SerNo);
        }
        dev["label"] = lblstr;
        results.push_back(dev);
      }
   }
   sdrplay_api_Close();
   return results;
}

static SoapySDR::Device *makeSDRPlay(const SoapySDR::Kwargs &args)
{
    return new SoapySDRPlay(args);
}

static SoapySDR::Registry registerSDRPlay("sdrplay3", &findSDRPlay, &makeSDRPlay, SOAPY_SDR_ABI_VERSION);

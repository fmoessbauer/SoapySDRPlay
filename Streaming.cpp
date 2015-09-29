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


SoapySDR::Stream *SoapySDRPlay::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &args)
{
    //check that direction is SOAPY_SDR_RX
    if (direction != SOAPY_SDR_RX) {
        throw std::runtime_error("SDRPlay is RX only, use SOAPY_SDR_RX");
    }

    //check the channel configuration
        //check than channels is either empty or [0]
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == "CF32")
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
        rxFloat = true;
    }
    else if (format == "CS16")
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
        rxFloat = false;
    }
    else
    {
        throw std::runtime_error("setupStream invalid format '" + format + "' -- Only CS16 and CF32 are supported by SoapySDRPlay, and CS16 is the native format.");
    }

//    if (args.find("buffer_packets") != args.end()) {
//        int numPackets_in = std::stoi(args.at("buffer_packets"));
//        if (std::isnan(numPackets_in) || numPackets_in == 0) {
//            SoapySDR_logf(SOAPY_SDR_DEBUG, "numPackets is 0 or not a number; defaulting to %d", numPackets);
//        } else {
//            numPackets = numPackets_in;
//        }
//    }
//    SoapySDR_logf(SOAPY_SDR_DEBUG, "Set numPackets to %d", numPackets);

    //use args to specify optional things like:
    //integer to float scale factors
    //number of transfers and or transfer size
    //depends what the hardware supports...

    //TODO: add optional SDRPlay stream flags here

    return (SoapySDR::Stream *)this;
}

void SoapySDRPlay::closeStream(SoapySDR::Stream *stream)
{
    xi.erase(xi.begin(), xi.end());
    xq.erase(xi.begin(), xi.end());
}

size_t SoapySDRPlay::getStreamMTU(SoapySDR::Stream *stream) const
{
    //how large is a transfer?
    //this value helps users to allocate buffers that will match the hardware transfer size
    return xi.size();
}

int SoapySDRPlay::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems)
{
    //probably can ignore flags and time and numElems
    //these are for timed burst features
    //however it might be nice to return SOAPY_SDR_NOT_SUPPORTED
    //when flags != or or numElems != 0

    //this function should actually enable the hardware to stream

    if (centerFreqChanged) {
        centerFreq = newCenterFreq;
        centerFreqChanged = false;
    }

    if (rateChanged) {
        rate = newRate;
        rateChanged = false;

        mir_sdr_Bw_MHzT bwCheck = getBwEnumForRate(rate);
        double bwCheckVal = getBwValueFromEnum(bwCheck);
        if (bwCheckVal != bw) {
            bw = bwCheckVal;
            bwChanged = false;
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Changed bandwidth for rate %f to %f", rate, bw);
        }
    } else if (bwChanged) {
        mir_sdr_Bw_MHzT eBw = getBwEnumForRate(newBw);
        bw = getBwValueFromEnum(eBw);
        bwChanged = false;
    }

    // Configure DC tracking in tuner
    mir_sdr_ErrT err;
    err = mir_sdr_SetDcMode(4,0);
    err = mir_sdr_SetDcTrackTime(63);
    err = mir_sdr_Init(newGr, rate/1000000.0, centerFreq/1000000.0, mirGetBwMhzEnum(bw), mir_sdr_IF_Zero, &sps);

    if (err != 0) {
        throw std::runtime_error("activateStream failed.");
    }

    numPackets = getOptimalPacketsForRate(rate, sps);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Set numPackets to %d", numPackets);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "stream sps: %d", sps);
    SoapySDR_logf(SOAPY_SDR_DEBUG, "stream numPackets*sps: %d", (numPackets*sps));


    // Alocate data buffers
    xi.resize(sps * numPackets);
    xq.resize(sps * numPackets);
    syncUpdate = 0;

    return 0;
}

int SoapySDRPlay::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs)
{
    //same idea as activateStream,
    //but disable streaming in the hardware
    mir_sdr_ErrT err;
    err = mir_sdr_Uninit();

    return 0;
}

int SoapySDRPlay::readStream(
    SoapySDR::Stream *stream,
    void * const *buffs,
    const size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    mir_sdr_ErrT err;

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    //check for data with timeout
    // TODO: Can we do?

    bool reInit = false;

    if (rateChanged)
    {
        rate = newRate;
        rateChanged = false;

        mir_sdr_Bw_MHzT bwCheck = getBwEnumForRate(rate);
        double bwCheckVal = getBwValueFromEnum(bwCheck);

        bwChanged = true;
        newBw = bwCheckVal;

        SoapySDR_log(SOAPY_SDR_DEBUG,"Changed sample rate");
    }

    if (centerFreqChanged)
    {
        double freqDiff = std::abs(centerFreq-newCenterFreq);
        centerFreq = newCenterFreq;
        centerFreqChanged = false;

        if (freqDiff < rate/2.0) {
            mir_sdr_SetRf(centerFreq, 1, 0);
        } else {
            reInit = true;
        }

        SoapySDR_log(SOAPY_SDR_DEBUG,"Changed center frequency");
    }

    if (bwChanged)
    {
        bw = newBw;
        bwChanged = false;
        reInit = true;
    }

    if (reInit)
    {
        // "For large ADC sample frequency changes a mir_sdr_Uninit and mir_sdr_Init at the new sample rate must be performed."
        err = mir_sdr_Uninit();
        err = mir_sdr_Init(newGr, rate/1000000.0, centerFreq/1000000.0, mirGetBwMhzEnum(bw), mir_sdr_IF_Zero, &sps);

        numPackets = getOptimalPacketsForRate(rate, sps);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "stream re-init sps: %d", sps);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "stream re-init numPackets*sps: %d", (numPackets*sps));

        xi.resize(sps * numPackets);
        xq.resize(sps * numPackets);

        xi_buffer.erase(xi_buffer.begin(), xi_buffer.end());
        xq_buffer.erase(xq_buffer.begin(), xq_buffer.end());

        if (err != 0) {
            throw std::runtime_error("Error resetting mir_sdr interface for bandwidth change.");
        }

    }

    // Prevent stalling if we've already buffered enough data..
    if (xi_buffer.size() < numElems)
    {
        // If we change frequency or bandwidth mid-stream then drop the unmatched samples
        // by setting the startPacket offset
        int startPacket = 0;

        //receive into temporary buffer
        for (int i = 0; i < numPackets; i++)
        {
            err = mir_sdr_ReadPacket(&xi[sps*i], &xq[sps*i], &fs, &grc, &rfc, &fsc);
            //return SOAPY_SDR_TIMEOUT when timeout occurs
            if (err != 0)
            {
                return SOAPY_SDR_TIMEOUT;
            }
            if (grc) {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "Gain change acknowledged from device. packet: %d", i);
                grChangedAfter = (i + 1) * sps; // indicate where change occurred
                grc = 0;
                mir_sdr_ResetUpdateFlags(1,0,0);
            }
            if (rfc) {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "Center frequency change acknowledged from device. packet: %d", i);
                mir_sdr_ResetUpdateFlags(0,1,0);
                startPacket = i;
            }
            if (fsc) {  // This shouldn't happen now but leaving it to see..
                SoapySDR_logf(SOAPY_SDR_DEBUG, "Rate change acknowledged from device. packet: %d", i);
                mir_sdr_ResetUpdateFlags(0,0,1);
                startPacket = i;
            }
        }

        // AGC
        if (grChangedAfter > 0) // do AGC if no update pending
        {
            double adcPower, ival, qval;
            for (int j = 0; j < (sps * numPackets); j++)
            {
                ival = xi[j];
                qval = xq[j];
                adcPower += (ival*ival) + (qval*qval);
            }
            adcPower /= double(sps*numPackets);

            if ((adcPower > double(adcHigh)) || (adcPower < double(adcLow))) {
                newGr = int(10.0 * log(adcPower / adcTarget));
                SoapySDR_logf(SOAPY_SDR_DEBUG, "AGC: Gain reduction changed from %d to %d", oldGr, newGr);
            }

            // reset flag in case no change is required
            grChangedAfter = 0;
            // only update if change is required
            if (newGr != oldGr) {
                // use absolute value
                err = mir_sdr_SetGr(newGr, 1, syncUpdate);
                grChangedAfter = -1;
                oldGr = newGr;
            }
        }

        //was numElems < than the hardware transfer size?
        //may have to keep part of that temporary buffer
        //around for the next call into readStream...
        if (startPacket < numPackets) {
            xi_buffer.insert(xi_buffer.end(),xi.begin()+(sps*startPacket),xi.end());
            xq_buffer.insert(xq_buffer.end(),xq.begin()+(sps*startPacket),xq.end());
        }
    }

    int returnedElems = (numElems>xi_buffer.size())?xi_buffer.size():numElems;

    if (!returnedElems) {
        return SOAPY_SDR_UNDERFLOW;
    }

    //convert into user's buff0
    if (rxFloat)
    {
        float *ftarget = (float *)buff0;
        for (int i = 0; i < returnedElems; i++)
        {
            ftarget[i*2] = ((float)xi_buffer[i]/(float)SHRT_MAX);
            ftarget[i*2+1] = ((float)xq_buffer[i]/(float)SHRT_MAX);
        }
    }
    else
    {
        short *starget = (short *)buff0;
        for (int i = 0; i < returnedElems; i++)
        {
            starget[i*2] = xi_buffer[i];
            starget[i*2+1] = xq_buffer[i];
        }
    }

    xi_buffer.erase(xi_buffer.begin(),xi_buffer.begin()+returnedElems);
    xq_buffer.erase(xq_buffer.begin(),xq_buffer.begin()+returnedElems);

    //return number of elements written to buff0
    return returnedElems;
}

int SoapySDRPlay::getOptimalPacketsForRate(double rate_in, int sps_in) {
    return ceil((rate_in / 30.0)/(float)sps_in);
}

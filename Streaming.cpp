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

std::vector<std::string> SoapySDRPlay::getStreamFormats(const int direction, const size_t channel) const 
{
    std::vector<std::string> formats;

    formats.push_back("CS16");
    formats.push_back("CF32");

    return formats;
}

std::string SoapySDRPlay::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const 
{
     fullScale = 32767;
     return "CS16";
}

SoapySDR::ArgInfoList SoapySDRPlay::getStreamArgsInfo(const int direction, const size_t channel) const 
{
    SoapySDR::ArgInfoList streamArgs;

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static void _rx_callback_A(
  short *xi,
  short *xq,
  sdrplay_api_StreamCbParamsT *params,
  unsigned int numSamples,
  unsigned int reset,
  void *cbContext)
{
    StreamData *pstream = (StreamData *)cbContext;
    return SoapySDRPlay::rx_callback(pstream, xi, xq, numSamples, 0);
}

static void _rx_callback_B(
  short *xi,
  short *xq,
  sdrplay_api_StreamCbParamsT *params,
  unsigned int numSamples,
  unsigned int reset,
  void *cbContext)
{
    StreamData *pstream = (StreamData *)cbContext;
    return SoapySDRPlay::rx_callback(pstream, xi, xq, numSamples, 1);
}

static void _gr_callback(
  sdrplay_api_EventT eventId,
  sdrplay_api_TunerSelectT tuner,
  sdrplay_api_EventParamsT *params,
  void *cbContext)
{
    StreamData *pstream = (StreamData *)cbContext;
    return pstream->_soapy_instance->gr_callback(pstream, eventId, tuner, params);
}

void SoapySDRPlay::rx_callback(StreamData * pstream, short *xi, short *xq, unsigned int numSamples, unsigned short channel)
{
  // retract block from pool
  // fill block
  // enqueue
//  SoapySDR_logf(SOAPY_SDR_INFO, "callback on %d", channel);

  // this should not be required as we always start a new block
  int spaceReqd = numSamples * pstream->_elements_per_sample * pstream->_shorts_per_word;

  StreamBlock * block{nullptr};
  // fetch a new block
  if(!pstream->_pool.try_dequeue(block)){
      pstream->_overflow_event = true;
      return;
  }
  assert(block != nullptr);

  block->resize(spaceReqd);
  auto * beg = block->data();

  if (pstream->_shorts_per_word == 1)
  {
     short *dptr = (short*) beg;
     for (unsigned i = 0; i < numSamples; i++)
     {
         *dptr++ = xi[i];
         *dptr++ = xq[i];
      }
  }
  else
  {
     float *dptr = (float*) beg;
     for (unsigned i = 0; i < numSamples; i++)
     {
        *dptr++ = (float)xi[i] / 32768.0f;
        *dptr++ = (float)xq[i] / 32768.0f;
     }
  }
  // release block (add to queue)
  if(!pstream->_queues[channel].try_enqueue(block)){
    throw std::runtime_error("try to add unavailable block");
  }
  return;
}

void SoapySDRPlay::gr_callback(
  StreamData * pstream,
  sdrplay_api_EventT eventId,
  sdrplay_api_TunerSelectT tuner,
  sdrplay_api_EventParamsT *params)
{
  auto tunerid = (tuner == sdrplay_api_Tuner_A) ? 0 : 1;

  switch(eventId){
    case sdrplay_api_GainChange:
      gRdB[tunerid] = params->gainParams.gRdB;
      if(gRdB[tunerid] < 200)
        current_gRdB[tunerid] = gRdB[tunerid];
      break;

    case sdrplay_api_PowerOverloadChange:
      sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck);
      // OVERLOAD DECTECTED
      break;
    default:
      break;
  }
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapySDRPlay::setupStream(const int direction,
                                            const std::string &format,
                                            const std::vector<size_t> &channels,
                                            const SoapySDR::Kwargs &args)
{
    // check the channel configuration
// TODO
//    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) 
//    {
//       throw std::runtime_error("setupStream invalid channel selection");
//    }
    
    unsigned shortsPerWord;
    // check the format
    if (format == "CS16") 
    {
        useShort = true;
        shortsPerWord = 1;
        bufferLength = bufferElems * elementsPerSample * shortsPerWord;
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
    } 
    else if (format == "CF32") 
    {
        useShort = false;
        shortsPerWord = sizeof(float) / sizeof(short);
        bufferLength = bufferElems * elementsPerSample * shortsPerWord;  // allocate enough space for floats instead of shorts
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
    } 
    else 
    {
       throw std::runtime_error( "setupStream invalid format '" + format +
                                  "' -- Only CS16 or CF32 are supported by the SoapySDRPlay module.");
    }

    std::lock_guard<std::mutex> lock(_buf_mutex);

    StreamData * pstream = new StreamData(DEFAULT_NUM_BUFFERS, DEFAULT_BUFFER_LENGTH, shortsPerWord, channels, this);
    return (SoapySDR::Stream *) pstream;
}

void SoapySDRPlay::closeStream(SoapySDR::Stream *stream)
{
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    if (streamActive)
    {
      sdrplay_api_Uninit(dev);
    }
    delete ((StreamData *) stream);

    streamActive = false;
}

size_t SoapySDRPlay::getStreamMTU(SoapySDR::Stream *stream) const
{
    // is a constant in practice
    return DEFAULT_BUFFER_LENGTH;
}

int SoapySDRPlay::activateStream(SoapySDR::Stream *stream,
                                 const int flags,
                                 const long long timeNs,
                                 const size_t numElems)
{
    if (flags != 0) 
    {
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    resetBuffer = true;
    sdrplay_api_ErrT err;
    auto * pstream = (StreamData*) stream;
    
    std::lock_guard <std::mutex> lock(_general_state_mutex);

    //Enable (= 1) API calls tracing,
    //but only for debug purposes due to its performance impact. 
    sdrplay_api_DebugEnable(dev, (sdrplay_api_DbgLvl_t)0);

    //temporary fix for ARM targets.
#if defined(__arm__) || defined(__aarch64__)
    deviceParams->mode = sdrplay_api_BULK;
    // TODO: Update does not seem to be required (at least no reason stated)
#endif

    sdrplay_api_CallbackFnsT cbFns;
    cbFns.StreamACbFn = _rx_callback_A;
    cbFns.StreamBCbFn = NULL;
    cbFns.EventCbFn = _gr_callback;

    // single channel on tuner 2
    if(pstream->_channels[0] == 1){
      cbFns.StreamACbFn = NULL;
      cbFns.StreamBCbFn = _rx_callback_A;
    }
    if(pstream->_channels.size() == 2){
      cbFns.StreamBCbFn = _rx_callback_B;
    }
    err = sdrplay_api_Init(dev, &cbFns, (void*)stream);

//    err = mir_sdr_StreamInit(&gRdB, sampleRate / 1e6, centerFrequency / 1e6, bwMode,
//                             ifMode, lnaState, &gRdBsystem, mir_sdr_USE_RSP_SET_GR, &sps,
//                             _rx_callback, _gr_callback, (void *)this);

    if (err != sdrplay_api_Success)
    {
       //throw std::runtime_error("StreamInit Error: " + std::to_string(err));
       return SOAPY_SDR_NOT_SUPPORTED;
    }
    for(auto & cidx : pstream->_channels){
      auto * channel = (cidx == 0) ? deviceParams->rxChannelA : deviceParams->rxChannelB;
      auto tuner = (cidx == 0) ? sdrplay_api_Tuner_A : sdrplay_api_Tuner_B;

      // decimation
      channel->ctrlParams.decimation.enable           = decEnable;
      channel->ctrlParams.decimation.decimationFactor = decM;
      channel->ctrlParams.decimation.wideBandSignal   = 1;

      // DC Correction
      channel->tunerParams.dcOffsetTuner.dcCal     = 4;
      channel->tunerParams.dcOffsetTuner.trackTime = 63;

      sdrplay_api_Update(dev, tuner, sdrplay_api_Update_Ctrl_Decimation);
    }

    streamActive = true;
    return 0;
}

int SoapySDRPlay::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0)
    {
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    std::lock_guard <std::mutex> lock(_general_state_mutex);

    if (streamActive)
    {
      sdrplay_api_Uninit(dev);
    }

    streamActive = false;

    return 0;
}

int SoapySDRPlay::readStream(SoapySDR::Stream *stream,
                             void * const *buffs,
                             const size_t numElems,
                             int &flags,
                             long long &timeNs,
                             const long timeoutUs)
{   
    if (!streamActive) 
    {
        return 0;
    }
    
    auto * pstream = (StreamData *) stream;
    void* source_buffs[2];

    // are elements left in the buffer? if not, do a new read.
    size_t unused = 0;
    if (pstream->_read_remaining == 0)
    {
        int ret = this->acquireReadBuffer(stream, unused, (const void **)&source_buffs, flags, timeNs, timeoutUs);

        if (ret < 0)
        {
            return ret;
        }
        pstream->_read_remaining = ret;
    } else {
      for(size_t i=0; i<pstream->_channels.size(); ++i){
        assert(nullptr != pstream->_cur_read_block[i]);
        source_buffs[i] = pstream->_cur_read_block[i]->data();
      }
    }

    size_t returnedElems = std::min((size_t)pstream->_read_remaining, numElems);

    for(size_t i=0; i<pstream->_channels.size(); ++i){
      // copy into user's buff0
      if (useShort)
      {
          std::memcpy(buffs[i], source_buffs[i], returnedElems * 2 * sizeof(short));
      }
      else
      {
          std::memcpy(buffs[i], (float *)(source_buffs[i]), returnedElems * 2 * sizeof(float));
      }
    }
    
    // bump variables for next call into readStream
    pstream->_read_remaining -= returnedElems;

    // return number of elements written to buff0
    if (pstream->_read_remaining != 0)
    {
        flags |= SOAPY_SDR_MORE_FRAGMENTS;
    }
    else
    {
        this->releaseReadBuffer(stream, unused);
    }
    return (int)returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapySDRPlay::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
  const auto * pstream = ((StreamData*) stream);
    return pstream->_streamblocks.size() / pstream->_channels.size();
}

int SoapySDRPlay::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    auto * pstream = (StreamData *) stream;

  for(size_t i=0; i<pstream->_channels.size(); ++i){
    buffs[i] = (void *)pstream->_cur_read_block[i]->data();
  }
  return 0;
}

int SoapySDRPlay::acquireReadBuffer(SoapySDR::Stream *stream,
                                    size_t &handle,
                                    const void **buffs,
                                    int &flags,
                                    long long &timeNs,
                                    const long timeoutUs)
{
    auto * pstream = (StreamData *) stream;
    if (resetBuffer || pstream->_overflow_event){
      // drain queue
      StreamBlock * item;
      for(auto & q : pstream->_queues){
        while(q.try_dequeue(item)){
          assert(item != nullptr);
          if(!pstream->_pool.try_enqueue(item))
            SoapySDR_log(SOAPY_SDR_WARNING, "failed to re-insert block");
        }
      }
      pstream->_overflow_event = false;
      if(resetBuffer){
        resetBuffer = false;
      } else {
        SoapySDR_log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
      }

    }
    // extract nchan blocks
    size_t nchan = pstream->_channels.size();
    for(size_t i=0; i<nchan; ++i){
      // this might lead to a timeout which is nchan * timeout long
      // but otherwise many timeouts happen
      if(!pstream->_queues[i].wait_dequeue_timed(pstream->_cur_read_block[i], std::chrono::microseconds(timeoutUs))){
        SoapySDR_logf(SOAPY_SDR_WARNING, "Timeout on %d", i);
        return SOAPY_SDR_TIMEOUT;
      }
      buffs[i] = pstream->_cur_read_block[i];
      assert(buffs[i] != nullptr);
    }
    auto size = pstream->_cur_read_block[0]->size();
    pstream->_read_remaining = size;
    for(size_t i=0;i<nchan; ++i){
      if(size != pstream->_cur_read_block[i]->size()){
        SoapySDR_log(SOAPY_SDR_WARNING, "blocks are not of equal size");
      }
    }

    // return number available
    return size;
}

void SoapySDRPlay::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle)
{
    auto * pstream = (StreamData *) stream;
    auto & block = pstream->_cur_read_block;
    for(auto b : block){
      if(b == nullptr) continue;
      if(!pstream->_pool.try_enqueue(b)){
        throw std::runtime_error("cannot re-insert block to object pool");
      }
      b = nullptr;
    }
}


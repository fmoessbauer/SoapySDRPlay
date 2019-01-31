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

static void _rx_callback(
  short *xi,
  short *xq,
  sdrplay_api_StreamCbParamsT *params,
  unsigned int numSamples,
  unsigned int reset,
  void *cbContext)
{
#if 1
    StreamData *pstream = (StreamData *)cbContext;
    return SoapySDRPlay::rx_callback(pstream, xi, xq, numSamples);
#endif
}

static void _gr_callback(
  sdrplay_api_EventT eventId,
  sdrplay_api_TunerSelectT tuner,
  sdrplay_api_EventParamsT *params,
  void *cbContext)
{
    StreamData *pstream = (StreamData *)cbContext;
    //return self->gr_callback(gRdB, lnaGRdB); // TODO
}

void SoapySDRPlay::rx_callback(StreamData * pstream, short *xi, short *xq, unsigned int numSamples)
{
  // retract block from pool
  // fill block
  // enqueue

  // this should not be required as we always start a new block
  int spaceReqd = numSamples * pstream->_elements_per_sample * pstream->_shorts_per_word;
  StreamBlock * block = pstream->_cur_write_block;
  // check space in currently opened block
  // TODO: what is decm
  auto decM = 1;
//  if((block->_size + spaceReqd) >= (pstream->_buffer_size / decM)){
//    // release current block
//    if(!pstream->_queue.try_enqueue(block)){
//      // TODO this must not happen (as number of blocks is fixed)
//      throw std::runtime_error("try to add unavailable block");
//    }

    // fetch a new block
    if(!pstream->_pool.try_dequeue(block)){
      pstream->_overflow_event = true;
      return;
    }
//  }
  // TODO: find way to merge individual streams

  auto * beg = block->_buffers[0].data() + block->_size;
  block->_size += spaceReqd;

  //SoapySDR_log(SOAPY_SDR_INFO, "test");
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
  // release block
  if(!pstream->_queue.try_enqueue(block)){
    throw std::runtime_error("try to add unavailable block");
  }
  return;
}

void SoapySDRPlay::gr_callback(StreamData * pstream, unsigned int gRdB, unsigned int lnaGRdB)
{
#if 0
    //Beware, lnaGRdB is really the LNA GR, NOT the LNA state !
 
    sdrplay_api_GainValuesT gainVals;

    mir_sdr_GetCurrentGain(&gainVals);


    if (gRdB < 200)
    {
        current_gRdB = gRdB;
    }

    if (gRdB < mir_sdr_GAIN_MESSAGE_START_ID)
    {
        // gainVals.curr is a calibrated gain value
    }
    else if (gRdB == mir_sdr_ADC_OVERLOAD_DETECTED)
    {
        mir_sdr_GainChangeCallbackMessageReceived();
        // OVERLOAD DECTECTED
    }
    else
    {
        mir_sdr_GainChangeCallbackMessageReceived();
        // OVERLOAD CORRECTED
    }
#endif
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

    StreamData * pstream = new StreamData(DEFAULT_NUM_BUFFERS, DEFAULT_BUFFER_LENGTH, shortsPerWord, channels);
    if(!pstream->_pool.try_dequeue(pstream->_cur_write_block)){
      throw std::runtime_error("no block in object pool");
    }
    assert(pstream->_cur_write_block->_buffers[0].size() > 0);
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
    cbFns.StreamACbFn = _rx_callback;
    cbFns.StreamBCbFn = NULL;
    cbFns.EventCbFn = _gr_callback;

    err = sdrplay_api_Init(dev, &cbFns, (void*)stream);

//    err = mir_sdr_StreamInit(&gRdB, sampleRate / 1e6, centerFrequency / 1e6, bwMode,
//                             ifMode, lnaState, &gRdBsystem, mir_sdr_USE_RSP_SET_GR, &sps,
//                             _rx_callback, _gr_callback, (void *)this);

    if (err != sdrplay_api_Success)
    {
       //throw std::runtime_error("StreamInit Error: " + std::to_string(err));
       return SOAPY_SDR_NOT_SUPPORTED;
    }
    // TODO: on which channel / tuner?
    //mir_sdr_DecimateControl(decEnable, decM, 1);

    //mir_sdr_SetDcMode(4,0);
    //mir_sdr_SetDcTrackTime(63);
    
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
    if (pstream->_read_remaining == 0)
    {
        // TODO
        size_t unused = 0;
        int ret = this->acquireReadBuffer(stream, unused, (const void **)&source_buffs, flags, timeNs, timeoutUs);

        if (ret < 0)
        {
            return ret;
        }
        pstream->_read_remaining = ret;
    } else {
      assert(nullptr != pstream->_cur_read_block);
      for(size_t i=0; i<pstream->_channels.size(); ++i){
        source_buffs[i] = pstream->_cur_read_block->_buffers[i].data();
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
        // todo
        this->releaseReadBuffer(stream, (size_t)pstream->_cur_read_block);
    }
    return (int)returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapySDRPlay::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return ((StreamData*) stream)->_streamblocks.size();
}

int SoapySDRPlay::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    auto * pstream = (StreamData *) stream;

  for(size_t i=0; i<pstream->_channels.size(); ++i){
    buffs[i] = (void *)pstream->_streamblocks[handle]._buffers[i].data();
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
      while(pstream->_queue.try_dequeue(item)){
        pstream->_pool.try_enqueue(item);
      }
      pstream->_overflow_event = false;
      if(resetBuffer){
        resetBuffer = false;
      } else {
        SoapySDR_log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
      }

    }
    StreamBlock * read_block = nullptr;
    if(!pstream->_queue.wait_dequeue_timed(read_block, std::chrono::microseconds(timeoutUs))){
      SoapySDR_log(SOAPY_SDR_WARNING, "Timeout");
      return SOAPY_SDR_TIMEOUT;
    }
    pstream->_cur_read_block = read_block;
    pstream->_read_remaining = read_block->_size;
    for(size_t i=0;i<pstream->_channels.size(); ++i){
      buffs[i] = read_block->_buffers[i].data();
    }

    // return number available
    return read_block->_size;
}

void SoapySDRPlay::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle)
{
    auto * pstream = (StreamData *) stream;
    auto * block = pstream->_cur_read_block;
    block->_size = 0;
    if(!pstream->_pool.try_enqueue(block)){
      throw std::runtime_error("cannot re-insert block to object pool");
    }
    pstream->_cur_read_block = nullptr;
}


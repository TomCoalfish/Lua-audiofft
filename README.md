# audiofft-
audiofft and convolution simple wrapper

# swig
* need to modify std::vector
* add T* data() to the interface

# a kind of document

<pre>
#include "AudioFFT.h"

void Example()
{
  const size_t fftSize = 1024; // Needs to be power of 2!
  std::vector<float> input(fftSize, 0.0f);
  std::vector<float> re(audiofft::AudioFFT::ComplexSize(fftSize));
  std::vector<float> im(audiofft::AudioFFT::ComplexSize(fftSize));
  std::vector<float> output(fftSize);
  audiofft::AudioFFT fft;
  fft.init(1024);
  fft.fft(input.data(), re.data(), im.data());
  fft.ifft(output.data(), re.data(), im.data());
}
</pre>

<pre>
require('audiofft')
input = audiofft.float_vector(1024,0.0f)
re    = audiofft.float_vector(audiofft.AudioFFT.ComplexSize(1024))
im    = audiofft.float_vector(audiofft.AudioFFT.ComplexSize(1024))
output = audiofft.float_vector(1024)
fft    = audiofft.AudioFFT() 
fft:init(1024)
fft:fft(input.data(),re.data(),im.data())
fft:ifft(output.data(), re.data(), im.data())
</pre>

<pre>
namespace audiofft
{
  class AudioFFT
  {
  public:

  AudioFFT();
  ~AudioFFT();

  void init(size_t size);
  void fft(const float* data, float* re, float* im);
  void ifft(float* data, const float* re, const float* im);
  static size_t ComplexSize(size_t size);
  }
 }
</pre>

<pre>
namespace fftconvolver
{
  class FFTConvolver
  {  
  public:
    FFTConvolver();  
    virtual ~FFTConvolver();

    /**
    * @brief Initializes the convolver
    * @param blockSize Block size internally used by the convolver (partition size)
    * @param ir The impulse response
    * @param irLen Length of the impulse response
    * @return true: Success - false: Failed
    */
    bool init(size_t blockSize, const Sample* ir, size_t irLen);

    /**
    * @brief Convolves the the given input samples and immediately outputs the result
    * @param input The input samples
    * @param output The convolution result
    * @param len Number of input/output samples
    */
    void process(const Sample* input, Sample* output, size_t len);

    /**
    * @brief Resets the convolver and discards the set impulse response
    */
    void reset();
  };  
} // End of namespace fftconvolver
</pre>

<pre>
namespace fftconvolver
{ 

/**
* @class TwoStageFFTConvolver
* @brief FFT convolver using two different block sizes
*
* The 2-stage convolver consists internally of two convolvers:
*
* - A head convolver, which processes the only the begin of the impulse response.
*
* - A tail convolver, which processes the rest and major amount of the impulse response.
*
* Using a short block size for the head convolver and a long block size for
* the tail convolver results in much less CPU usage, while keeping the
* calculation time of each processing call short.
*
* Furthermore, this convolver class provides virtual methods which provide the
* possibility to move the tail convolution into the background (e.g. by using
* multithreading, see startBackgroundProcessing()/waitForBackgroundProcessing()).
*
* As well as the basic FFTConvolver class, the 2-stage convolver is suitable
* for real-time processing which means that no "unpredictable" operations like
* allocations, locking, API calls, etc. are performed during processing (all
* necessary allocations and preparations take place during initialization).
*/
class TwoStageFFTConvolver
{  
public:
  TwoStageFFTConvolver();  
  virtual ~TwoStageFFTConvolver();
  
  /**
  * @brief Initialization the convolver
  * @param headBlockSize The head block size
  * @param tailBlockSize the tail block size
  * @param ir The impulse response
  * @param irLen Length of the impulse response in samples
  * @return true: Success - false: Failed
  */
  bool init(size_t headBlockSize, size_t tailBlockSize, const Sample* ir, size_t irLen);

  /**
  * @brief Convolves the the given input samples and immediately outputs the result
  * @param input The input samples
  * @param output The convolution result
  * @param len Number of input/output samples
  */
  void process(const Sample* input, Sample* output, size_t len);

  /**
  * @brief Resets the convolver and discards the set impulse response
  */
  void reset();
  
};
  
} // End of namespace fftconvolver
</pre>

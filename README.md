# audiofft
audiofft and convolution simple wrapper

```lua
require('audiofft')
input = audiofft.float_vector(1024,0.0)
re    = audiofft.float_vector(audiofft.AudioFFT.ComplexSize(1024))
im    = audiofft.float_vector(audiofft.AudioFFT.ComplexSize(1024))
output = audiofft.float_vector(1024)
fft    = audiofft.AudioFFT() 
fft:init(1024)
fft:fft(input:data(),re:data(),im:data())
fft:ifft(output:data(),re:data(), im:data())
```

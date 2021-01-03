import numpy as np
import matplotlib.pyplot as plt
import math
import timeit
from scipy.fft import fft, fftfreq, rfft, rfftfreq, irfft

_tstart_stack = []
def tic():
    _tstart_stack.append(timeit.default_timer())

def toc(fmt="Elapsed: %s s"):
    print(fmt % (timeit.default_timer() - _tstart_stack.pop()))
    

noisy_signal = np.random.normal(0,20,1000)
t = np.linspace(0,10,1000)

figure,axs = plt.subplots(7,1)
axs[0].plot(t,noisy_signal)

template_signal = np.zeros(1000)
template_signal[0:100] = 200*np.sin(np.linspace(0,2*math.pi,100))

peak_signal = 500*np.sin(2*np.linspace(0,math.pi,50))
noisy_signal[600:650] = peak_signal + noisy_signal[600:650]


cor_signal = np.correlate(noisy_signal,peak_signal,'same')
tic()
conv_signal = np.convolve(noisy_signal,peak_signal,'same')
toc()

tic()
f_noisy_signal = rfft(noisy_signal)
f_peak_signal = rfft(peak_signal)
f_peak_padded = np.zeros(len(f_noisy_signal))
f_peak_padded[0:len(f_peak_signal)] = f_peak_signal
f_conv = irfft(f_noisy_signal*f_peak_padded)
toc()


axs[1].plot(t[0:50],peak_signal)
axs[2].plot(t,noisy_signal)
axs[3].plot(cor_signal)
axs[4].plot(conv_signal)
axs[5].plot(f_conv)
plt.show()
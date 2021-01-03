import serial
#import timeit
import time
import timeit
import numpy as np
#import sys
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, rfft, rfftfreq
#np.set_printoptions(threshold=sys.maxsize)

_tstart_stack = []
def tic():
    _tstart_stack.append(timeit.default_timer())

def toc(fmt="Elapsed: %s s"):
    print(fmt % (timeit.default_timer() - _tstart_stack.pop()))
    

# Variables init
N = 5000
i = 0
A = np.zeros((N,3), dtype=int)
print('setup init...')
# Serial_com init, arduino reset
ser = serial.Serial('COM8', 115200)
ser.flushInput()
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)

while i < N:
    data = ser.read(6)
    A[i,0] = int.from_bytes(data[0:2],'big', signed=True)
    A[i,1] = int.from_bytes(data[2:4],'big', signed=True)
    A[i,2] = int.from_bytes(data[4:6],'big', signed=True)
    i+=1

# -------------------------------------------------
# Closing the serial port communication

ser.flushInput()
ser.close()
#toc = timeit.default_timer()

# -------------------------------------------------
# RFFT algorithm, freq analysis
Axf = rfft(A[2:,0])
Ayf = rfft(A[2:,1])
Azf = rfft(A[2:,2])
xf = rfftfreq(N-2, 1/1000)
# -------------------------------------------------
# Data Plotting

print('printing data...')

# Acceleration data plot
fig, axs = plt.subplots(3,2)
axs[0, 0].plot(A[2:,0])
axs[0, 0].set_title('Accel_X')
axs[1, 0].plot(A[2:,1])
axs[1, 0].set_title('Accel_Y')
axs[2, 0].plot(A[2:,2])
axs[2, 0].set_title('Accel_Z')

# FFT data plot
axs[0, 1].plot(xf, np.abs(Axf))
axs[0, 1].set_title('FFT_X')
axs[1, 1].plot(xf, np.abs(Ayf))
axs[1, 1].set_title('FFT_Y')
axs[2, 1].plot(xf, np.abs(Azf))
axs[2, 1].set_title('FFT_Z')



#np.savetxt("Punch.txt", A[2:,:])
plt.show()
#!/usr/bin/python


import sys
import time
import matplotlib.pyplot as plt
import numpy as np

from rpc import SteveControl, UsbRpcConnection

conn = UsbRpcConnection()
ctl = SteveControl( conn )

resp = ctl.measure_servo_response(2000,1500,200)

plt.subplot(2,1,1)
plt.plot(resp['time'],resp['setpoint'],label="Setpoint")
plt.plot(resp['time'],resp['error'],label="Position")
plt.plot(resp['time'],resp['drive'],label="Drive")
plt.legend(framealpha=0.5)
plt.subplot(2,1,2)
#np.abs(fftshift(A))
A = np.log10(np.abs(np.fft.fftshift(np.fft.fft(resp['error']))))
freq = np.linspace(-21341/2, 21341/2, len(A))
plt.plot(freq,A,label="FFT(Position)")
plt.legend(framealpha=0.5)

plt.show()

while True:
    s=conn.console_read()
    if s!=None:
        sys.stdout.write(s)
    time.sleep(0.01);

#conn.close()
#sys.exit(0)

n_samples = 8192
t_samples = ctl.test_adc_acquisition(n_samples, 16)

samplerate = (21341 * 480 / 15)



t_t = numpy.linspace(0, 100000.0/samplerate * n_samples, n_samples)

print(t_t)
plt.plot(t_t,t_samples,label="Samples")
#plt.plot(t_t,err_t,label="error")
#plt.plot(t_t,y_t,label="drive")
plt.legend(framealpha=0.5)
plt.show()

conn.close()
#!/usr/bin/python


import sys
import time
from rpc import SteveControl, UsbRpcConnection

conn = UsbRpcConnection()
ctl = SteveControl( conn )

#ctl.esc_set_speed(10.0)

#while True:
#    s=conn.console_read()
#    if s!=None:
#        sys.stdout.write(s)
#    time.sleep(0.01);

#conn.close()
#sys.exit(0)

n_samples = 8192
t_samples = ctl.test_adc_acquisition(n_samples, 16)

samplerate = (21341 * 480 / 15)

import matplotlib.pyplot as plt
import numpy

t_t = numpy.linspace(0, 100000.0/samplerate * n_samples, n_samples)

print(t_t)
plt.plot(t_t,t_samples,label="Samples")
#plt.plot(t_t,err_t,label="error")
#plt.plot(t_t,y_t,label="drive")
plt.legend(framealpha=0.5)
plt.show()

conn.close()
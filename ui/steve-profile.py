#!/usr/bin/python


import sys
import time
import matplotlib.pyplot as plt
import numpy

from rpc import SteveControl, UsbRpcConnection

conn = UsbRpcConnection()
ctl = SteveControl( conn )

radial_steps = 20
radial_count = 65
samples_per_step = 100

ctl.ldrive_go_home()

while ctl.ldrive_check_idle() == False:
    time.sleep(0.3)

f_profile=open("height.txt","wb");

for r in range(0, radial_count):
    print("Radius %d" % r)
    prof=ctl.profile_height(3400, samples_per_step);

    f_profile.write("r %d step %d samples %d\n" % (r, radial_steps, samples_per_step))
    print(prof)
    for i in range(0, samples_per_step):
        print(i)
        f_profile.write("%d %d\n" % (prof['rho'][i], prof['h'][i]))

    ctl.ldrive_step( -radial_steps )
    while ctl.ldrive_check_idle() == False:
        time.sleep(0.3)

f_profile.close()

#for p in prof:
#    print(p)
#while True:
    #s=conn.console_read()
    #if s!=None:
        #sys.stdout.write(s)
    #time.sleep(0.01);

#conn.close()
#sys.exit(0)

#print(prof['rho'])
#plt.plot(prof['rho'], prof['h'],label="Height profile vs radial position")
#plt.legend(framealpha=0.5)
#plt.show()


#n_samples = 8192
#t_samples = ctl.test_adc_acquisition(n_samples, 16)

#samplerate = (21341 * 480 / 15)



#t_t = numpy.linspace(0, 100000.0/samplerate * n_samples, n_samples)

#print(t_t)
#plt.plot(t_t,t_samples,label="Samples")
#plt.plot(t_t,err_t,label="error")
#plt.plot(t_t,y_t,label="drive")
#plt.legend(framealpha=0.5)
#plt.show()

conn.close()
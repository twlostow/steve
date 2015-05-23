#!/usr/bin/python

import struct
import array
import signal
import sys
import time
import serial
import threading
import Queue
import numpy as np
from scipy.interpolate import griddata

class UsbRpcMessage:
    def __init__(self,id,data):
        self.id=id
        self.data=data

    def size(self):
        return len(self.data)

class UsbRpcReadoutThread(threading.Thread):
    def __init__(self,conn_):
        super(UsbRpcReadoutThread, self).__init__()
        self.conn=conn_
        self.running=False

    def run(self):
        self.running=True
        s=""
        while self.running:
            c=self.conn.ser.read(1)
            if(c == chr(0x01)):
                header = self.conn.ser.read(4);

                rsp_size, rsp_id = struct.unpack(">HH", header)
#                print("Got response %x %x" % (rsp_size, rsp_id))
                rsp_data = self.conn.ser.read(rsp_size)
                rsp = UsbRpcMessage(rsp_id, rsp_data)
                self.conn.response_queue.put(rsp)
            else:
                s += c
                if c == '\n':
                    self.conn.console_queue.put(s)
                    s=""

    def stop(self):
        self.running = False


class UsbRpcConnection:

    def __init__(self,dev_="/dev/ttyUSB1", baudrate_=115200):
        self.ser = serial.Serial(port=dev_,baudrate=baudrate_,timeout=0.1,rtscts=False)
        self.console_queue = Queue.Queue()
        self.readout_thread = UsbRpcReadoutThread(self)
        self.readout_thread.start()
        self.response_queue = Queue.Queue()
        signal.signal(signal.SIGINT, self._handler)

    def send(self,rq):
        self.ser.write(chr(0x01))
        #print("Send %d bytes" % rq.size())
        self.ser.write(chr((rq.size() >> 8) & 0xff))
        self.ser.write(chr((rq.size() >> 0) & 0xff))
        self.ser.write(chr((rq.id >> 8) & 0xff))
        self.ser.write(chr((rq.id >> 0) & 0xff))
        self.ser.write(rq.data)
        pass

    def receive(self):
        return self.response_queue.get()

    def request(self,rq):
        self.send(rq)
        return self.response_queue.get()

    def console_read(self):
        return None if self.console_queue.empty() else self.console_queue.get()

    def _handler(self,signum, frame):
        sys.stderr.write("[rpc] closing connection...\n")
        self.close()
        sys.exit(0)

    def close(self):
        self.readout_thread.stop()
        self.readout_thread.join()

class SteveHeightmap:
    def __init__(self, filename):
        lines=open(filename,"rb").readlines()
        n=0
        points = []
        values = []

        while n < len(lines):
            tok = lines[n].split()
            n+=1
            if(tok[0] == 'r'):
                samples = int(tok[5])
                r_idx = int(tok[1])
                r_step = int(tok[3])
                r = r_idx * r_step
        #   print(samples)
                for i in range(0,samples):
                    tok = lines[n].split()
                    rho = float(tok[0]) / 100
                    h = float(tok[1])
                    points.append((r, rho))
                    values.append(h)
                    points.append((r, rho + 360))
                    values.append(h)
                    points.append((r, rho - 360))
                    values.append(h)
#       print(rho, h)
                    n+=1


        #print(points)
        #print(values)
        self.grid_r, self.grid_rho = np.mgrid[0:1280:1281j,0:359:360j]
        self.grid_z = griddata(points, values, (self.grid_r, self.grid_rho), method='linear')

    def map_for_ring(self, r):
        m=[]
        for a in range(0,360,2):
            m.append(int(self.grid_z[r][a]))
        #print(m)
        return m

class SteveControl:
    ID_ADC_TEST = 0x1
    ID_ESC_SET_SPEED = 0x2
    ID_ESC_GET_SPEED = 0x3
    ID_SERVO_RESPONSE = 0x4
    ID_PROFILE_HEIGHT = 0x5
    ID_LDRIVE_STEP = 0x6
    ID_LDRIVE_READ_ENCODER = 0x7
    ID_LDRIVE_CHECK_IDLE = 0x8
    ID_LDRIVE_GO_HOME = 0x9
    ID_SERVO_SET_HEIGHTMAP = 0xa
    ID_SERVO_ENABLE_HEIGHTMAP = 0xb
    ID_ETCH_SINGLE_SPOT = 0xc


    def __init__(self, conn):
        self.conn = conn

    def test_adc_acquisition(self, n_samples, n_avg = 1):
        self.conn.send( UsbRpcMessage( self.ID_ADC_TEST, struct.pack(">II", n_samples, n_avg) ) )
        rsp = self.conn.receive()
        return struct.unpack("<%dH" % n_samples, rsp.data)

    def esc_set_speed(self, speed):
        self.conn.send( UsbRpcMessage( self.ID_ESC_SET_SPEED, struct.pack(">I", int(speed * 1000.0)) ) )

    def esc_get_speed(self, speed):
        self.conn.send( UsbRpcMessage( self.ID_ESC_GET_SPEED ) )
        rsp = self.conn.receive()
        d = struct.unpack("<I", rsp.data)
        return float(d) / 1000.0

    def measure_servo_response(self, init_setpoint, target_setpoint,dvdt=1):
        self.conn.send( UsbRpcMessage( self.ID_SERVO_RESPONSE, struct.pack(">III", init_setpoint, target_setpoint, dvdt) ) )
        rsp = self.conn.receive()
        n_samples = rsp.size() / 6
        rv = {'time' : [], 'setpoint':[], 'error' : [], 'drive': []}
        samplerate = (21341)

        import numpy

        t_t = numpy.linspace(0, 1000.0/samplerate * n_samples, n_samples)

        #print("rsp size", rsp.size())
        for i in range(0, n_samples):
            s,e,d=struct.unpack("<hhh", rsp.data[i*6:(i+1)*6])
            #print(s,e,d)
            rv['time'].append(t_t[i])
            rv['setpoint'].append(s)
            rv['error'].append(e)
            rv['drive'].append(d)
        return rv

    def profile_height(self, init_setpoint, n_points):
        self.conn.send( UsbRpcMessage( self.ID_PROFILE_HEIGHT, struct.pack(">II", init_setpoint, n_points) ) )
        rsp = self.conn.receive()

        print("ProfH size %d" % rsp.size())

        n_samples = rsp.size() / 8
        rv = []

        for i in range(0, n_samples):
            h,r=struct.unpack("<II", rsp.data[i*8:(i+1)*8])
            rv.append((r,h))
            print(r,h)

        rv3 = sorted(rv, key=lambda tup: tup[0])

        rv2 = {'rho' : [], 'h':[] }
        for s in rv3:
            rv2['rho'].append(s[0])
            rv2['h'].append(s[1])

        return rv2

    def ldrive_step(self, dir):
        self.conn.send( UsbRpcMessage( self.ID_LDRIVE_STEP, struct.pack(">i", dir) ) )

    def ldrive_read_encoder(self):
        self.conn.send( UsbRpcMessage( self.ID_LDRIVE_READ_ENCODER, struct.pack(">i", 1) ) )
        rsp = self.conn.receive()
        i,q=struct.unpack("<ii", rsp.data)
        return i,q

    def ldrive_check_idle(self):
        self.conn.send( UsbRpcMessage( self.ID_LDRIVE_CHECK_IDLE, struct.pack(">i", 1) ) )
        rsp = self.conn.receive()
        idle = struct.unpack("<i", rsp.data)
        return idle[0]

    def ldrive_go_home(self):
        self.conn.send( UsbRpcMessage( self.ID_LDRIVE_GO_HOME, struct.pack(">i", 1) ) )

    def set_heightmap(self, hmap):
        print(hmap)
        msg =  UsbRpcMessage( self.ID_SERVO_SET_HEIGHTMAP, struct.pack(">180I", *hmap) )
        #print('HMAPS:', msg.size())
        self.conn.send(msg)
        #pass

    def enable_heightmap(self, enable, height):
        self.conn.send( UsbRpcMessage( self.ID_SERVO_ENABLE_HEIGHTMAP, struct.pack(">II", enable, height) ) )

    def etch_single_spot(self):
        self.conn.send( UsbRpcMessage( self.ID_ETCH_SINGLE_SPOT, struct.pack(">i", 1) ) )

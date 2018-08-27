#!/usr/bin/python
# -*- coding: utf-8 -*-

from mbedrpc import *
#from PID import *
import threading, thread
import time
from serial import SerialException

class Motor:
    def __init__(self, a1, a2):
        self.a1=a1
        self.a2=a2

    def cmd(self, speed):
        if speed >=0:
            self.a1.write(speed)
            self.a2.write(0)
        else:
            self.a1.write(0)
            self.a2.write(-speed)

class Zumy:
    def __init__(self, dev='/dev/ttyACM0'):
        self.mbed=SerialRPC(dev, 115200)
        a1=PwmOut(self.mbed, p21)
        a2=PwmOut(self.mbed, p22)
        b1=PwmOut(self.mbed, p23)
        b2=PwmOut(self.mbed, p24)

        #Setting motor PWM frequency
        pwm_freq = 50.0
        a1.period(1/pwm_freq)
        a2.period(1/pwm_freq)
        b1.period(1/pwm_freq)
        b2.period(1/pwm_freq)
        
        self.m_right = Motor(a1, a2)
        self.m_left = Motor(b1, b2)
        self.an = AnalogIn(self.mbed, p20)

        self.data = []

	self.imu_data = RPCFunction(self.mbed, "gid")
	self.enc_data = RPCFunction(self.mbed, "ged")
	self.rst = RPCFunction(self.mbed, "rst")
	#self.wd_init = RPCFunction(self.mbed, "wdinit")
        #self.wd_init.run("")

	self.rlock=threading.Lock()

        # Data update thread
        self.init_data()
        #data_thread = threading.Thread(target=self.update_data)
        #data_thread.start()
        thread.start_new_thread(self.update_data, ())

    def init_data(self):
        while(not self.data):
           time.sleep(0.01) # update data at 100Hz to not starve the main thread
           try:
              self.rlock.acquire()
              self.data = self.enc_data.run("").split(',') + self.imu_data.run("").split(',')
              self.rlock.release()
           except SerialException:
              print "serial exception in update_data!"
              pass



    def update_data(self):
        while(1):
           time.sleep(0.01) # update data at 100Hz to not starve the main thread
           try:
              self.rlock.acquire()
              self.data = self.enc_data.run("").split(',') + self.imu_data.run("").split(',') 
              self.rlock.release()
           except SerialException:
              print "serial exception in update_data!"
              pass

    def get_data(self):
        self.rlock.acquire()
        data = self.data
        self.rlock.release()
        return data

    def read_data(self):
        print self.get_data()
        print "-------------------"

    def reset(self):
        self.rst.run("")

    def cmd(self, left, right):
        self.rlock.acquire()
        self.m_left.cmd(left)
        self.m_right.cmd(right)
        self.rlock.release()
 
    def read_voltage(self):
        self.rlock.acquire()
        try:
          ain=self.an.read()*3.3
        except SerialException:
          pass
        self.rlock.release()
        volt=ain*(4.99+15.8) / 4.99
        return volt

    def read_enc(self):
      data = self.get_data()
      if len(data) == 8:
          # Get last two elements from data list and r_enc is the first element
          rval = data[:2]
          rval = [int(rval[1]), int(rval[0])]
      else:
          print "fail to read encoder data"
          rval = []
      return rval

    def read_imu(self):
      data = self.get_data()
      if len(data) == 8:
          # Get data list but the last two elements, and convert to float
          rval = self.data[2:]
          rval = [float(i) for i in rval]
      else:
          print "fail to read imu data"
          rval = []
      return rval

if __name__ == '__main__':
    z=Zumy()
  #  z.cmd(0.1,0.1)

    time.sleep(0.1)
    z.cmd(0,0)
    z.reset() 


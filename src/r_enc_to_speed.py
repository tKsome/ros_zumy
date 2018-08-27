#!/usr/bin/python
import roslib
import sys
import rospy
from std_msgs.msg import String,Int16,Float64
from threading import Condition

import socket,time
#last_enc = 0 
#l_encv = Float64()
#trans_factor = 1167
#current_time = time.time()
#last_time = current_time
#class l_encTospeed:
class rspeed:
  def __init__(self):
    self.last_enc = 0
   # global last_enc
    self.r_encv = Float64()
    rospy.init_node('enc_TO_speed', anonymous = True)
    #12 ticks/encoder_back_shaft_rotation * (100.37encoder_back_shaft_rotation /1 output_shaft_rotation) * 1 output_shaft_rotation / (circunfrence = pi * d = 3.15 inches * 0.0254 meters/inch) = 12 * 100.37 /(pi * 1.5 * 0.0254) = 5012.16 ticks/meter
    self.trans_factor = 5012.16 #ticks/meter
    #global trans_factor
    self.lock = Condition()
    self.rate = rospy.Rate(70.0)
    #global rate
    self.name = socket.gethostname()
    self.current_time = time.time()
   # global current_time
    self.last_time = self.current_time
    #global last_time
    #global l_encv
    rospy.Subscriber('/r_enc', Int16, self.enc_to_speed, queue_size = 10)
    self.r_encspeed = rospy.Publisher('/r_speed', Float64, queue_size=10)
    #while not rospy.is_shutdown():
    #    l_encspeed.publish(l_encv)
    #    rate.sleep()
    #rospy.spin()
  def enc_to_speed(self, msg):
    #self.l_encv = Float64()
    new_enc = msg.data
   # global last_enc
   # global last_time
   # global current_time
    diff = new_enc - self.last_enc
    meter = float(diff /self.trans_factor)
    self.current_time = time.time()
    delta_time = self.current_time - self.last_time
    rv = float(meter * delta_time)
    self.lock.acquire()
    self.last_enc = new_enc
    #rospy.lock.release()
    self.last_time = self.current_time
    self.lock.release()
    self.r_encv.data = rv
   # l_encspeed.publish(l_encv)
   # rate.sleep()
   # global rate
   # global l_encv

  def run(self):
    while not rospy.is_shutdown():
      enc_msg = Float64()
      self.lock.acquire()
      enc_msg.data = self.r_encv.data
      self.r_encspeed.publish(enc_msg)
      self.lock.release()
      self.rate.sleep()


if __name__ == '__main__':
  zlv = rspeed()
  zlv.run()

#!/usr/bin/env python
#import matplotlib
#matplotlib.use('Agg')
import numpy as np
import rospy
import roslib
#from zumy import Zumy
import socket, time
import pylab
#import matplotlib
from geometry_msgs.msg import Twist
from threading import Condition
from std_msgs.msg import Int16,Float64
from ros_zumy.msg import kalman

#matplotlib.use('Agg')
class Kalman:   #kalman filter for the message of IMU
  def __init__(self):
    #self.zumy = Zumy()
    self.x_last = 0.0 # the best value of x last time for angular_vel
    self.p_last = 0.03 # the best value of x`covarience last time
    self.Q = 0.001 # process noise
    self.R = 0.005 # measure noise 
    self.kg = 0.0 # the parameter of kalman
    self.x_mid = 0.0 # middle of x
    self.x_now = 0.0 # x now!
    self.p_mid = 0.0 # middle of x`covarience 
    self.p_now = 0.0 
    self.z_real = Float64() # the value you make
    self.z_measure = Float64() # the value sensor measure
    self.sumerror_kalman = 0.0 # the total of error of kalman`parameter
    self.sumerror_measure = 0.0 # total of error of measure
    self.x_last = self.z_real
    self.x_mid = self.x_last
#-----------------------------------------------------------------------
    self.xx_lase = 0.0 # for linear_vel
    self.pp_last = 0.0001 # the best value of x`covarience last time
    self.QQ = 0.0001 # process noise
    self.RR = 0.001496 # measure noise 
    self.kgkg = 0.0 # the parameter of kalman
    self.xx_mid = 0.0 # middle of x
    self.xx_now = 0.0 # x now!
    self.pp_mid = 0.0 # middle of x`covarience 
    self.pp_now = 0.0
    self.zz_real = Float64() # the value you make
    self.zz_measure = Float64() # the value sensor measure
    self.ssumerror_kalman = 0.0 # the total of error of kalman`parameter
    self.ssumerror_measure = 0.0 # total of error of measure
    self.xx_last = self.zz_real
    self.xx_mid = self.xx_last
#------------------------------------------------------------------------
    self.return_value = Float64()
    self.return_linear = Float64()
    # set the parameter of this system
    rospy.init_node('kalman_filter', anonymous = True)
    self.lock = Condition()
    self.rate = rospy.Rate(600.0)
    rospy.Subscriber('/kalman_param', kalman, self.kalman_go, queue_size = 5)
    self.kalman_returnmsg = rospy.Publisher('/cmd_vel',Twist,queue_size = 5)
    self.kalman_linear = rospy.Publisher('/kalman_linear',Float64,queue_size = 5) 
    self.kalman_angular = rospy.Publisher('/kelman_angular',Float64,queue_size = 5)
    

  def kalman_go(self,msg):
    self.z_real.data = msg.keyboard_angular
    self.lock.acquire()
    self.x_last = self.z_real.data
    self.x_mid = self.x_last
    self.p_mid = self.p_last + self.Q

    self.kg = self.p_mid / (self.p_mid + self.R)
    self.z_measure.data = msg.measure_angular
    #self.z_real.data = msg.keyboard_angular
    self.x_now = self.x_mid + self.kg * (self.z_measure.data - self.x_mid)
    self.p_now = (1 - self.kg)* self.p_mid
    self.p_last = self.p_now
    self.x_last = self.x_now
    self.return_value.data = self.x_now*1.1
   # self.return_linear.data = msg.linear_vel
    #self.sumerror_kalman +=
   # pylab.figure()
   # pylab.plot(self.z_measure,'k--',label='measurement of noisy')
   # pylab.plot(self.return_value,'b-',label='filtered value') 
   # pylab.axhline(self.z_real,color='g',label='truth value') 
   # pylab.legend()
   # pylab.xlabel('time')
   # pylab.ylabel('angular_vel')
    
   # pylab.show()
    self.lock.release()
   #_________________________________________________________________

    self.zz_real.data = msg.keyboard_linear
    self.lock.acquire()
    self.xx_last = self.zz_real.data
    self.xx_mid = self.xx_last
    self.pp_mid = self.pp_last + self.QQ

    self.kgkg = self.pp_mid / (self.pp_mid + self.RR)
    self.zz_measure.data = msg.linear_vel
    #self.z_real.data = msg.keyboard_angular
    self.xx_now = self.xx_mid + self.kgkg * (self.zz_measure.data - self.xx_mid)
    self.pp_now = (1 - self.kgkg)* self.pp_mid
    self.pp_last = self.pp_now
    self.xx_last = self.xx_now
    self.return_linear.data = self.xx_now*1.1    
    self.lock.release()


  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      twist_msg = Twist()
      twist_msg.linear.x = self.return_linear.data*1.1 
     # twist_msg.linear.x = self.zz_measure.data
      twist_msg.linear.y = 0
      twist_msg.linear.z = 0
      twist_msg.angular.x = 0
      twist_msg.angular.y = 0
      twist_msg.angular.z = self.return_value.data*1.1
     # twist_msg.angular.z = self.z_measure.data
     # self.lock.acquire()
      self.kalman_linear.publish(twist_msg.linear.x)
      self.kalman_angular.publish(twist_msg.angular.z)
      self.kalman_returnmsg.publish(twist_msg)
      self.lock.release()
      #pylab.figure()
      #pylab.plot(self.z_measure,'k--',label = 'measurement of noisy')
      #pylab.plot()
      self.rate.sleep()


if __name__ == '__main__':
  kalman_ojbk = Kalman()
  kalman_ojbk.run()


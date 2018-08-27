#!/usr/bin/env python
import roslib
import rospy
import numpy
import socket, time
from geometry_msgs.msg import Twist
from zumy import Zumy
from threading import Condition
from std_msgs.msg import String,Int16,Float64
class PID:#PID controller
    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.kd = D
  
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.zumy = Zumy()
        self.clear()
        self.setKp(0.5)
        self.setKi(0.4)
        self.setKd(0.0001)
        rospy.init_node('PID',anonymous= True)
        self.rate = rospy.Rate(70.0)
        self.lock = Condition()
        rospy.Subscriber('/cmd_vel1',Twist,self.cmd_callback,queue_size = 1)
        rospy.Subscriber('/PID_gethey',Float64,self.PID_go,queue_size =1)
        self.sudu_pub = rospy.Publisher('/vel',Float64,queue_size = 1)
        self.a = 0
       # self.clear()
    def cmd_callback(self,msg):
        self.v_set = msg.linear.x
        self.a = msg.angular.z
    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 5.0

        self.output = 0.0

    def PID_go(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value.data

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def run(self):
      while not rospy.is_shutdown():
        #self.lock.acquire()
        self.sudu_pub.publish(self.output)
        v = self.output
        aa = self.a
        r = v + 0.2*aa
        l = v - 0.2*aa
        self.lock.acquire()
        self.zumy.cmd(l,r)
        self.lock.release()
        self.rate.sleep()
    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time
if __name__ == '__main__':
  heihei = PID()
  heihei.run()

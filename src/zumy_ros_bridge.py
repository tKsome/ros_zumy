#!/usr/bin/python
import roslib
import rospy
import numpy
#import socket,time
#from PID import PID
#from PID_for_IMU import PID_another
#from l_enc_to_speed import l_encTospeed
from geometry_msgs.msg import Twist
from ros_zumy.msg import kalman
from threading import Condition
from zumy import Zumy
from std_msgs.msg import String,Header,Int16,Float64
from sensor_msgs.msg import Imu

import socket,time
enc_data_r = 0
enc_data_l = 0

class ZumyROS:	
  def __init__(self):
    self.zumy = Zumy()
   # self.PID = PID()
  #  self.PID_another = PID_another()
   # self.l_enc_v = l_encTospeed()
    rospy.init_node('zumy_ros')
    #self.cmd = (0,0)
    #self.rate = rospy.Rate(50.0)
    rospy.Subscriber('/cmd_vel1', Twist, self.cmd_callback,queue_size=1)
   # rospy.Subscriber('/vel',Float64,self.updata_vel,queue_size = 1)
   # rospy.Subscriber('/l_speed', Float64, self.l_enc_to_speed, queue_size = 10)
   # rospy.Subscriber('/r_speed', Float64, self.r_enc_to_speed, queue_size = 10)
   # rospy.Subscriber('/angular_return',Float64, self.kalman_callback, queue_size = 5)
    self.lock = Condition()
   # self.rate = rospy.Rate(1.0)
    self.name = socket.gethostname()
   # self.current_time = time.time()
    global enc_data_r
    global enc_data_l
   # global a
   # self.PID.clear()#set the arguments of PID 
   # self.PID.setKp(0.115)
   # self.PID.setKi(0.0006)
   # self.PID.setKd(0.0)
   # self.PID.setSampleTime(0.0001)
   # self.PID.setWindup(15)
   # self.PID_another.clear()
   # self.PID_another.setKp(0.45)
   # self.PID_another.setKi(0.035)
   # self.PID_another.setKd(0)
   # self.PID_another.setSampleTime(0.001)
  #  self.PID_another.setWindup(15)
    self.rate = rospy.Rate(70.0)
   # self.heartBeat = rospy.Publisher('heartBeat', String, queue_size=5)
    self.imu_pub = rospy.Publisher('Imu', Imu, queue_size = 1)
    self.r_enc_pub = rospy.Publisher('/r_enc', Int16, queue_size = 1)
    self.l_enc_pub = rospy.Publisher('/l_enc', Int16, queue_size = 1)
   # self.ly_twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   # self.PID_pub = rospy.Publisher('/PID_gethey',Float64,queue_size=1)
    self.kalman_pub = rospy.Publisher('/kalman_param', kalman, queue_size = 5)
    self.nonekalman_linear = rospy.Publisher('/nonekalman_linear',Float64,queue_size=5)
    self.nonekalman_angular = rospy.Publisher('/nonekalman_angular',Float64,queue_size=5)
    self.enc_angular = rospy.Publisher('/enc_angular',Float64,queue_size=5)
   # self.sudu_pub = rospy.Publisher('/vel',Float64,queue_size = 1)
    self.linear_pub = rospy.Publisher('/linear_vel',Float64,queue_size = 1)
    self.angular_pub = rospy.Publisher('/angular_vel',Float64,queue_size = 1)
    self.imu_count = 0
    self.a = 0
    self.v = 0
    self.enc_data_r = Int16()
    self.enc_data_l = Int16()
    self.enc_data_r.data = 0
    self.enc_data_l.data = 0
    self.hey = 0
   # global enc_data_r
   # global enc_data_l
   # global a
   # global radius
    ly_date = [enc_data_r , enc_data_l]
    global ly_date
    self.enc_r_last = 0
    self.enc_v_last = 0
    self.radius = 3.81 / 2
    # the robot`s wheel will make 1168 ticks/meter, and get the parament:1168*radius/100 = 0.212504
    self.param = 0.213
    #enc_data_r = 0
    #enc_data_l = 0
   # global enc_data_r
   # global enc_data_l
    numpy.save('ly_enc.npy',ly_date)
    self.current_time = time.time()
    self.last_time = self.current_time

  def kalman_callback(self,data):
    ll = data.data

 # def l_enc_to_speed(self, data):
   # print data.data,'l_vdata'
 # def r_enc_to_speed(self, data):
   # print data.data,'r_vdata'
#  def updata_vel(self,msg):
   # v = msg.data
   # r = v +0.2*self.a
   # l = (v/1.15) - 0.2*self.a
    #self.lock.acquire()
    #self.zumy.cmd(l,r)
    #self.lock.release()
  def cmd_callback(self, msg):
   # lv = 0.6#???
   # la = 0.4#???
   # v = msg.linear.x
   # a = msg.angular.z
   # vv = v / 2
   # self.PID.SetPoint = msg.linear.x
   # print self.PID.SetPoint
    
    self.v = msg.linear.x
    self.a = msg.angular.z
    v = self.v
   # global a
   # global radius
   # vv = v*2
   # self.linear_pub.publish(vv/10)
   # self.PID.setPoint(vv)
   # vvv = self.PID.setPoint(vv)
   # print vvv #the PID about linear_vel
    r = v + (self.param*self.a)
    l = (v/1.15) - (self.param*self.a)
    self.lock.acquire()
   # vv = v*2

   # self.PID.setPoint(vv)
   # self.PID_another.setPoint(self.a)
    # self.cmd = (l,r)
    self.zumy.cmd(l,r)
    print 'gogogogogogogogogogogogogogogo'
    self.lock.release()

  def enc_vel(self, rvv, lvv):
    self.lock.acquire()
    zumyv = ((rvv - lvv) / 2)
   # return zumyv
    self.lock.release()
    return zumyv

  def enc_rad(self, rvv, lvv):
    self.lock.acquire()
    zumyr = -((rvv + lvv) / 0.09) 
    #return zumyr
    self.lock.release()
    return zumyr
    
  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.current_time = time.time()
      self.last_tiem = self.current_time
     # self.zumy.cmd(0.1,0.1)
      # print imu and enc data
     # self.zumy.read_data()
      imu_data = self.zumy.read_imu()
      enc_data = self.zumy.read_enc()
     # imu_data = [0,0,0,0,0,0]
     # enc_data = [0,0]
      self.lock.release()
      twistImu = Twist()
      kalman_msg = kalman()
      if len(imu_data) == 6:     
          imu_msg = Imu()
          imu_msg.header = Header(self.imu_count,rospy.Time.now(),self.name)
          imu_msg.linear_acceleration.x = 9.81 * imu_data[0]
          imu_msg.linear_acceleration.y = 9.81 * imu_data[1]
          imu_msg.linear_acceleration.z = 9.81 * imu_data[2]
          imu_msg.angular_velocity.x = 3.14 / 180.0 * imu_data[3]
          imu_msg.angular_velocity.y = 3.14 / 180.0 * imu_data[4]
          imu_msg.angular_velocity.z = 3.14 / 180.0 * imu_data[5]
          #twistImu.angular.x = 0
          #twistImu.angular.y = 0
          #twistImu.angular.z = imu_msg.angular_velocity.z
          self.imu_pub.publish(imu_msg)
         # kalman_msg = kalman()
          kalman_msg.keyboard_linear = self.v
          kalman_msg.keyboard_angular = -self.a*3.14
          vvv = self.v
          aaa = self.a*3.14
          self.angular_pub.publish(-aaa)
          self.linear_pub.publish(vvv)
          kalman_msg.measure_angular = imu_msg.angular_velocity.z
         # self.kalman_pub.publish(kalman_msg)
          self.nonekalman_angular.publish(kalman_msg.measure_angular)
      
      if len(enc_data) == 2:
          enc_msg = Int16()
          enc_dif_r = Int16()
          enc_dif_l = Int16()
          #global enc_data_r
          #global enc_data_l
          self.lock.acquire()
          ly_daenc = numpy.load('ly_enc.npy')
          enc_msg.data = enc_data[0]
          self.lock.release()
          self.r_enc_pub.publish(enc_msg)
          #ly_daenc[0] = enc_data[0]
          self.lock.acquire()
          enc_data_r = ly_daenc[0]
          enc_dif_r.data = enc_msg.data - enc_data_r # difference value
          enc_msg.data = enc_data[1]
          self.lock.release()
          self.l_enc_pub.publish(enc_msg)
          #ly_daenc[1] = enc_data[1]
          self.lock.acquire()
          enc_data_l = ly_daenc[1]
          enc_dif_l.data = enc_msg.data - enc_data_l # difference value
          ly_daenc = enc_data
          numpy.save('ly_enc.npy',ly_daenc)
          #this get the robocar`s wheels of velocity
          # velocity =(( encoder`s value of change in one cycle ) / total number of encoder) * wheel`s perimeter
          enc_sr = (float(enc_dif_r.data)/120.0)*30*3.14 
         # enc_vr =float(enc_dif_r /1167)*17.25 # it don`t work!!!wtf
          enc_vr = enc_sr * 0.01725
          enc_sl = (float(enc_dif_l.data)/120.0)*30*3.14
         # enc_vl =float(enc_dif_l /1167)*17.25
          enc_vl = enc_sl * 0.01725
          
          #print enc_dif_r,'dif_r'
          #print enc_dif_l,'dif_l'
          #print enc_sr,'sr'
          #print enc_sl,'sl'
          #print enc_vr,'vr'
          #print enc_vl,'vl'
          self.lock.release()
          enc_v_v = self.enc_vel(enc_vr,enc_vl)
          enc_v = enc_v_v/4.4 
          if self.v > self.hey or self.v < self.hey:
            if enc_v == self.hey:
              enc_v = self.enc_v_last
          self.enc_v_last = enc_v
          print self.enc_v_last ,'enc_v'
          enc_r_r = self.enc_rad(enc_vr,enc_vl)
          enc_r = enc_r_r/3 
          if self.a > self.hey or self.a < self.hey:
            if enc_r == 0:
              enc_r = self.enc_r_last
          self.enc_r_last = enc_r
          print self.enc_r_last ,'enc_r'
          
          twistImu.linear.x = self.enc_v_last
          twistImu.linear.y = 0
          twistImu.linear.z = 0
          twistImu.angular.x = 0
          twistImu.angular.y = 0
          twistImu.angular.z = self.enc_r_last
          kalman_msg.linear_vel = self.enc_v_last
          self.kalman_pub.publish(kalman_msg)
          self.enc_angular.publish(self.enc_r_last)
          self.nonekalman_linear.publish(self.enc_v_last)
          self.current_time = time.time()
         # time_dif = self.current_time - self.last_time
         # self.last_time = self.current_time
         # print time_dif,'tiem_dif'
         # numpy.save('ly_enc.npy',ly_daenc)
          #self.ly_twist_pub.publish(twistImu)
          
          #enc_data_r = enc_data[0]#keep last value
         # enc_data_l = enc_data[1]
         # print enc_data_r
         # print enc_data_l
       #   self.lock.acquire()
   #       linear_output = self.PID.update(self.enc_v_last)
         # angular_output = self.PID_another.update(enc_r)
         # print output,'output'
         # print linear_output,'linear_output'
    #      v = linear_output/2
    #      r = v + (0.2*self.a)
    #      l =(v/1.15) - (0.2*self.a)
         # print a,'a' 
         # self.lock.acquire()
     #     vv = v*2
     #     self.sudu_pub.publish(vv)
     #     self.lock.acquire()
     #     self.zumy.cmd(l,r)
     #     self.lock.release()
          time_dif = self.current_time - self.last_time
          self.last_time = self.current_time
         # print time_dif,'tiem_dif'

         # self.sudu_pub.publish(vv)
      #self.ly_twist_pub.publish(twistImu)
      self.rate.sleep()

    # If shutdown, turn off motors
     # self.zumy.cmd(0,0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()

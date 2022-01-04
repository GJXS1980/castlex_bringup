#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import serial
import time
import os, stat
#import threading
import rospy
from serial.serialutil import SerialException

from math import sin, cos, pi, sqrt
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32, Float32MultiArray
import math
import numpy as np

from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Stm32ROS():
  def __init__(self):
    # 初始化节点，注册节点名
    rospy.init_node('Stm32_node', log_level=rospy.INFO)
    # 当该节点正在运行时，清除该节点
    rospy.on_shutdown(self.shutdown)
    # TODO 遍历ttyUSB,查找下位机串口
    # TODO 设备ttyUSB权限设置
    #在launch文件中获取参数

    # os.chmod(rospy.get_param("~port", "/dev/ttyUSB0"), stat.S_IRWXO) # 其他用户有全部权限
    self.port = rospy.get_param("~port", "/dev/stm32")
    self.baud = int(rospy.get_param("~baud", 115200))
    self.timeout = rospy.get_param("~timeout", 0.5)
    self.base_frame = rospy.get_param("~base_frame", "base_link")
    self.wheel_diameter = float(rospy.get_param("~wheel_diameter", 0.1)) # unit: m
    self.encoder_per_loop = int(rospy.get_param("~encoder_per_loop", 3176)) # 轮子转一圈编码输出多少脉冲

    # 货仓初始位置角度矫正
    self.angle_start =int(rospy.get_param("~angle_start", 0)) 
    self.angle_end = int(rospy.get_param("~angle_end", 135)) 

    # 轮子到底盘中心的距离设置
    self.length_a = float(rospy.get_param("~length_a",0.105))    #a轮子之间的距离
    self.length_b = float(rospy.get_param("~length_b",0.105))    #b轮子之间的距离
    self.length_c = float(rospy.get_param("~length_c",0.105))    #c轮子之间的距离

    # 三轮与底盘坐标系夹角
    self.angle_a = (float(rospy.get_param("~angle_a", 30))) * pi / 180.0    #底盘自身坐标系与a轮夹角
    self.angle_b = (float(rospy.get_param("~angle_b", 30))) * pi / 180.0    #底盘自身坐标系与b轮夹角
    self.angle_c = (float(rospy.get_param("~angle_c", 0))) * pi / 180.0   #底盘自身坐标系与c轮夹角

    self.rate = int(rospy.get_param("~rate", 100))
    self.safely = int(rospy.get_param("~safely", 0))
    self.meter_to_encoder = float(self.encoder_per_loop) / (self.wheel_diameter * pi)
    self.encoder_to_meter = (self.wheel_diameter * pi) / float(self.encoder_per_loop)

    #定义里程计相关变量
    self.x, self.y, self.th = 0, 0, 0
    self.dx, self.dy, self.dth  = 0, 0, 0
    self.vx, self.vy, self.vth,  = 0, 0, 0
    self.vA, self.vB, self.vC, self.vD = 0, 0, 0, 0
    # RPY三个角度初始化
    self.th_r, self.th_p, self.th_y = 0, 0, 0
    self.gyro_e_data = [0, 0, 0]
    self.odom_now = rospy.Time.now()
    self.odom_last = self.odom_now
    self.odom_dt = self.odom_now - self.odom_last

    self.turn_off , self.turn_on = True, False

    # 定义速度控制相关变量
    self.last_cmd_vel = time.time()
    self.cmd_x, self.cmd_y, self.cmd_th = 0.0, 0.0, 0.0
    self.cmd_vA, self.cmd_vB, self.cmd_vC, self.cmd_vD = 0, 0, 0, 0

    self.cmd_flag = False     # 确认是否进入回调函数获取速度控制指令，避免一直下发速度控制

    #陀螺儀相關參數
    self.gyro_data_r, self.gyro_offset_r, self.gyro_vel_r = 0.0, 0, 0,
    self.gyro_data_p, self.gyro_offset_p, self.gyro_vel_p = 0.0, 0, 0,
    self.gyro_data_y, self.gyro_offset_y, self.gyro_vel_y = 0.0, 0.0-0, 0.0
    self.gyro_now, self.gyro_last = 0, 0
    self.gyro_data_r1, self.gyro_data_p1, self.gyro_data_y1 = 0, 0, 0

    # 校准矩阵
    # 原来矩阵
    T_cal = np.mat([[0.94815943, 0.15890221, -0.27521587], [-0.16833122, 0.98567104, -0.01082614], [0.26955202, 0.05659233, 0.9613215]])
    # 求逆矩阵
    self.T_cal = T_cal.I

    # new add
    # 传感器数据的flag
    self.sensor_tag, self.disinfect_tag = 306, 306
    self.light = -1
    self.Warehouse = 0   #默认关闭货仓
    self.Warehouse_light = 0 
    # self.sensor_status_tag = 306

    self.last_encoder_count = [0, 0, 0, 0]    # 编码值
    self.interCharTimeout = self.timeout / 200    # 单个字节读写超时
    self.now = time.time()
    self.last = self.now
    self.speed_query_cycle = 0

    r = rospy.Rate(self.rate)
    # 订阅运动控制消息
    rospy.Subscriber(rospy.get_param("~input_cmd_vel", "/cmd_vel"), Twist, self.cmd_vel_callback)
    # 初始化里程计信息(Odometry)
    self.odom = Odometry()
    # 定义发布者发布里程计信息
    self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 5)
    self.odomBroadcaster = TransformBroadcaster()

    # new add
    # 订阅传感器是否要开启消息
    rospy.Subscriber("sensor_switch", Int32, self.sensor_switch_number)
    # 订阅紫外线消杀话题
    rospy.Subscriber("ultraviolet_disinfection", Int32, self.light_control)
    # 订阅貨艙是否要开启消息
    rospy.Subscriber("Warehouse_control", Int32, self.Warehouse_control)
    # 货仓呼吸灯话题
    rospy.Subscriber("Warehouse_light_control", Int32, self.Warehouse_light_control)
    # 订阅喷雾消毒话题
    rospy.Subscriber("spray_kill", Int32, self.disinfect_switch_number)

    # new add
    #　发布防碰撞传感器信息话题
    self.AntiCol_pub = rospy.Publisher('AntiCollision_data', Int32, queue_size = 1)
    #　发布超声波传感器信息话题
    self.ultrasonic_pub = rospy.Publisher('/ultrasonic_data', Float32MultiArray, queue_size = 1)
    #　发布防跌落传感器信息话题
    self.FallPre_pub = rospy.Publisher('/FallPrevention_data', Float32MultiArray, queue_size = 1)

    # 发布移动距离和角度
    self.odom_test = rospy.Publisher('/odom_test_data', Float32MultiArray, queue_size = 1)

    #　发布陀螺仪数据信息话题
    self.gyro_pub = rospy.Publisher('/gyro_data', Float32, queue_size = 1)

    #　发布电量信息话题
    self.battery_pub = rospy.Publisher('/battery_capacity', Float32, queue_size = 1)

    #定义发布者发布转速用于调PID
    self.wheel_vel_pub = rospy.Publisher("/wheel_vel_single", Int32, queue_size = 10)

    # 连接STM32
    self.serial = self.serial_connect()
    # 清除编码值
    self.reset_encoder()

    # 清空传感器数据
    self.reset_sensor()
    self.cmd_vel_x_tmp, self.cmd_vel_y_tmp = 0, 0
    self.closely = 0 #超聲波距離標志值，0,1,2,
    self.stop_flag = 0 #急停
    # i = 0
    # #陀螺仪连续采样200次，进行均值滤波，粗略计算温飘值
    # for i in range(0,200):
    # 	# time.sleep(0.005) #5ms採樣一次
    #   self.gyro_offset_r += self.get_gyro_data(0.000)[0]
    #   self.gyro_offset_p += self.get_gyro_data(0.000)[1]
    #   self.gyro_offset_y += self.get_gyro_data(0.000)[2]

    # self.gyro_data_r, self.gyro_data_p, self.gyro_data_y = 0, 0, 0
    # #粗略計算溫漂
    # self.gyro_offset_r, self.gyro_offset_p, self.gyro_offset_y  = (self.gyro_offset_r / 200), (self.gyro_offset_p / 200), (self.gyro_offset_y / 200)   
    # # print(self.gyro_offset)
    self.sensor_switch(self.sensor_tag)
    self.disinfect_switch(self.disinfect_tag)
    # rospy.sleep(18)
    self.gyro_offset_y = self.get_gyro_data(0.001)[2]
      # self.gyro_offset_y = self.gyro_offset_y / 200
      
    self.execute_array('G(0)(0)(0)(1)')
    time.sleep(0.01)
    print("Gyroscope is ready...........")
    print("下位机连接成功")

    while not rospy.is_shutdown():
      self.get_gyro_data(0.000)
      self.gyro_pub.publish(-(self.gyro_data_y * 180 / pi))  # 发布陀罗仪数据
      #　里程计的计算
      self.Odometry_calculate()
      # 获取超声波、防跌落、防碰撞传感器数据
      if self.sensor_tag == 306:
        self.get_sensor(0.001)
      elif self.sensor_tag != 306:
        self.sensor_switch(self.sensor_tag)
      else:
        pass
      # 开启消毒功能
      if self.disinfect_tag != 306 or self.light != -1:
        self.disinfect_switch(self.disinfect_tag)
        self.light_control_switch(self.light)
        self.light = -1
      # 货仓控制
      if self.Warehouse != -1:
        if self.Warehouse == 1:
          self.execute_array('s(1)')
        elif self.Warehouse == 0 :
            self.execute_array('s(0)')
        self.Warehouse = -1 

        #LED
      if self.Warehouse_light != -1:
        led = "x("+str(self.Warehouse_light)+")"
        self.execute_array(led)
        self.Warehouse_light  = -1
        # else:
        #   pass

      voltage = (float(self.get_battery(0.0)[0])*100)
      # print str(voltage) + '%'
      self.battery_pub.publish(voltage)

      if self.cmd_flag is True:
        self.set_speed(self.cmd_vA, self.cmd_vB, self.cmd_vC, self.cmd_vD)
        self.cmd_flag = False
    self.shutdown()
######################################################################################

  def light_control(self,data):
    self.light = data.data

  def Warehouse_control(self,data):
    self.Warehouse = data.data

  def Warehouse_light_control(self,data):
    self.Warehouse_light = data.data


# 启动消毒功能
  def light_control_switch(self, data):
    if data == 1 and self.turn_off:
      self.execute_ack("ZON")
      self.turn_off, self.turn_on = False, True

    elif data == 0 and self.turn_on:
      self.execute_ack("ZOFF")
      self.turn_on, self.turn_off = False, True

    else:
      pass
    # 复位self.disinfect_tag
    self.light = -1


  def serial_connect(self):
    try:
      rospy.loginfo("Connecting to STM32 on port " + self.port + "...")
      self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout, write_timeout=self.timeout)
      #等待1s
      rospy.loginfo("Sleeping for 1 second...")
      # time.sleep(0.01)
      serial_status = self.serial.isOpen()
      #向下位机查询波特率测试是否连接成功
      #test = self.get_baud()
      if(serial_status): #and test == self.baud):
        rospy.loginfo("Connection test successful.")
        rospy.loginfo("Serial health : " + str(serial_status))
        rospy.loginfo("Connected to STM32 on port " + self.port + " at " + str(self.baud) + " baud")
        rospy.loginfo("STM32 is ready...")
        return self.serial
      else:
        raise SerialException
    except SerialException:
      rospy.logerr("Cannot connect to STM32!")
      os._exit(1)

  def serial_open(self):
    #打开串口
    self.serial.open()

  def serial_close(self):
    #关闭串口
    self.serial.close()

  def serial_write(self, cmd):
    #向下位机串口写入控制指令
    self.serial.write((cmd + '\r\n').encode())

  #该函数用于接受下位机发送的所有字符（未提取数据）
  def serial_recv(self, timeout=0.5):
    timeout = min(timeout, self.timeout)
    c = ''
    value = ''
    attempts = 0
    while c != '\n':
      c = self.serial.read(1)
      c = bytes.decode(c) # byte --> str
      value += c
      attempts += 1 
      if attempts * self.interCharTimeout > timeout:
        return None
    # print "123"
    #print value
    # value = bytes.decode(value)
    value = value.strip('\r\n')   #移除字符串头尾指定的字符序列
    return value

  #等待下位机接受并回答‘OK’，用于提高通信确定性
  def recv_ack(self):
    ack = self.serial_recv(self.timeout)
    return ack

  #下位机返回单个整形数字时，提取（主要用于读取波特率、传感器等），用于一次通信非实时性
  def recv_int(self):
    value = self.serial_recv(self.timeout)
    try:
      return int(value)
    except:
      return None

  #下位机返回数组时，提取(主要用于读取编码值)，用于一次通信非实时性
  def recv_array(self):
    try:
      values = self.serial_recv(self.timeout).split("\t")
      # print "sss"
      #print values
      '''
      split() 通过指定分隔符对字符串进行切片，如果参数 num 有指定值，则分隔 num+1 个子字符串
      str.split(str="", num=string.count(str)).

      str -- 分隔符，默认为所有的空字符，包括空格、换行(\n)、制表符(\t)等。
      num -- 分割次数。默认为 -1, 即分隔所有。

      返回值：返回分割后的字符串列表。
      '''
      return values
      #map() 会根据提供的函数对指定序列做映射。
    except:
      return []

  #执行指令，包含发送指令和接受应答(读取单个int数值)
  def execute(self, cmd):
    try:
      self.serial.flushInput()
    except:
      pass

    ntries = 1    #允许重发次数
    attempts = 0  #已重发次数

    try:
      self.serial.write((cmd + '\r\n').encode())
      value = self.serial_recv(self.timeout)
      #可能通信出错，尝试重发指令
      while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
        try:
          self.serial.flushInput()
          self.serial.write((cmd+'\r\n').encode())
          value = self.recv(self.timeout)
        except:
          rospy.logerr("Exception executing command: " + cmd)
        attempts += 1
    except:
      rospy.logerr("Exception executing command: " + cmd)
      value = None

    return int(value)

  #执行指令，包含发送和接受确认（读取下位机发送过来的数组）
  def execute_array(self, cmd):
    try:
      self.serial.flushInput()
    except:
      pass

    ntries = 1    #允许重发次数
    attempts = 0  #已重发次数

    try:
      self.serial.write((cmd + '\r\n').encode())
      values = self.recv_array()
      while attempts < ntries and (values == '' or values == 'Invalid Command' or values == None):
        try:
          self.serial.flushInput()
          self.serial.write((cmd + '\r\n').encode())
          value = self.recv_array()
        except:
          rospy.logerr("Exception executing command: " + cmd)
        attempts += 1
    except:
      rospy.logerr("Exception executing command: " + cmd)
      raise SerialException
      return []

    return values
  
  #执行指令，包含发送和接受确认（根据‘OK’确认是否通信成功）
  def execute_ack(self, cmd):
    try:
      #print("flush")
      self.serial.flushInput()
    except:
      pass

    ntries = 1    #允许重发次数
    attempts = 0  #已重发次数

    try:
      self.serial.write((cmd + '\r\n').encode())
      #rospy.loginfo(cmd)
      ack = self.serial_recv(self.timeout)
      #rospy.loginfo(ack)
      while attempts < ntries and (ack != 'OK'):
        try:
          self.serial.flushInput()
          self.serial.write((cmd + '\r\n').encode())
          ack = self.recv_ack(self.timeout)
        except:
          rospy.logerr("Exception executing command: " + cmd)
          
    except:
      rospy.logerr("Exception executing command: " + cmd)
      raise SerialException
      return 0
    #print ack
    return ack

# new add
# 打开消毒功能
  def disinfect_switch_number(self, data):
    self.disinfect_tag = data.data

# 打开超声波传感器
  def sensor_switch_number(self, data):
    self.sensor_tag = data.data


# new add
# 打开超声波传感器
  def sensor_switch(self, data):
    # 只开启超声波传感器
    if self.sensor_tag == 1:
      self.execute_array('G(0)(0)(1)(1)')

    # 只开启防跌落传感器
    elif self.sensor_tag == 2:
      self.execute_array('G(0)(1)(0)(1)')

    # 开启防跌落和超声波传感器
    elif self.sensor_tag == 3:
      self.execute_array('G(0)(1)(1)(1)')

    # 只开启防碰撞传感器
    elif self.sensor_tag == 4:
      self.execute_array('G(1)(0)(0)(1)')

    # 开启超声波和防碰撞传感器
    elif self.sensor_tag == 5:
      self.execute_array('G(1)(0)(1)(1)')

    # 开启防碰撞与防跌落
    elif self.sensor_tag == 6:
      self.execute_array('G(1)(1)(0)(1)')

    # 防跌落、防碰撞和超声波传感器全开
    elif self.sensor_tag == 7:
      self.execute_array('G(1)(1)(1)(1)')

    # 传感器全关闭
    elif self.sensor_tag == 0:
      self.execute_array('G(0)(0)(0)(0)')
    else:
      pass
    # 复位self.sensor_tag
    self.sensor_tag = 306

# 启动消毒功能
  def disinfect_switch(self, data):
    # 启动１级消毒模式
    if self.disinfect_tag == 1:
      self.execute_array('x(8399)')

    # 启动２级消毒模式
    elif self.disinfect_tag == 2:
      self.execute_array('x(8000)')

    # 启动３级消毒模式
    elif self.disinfect_tag == 3:
      self.execute_array('x(7400)')

    # 传感器全关闭
    elif self.disinfect_tag == 0:
      self.execute_array('x(0)')
    else:
      pass
    # 复位self.disinfect_tag
    self.disinfect_tag = 306


######################################################################################################

  #向下位机查询波特率（一般用于测试是否通信成功）
  def get_baud(self):
    try:
      return int(self.execute('b'))
    except:
      return None

  #向下位机查询三轮的编码值
  def get_encoder_counts(self):
    values = self.execute_array('e')
    #print values
    try:
      values = list(map(int, values))
    except:
      values = []
    if len(values) != 5:
      rospy.logerr("Encoder count was not 5")
      raise SerialException
      return None
    else:
      #print values
      return values

  #向下位機查詢電源電壓，用於計算電量
  def get_battery(self, data):
    values = self.execute_array('b')  
    time.sleep(data) #data s採樣一次
    # values1 = values
    return values

# new add
# 向下位机查询传感器数据(超声波、防碰撞、防跌落)
  def get_sensor(self, data):
    values = self.execute_array('u')
    if len(values) != 8:
      rospy.logerr("sensor count was not 8")
      print(len(values))
      raise SerialException
      return None
    else:
      time.sleep(data) #data s採樣一次
      self.ultrasonic, self.FallPre, self.AntiCol = (float(values[0]), float(values[1]), float(values[2])), (int(values[3]), int(values[4]), int(values[5])), int(values[6])
      self.ultrasonic_pub.publish(Float32MultiArray(data = self.ultrasonic))
      self.FallPre_pub.publish(Float32MultiArray(data = self.FallPre))
      self.AntiCol_pub.publish(self.AntiCol)

  # 向下位机查询传感器数据(陀螺仪)
  def get_gyro_data(self, data):
    values = self.execute_array('g')
    time.sleep(data) #data s採樣一次
    if len(values) != 3:
      rospy.logerr("gyro count was not 3")
      print(len(values))
      raise SerialException
      return None
    else:
      # 对陀螺仪的数据进行处理(改变坐标系方向:x = -x';y = y'; z = -z')
      # gyro_values = (float(values[0]), float(values[1]), float(values[2]), float(values[3])*pi/180, -float(values[4])*pi/180, -float(values[5])*pi/180, float(values[6]), float(values[7]), float(values[8]))
      gyro_values = (float(values[0]), float(values[1]), float(values[2]))
      # print(gyro_values)
      # self.gyro_now = time.time()
    # if self.vA != 0 or self.vB != 0 or self.vC != 0 or self.vD != 0:
        # 由于里程计的角速度用编码器，所以下面的陀螺仪角速度用不上
        # self.gyro_vel_r, self.gyro_vel_p, self.gyro_vel_y = float(gyro_values[3] - self.gyro_offset_r), float(gyro_values[4] - self.gyro_offset_p), float(gyro_values[5] - self.gyro_offset_y)
       	# # self.gyro_vel_y = float(gyro_values[5] - self.gyro_offset_y)

        # # imu获取的是角速度，这里要计算角度，即求速度的积分，陀螺仪积分角度 += 角速度 * dt
        # self.gyro_data_r += float(gyro_values[3] - self.gyro_offset_r)*(self.gyro_now - self.gyro_last)
        # self.gyro_data_p += float(gyro_values[4] - self.gyro_offset_p)*(self.gyro_now - self.gyro_last)
        # self.gyro_data_y += float(gyro_values[5] - self.gyro_offset_y)*(self.gyro_now - self.gyro_last)

        # # 欧拉角转旋转矩阵
        # e = [self.gyro_data_y, self.gyro_data_p, self.gyro_data_r]
        # R = np.asmatrix(self.eulerAnglesToRotationMatrix(e))	# 使用np.asmatrix()函数将arry()转换成mat()矩阵格式

        # # 把陀螺仪的坐标系进行变换(校准)
        # T = np.asarray(self.T_cal*R)

        # # 旋转矩阵变换成欧拉角
        # self.gyro_e_data = self.rotationMatrixToEulerAngles(T)
        # # print(self.gyro_e_data)
        # self.gyro_data_y1, self.gyro_data_p1, self.gyro_data_r1 = self.gyro_e_data[0]*180.0/pi, self.gyro_e_data[1]*180.0/pi, self.gyro_e_data[2]*180.0/pi
      if gyro_values[2] < 0:
        self.gyro_data_r, self.gyro_data_p, self.gyro_data_y = gyro_values[0] * pi / 180.0, gyro_values[1] * pi / 180.0, (360 + gyro_values[2]) * pi / 180.0	# 根据陀罗仪安装的方向变号
      else:
        self.gyro_data_r, self.gyro_data_p, self.gyro_data_y = gyro_values[0]*pi/180.0, gyro_values[1]*pi/180.0, gyro_values[2]*pi/180.0	# 根据陀罗仪安装的方向变号
    return gyro_values
######################################################################################################

  #获取移动底盘速度
  def get_speed(self):
    self.now = time.time()
    self.speed_query_cycle = self.now - self.last    #计算速度查询周期
    #print self.speed_query_cycle
    self.last = self.now
    now_encoder = self.get_encoder_counts()[0:4] # 用第四个电机接口
    d_encoder = [0, 0, 0, 0]
    encoder_speed = [0, 0, 0, 0]
    meter_speed = [0, 0, 0, 0]
    # 急停
    # print self.get_encoder_counts()
    #self.stop_flag = self.get_encoder_counts()[4] #realtime read 急停，標志位放在編碼器數組的第五個位置
    for i in range(len(now_encoder)):
        d_encoder[i] = now_encoder[i] - self.last_encoder_count[i]    #计算编码差值
        encoder_speed[i] = (float(d_encoder[i]) / self.speed_query_cycle)    #编码差 / 时间 = 速度（ticks/s)
    self.last_encoder_count = now_encoder

    for index in range(len(encoder_speed)):
        meter_speed[index] = float(encoder_speed[index]) * self.encoder_to_meter
    self.wheel_vel_pub.publish(meter_speed[0])
    # print meter_speed
    return [d_encoder, encoder_speed, meter_speed]    #返回编码变化量、速度(ticks/s)、速度(m/s)

  #清空下位机编码值
  def reset_encoder(self):
    return self.execute_ack('r')

  # 清空传感器数据
  def reset_sensor(self):
    return self.execute_ack('R')

  #向下位机发送速度控制指令
  def set_speed(self, motor_a, motor_d, motor_c, motor_b):
    motor_d = 0
    temp_translate = 100.0#1.476
    motor_a = float(motor_a) * self.meter_to_encoder / temp_translate; 
    motor_b = float(motor_b) * self.meter_to_encoder / temp_translate; 
    motor_c = float(motor_c) * self.meter_to_encoder / temp_translate;

    #print ([motor_a, motor_b, motor_c])

    return self.execute_ack('m(%f)(%f)(%f)(%f)(30)' %(motor_a, motor_d, motor_c, motor_b))

  #停车
  def stop(self):
    self.set_speed(0, 0, 0, 0)
    '''
    values = self.execute_array('a')

    try:
      values = map(int, values)
    except:
      values = []

    if len(values) != self.num_analog_port:
      rospy.logerr("Analog port count was not " + str(self.num_analog_port))
      raise SerialException
      return None
    else:
      return values
      '''

  #关闭底盘节点时必须调用该函数释放串口
  def shutdown(self):
    try:
      rospy.loginfo("Stopping the robot...")
      #rospy.sleep(1)
    except:
      pass
    #停车，关闭串口
    try:
      # new add
      # 关闭传感器()
      self.execute_array('G(0)(0)(0)(0)')
      self.execute_array('x(0)')
      #self.execute_array('ZOFF')
      #self.stop()
      self.serial_close()
    except:
      pass
    finally:
      rospy.loginfo("Serial port closed.")
      os._exit(0)

  #在线整定PID参数
  def update_pid(self, wheel_num, kp, ki, kd):
    return self.execute_ack('p(%d)(%d)(%d)(%d)' %(wheel_num, kp, ki, kd))

  # cmd_vel回调函数
  def cmd_vel_callback(self, req):
    #接收速度控制消息(cmd_vel)
    self.last_cmd_vel = time.time()
    self.cmd_x = req.linear.x    # m/s
    self.cmd_y = req.linear.y   # m/s
    self.cmd_th = req.angular.z  # rad/s #速度反了，加了负号不符合公式，根据底盘接线调整
#    if self.cmd_th > 0.8 :
#        self.cmd_th = 0.8        #限速
#    elif self.cmd_th < -0.8 :
#        self.cmd_th = -0.8        #限速

    if math.isnan(self.cmd_th):
      self.cmd_th = 0

    #2019.07.29增加内容
    #if abs(self.cmd_x) >= 0.2:
      #self.cmd_x = (self.cmd_x / abs(self.cmd_x)) * 0.2
      #rospy.loginfo("Current linear velocity exceeding the limit is limited to 0.240m/s.")
    #elif abs(self.cmd_x) <= -0.2:
      #self.cmd_x = (self.cmd_x / abs(self.cmd_x)) * 0.2
      #rospy.loginfo("test.")

    #if abs(self.cmd_y) >= 0.2:
      #self.cmd_y = (self.cmd_y / abs(self.cmd_y)) * 0.2
      #rospy.loginfo("Current linear velocity exceeding the limit is limited to 0.240m/s.")
    #if abs(self.cmd_th) <= 0.5 and abs(self.cmd_th) > 0:
      #self.cmd_th = (self.cmd_th / abs(self.cmd_th)) * 0.5
      #rospy.loginfo("Current linear velocity exceeding the limit is limited to 0.240m/s.")
    #if abs(self.cmd_th) >= 1.5:
      #self.cmd_th = (self.cmd_th / abs(self.cmd_th)) * 1.5
    # if abs(self.cmd_x) <= -0.2:
    #   self.cmd_x = (self.cmd_x / abs(self.cmd_x)) * -0.2
    #   rospy.loginfo("Current linear velocity exceeding the limit is limited to 0.240m/s.")

    self.cmd_vA, self.cmd_vB, self.cmd_vC, self.cmd_vD = self.reverse_kinematics(self.cmd_x, self.cmd_y, self.cmd_th)
    self.cmd_flag = True

# 逆向运动学求解
  def forward_kinematics_roll(self, speed_a, speed_b, speed_c, speed_d):
    a1 = ((self.length_b + self.length_c*sin(self.angle_b))/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + sin(self.angle_b)*cos(self.angle_a)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c))
    a2 = -(self.length_a + sin(self.angle_a)*self.length_c)/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    a3 = -(sin(self.angle_b)*self.length_a -sin(self.angle_a)*self.length_b)/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + cos(self.angle_b)*sin(self.angle_a)*self.length_c)
    b1 = -(cos(self.angle_b)*self.length_c)/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    b2 = -(cos(self.angle_a)*self.length_c)/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    b3 = (cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a)/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    c1 = -(cos(self.angle_b))/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    c2 = -(cos(self.angle_a))/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    c3 = -(cos(self.angle_a)*sin(self.angle_b) + cos(self.angle_b)*sin(self.angle_a))/(cos(self.angle_a)*self.length_b + cos(self.angle_b)*self.length_a + cos(self.angle_a)*sin(self.angle_b)*self.length_c + sin(self.angle_a)*cos(self.angle_b)*self.length_c)
    v_x = [a1, a2, a3]
    v_y = [b1, b2,b3] 
    v_th = [c1, c2, c3]
    # 求逆运动学系数矩阵
    V = np.asarray(np.mat([v_x, v_y, v_th])*np.mat([[speed_a], [speed_d], [speed_c]]))	# 乘速度
    vx, vy, vth = (V[0][0]*cos(self.gyro_data_y) - V[1][0]*sin(self.gyro_data_y)), (V[0][0]*sin(self.gyro_data_y) + V[1][0]*cos(self.gyro_data_y)), V[2][0]	# 提取每个电机的速度
    v = [vx, vy, vth]

    # print(v)
    return [vx, vy, vth]

  # 正向运动学，由底盘整体速度推算轮子对应转速(m/s)
  def reverse_kinematics(self, speed_x, speed_y, speed_th):
    v_a = [cos(self.angle_a), -sin(self.angle_a), -self.length_a] 
    v_b = [-cos(self.angle_b), -sin(self.angle_b), -self.length_b]
    v_c = [0, 1, -self.length_c]
    V = np.asarray(np.mat([v_a, v_b, v_c])*np.mat([[speed_x], [speed_y], [speed_th]]))	# 乘速度
    va, vb, vc = V[0][0], V[1][0], V[2][0]	# 提取每个电机的速度
    k_speed = [va, 0, vc, vb]
    # print(k_speed)    
    return k_speed

  def isRotationMatrix(self, R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

    # 旋转矩阵转换成欧拉角
  def rotationMatrixToEulerAngles(self, R):
    assert(self.isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
      x = math.atan2(R[2,1] , R[2,2])
      y = math.atan2(-R[2,0], sy)
      z = math.atan2(R[1,0], R[0,0])
    else :
      x = math.atan2(-R[1,2], R[1,1])
      y = math.atan2(-R[2,0], sy)
      z = 0
    return np.array([x, y, z])

	# 欧拉角转换成旋转矩阵
  def eulerAnglesToRotationMatrix(self, theta):
    R_x = np.array([[1,         0,                  0                   ],
  		[0,         math.cos(theta[0]), -math.sin(theta[0]) ],
  		[0,         math.sin(theta[0]), math.cos(theta[0])  ]
  		])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
  			[0,                     1,      0                   ],
  			[-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
  			])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
  		[math.sin(theta[2]),    math.cos(theta[2]),     0],
  		[0,                     0,                      1]
  		])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

  #计算里程计信息并发布到话题
  def Odometry_calculate(self):
    d_encoder, encoder_speed, meter_speed = self.get_speed()
    #print [d_encoder, encoder_speed, meter_speed]

    self.vA, self.vB, self.vC, self.vD = meter_speed   #获取三个轮子的m/s速度值
    #print [self.vC, self.vA, self.vB, self.vD]

    # 获取陀螺仪上面的x,y方向的线速度和角速度
    self.vx, self.vy, self.vth = self.forward_kinematics_roll(self.vA, self.vB, self.vC, self.vD)
    self.x += (self.vx*self.speed_query_cycle)  # 计算在x方向的位移
    # a = self.vx*self.speed_query_cycle
    self.y += (self.vy*self.speed_query_cycle)  # 计算在y方向的位移

    self.th_y = self.gyro_data_y 
    # print(self.th_y)
    # test = (self.x, self.y, self.th_y)

    # self.odom_test.publish(Float32MultiArray(data = test))

    #　计算四元数
    quaternion = Quaternion()
    #　RPY到四元数的变换
    quat = quaternion_from_euler (0, 0, self.th_y, 'rxyz')	# 另外两个角度要测试
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]

    # quaternion.x = 0
    # quaternion.y = 0
    # quaternion.z = sin(self.th_y/2.0)
    # quaternion.w = cos(self.th_y/2.0)

    self.odom_now = rospy.Time.now()
    self.odomBroadcaster.sendTransform(
    (self.x, self.y, 0),
    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    self.odom_now,
    "base_link",
    "odom"
    )

    self.odom.header.frame_id = "odom" 
    self.odom.child_frame_id = self.base_frame
    self.odom.header.stamp = self.odom_now
    self.odom.pose.pose.position.x = self.x
    self.odom.pose.pose.position.y = self.y
    self.odom.pose.pose.position.z = 0
    self.odom.pose.pose.orientation = quaternion
    
    self.odom.pose.covariance = [1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9]
    
    self.odom.twist.twist.linear.x = self.vx
    self.odom.twist.twist.linear.y = self.vy
    self.odom.twist.twist.angular.z = self.vth
    
    self.odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9]
    self.odom_pub.publish(self.odom)
    

'''
if __name__ == '__main__':
  try:
    mySTM32 = Stm32ROS()
  except SerialException:
    rospy.logerr("Serial exception trying to open port.")
    os._exit(0)
'''
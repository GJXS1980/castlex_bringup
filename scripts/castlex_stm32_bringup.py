#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from castlex_stm32_bringup_python3 import Stm32ROS

if __name__ == '__main__':
  try:
    mySTM32 = Stm32ROS()
  except SerialException:
    rospy.logerr("Serial exception trying to open port.")
    os._exit(0)

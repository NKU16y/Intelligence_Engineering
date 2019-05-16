#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import os
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
class map_update:
    
    def __init__(self):
        rospy.init_node("map_update")
        self.map_update_pub=rospy.Publisher("/path_update",OccupancyGrid,queue_size=15)
        #地图相关
        self.origin_x = 0
        self.origin_y = 0
        self.height = 0
        self.resolution = 0
        self.width = 0
        self.map_raw=[]
        #self.map_sub=rospy.Subscriber("/map",OccupancyGrid,self.mapCallback)
        #激光相关
        self.angle_min=0.0
        self.angle_max=0.0
        self.angle_increment=0.0
        self.scan_time=0.0
        self.range_min=-math.pi/6
        self.range_max=math.pi/6
        self.isob=False
        self.laser_sub=rospy.Subscriber("/scan",LaserScan,self.laserCallback)
        #里程计相关
        self.x = 0.0
        self.y = 0.0
        self.wx= 0.0
        self.wy= 0.0
        self.xm= 0.0
        self.ym= 0.0
        self.theta=0.0
        self.odo_sub=rospy.Subscriber("/sensors_fusion/odom",Odometry,self.odomCallback)
        self.dis=0.5
        #self.update_map()
        rospy.Rate(1)
        rospy.spin()
    
    def World2map(self,xw,yw):
        xm=xw-self.height-self.origin_y/self.resolution
        ym=yw-self.origin_x/self.resolution
        return xm,ym
    
    def update_map(self):
        if self.isob:
            print("update map")
            self.wx=self.x+self.dis*math.cos(self.theta)
            self.wy=self.y+self.dis*math.sin(self.theta)
            self.xm,self.ym=self.World2map(self.wx,self.wy)
            U_map=self.map_raw.reshape((self.height, self.width))
            for i in range(self.xm-5,self.xm+5):
                for j in range(self.ym-5,self.ym+5):
                    U_map[i][j]=0
            Up_map=np.array(U_map,dtype=np.int8)
            Upp_map=OccupancyGrid(Up_map)
            self.map_update_pub.publish(Upp_map)

            
    def mapCallback(self, msg):
        self.origin_x = msg.info.origin.position.y
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_raw=np.array(msg.data,dtype=np.int8)

    def laserCallback(self,msg):
        self.angle_min=msg.angle_min
        self.angle_max=msg.angle_max
        self.angle_increment=msg.angle_increment
        min_distance=10
        range_index_start = int((self.range_min - self.angle_min)/self.angle_increment)
        range_index_end = int ((self.range_max - self.angle_min)/self.angle_increment)
        #寻找范围内最小的距离
        for i in range(range_index_start,range_index_end):
            if msg.ranges[i]<min_distance and msg.ranges[i]>0.0:
                min_distance = msg.ranges[i]
            if min_distance <= 0.5:
                self.isob=True
            else:
                self.isob =False
        print (range_index_start)
        print (range_index_end)
        print (min_distance)
        print (self.isob)
    
    def odomCallback(self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        self.theta=math.acos(1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z)
        print("odom_data----------'\n'")
        print(msg.pose.pose.orientation.w)
        print(msg.pose.pose.orientation.x)
        print(msg.pose.pose.orientation.y)
        print(msg.pose.pose.orientation.z)
        print(math.cos(self.theta))
        print(self.theta)
if __name__ == '__main__':
    map_update()
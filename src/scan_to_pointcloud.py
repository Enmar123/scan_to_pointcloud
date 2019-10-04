#!/usr/bin/env python

"""
Spyder Editor

This temporary script file is located here:
/home/arc/.spyder2/.temp.py
"""
import rospy
import math as m

import tf
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PointStamped

##############################################################################

class Node:
    
    def __init__(self):
        # General Variables
        self.ref_frame = "/map" # If SLAM got messed up change "/map" to "/odom"
        
        # ROS variables              
        self.point_msg = PointStamped()
        self.point_msg.header.frame_id = "rplidar_laser"
        self.point_msg.point.z = 0.0
        self.tl = tf.TransformListener(True, rospy.Duration(10))
        self.lp = lg.LaserProjection()
        # IMPORTANT: For older scans subscribe to "rplidar_scan" instead of "scan1"
        rospy.Subscriber("scan1", LaserScan, self.processLaser2)
        self.pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
        
        rospy.spin()

# ------------------ METHODS ------------------
    
    
    def processLaser2(self, msg):
        #convert LaserScan to PointCloud2
        pc2_msg = self.lp.projectLaser(msg)       
        point_generator = pc2.read_points(pc2_msg)
        for point in point_generator:
            if not m.isnan(point[0]):
                if not m.isnan(point[1]):
                    self.point_msg.point.x = point[0]
                    self.point_msg.point.y = point[1]
                    #get latest transform time                    
                    time_tf = self.tl.getLatestCommonTime("/rplidar_laser", self.ref_frame) 
                    # to prevent extapolation errors make the "point timestamp" = "transfom timestamp"
                    self.point_msg.header.stamp = time_tf
                    #point message at new reference frame                    
                    new_point_msg = self.tl.transformPoint(self.ref_frame, self.point_msg)
                    self.pointWrite(new_point_msg)


    #takes a point_msg and writes xyz to a .txt file    
    def pointWrite(self,point_msg):
        x = point_msg.point.x
        y = point_msg.point.y
        z = point_msg.point.z
        
        info = "%s %s %s" %(x,y,z)
        point_file.write(info + "\n")
        
        rospy.loginfo("point_written")

##############################################################################

# gets number from a txt file 
def readFileNum():
    file_a = open("_log/memory_file.txt","r")
    number = int(file_a.read()) + 1
    file_a.close()
   
    file_a = open("_log/memory_file.txt","w")
    file_a.write(str(number))    
    file_a.close()

    return number

##############################################################################

if __name__ == '__main__':
    file_num = readFileNum()
    point_file = open("DATA/pointclouds/point_file_%d.txt" %file_num,"w")    
    
    rospy.init_node('scan_to_pointcloud', anonymous=False)
    print("")    
    print("--- READY TO PROCESS BAGFILE ---")    
    try:
        node = Node()
    except rospy.ROSInterruptException:  pass
  
    point_file.close()

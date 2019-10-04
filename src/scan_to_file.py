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
        # Start ROS
        rospy.init_node('scan_to_file', anonymous=False)
        
        # Get Ros Params
        self.world_frame_id = rospy.get_param("~world_frame_id")
        self.scan_frame_id = rospy.get_param("~scan_frame_id")
        self.scan_topic = rospy.get_param("~scan_topic")
        self.save_to = rospy.get_param("~save_to")
        
        
        # ROS Subs and pubs
        rospy.Subscriber(self.scan_topic, LaserScan, self.laser_callback)
        
        # Regual variables
        self.mem_file = "memory.txt"
        filename = "pointcloud_%d.txt"%(self.get_counter())
        self.f = open(self.save_to + filename, "w")
        self.update_counter()
        
        # ROS variables              
        self.point_msg = PointStamped()
        self.point_msg.header.frame_id = self.scan_frame_id
        self.point_msg.point.z = 0.0
        self.tl = tf.TransformListener(True, rospy.Duration(10))
        self.lp = lg.LaserProjection()
        
        rospy.loginfo("scan_to_file started, ready to process LaserScan")
        rospy.spin()

# ------------------ METHODS ------------------
    
    def laser_callback(self, laser_msg):
        #convert LaserScan to PointCloud2
        pc2_msg = self.lp.projectLaser(laser_msg)       
        point_generator = pc2.read_points(pc2_msg)
        for point in point_generator:
            if not m.isnan(point[0]):
                if not m.isnan(point[1]):
                    self.point_msg.point.x = point[0]
                    self.point_msg.point.y = point[1]
                    #get latest transform time     
                    #time_tf = self.tl.getLatestCommonTime("/" + self.scan_frame_id, self.world_frame_id) 
                    time_tf = self.tl.getLatestCommonTime(self.scan_frame_id, self.world_frame_id) 
                    # to prevent extapolation errors make the "point timestamp" = "transfom timestamp"
                    self.point_msg.header.stamp = time_tf
                    #point message at new reference frame                    
                    new_point_msg = self.tl.transformPoint(self.world_frame_id, self.point_msg)
                    self.pointWrite(new_point_msg)    

    # Takes a point_msg and writes xyz to a .txt file    
    def pointWrite(self,point_msg):
        x = point_msg.point.x
        y = point_msg.point.y
        z = point_msg.point.z
        
        info = "%s %s %s" %(x,y,z)
        self.f.write(info + "\n")
        
        rospy.loginfo("point_written")
        
    # Gets counter from a memory.txt file
    def get_counter(self):       
        try:        
            with open(self.save_to + self.mem_file, "r") as f:            
                return int(f.readlines()[0])
        except (IndexError, IOError) as e:
            with open(self.save_to + self.mem_file, "w") as f:
                f.write("0")
            return self.get_counter()
                
    def update_counter(self):
        count = self.get_counter()
        with open(self.save_to + self.mem_file, "w") as f:
            f.write(str(count + 1))


##############################################################################

if __name__ == '__main__':
    
    try:
        node = Node()
    except rospy.ROSInterruptException:
        node.f.close()        
        pass
        

#!/usr/bin/env python

"""
Spyder Editor

This temporary script file is located here:
/home/arc/.spyder2/.temp.py
"""
import rospy

import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, LaserScan

##############################################################################

class Converter:
    def __init__(self):
        rospy.loginfo("Starting scan_to_pc2 converter")
        scan_name = rospy.get_param("~scan_name")
        pc2_name =  rospy.get_param("~pc2_name")
        #rospy.loginfo("%s is %s", rospy.resolve_name('/scan_in'), scan_name)

        rospy.Subscriber(scan_name, LaserScan, self.laser_callback)
        self.pc2_pub = rospy.Publisher(pc2_name, PointCloud2, queue_size=10)
        
        self.lp = lg.LaserProjection()

        rospy.spin()

    ###################
    ### ROS METHODS ###
    ###################

    def laser_callback(self, laser_msg):
        #convert LaserScan to PointCloud2
        pc2_msg = self.lp.projectLaser(laser_msg)
        self.pc2_pub.publish(pc2_msg)        


##############################################################################

if __name__ == '__main__':
    rospy.init_node('scan_to_pc2', anonymous=False)   
    try:
        foo = Converter()
    except rospy.ROSInterruptException:
        pass

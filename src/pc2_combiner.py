#!/usr/bin/env python

"""
Spyder Editor

This temporary script file is located here:
/home/arc/.spyder2/.temp.py
"""
import rospy

#import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2 #, LaserScan

##############################################################################

class RosNode:
    def __init__(self):
        # Log Info
        rospy.loginfo("Starting Pointcloud Combiner")
        
        # Get Params        
        in1_name = rospy.get_param("~in1_name")
        in2_name = rospy.get_param("~in2_name")
        out_name = rospy.get_param("~out_name")

        # Set up subs and pubs
        rospy.Subscriber(in1_name, PointCloud2, self.callback)
        rospy.Subscriber(in2_name, PointCloud2, self.callback)
        self.pc2_pub = rospy.Publisher(out_name, PointCloud2, queue_size=10)

        rospy.spin()

    ###################
    ### ROS METHODS ###
    ###################

    def callback(self, pc2_msg):
        # Publish the message under new name
        self.pc2_pub.publish(pc2_msg)        


##############################################################################

if __name__ == '__main__':
    rospy.init_node('pc2_combiner', anonymous=False)   
    try:
        rn = RosNode()
    except rospy.ROSInterruptException:
        pass

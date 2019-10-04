#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import tf

import numpy as np
import struct

def pc2msg_to_points_old(msg):
    points = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            x = point[0]
            y = point[1]
            z = point[2]
            points.append([x,y,z])
            #print(point)
    return points
    
def pc2msg_to_points(msg):
    points = list(sensor_msgs.point_cloud2.read_points(msg, skip_nans=True))
    return points


    
def transform_points(trans, quat, points):
    points_new = []
    for point in points:
        xyz = point[0:3]
        xyz = qv_mult(quat, xyz)
        xyz = translation(trans, xyz)
        #print(xyz)        
        point_new = list(xyz) + list(point[3:])
        #print(xyz)
        points_new.append(point_new)
    return points_new
    
def translation(trans, xyz):
    point_new = trans + np.array(xyz)
    return point_new
    
def qv_mult(q1, v1):
    v1_new = tf.transformations.unit_vector(v1)
    q2 = list(v1_new)
    q2.append(0.0)
    unit_vector = tf.transformations.quaternion_multiply(
                    tf.transformations.quaternion_multiply(q1, q2), 
                    tf.transformations.quaternion_conjugate(q1)
                    )[:3]
    
    vector_len = np.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
    vector = unit_vector * vector_len
    return vector
    
def binary(num):
    return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))


def rgbval_to_rgb(rgbval):
    binTest = binary(rgbval)
    bin1 = binTest[ 0: 8]
    bin2 = binTest[ 8:16]
    bin3 = binTest[16:24]
    #bin4 = binTest[24:32]
    rgb = [int(bin1,2),int(bin2,2),int(bin3,2)]
    return rgb
    
def expand_names(names):
    names_new = []
    for name in names:
        if name == "rgb":
            names_new = names_new + ["red", "green", "blue"]
        else:
            names_new.append(name)
    return names_new

class RosNode:
    def __init__(self):
        rospy.loginfo("Starting Node: pc2_to_file")        
        rospy.init_node("pc2_to_file")        
        # Load ROS Params
        
        # Load Class Params        
        self.is_first_callback = True
        self.n_points = 0
        #self.f = open(self.save_to + "point_cloud_%d"%(self.get_counter()) + self.save_as, "w")         
        
        # ROS Subs
        self.listener = tf.TransformListener()
        rospy.Subscriber( "/rs435_camera/depth/color/points", PointCloud2, self.callback )
        rospy.spin()
        
    def callback(self, msg):
        if self.is_first_callback:
            # build header with field data
            self.is_first_callback = False            
            names = [field.name for field in msg.fields]
            names = expand_names(names)
            print names
            #self.writeHeader(names)
            
        while not rospy.is_shutdown():
            try:
                (trans, quat) = self.listener.lookupTransform("ekf_odom", msg.header.frame_id, rospy.Time(0) )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            break
        
        points = pc2msg_to_points(msg)
        points_new = transform_points(trans, quat, points)
        #self.write(points_new)
        #print(points[0])
        
    def writeHeader(self, names):
        a = 'ply'
        b = 'format ascii 1.0'
        c = 'element vertex %s'%(str(len(self.data)))
        lines = [a, b, c]
        for name in names:
            desc  = "property float %s"%(name)
            lines.append(desc)
        j = 'end_header'
        lines.append(j)
        for line in lines:
            self.f.write(line + '\n')
        
        
        
if __name__=="__main__":
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        node.f.close()        
        pass
        
        
        

#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import tf

#import io
import time
import struct
import in_place
#import fileinput
import numpy as np
from datetime import datetime
#from pyquaternion import Quaternion

#def pc2msg_to_points_old(msg):
#    points = []
#    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
#            x = point[0]
#            y = point[1]
#            z = point[2]
#            points.append([x,y,z])
#            #print(point)
#    return points
#    
#def pc2msg_to_points(msg):
#    points = np.array(sensor_msgs.point_cloud2.read_points(msg, skip_nans=True))
#    return points
    
def pc2msg_to_points(msg):
    points = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        values = []
        for value in point:            
            values.append(value)
        points.append(values)
    return points

def expand_color(points, names):
    i_rgb = None
    try:
        i_rgb = names.index("rgb")
    except ValueError:
        pass
    points_new = []
    if i_rgb is not None:
        for point in points:
            point_new = point[0:i_rgb]
            point_new = point_new + rgbval_to_rgb(point[i_rgb])
            point_new = point_new + point[i_rgb+1:]
            points_new.append(point_new)
    else:
        return points
    return points_new
    
def transform_points(trans, quat, points):
    """ transfomrs the xyz of point data, (rate = 45 using sicktim laser) """ 
    points_new = []
    for point in points:
        xyz = point[0:3]
        xyz = qv_mult(quat, xyz)
        xyz = translation(trans, xyz)       
        point_new = xyz + point[3:]
        points_new.append(point_new)
    return points_new
    
def translation(trans, xyz):
    x = trans[0] + xyz[0]
    y = trans[1] + xyz[1]
    z = trans[2] + xyz[2]
    return [x,y,z]
    
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
    return list(vector)
    
def binary(num):
    return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))

def rgbval_to_rgb(rgbval):
    binTest = binary(rgbval)
    bin1 = binTest[ 0: 8]
    bin2 = binTest[ 8:16]
    bin3 = binTest[16:24]
    bin4 = binTest[24:32]
    rgb = [int(bin2,2),int(bin3,2),int(bin4,2)] #This is the read order for rgb
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
        self.target_frame = rospy.get_param('~target_frame')
        pc2_topic         = rospy.get_param('~pc2_topic')
        save_to           = rospy.get_param('~save_to')
        
        # Load Class Params
        now = datetime.now()
        self.date_time = now.strftime("%Y-%m-%d-%H-%M-%S")
        self.filepath = save_to + "point-cloud_%s.ply"%(self.date_time)

        self.is_first_callback = True
        self.n_points = 0
        self.names_old = None
        self.names_new = None
        self.seq = 0
        self.times = []
        
        f = open(self.filepath, "w")
        f.close()
        
        # ROS Process
        self.listener = tf.TransformListener()
        rospy.Subscriber( pc2_topic, PointCloud2, self.callback )
        #rospy.Subscriber( "pointcloud1", PointCloud2, self.callback )
        rospy.on_shutdown(self.my_hook)
        rospy.spin()
    
    def my_hook(self):
        # Wait for file to fisnish writing
#        while not rospy.is_shutdown:
#            try:
#                self.f.close()
#            except:
#                time.sleep(0.1)
#                continue
#            break
        rospy.loginfo("shutting down: writing element vertex")    
        # Replace point-count line
        i_lines = 0
        with in_place.InPlace(self.filepath) as myfile:
            for line in myfile:
                if i_lines == 2:
                    line = 'element vertex %s\n'%(str(self.n_points))
                else:
                    pass
                i_lines += 1
                myfile.write(line)

    def writeHeader(self, names):
        a = 'ply'
        b = 'format ascii 1.0'
        c = 'element vertex %s'%(str(self.n_points))
        lines = [a, b, c]
        for name in names:
            if name == "red" or name == "green" or name == "blue":
                desc  = "property uchar %s"%(name)
            else:
                desc  = "property float %s"%(name)
            lines.append(desc)
        j = 'end_header'
        lines.append(j)
        with open(self.filepath, "a") as myfile:
            for line in lines:
                myfile.write(line + '\n')
            
    def writePoints(self, points):
        with open(self.filepath, "a") as myfile:
            for line in points:
                str_list = [str(val) for val in line]
                my_line = " ".join(str_list) + "\n"
                myfile.write(my_line)
    
    def check_rate(self, time):
        self.times.append(time)
        if len(self.times) > 100:
            self.times.pop(0)     
        rate = 1/(sum(self.times)/len(self.times))
        return rate         
        
    def callback(self, msg):
        t0 = time.time()
        if self.is_first_callback:
            # Build header with field data
            self.is_first_callback = False            
            self.names_old = [field.name for field in msg.fields]
            self.names_new = expand_names(self.names_old)
            self.writeHeader(self.names_new)
            
        while not rospy.is_shutdown():
            try:
                (trans, quat) = self.listener.lookupTransform( self.target_frame, msg.header.frame_id, rospy.Time(0) )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            break
        
        points = pc2msg_to_points(msg)                 # Extract info from message data 
        points = transform_points(trans, quat, points) # Transform position data
        points = expand_color(points, self.names_old)  # Unpack RGB data
        self.writePoints(points)                       # Write points to file
        self.n_points += len(points)                   # Update pointcloud size 
        
        t1 = time.time()
        #rospy.loginfo("rate = %s"%str(self.check_rate(t1-t0)))
        rospy.loginfo(points[0])

        
if __name__=="__main__":
    try:
        node = RosNode()
    except rospy.ROSInterruptException:        
        pass
        
        
        

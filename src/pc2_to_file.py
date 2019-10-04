#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import tf

import io
import time
import struct
import fileinput
import numpy as np
from datetime import datetime
from pyquaternion import Quaternion

def pc2msg_to_points_old(msg):
    points = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            x = point[0]
            y = point[1]
            z = point[2]
            points.append([x,y,z])
            #print(point)
    return points
    
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
    
def transform_points2(trans, quat, points):
    """Attempt to speed up tf_points using list comp, cant quite get it to 
       work (rate = ??)"""
    #print points[0]
    points_new = [qv_mult(quat, point[0:3]) for point in points]  + [1,2,3] 
    #print points_new[0]
    points_new = [[1,2,3] for point in points]  + [1,2,3] 
    #points = [np.concatenate(translation(trans, point[0:3]), point[3:]) for point in points]
    return points
    
def transform_points3(trans, quat, points):
    """Attempt to speed up tf_points using another quaternion calc but it
       is actually slower (rate = 33)"""
    points = np.array(points)
    vs = points[:,0:3]
    # Rotate Points
    w,x,y,z = quat[3], quat[0], quat[1], quat[2]
    q = Quaternion(w,x,y,z)
    vs_mod = []
    for v in vs:
        vs_mod.append(q.rotate(v))
    # Translate points
    vs_mod = np.array(vs_mod) + trans
    # Recombine
    points[:,0:3] = vs_mod
    #print points[0]    
    return points
    
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
        now = datetime.now()
        self.date_time = now.strftime("%Y-%m-%d-%H-%M-%S")
        self.saveto = "/home/arc/pointclouds/jason/"
        self.filepath = self.saveto + "point-cloud_%s.ply"%(self.date_time)

        self.is_first_callback = True
        self.n_points = 0
        self.names_old = None
        self.names_new = None
        self.seq = 0
        self.times = []
        
        self.f = open(self.filepath, "w")         
        
        # ROS Process
        self.listener = tf.TransformListener()
        rospy.Subscriber( "/rs435_camera/depth/color/points", PointCloud2, self.callback )
        #rospy.Subscriber( "pointcloud1", PointCloud2, self.callback )
        #rospy.on_shutdown(self.my_hook)
        rospy.spin()
#    
#    def my_hook(self):
#        self.f.close()
#
#        with open(self.filepath, 'a') as f:
#            try:
#                f.writelines('element vertex %s\n'%(str(self.n_points)))[2]
#            except IOError:
#                f.close()
#            finally:
#                f.close()
    
    def my_hook(self):
        #while self.is_writing:
        self.f.close()
        i = 0
        for  line in fileinput.FileInput(self.filepath, inplace=1):
            if line == 2:
                line = 'element vertex %s\n'%(str(self.n_points))
                break
            else:
                i += 1
        
        

    def writeHeader(self, names):
        a = 'ply'
        b = 'format ascii 1.0'
        c = 'element vertex %s'%(str(self.n_points))
        lines = [a, b, c]
        for name in names:
            desc  = "property float %s"%(name)
            lines.append(desc)
        j = 'end_header'
        lines.append(j)
        for line in lines:
            self.f.write(line + '\n')
            
    def writePoints(self, points):
        for line in points:
            for value in line:
                self.f.write(str(value) + " ")
            self.f.write("\n")
    
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
                (trans, quat) = self.listener.lookupTransform("ekf_odom", msg.header.frame_id, rospy.Time(0) )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            break
        
        points = pc2msg_to_points(msg)                 # Extract info from message data 
        points = transform_points(trans, quat, points) # Transform position data
        points = expand_color(points, self.names_old)  # Unpack RGB data
        self.writePoints(points)                       # Write points to file
        self.n_points += len(points)                   # Update pointcloud size 
        
        t1 = time.time()
        rospy.loginfo("rate = %s"%str(self.check_rate(t1-t0)))

        
if __name__=="__main__":
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        #node.f.close()        
        pass
        
        
        

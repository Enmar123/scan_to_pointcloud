#!/usr/bin/env python

"""
My notes:
Tested up to 20000 pts at 1hz
fails at 20000 pts at 5hz
"""
import rospy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
import sensor_msgs.point_cloud2
import tf

import threading
import struct
import in_place
import numpy as np
from datetime import datetime
    
def pc2_read_data(msg):
    points = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        values = []
        for value in point:            
            values.append(value)
        points.append(values)
    return points
    
def pc2_to_pts(msg):
    pts = pc2_read_data(msg)     # Extracts field data based on dtype
    pts = convert_rgb(pts, msg)  # Unpacks rgb field if it is present
    print pts[0]    
    return pts
    
### RGB section ### ----------------------------------------------------------
    
def convert_dtype6(i):         #for point field with dtype 6
    i = i % 4294967296
    n4 = i % 256
    i = i / 256
    n3 = i % 256
    i = i / 256
    n2 = i % 256
    n1 = i / 256
    #return (n1,n2,n3,n4)
    rgb = [n2, n3, n4]
    return rgb

def binary(num):
    return ''.join(bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))

def convert_dtype7(rgbval):   # for point field with dtype 7
    binTest = binary(rgbval)
    bin1 = binTest[ 0: 8]
    bin2 = binTest[ 8:16]
    bin3 = binTest[16:24]
    bin4 = binTest[24:32]
    rgb = [int(bin2,2),int(bin3,2),int(bin4,2)] #This is the read order for rgb
    return rgb
    
def convert_type(dtype, rgbval):
    if dtype == 6:
        return convert_dtype6(rgbval)
    elif dtype == 7:
        return convert_dtype7(rgbval)
    
def convert_rgb(pts, msg):
    i_rgb = 0
    dtype = None
    for field in msg.fields:
        if field.name == "rgb":
            dtype = field.datatype
            break
        else:
            i_rgb += 1
        
    if dtype == None:
        return pts
    else:
        pts_new = []
        for pt in pts:
            rgbval = pt[i_rgb]
            rgb = convert_type(dtype, rgbval)
            pt_new = pt[0:i_rgb] + rgb + pt[i_rgb+1:]
            pts_new.append(pt_new)
        return pts_new
  
def expand_names(names):
    names_new = []
    for name in names:
        if name == "rgb":
            names_new = names_new + ["red", "green", "blue"]
        else:
            names_new.append(name)
    return names_new      
    
### XYZ Section ### -----------------------------------------------------------
    
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
    
def thread_function(ptss, index, trans, quat, points):
    pts = transform_points(trans, quat, points)     # Transform position data
    ptss[index] = pts                               # Store transformed points
    
def pts_tf_threader(n_threads, trans, quat, pts):
    ptss= [None]*n_threads
    # figure out how to divide up the points
    n_pts = len(pts)  
    l = n_pts/n_threads                 # len of typical thread
    ln = n_pts - l*(n_threads-1)        # len of longer thread
    # dividee up the points
    for i in range(n_threads):
        if i == 0:
            ptss[i] = pts[0:ln]
        else:
            ptss[i] = pts[ln + (i-1)*l:ln + i*l]
    
    threads = list()
    for index in range(n_threads):
        x = threading.Thread(target=thread_function, args=(ptss, index, trans, quat, ptss[index]))
        threads.append(x)
        x.start()

    for index, thread in enumerate(threads):
        thread.join()
        
    pts = [pt for pt in pts for pts in ptss]
    return pts


### Rosnode Section ### ------------------------------------------------------

class RosNode:
    def __init__(self):
        rospy.loginfo("Starting Node: pc2_to_file")        
        rospy.init_node("pc2_to_file")        
        # Load ROS Params
        self.target_frame = rospy.get_param('~target_frame')
        pc2_topic         = rospy.get_param('~pc2_topic')
        save_to           = rospy.get_param('~save_to')
        
#        self.target_frame = "ekf_odom"
#        pc2_topic         = "rs435_camera/depth/color/points"
#        save_to           = "/home/arc/pointclouds/jason/"
        
        # Load Class Params
        now = datetime.now()
        self.date_time = now.strftime("%Y-%m-%d-%H-%M-%S")
        self.filepath = save_to + "point-cloud_%s.ply"%(self.date_time)

        self.is_first_callback = True
        self.n_points = []
        self.names_old = None
        self.names_new = None
        self.seq = 0
        self.times = []
        self.f = None
        self.is_exiting = False
                
        # ROS Process
        self.listener = tf.TransformListener()
        rospy.Subscriber( pc2_topic, PointCloud2, self.callback )
        self.pub = rospy.Publisher( "is_recorded", Empty, queue_size=10 )
        rospy.on_shutdown(self.my_hook)
        rospy.spin()
    
    def my_hook(self):
        self.is_exiting = True
        if self.f is None:
            pass
        else:
            i_lines = 0
            with in_place.InPlace(self.filepath) as myfile:
                for line in myfile:
                    if i_lines == 2:
                        line = 'element vertex %s\n'%(str(sum(self.n_points)))
                    else:
                        pass
                    i_lines += 1
                    myfile.write(line)
        print("pts =", (sum(self.n_points)))

    def writeHeader(self, names):
        a = 'ply'
        b = 'format ascii 1.0'
        c = 'element vertex %s'%("000")
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
        if self.is_first_callback:
            # Create file
            self.f = open(self.filepath, "w")
            self.f.close()
            # Build header with field data
            self.is_first_callback = False            
            self.names_old = [field.name for field in self.msg.fields]
            self.names_new = expand_names(self.names_old)
            self.writeHeader(self.names_new)
            
        with open(self.filepath, "a") as myfile:
            for line in points:
                str_list = [str(val) for val in line]
                my_line = " ".join(str_list) + "\n"
                myfile.write(my_line)
                
    def get_tf(self):
        while not rospy.is_shutdown():
            try:
                (trans, quat) = self.listener.lookupTransform( self.target_frame, self.msg.header.frame_id, rospy.Time(0) )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            break
        return (trans, quat)
    
        
    def callback(self, msg):
        if self.is_exiting == True:
            pass                             # ignore callbacks if exiting
        else:
            self.n_points.append(int(len(msg.data)/msg.point_step))            # Update pointcloud size 
            self.msg = msg
            if msg.data == []:              # if message holds no data
                pass                        # do nothing
            else:
                (trans, quat) = self.get_tf()                  # Obtain the transform between the points frame_id and the goal frame_id
                        
                points = pc2_to_pts(msg)                                        # Extract point info from message data 
                #points = transform_points(trans, quat, points)                  # Transform position data
                points = pts_tf_threader(4, trans, quat, points)           # Transform position data (threaded)
                self.writePoints(points)                                        # Write points to file
                self.pub.publish(Empty())                                       # Pub when data is_recorded (to check hz)

        
if __name__=="__main__":
    try:
        node = RosNode()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        pass   

        
        
        

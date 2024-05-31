import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs


import numpy as np
# import open3d as o3d

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        # self.o3d_pcd = o3d.geometry.PointCloud()


        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/camera/depth/color/points',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        self.points=None
                
    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from 
        # the ROS1 package. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        gen = read_points(msg, skip_nans=True)
        int_data = list(gen)
        # print(int_data)
        xyz = np.empty((len(int_data), 3))
        rgb = np.empty((len(int_data), 3))
        idx = 0
        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            # xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            # rgb = np.append(rgb,[[r,g,b]], axis = 0)
            
            xyz[idx] = [x[0],x[1],x[2]]
            rgb[idx] = [r,g,b]
            idx = idx + 1
        # anything that is infinity is made as max range of depth as per datasheet

        # xyz[np.isinf(xyz)] = 8
        self.points=xyz # Point cloud from ROS topic 
        # print(self.points.shape)
        # print(xyz)
        px=xyz[:,0]
        py=xyz[:,1]
        pz=xyz[:,2]
        avg_x = np.mean(px)
        avg_y = np.mean(py)
        avg_z = np.mean(pz)
        # print("Average X: ", np.round(avg_x, 2),"Average Y: ",np.round(avg_y, 2),"Average Z: ",np.round(avg_z, 2))
        
        #FLITER Y VAL 
        min_y = 0.2       # minimum value for y
        max_y = 0.1
        # mask_y = (py >= min_y) & (py <= max_y)
        # mask_y= (np.abs(py) >= min_y) & (np.abs(py) <= max_y)
        mask_y= (np.abs(py) <= max_y)
        y_filtered= xyz[mask_y]

        #FILTER X VALS 
        px=y_filtered[:,0]
        py=y_filtered[:,1]
        pz=y_filtered[:,2]
        #x vals
        min_x = -0.2       # minimum value for x
        max_x = 0.2
        mask_x = (np.abs(px)<=max_x)
        x_filtered= y_filtered[mask_x]

        #FILTER Z VALS
        px=x_filtered[:,0]
        py=x_filtered[:,1]
        pz=x_filtered[:,2]

        # print(pz)
        min_z_value = 1.0
        mask_z = pz < min_z_value
        z_filtered = pz[mask_z]

        points_filtered = x_filtered[mask_z]
        #Print max value of z from z_filtered
        z_max= np.max(z_filtered)

        # Print the filtered z values
        print(z_filtered,np.mean(z_filtered),"MAX_Z",z_max)

        # Optionally, print the corresponding x, y, z points
        # print(points_filtered)


        #  #converting from ROS to PX4 coordianates 
        # data[data[:, 2] < 2] = [np.nan, np.nan, np.nan]
        # data[data[:, 2] > 6] = [np.nan, np.nan, np.nan]
        # data = data[:, [0, 1, 2]]  * np.array([1, -1, -1])
        # #PX4 coordinates to World Coordinates 
        # data = data[:, [1, 0, 2]]  * np.array([1, 1, -1])

        
      
        


## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later: 
# https://github.com/ros2/common_interfaces
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt



def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    # print(pcd_listener.points)
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

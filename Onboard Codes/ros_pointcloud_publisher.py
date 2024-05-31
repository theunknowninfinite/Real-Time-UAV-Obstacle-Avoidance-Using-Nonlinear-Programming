import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
from std_msgs.msg import Float32MultiArray

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

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        # self.o3d_pcd = o3d.geometry.PointCloud()

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/camera/depth/color/points',                      # topic
            self.listener_callback,      # Function to call
            10                           # QoS
        )
        self.points=None

        # Set up a publisher to publish the obstacle point
        self.obstacle_publisher = self.create_publisher(
            Float32MultiArray, '/obstacle_point', qos_profile)

                
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
        # self.points = xyz # Point cloud from ROS topic 

        # Filter 
        # Remove all values of xyz, where abs(y) > y_threshold
        y_threshold = 0.01
        xyz = xyz[np.abs(xyz[:, 1]) < y_threshold]

        # Filter 
        # Remove all values of xyz, where abs(x) > x_threshold
        x_threshold = 0.5
        xyz = xyz[np.abs(xyz[:, 0]) < x_threshold]

        # Filter 
        # Remove all values of xyz, where z > z_threshold
        z_threshold = 2.5
        xyz = xyz[xyz[:, 2] < z_threshold]

        # Filter 
        # Remove all values of xyz, where x y and z are all 0
        xyz = xyz[~np.all(xyz == 0, axis=1)]

        print("Number of points: ", len(xyz))

        # If there are points which 
        if len(xyz) > 0:
            # print("Number of points before filtering: ", len(xyz))

            # Use the Radius Outlier Removal filter to remove outliers
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz)
            # cl, ind = pcd.remove_radius_outlier(nb_points=10, radius=0.1)
            # xyz = np.asarray(pcd.points)

            # # Apply statistical oulier removal
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz)
            # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            # xyz = np.asarray(pcd.points)

            # Apply voxel downsampling
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz)
            # pcd = pcd.voxel_down_sample(voxel_size=0.05)
            # xyz = np.asarray(pcd.points)

            # In every xyz point, check how many points are near it in a radius of 0.1
            # If there are more than 10 points near it, keep it
            # If there are less than 10 points near it, remove it
            # xyz = np.array([point for point in xyz if np.sum(np.linalg.norm(xyz - point, axis=1) < 0.1) > 10])

            # print("Number of points after filtering: ", len(xyz))

            def gaussian_filter(xyz, sigma=0.1):
                # Create a copy of the xyz array
                xyz_filtered = np.copy(xyz)

                # For every point in the xyz array
                for i, point in enumerate(xyz):
                    # Calculate the distance between the point and all other points
                    dist = np.linalg.norm(xyz - point, axis=1)

                    # Calculate the gaussian kernel
                    kernel = np.exp(-dist / (2 * sigma ** 2))

                    # Calculate the weighted average of the z values
                    xyz_filtered[i, 2] = np.sum(kernel * xyz[:, 2]) / np.sum(kernel)

                return xyz_filtered

            # Apply a gaussian filter to the points
            xyz = gaussian_filter(xyz, sigma=0.1)

            # Print the average z value
            # print("Average z: ", np.round(np.mean(xyz[:, 2]), 2))    

            # Print the maximum z value
            print("Max z: ", np.round(np.max(xyz[:, 2]), 2), ", the full point: ", np.round(xyz[np.argmax(xyz[:, 2]), :], 2))

            # Print the minimum z value
            print("Min z: ", np.round(np.min(xyz[:, 2]), 2), ", the full point: ", np.round(xyz[np.argmin(xyz[:, 2]), :], 2))

            # Print all points at z = 0
            # print("All points at z = 0: ", np.round(xyz[xyz[:, 2] == 0, :], 2))

            # Print the average of z values where z > 0
            if np.round(np.max(xyz[:, 2]), 2) > 0:
                print("Average z where z > 0: ", np.round(np.mean(xyz[xyz[:, 2] > 0, 2]), 2))
            else:
                print("Average z where z > 0: 0")


            # Convert camera coordinates to drone coordinates in NED frame
            # camera x -> drone y
            # camera y -> drone z
            # camera z -> drone x
            xyz = xyz[:, [2, 0, 1]] 

            # Take 10 closest points to the drone, and take their average to get the point closest to the drone
            closest_points = xyz[np.argsort(np.linalg.norm(xyz, axis=1))[:10]]

            # Calculate the average of the closest points
            obstacle_predicted_point = np.mean(closest_points, axis=0)

            # Convert the obstacle_predicted_point to float32
            # obstacle_predicted_point = obstacle_predicted_point.astype(np.float32)

            # Print the predicted obstacle point
            print("Predicted obstacle point: ", np.round(obstacle_predicted_point, 2))

            # Print the point closest to the drone
            print("Closest point to drone: ", np.round(xyz[np.argmin(np.linalg.norm(xyz, axis=1)), :], 2))

            # Publish the obstacle point
            msg = Float32MultiArray()
            msg.data = obstacle_predicted_point.tolist()
            self.obstacle_publisher.publish(msg)
            
        else:
            print("Enviorment is obstacle free")

            # Publish no obstacle point
            msg = Float32MultiArray()
            msg.data = [-1.0, -1.0, -1.0]
            self.obstacle_publisher.publish(msg)


        print("\n")

        #  #converting from ROS to PX4 coordianates 
        # data[data[:, 2] < 2] = [np.nan, np.nan, np.nan]
        # data[data[:, 2] > 6] = [np.nan, np.nan, np.nan]
        # data = data[:, [0, 1, 2]]  * np.array([1, -1, -1])
        # #PX4 coordinates to World Coordinates 
        # data = data[:, [1, 0, 2]]  * np.array([1, 1, -1])

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

# import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import time 


class DepthEstimation(Node):
    "Node for accessing stereo camera data and computing depth"

    def __init__(self)->None:
        super().__init__("depth_estimation")

        # Create subscribers
        self.stereo_camera_front_subscriber = self.create_subscription(
            Image, "/stereo_front", 
            self.stereo_camera_cb, 10)

    def stereo_camera_cb(self, msg):
        # Access the data field
        gray_data = msg.data
        print(gray_data[0])
        print("\n\n\n\n\n\n\n")
        time.sleep(3)

def main(args=None) -> None:
    print("Accessing stereo camera data")
    rclpy.init(args=args)
    depth_estimation = DepthEstimation()
    rclpy.spin(depth_estimation)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
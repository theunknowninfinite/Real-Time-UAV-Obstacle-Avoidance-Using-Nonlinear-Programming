import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.counter = 0
        self.wp = 0
        self.dt = 0.050
        self.wait_time = 3

        # Initialize trajectory parameters
        self.theta = 0
        self.step_size = 2*np.pi/1000

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_setpoint(self, position: list, velocity: list, acceleration: list, yaw: float, yaw_rate: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.yaw = yaw
        msg.yawspeed = yaw_rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {np.round(position, 2)}")


    # def publish_position_setpoint(self, x: float, y: float, z: float):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = [x, y, z]
    #     msg.yaw = 1.57079  # (90 degree
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def dist2wp(self, wp_x, wp_y, wp_z) -> float:
        """Calculate Euclidean distance to the waypoint"""
        return float((wp_x - self.vehicle_local_position.x)**2 +
                     (wp_y - self.vehicle_local_position.y)**2 + 
                     (wp_z - self.vehicle_local_position.z)**2 )**0.5


    def timer_callback(self) -> None:
        """Callback function to publish drone postion"""
        self.publish_offboard_control_heartbeat_signal()
        # self.engage_offboard_mode()

        # Check if the drone is in offboard mode
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            # Define a constant height above which the trajectory should be executed
            z_const = - 1.5
            # Define the radius of the circular trajectory
            r = 0.5
            # Define the frequency of the z axis sinusoidal trajectory
            frequency = 4.0
            # Define amplitude of the z axis sinusoidal trajectory
            amplitude = 0.1

            # Get position setpoints
            self.x = r * np.cos(self.theta)
            self.y = r * np.sin(self.theta)
            self.z = -(amplitude * np.sin(frequency*self.theta)) + z_const
            position = [self.x, self.y, self.z]

            # Calculate the velocity setpoints
            self.vx = -r * np.sin(self.theta)
            self.vy = r * np.cos(self.theta)
            self.vz = -1 * np.cos(frequency*self.theta)
            velocity = [self.vx, self.vy, self.vz]

            # Calculate the acceleration setpoints
            self.ax = -r * np.cos(self.theta)
            self.ay = -r * np.sin(self.theta)
            self.az = 1 * np.sin(frequency*self.theta)
            acceleration = [self.ax, self.ay, self.az]

            # Calculate the yaw angle and yaw rate to the center of the circle
            self.yaw = np.pi + np.arctan2(self.y, self.x) 
            # Wrap it from -pi to pi
            self.yaw = (self.yaw + np.pi) % (2*np.pi) - np.pi
            
            # Calculate the yaw rate
            yaw_next = np.pi + np.arctan2(self.y + self.vy*self.dt, self.x + self.vx*self.dt)
            # Wrap it from -pi to pi
            yaw_next = (yaw_next + np.pi) % (2*np.pi) - np.pi

            # Calculate the yaw rate
            yaw_differece = yaw_next - self.yaw
            # Wrap it from -pi to pi
            yaw_differece = (yaw_differece + np.pi) % (2*np.pi) - np.pi
            self.yaw_rate = yaw_differece / self.dt

            # Publish the trajectory setpoint
            self.publish_setpoint(position, velocity, acceleration, self.yaw, self.yaw_rate)

            # Increment theta
            self.theta += self.step_size

            # Wrap theta from 0-2pi
            self.theta = self.theta % (2*np.pi)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

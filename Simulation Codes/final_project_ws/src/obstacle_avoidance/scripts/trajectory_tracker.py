import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

from gekko import GEKKO
import matplotlib.pyplot as plt

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
        self.dt = 0.05
        self.wait_time = 7 / self.dt

        # Set the goal variables
        self.goal_points = [(3, 0, -1), (0, 0, -1)]
        # self.goal_points = [(10.0, 0.0, -1.0), (0.0, 10.0, -1.0), (-10.0, 0.0, -1.0)]
        self.goal_counter = 0
        self.goal_wait_counter = 1
        self.goal_reached = False
        self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
        print('Goal Position: ', self.x_goal, self.y_goal, self.z_goal)

        # Flags
        self.ready = False # Flag to check if the drone is ready to start the mission

        # Variables for waypoint navigation
        self.waypoints = False
        self.waypoint_counter = 1   # Set to 1 to make it possible to calculate yaw w.r.t the previous waypoint


        # Create a timer to publish control commands
        self.controller_timer = self.create_timer(self.dt, self.controller)
        self.trajectory_planner_timer = self.create_timer(self.dt, self.trajectory_generator)

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
        # self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def trajectory_generator(self):
        """
        Generate a trajectory for the drone to follow.

        self.waypoints is set to False if the trajectory is not yet generated
        Once the trajectory is generated, self.waypoints is a list of tuples 
        containing the x, y, z coordinates of the waypoints
        """
        
        # Generate waypoints only if:
        # > the drone is ready and 
        # > the waypoints are not yet generated
        # > the goal is not yet reached
        if self.ready and not self.waypoints and not self.goal_reached:

            # Define the initial conditions
            X0 = self.vehicle_local_position.x
            Y0 = self.vehicle_local_position.y
            Z0 = self.vehicle_local_position.z
            vx0, vy0, vz0 = 0, 0, 0

            print("Optimizer starting position: ", X0, Y0, Z0)

            # Define the final conditions
            Xf = self.x_goal
            Yf = self.y_goal
            Zf = self.z_goal

            # Set the limits
            v_max, v_min = 3, -3
            a_max, a_min = 1, -1

            m = GEKKO(remote=False)

            # Define the time points
            steps = min(int(((X0-Xf)**2 + (Y0-Yf)**2 + (Z0-Zf)**2)**0.5)+5, 10)
            T = 5
            m.time = np.linspace(0, T, steps)

            # Define the state variables
            X = m.Var(value=X0)
            Y = m.Var(value=Y0)
            Z = m.Var(value=Z0)
            vx = m.Var(value=vx0, lb=v_min, ub=v_max)
            vy = m.Var(value=vy0, lb=v_min, ub=v_max)
            vz = m.Var(value=vz0, lb=v_min, ub=v_max)

            # Define the control variables
            ax = m.MV(value=0, lb=a_min, ub=a_max)
            ay = m.MV(value=0, lb=a_min, ub=a_max)
            az = m.MV(value=0, lb=a_min, ub=a_max)
            ax.STATUS, ay.STATUS, az.STATUS = 1, 1, 1            

            tf = m.FV(value=1, lb=0.1, ub=T)
            tf.STATUS = 1

            # Define the differential equations
            m.Equation(X.dt() == vx*tf)
            m.Equation(Y.dt() == vy*tf)
            m.Equation(Z.dt() == vz*tf)

            m.Equation(vx.dt() == ax*tf)
            m.Equation(vy.dt() == ay*tf)
            m.Equation(vz.dt() == az*tf)

            # Define the final conditions
            m.fix(X, pos=len(m.time)-1, val=Xf)
            m.fix(Y, pos=len(m.time)-1, val=Yf)
            m.fix(Z, pos=len(m.time)-1, val=Zf)
            m.fix(vx, pos=len(m.time)-1, val=0)
            m.fix(vy, pos=len(m.time)-1, val=0)
            m.fix(vz, pos=len(m.time)-1, val=0)

            # Calculate the distance between the initial and final points
            distance = np.sqrt((Xf-X0)**2 + (Yf-Y0)**2 + (Zf-Z0)**2)
            # Set the speed to reach the final point
            velocity = 0.5

            # Define your scheduled time to reach the final conditions
            tf_scheduled = distance / velocity

            A = 1.0 # Define the weight for prioritizing faster trajectories, maximizing the speed to reach final conditions

            mayers_term = A*(tf-tf_scheduled)**2

            cost = mayers_term #+ desired_position

            # Define the objective
            m.Obj(cost)

            # Solve the optimization problem
            m.options.IMODE = 6
            m.options.SOLVER = 1
            m.solve(disp=False)

            # Print the minimum cost
            print('Minimum Cost: ' + str(m.options.OBJFCNVAL))

            # Print final travel time
            print('Final Time: ' + str(tf.value[0]))

            # Given the X, Y, Z points, interpolate points between each point to get a smooth trajectory
            self.waypoints = [(X[0], Y[0], Z[0])]

            # Make steps a function between the initial and final point 
            steps = min(int(((X0-Xf)**2 + (Y0-Yf)**2 + (Z0-Zf)**2)**0.5)+2, 10)

            for i in range(len(X)-1):
                # Make sure to not include the first point, as it is already included
                x = np.linspace(X[i], X[i+1], steps)[1:]
                y = np.linspace(Y[i], Y[i+1], steps)[1:]
                z = np.linspace(Z[i], Z[i+1], steps)[1:]

                # Append the waypoint only if it is not equal to the previous waypoint
                if (x[0], y[0], z[0]) != self.waypoints[-1]:
                    self.waypoints += [(x[j], y[j], z[j]) for j in range(len(x))]
            print("Number of Waypoints: ", len(self.waypoints))
            
            # Make a 2D plot of the position
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoints))], [v[0] for v in self.waypoints], 'r')
            # ax[0].set_title('X')
            # ax[1].plot([i for i in range(len(self.waypoints))], [v[1] for v in self.waypoints], 'g')
            # ax[1].set_title('Y')
            # ax[2].plot([i for i in range(len(self.waypoints))], [v[2] for v in self.waypoints], 'b')
            # ax[2].set_title('Z')
            # plt.show()

            # Print the waypoints
            # print("Waypoints: ", self.waypoints)

            # Generate the velocity setpoints
            dt = tf_scheduled / len(self.waypoints)
            self.waypoint_velocities = []
            for i in range(len(self.waypoints)-1):
                x = (self.waypoints[i+1][0] - self.waypoints[i][0]) / dt
                y = (self.waypoints[i+1][1] - self.waypoints[i][1]) / dt
                z = (self.waypoints[i+1][2] - self.waypoints[i][2]) / dt
                self.waypoint_velocities.append((x, y, z))
            # Pad the last velocity to match the number of waypoints
            while len(self.waypoint_velocities) < len(self.waypoints):
                # self.waypoint_velocities.append(self.waypoint_velocities[-1])
                self.waypoint_velocities.append((0.0, 0.0, 0.0))
            print("Number of Velocities: ", len(self.waypoint_velocities))

            # Make a 2D plot of the velocities
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoint_velocities))], [v[0] for v in self.waypoint_velocities], 'r')
            # ax[0].set_title('X Velocity')
            # ax[1].plot([i for i in range(len(self.waypoint_velocities))], [v[1] for v in self.waypoint_velocities], 'g')
            # ax[1].set_title('Y Velocity')
            # ax[2].plot([i for i in range(len(self.waypoint_velocities))], [v[2] for v in self.waypoint_velocities], 'b')
            # ax[2].set_title('Z Velocity')
            # plt.show()
                
            # Generate acceleration setpoints
            dt = tf_scheduled / len(self.waypoints)
            self.waypoint_accelerations = []
            for i in range(len(self.waypoint_velocities)-1):
                x = (self.waypoint_velocities[i+1][0] - self.waypoint_velocities[i][0]) / dt
                y = (self.waypoint_velocities[i+1][1] - self.waypoint_velocities[i][1]) / dt
                z = (self.waypoint_velocities[i+1][2] - self.waypoint_velocities[i][2]) / dt
                self.waypoint_accelerations.append((x, y, z))
            # Pad the last acceleration to match the number of waypoints
            while len(self.waypoint_accelerations) < len(self.waypoints):
                # self.waypoint_accelerations.append(self.waypoint_accelerations[-1])
                self.waypoint_accelerations.append((0.0, 0.0, 0.0))
            print("Number of Accelerations: ", len(self.waypoint_accelerations))

            # Make a 2D plot of the accelerations
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoint_accelerations))], [v[0] for v in self.waypoint_accelerations], 'r')
            # ax[0].set_title('X Acceleration')
            # ax[1].plot([i for i in range(len(self.waypoint_accelerations))], [v[1] for v in self.waypoint_accelerations], 'g')
            # ax[1].set_title('Y Acceleration')
            # ax[2].plot([i for i in range(len(self.waypoint_accelerations))], [v[2] for v in self.waypoint_accelerations], 'b')
            # ax[2].set_title('Z Acceleration')
            # plt.show()

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

    def publish_position_setpoint(self, position: list, velocity: list, acceleration: list, yaw: float):
        """Publish the trajectory setpoint."""

        print(f"Current Position: {self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z}")
        print(f"Publishing waypoint: {position[0], position[1], position[2]}")
        print(f"Publishing velocity: {np.round(velocity, 2)}")
        print(f"Publishing acceleration: {np.round(acceleration, 2)}")
        print(f"Publishing yaw: {np.round(yaw, 2)}")
        goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
        print('Goal Distance: ', goal_distance)

        msg = TrajectorySetpoint()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.yaw = yaw  # Always facing the goal point 
        msg.yawspeed = 0.1
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
    # def publish_position_setpoint(self, position: list, velocity: list, yaw: float):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = position
    #     msg.velocity = velocity
    #     msg.yaw = yaw  # Always facing the goal point 
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    #     self.get_logger().info(f"Publishing waypoint: {np.round(position, 2)}")
    #     self.get_logger().info(f"Publishing yaw: {np.round(yaw, 2)}")
    #     goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
    #     print('Goal Distance: ', goal_distance)
            
    # def publish_position_setpoint(self, position: list, yaw: float):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = position
    #     msg.yaw = yaw  # Always facing the goal point 
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    #     self.get_logger().info(f"Publishing waypoint: {np.round(position, 2)}")
    #     self.get_logger().info(f"Publishing yaw: {np.round(yaw, 2)}")
    #     goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
    #     print('Goal Distance: ', goal_distance)

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

    def controller(self) -> None:
        """Callback function to publish drone postion"""
        self.publish_offboard_control_heartbeat_signal()
        self.engage_offboard_mode()

        # Check if the drone is in offboard mode
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            # Print the current vehicle position
            # print('Current Position: ', self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z)
            # print('Goal Position: ', self.x_goal, self.y_goal, self.z_goal)
            # goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
            # print('Goal Distance: ', goal_distance)

            # GETTING THE DRONE READY TO START THE MISSION
            if not self.ready:

                # Distance threshold
                distance_threshold = 0.5
                goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
                print('Goal Distance: ', goal_distance)

                # Check if the drone has not reached to the goal point, then orient the drone to the goal
                if not self.goal_reached:

                    # Set the desired yaw
                    self.yaw_desired = np.arctan2(self.y_goal - self.vehicle_local_position.y, self.x_goal - self.vehicle_local_position.x)

                    # Set the waypoint to the current position
                    self.waypoint = [self.vehicle_local_position.x, 
                                     self.vehicle_local_position.y, 
                                     self.vehicle_local_position.z]
                    
                    # Set the waypoint velocity to zero
                    self.waypoint_velocity = [0.0, 0.0, 0.0]

                    # Set the waypoint acceleration to zero
                    self.waypoint_acceleration = [0.0, 0.0, 0.0]
                    
                    print('Adjusting drone to face the goal')
                            
                # Check if the drone is facing the goal point, and the drone is not at the goal point
                if np.abs(self.yaw_desired - self.vehicle_local_position.heading) < 0.1 and not self.goal_reached:
                    # Drone is now ready to start the mission
                    self.ready = True
                    print('Drone facing the goal point, ready to start the mission')
                    print('Waiting for waypoints...')

            # WAYPOINT NAVIGATION STATE
            # Check if the waypoints are generated
            if self.waypoints and self.ready:

                # Calculate the distance to the current waypoint
                distance_to_waypoint = self.dist2wp(self.waypoints[self.waypoint_counter][0], 
                                                    self.waypoints[self.waypoint_counter][1], 
                                                    self.waypoints[self.waypoint_counter][2])

                # Check if the drone is close to the waypoint
                if distance_to_waypoint < 0.2:
                    self.waypoint_counter += 1
                
                # Limit the waypoint counter
                self.waypoint_counter = min(self.waypoint_counter, len(self.waypoints)-1)

                # Calculate the desired yaw, facing the next waypoint
                self.yaw_desired = np.arctan2(self.waypoints[self.waypoint_counter][1] - self.waypoints[self.waypoint_counter-1][1], 
                                              self.waypoints[self.waypoint_counter][0] - self.waypoints[self.waypoint_counter-1][0])
                # Set the waypoint position
                self.waypoint = [self.waypoints[self.waypoint_counter][0],
                                 self.waypoints[self.waypoint_counter][1],
                                 self.waypoints[self.waypoint_counter][2]]
                
                # Set the waypoint velocity
                self.waypoint_velocity = [self.waypoint_velocities[self.waypoint_counter][0],
                                          self.waypoint_velocities[self.waypoint_counter][1], 
                                          self.waypoint_velocities[self.waypoint_counter][2]]
                
                # Set the waypoint acceleration
                self.waypoint_acceleration = [self.waypoint_accelerations[self.waypoint_counter][0],
                                              self.waypoint_accelerations[self.waypoint_counter][1], 
                                              self.waypoint_accelerations[self.waypoint_counter][2]]

                # Print distance to current waypoint
                print('Distance to waypoint: ', distance_to_waypoint)

                # Print how many waypoints have been reached
                print(f'Waypoint {self.waypoint_counter} reached out of {len(self.waypoints)-1} waypoints')

                # If at the final waypoint, set the waypoints to False
                if self.waypoint_counter == len(self.waypoints)-1:
                    self.waypoints = False
                    self.waypoint_counter = 1   # Set to 1 to make it possible to calculate yaw w.r.t the previous waypoint
                    self.ready = False
                    self.goal_reached = True
                    print('All waypoints reached, mission completed')

            # GOAL REACHED STATE
            if self.goal_reached:
                self.goal_wait_counter += 1

                # If the goal wait counter is greater than the wait time, switch to the next goal
                if self.goal_wait_counter > self.wait_time:
                    self.goal_wait_counter = 0
                    self.goal_reached = False

                    # # Increment and loop the goal counter
                    self.goal_counter += 1
                    self.goal_counter = self.goal_counter % len(self.goal_points)

                    # Set the new goal position
                    self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]

                    print('New goal position: ', self.x_goal, self.y_goal, self.z_goal)                    


            # Print the variables
            print('Current goal: ', self.x_goal, self.y_goal, self.z_goal)
            print('Ready: ', self.ready)
            print('Goal Reached: ', self.goal_reached)
            if self.waypoints:
                print('Waypoints: Yes')
            else:
                print('Waypoints: No')


            # Publish the position setpoint
            self.publish_position_setpoint(self.waypoint, self.waypoint_velocity,
                                           self.waypoint_acceleration, self.yaw_desired)
            print("\n")
            # self.publish_position_setpoint(self.waypoint, self.waypoint_velocity, self.yaw_desired)
            # self.publish_position_setpoint(self.waypoint, self.yaw_desired)

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
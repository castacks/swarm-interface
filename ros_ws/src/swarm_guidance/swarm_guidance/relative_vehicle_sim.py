#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,\
                            VehicleOdometry
from std_msgs.msg import String
import time, json
import numpy as np
from geometry_msgs.msg import PoseStamped
from swarm_custom_msgs.msg import NeighbourRelativePose
import numpy as np

class RelativeVehicleSim(Node):
    """Communication node for passing agent informations (should be used only in sim)"""

    def __init__(self) -> None:
        super().__init__('relative_vehicle_sim')

        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('total_vehicles', 0)
        vehicle_id_param_obj = self.get_parameter('vehicle_id')
        total_vehicles_param_obj = self.get_parameter('total_vehicles')
        self.vehicle_id = vehicle_id_param_obj.value
        self.total_vehicles = total_vehicles_param_obj.value
        print('the vehicle id is : ', self.vehicle_id)
        print('total vehicles are : ', self.total_vehicles, '\n\n')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        sim_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'px4_{self.vehicle_id}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'px4_{self.vehicle_id}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'px4_{self.vehicle_id}/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, f'px4_{self.vehicle_id}/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'px4_{self.vehicle_id}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)


        # print('i am failing here')
        self.vehicle_sim_pose_subscriber = self.create_subscription(
            PoseStamped, f'/drone{self.vehicle_id}/state/pose', self.vehicle_sim_pose_callback, sim_qos_profile
        )

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_sim_pose = PoseStamped()
        self.takeoff_height = -5.0


        # below are publishers and subscribers that are used to share position information (they are not related to px4)
        self.vehicle_position_publisher = self.create_publisher(
            String, f'multi_vehicle/position', qos_profile
        )

        self.vehicle_position_subscriber = self.create_subscription(
            String, f'multi_vehicle/position', self.vehicle_position_callback, qos_profile
        )
        self.vehicle_position = String()
        self.neighbour_vehicle_relative_position_info = np.zeros((self.total_vehicles, 3))

        # timer to publish position info
        self.position_timer = self.create_timer(0.08, self.position_timer_callback)


        # guidance algorithm timer
        self.guidance_timer = self.create_timer(0.2, self.guidance_timer_callback)

        # Create a timer to publish control commands
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.kp = 0.2

    def vehicle_position_callback(self, vehicle_position):
        # print('\nreceived position data : ', vehicle_position)
        '''Callback function for vehicle_position topic subscriber'''
        vehicle_position_info = json.loads(vehicle_position.data)
        # print('unpacked data : ', vehicle_position_info)

        if vehicle_position_info['vehicle_id'] != self.vehicle_id:
            neighbour_vehicle_position = vehicle_position_info['position']
            neighbour_vehicle_relative_position = [neighbour_vehicle_position[0] - self.vehicle_sim_pose.pose.position.y,
                                                   neighbour_vehicle_position[1] - self.vehicle_sim_pose.pose.position.x,
                                                   -(neighbour_vehicle_position[2] - self.vehicle_sim_pose.pose.position.z)]
            
            relative_pose_stamped = PoseStamped()
            relative_pose_stamped.header.frame_id = str(vehicle_position_info['vehicle_id'])
            relative_pose_stamped.pose.position.x = neighbour_vehicle_relative_position[0]
            relative_pose_stamped.pose.position.y = neighbour_vehicle_relative_position[1]
            relative_pose_stamped.pose.position.z = neighbour_vehicle_relative_position[2]
            

            # print(relative_pose_stamped.pose.position)

            self.neighbour_vehicle_relative_position_info[vehicle_position_info['vehicle_id']-1, :] = neighbour_vehicle_relative_position
    
    def vehicle_sim_pose_callback(self, vehicle_sim_pose):
        '''receive the pose of the vehicle directly from the simulation'''
        self.vehicle_sim_pose = vehicle_sim_pose
        # print(self.vehicle_sim_pose)

    def vehicle_odometry_callback(self, vehicle_odometry):
        '''Callback function for vehicle_odometry topic subscriber.'''
        self.vehicle_odometry = vehicle_odometry

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    
    def takeoff(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param5=1.0,
                                                    param6=1.0,
                                                    param7=1.0,)
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
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        # msg.position = [x, y, z]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        # msg.yaw = 1.57079  # (90 degree)
        msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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
        msg.target_system = self.vehicle_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def get_rotation_matrix_from_quaternion(self):
        quaternions = self.vehicle_odometry.q
        q0 = quaternions[0]
        q1 = quaternions[1]
        q2 = quaternions[2]
        q3 = quaternions[3]

        r0_0 = 2 * (q0 * q0 + q1 * q1) - 1
        r0_1 = 2 * (q1 * q2 - q0 * q3)
        r0_2 = 2 * (q1 * q3 + q0 * q2)

        r1_0 = 2 * (q1 * q2 + q0 * q3)
        r1_1 = 2 * (q0 * q0 + q2 * q2) - 1
        r1_2 = 2 * (q2 * q3 - q0 * q1)

        r2_0 = 2 * (q1 * q3 - q0 * q2)
        r2_1 = 2 * (q2 * q3 + q0 * q1)
        r2_2 = 2 * (q0 * q0 + q3 * q3) - 1

        rotation_matrix = np.array([[r0_0, r0_1, r0_2],
                                    [r1_0, r1_1, r1_2],
                                    [r2_0, r2_1, r2_2]])
        
        return rotation_matrix

        


    def convert_body_vel_to_spatial_vel(self, vx_b, vy_b, vz_b):
        rotation_matrix = self.get_rotation_matrix_from_quaternion()
        velocity_body = np.array([[vx_b],
                                  [vy_b], 
                                  [vz_b]])
        velocity_spatial = rotation_matrix @ velocity_body
        # return velocity_spatial
        return velocity_spatial[0][0], velocity_spatial[1][0], velocity_spatial[2][0]
        


    def position_timer_callback(self):
        sim_pose = [self.vehicle_sim_pose.pose.position.y,
                    self.vehicle_sim_pose.pose.position.x,
                    self.vehicle_sim_pose.pose.position.z]
        vehicle_info = json.dumps({'position':sim_pose,
                                   'vehicle_id':self.vehicle_id,
                                   'timestamp':time.time()})
        position_data = String()
        position_data.data = vehicle_info
        self.vehicle_position_publisher.publish(position_data)

    # def follow_sine_pattern(self, t, amplitude=1.0, frequency=1.0):
    #     # omega = 2 * np.pi * frequency
    #     # x = t
    #     # y = amplitude * np.sin(omega * t)
        
    #     # x_err = x - self.vehicle_odometry.position[0]
    #     # y_err = y - self.vehicle_odometry.position[1]

    #     # heading_error = np.arctan2(y_err, x_err)
    #     # vx = 1.2 *  np.cos(heading_error)
    #     # vy = 1.2 * np.sin(heading_error)
    #     # vz = 0.0
    #     omega = 2 * np.pi * frequency
    #     vx = 1.0                   # constant forward speed along x
    #     vy = amplitude * omega * np.cos(omega * t)
    #     vz = 0.0
    #     return float(vx), float(vy), float(vz)
    
    def guidance_timer_callback(self):
        # print('\n\n',self.neighbour_vehicle_position_info)

        zero_row_mask = np.all(self.neighbour_vehicle_relative_position_info == 0, axis=1)
        neighbors_relative_filtered = self.neighbour_vehicle_relative_position_info[~zero_row_mask]

        # print('\n\n', neighbors_relative_filtered)
        # print('\n\n', self.neighbour_vehicle_relative_position_info)



    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        
        # if self.vehicle_status.armed == VehicleStatus.ARMING_STATE_DISARMED:
        #     self.arm()
        
        # if self.vehicle_status_subscriber.armed == VehicleStatus.ARMING_STATE_ARMED and self.vehicle_status.takeoff_time == 0:
        #     self.takeoff()
        
        # if self.vehicle_status.takeoff_time > 0 and self.vehicle_odo.position[-1] < self.takeoff_height - 1:
        #     self.engage_offboard_mode()

        # if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(2.0, 0.0, 0)
        
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_odometry.position[-1] > self.takeoff_height-2 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print('going up')
            self.publish_position_setpoint(0.0, 0.0, -0.3)
            print('\n\n')

        elif self.vehicle_odometry.position[-1] <= self.takeoff_height-2:
            print('\nmoving forward')

            # # scenario 1
            # if self.vehicle_id == 1:
            #     self.publish_position_setpoint(2.0, 0.0, 0.0)
            # else:
            #     self.publish_position_setpoint(-2.0, 0.0, 0.0)

            # scenario 2
            if self.vehicle_id == 2:
                self.publish_position_setpoint(0.0, 2.5, 0.0)
                # vel_x, vel_y, vel_z = self.convert_body_vel_to_spatial_vel(0.0, 2.5, 0.0)
                # vel_x, vel_y, vel_z = self.follow_sine_pattern(0.5, 2.0, 0.5)
                # self.publish_position_setpoint(vel_x, vel_y, vel_z)
                
            else:
                zero_row_mask = np.all(self.neighbour_vehicle_relative_position_info == 0, axis=1)
                neighbors_relative_filtered = self.neighbour_vehicle_relative_position_info[~zero_row_mask]
                
                if self.vehicle_id == 1:
                    x_follow_vel = float(0.002 * neighbors_relative_filtered[0][0])
                    y_follow_vel = float(self.kp * neighbors_relative_filtered[0][1])
                    z_follow_vel = float(self.kp * neighbors_relative_filtered[0][2])

                    self.publish_position_setpoint(x_follow_vel, y_follow_vel, z_follow_vel)
                
                elif self.vehicle_id == 3:
                    x_follow_vel = float(0.002 * neighbors_relative_filtered[-1][0])
                    y_follow_vel = float(self.kp * neighbors_relative_filtered[-1][1])
                    z_follow_vel = float(self.kp * neighbors_relative_filtered[-1][2])

                    self.publish_position_setpoint(x_follow_vel, y_follow_vel, z_follow_vel)

                
            

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1



def main(args=None) -> None:
    print('Starting relative vehicle sim control node...')
    rclpy.init(args=args)
    relative_vehicle_sim_control = RelativeVehicleSim()
    rclpy.spin(relative_vehicle_sim_control)
    relative_vehicle_sim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)





# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
# import numpy as np

# class OffboardControl(Node):
#     """Node for controlling a vehicle in offboard mode."""

#     def __init__(self) -> None:
#         super().__init__('offboard_control_takeoff_and_land')

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Create publishers
#         self.offboard_control_mode_publisher = self.create_publisher(
#             OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         self.trajectory_setpoint_publisher = self.create_publisher(
#             TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
#         self.vehicle_command_publisher = self.create_publisher(
#             VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

#         # Create subscribers
#         self.vehicle_local_position_subscriber = self.create_subscription(
#             VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
#         self.vehicle_status_subscriber = self.create_subscription(
#             VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
#         self.vehicle_odometry_subscriber = self.create_subscription(
#              VehicleOdometry, f'/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile
#          )

#         # Initialize variables
#         self.offboard_setpoint_counter = 0
#         self.vehicle_local_position = VehicleLocalPosition()
#         self.vehicle_status = VehicleStatus()
#         self.vehicle_odometry = VehicleOdometry()
#         self.takeoff_height = -5.0

#         # Create a timer to publish control commands
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def vehicle_odometry_callback(self, vehicle_odometry):
# #         '''Callback function for vehicle_odometry topic subscriber.'''
#         self.vehicle_odometry = vehicle_odometry

#     def vehicle_local_position_callback(self, vehicle_local_position):
#         """Callback function for vehicle_local_position topic subscriber."""
#         self.vehicle_local_position = vehicle_local_position

#     def vehicle_status_callback(self, vehicle_status):
#         """Callback function for vehicle_status topic subscriber."""
#         self.vehicle_status = vehicle_status

#     def arm(self):
#         """Send an arm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
#         self.get_logger().info('Arm command sent')

#     def disarm(self):
#         """Send a disarm command to the vehicle."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
#         self.get_logger().info('Disarm command sent')

#     def engage_offboard_mode(self):
#         """Switch to offboard mode."""
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
#         self.get_logger().info("Switching to offboard mode")

#     def land(self):
#         """Switch to land mode."""
#         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
#         self.get_logger().info("Switching to land mode")

#     def publish_offboard_control_heartbeat_signal(self):
#         """Publish the offboard control mode."""
#         msg = OffboardControlMode()
#         msg.position = False
#         msg.velocity = True
#         msg.acceleration = False
#         msg.attitude = False
#         msg.body_rate = False
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.offboard_control_mode_publisher.publish(msg)

#     # def publish_position_setpoint(self, x: float, y: float, z: float):
#     #     """Publish the trajectory setpoint."""
#     #     msg = TrajectorySetpoint()
#     #     msg.position = [x, y, z]
#     #     msg.yaw = 1.57079  # (90 degree)
#     #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#     #     self.trajectory_setpoint_publisher.publish(msg)
#     #     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

#     def publish_position_setpoint(self, x: float, y: float, z: float):
#         """Publish the trajectory setpoint."""
#         msg = TrajectorySetpoint()
#         # msg.position = [x, y, z]
#         msg.position = [float('nan'), float('nan'), float('nan')]
#         msg.velocity = [x, y, z]
#         # msg.yaw = 1.57079  # (90 degree)
#         msg.yaw = float('nan')
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.trajectory_setpoint_publisher.publish(msg)
#         self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

#     def publish_vehicle_command(self, command, **params) -> None:
#         """Publish a vehicle command."""
#         msg = VehicleCommand()
#         msg.command = command
#         msg.param1 = params.get("param1", 0.0)
#         msg.param2 = params.get("param2", 0.0)
#         msg.param3 = params.get("param3", 0.0)
#         msg.param4 = params.get("param4", 0.0)
#         msg.param5 = params.get("param5", 0.0)
#         msg.param6 = params.get("param6", 0.0)
#         msg.param7 = params.get("param7", 0.0)
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#         self.vehicle_command_publisher.publish(msg)

#     def get_rotation_matrix_from_quaternion(self):
#         quaternions = self.vehicle_odometry.q
#         q0 = quaternions[0]
#         q1 = quaternions[1]
#         q2 = quaternions[2]
#         q3 = quaternions[3]

#         r0_0 = 2 * (q0 * q0 + q1 * q1) - 1
#         r0_1 = 2 * (q1 * q2 - q0 * q3)
#         r0_2 = 2 * (q1 * q3 + q0 * q2)

#         r1_0 = 2 * (q1 * q2 + q0 * q3)
#         r1_1 = 2 * (q0 * q0 + q2 * q2) - 1
#         r1_2 = 2 * (q2 * q3 - q0 * q1)

#         r2_0 = 2 * (q1 * q3 - q0 * q2)
#         r2_1 = 2 * (q2 * q3 + q0 * q1)
#         r2_2 = 2 * (q0 * q0 + q3 * q3) - 1

#         rotation_matrix = np.array([[r0_0, r0_1, r0_2],
#                                     [r1_0, r1_1, r1_2],
#                                     [r2_0, r2_1, r2_2]])
        
#         return rotation_matrix

        


#     def convert_body_vel_to_spatial_vel(self, vx_b, vy_b, vz_b):
#         rotation_matrix = self.get_rotation_matrix_from_quaternion()
#         velocity_body = np.array([[vx_b],
#                                   [vy_b], 
#                                   [vz_b]])
#         velocity_spatial = rotation_matrix @ velocity_body
#         # print(velocity_spatial)
#         return velocity_spatial[0][0], velocity_spatial[1][0], velocity_spatial[2][0]

#     def timer_callback(self) -> None:
#         """Callback function for the timer."""
#         self.publish_offboard_control_heartbeat_signal()

#         if self.offboard_setpoint_counter == 10:
#             self.engage_offboard_mode()
#             self.arm()

#         if self.vehicle_odometry.position[-1] > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
#             print('going up')
#             self.publish_position_setpoint(0.0, 0.0, -1.5)

#         elif self.vehicle_odometry.position[-1] <= self.takeoff_height:
#             print('moving forward')
#             vel_x, vel_y, vel_z = self.convert_body_vel_to_spatial_vel(2.0, 0.0, 0.0)
#             self.publish_position_setpoint(vel_x, vel_y, vel_z)

#         if self.offboard_setpoint_counter < 11:
#             self.offboard_setpoint_counter += 1


# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)

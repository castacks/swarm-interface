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
from pyquaternion import Quaternion


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def get_rotation_matrix_from_quaternion(quaternions):
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


def convert_body_vel_to_spatial_vel(vx_b, vy_b, vz_b, quaternions):
    rotation_matrix = get_rotation_matrix_from_quaternion(quaternions)
    velocity_body = np.array([[vx_b],
                                [vy_b], 
                                [vz_b]])
    velocity_spatial = rotation_matrix @ velocity_body
    # return velocity_spatial
    return velocity_spatial[0][0], velocity_spatial[1][0], velocity_spatial[2][0]


def get_relative_position_in_body(relative_position_in_spatial, quaternions):
    rotation_matrix = get_rotation_matrix_from_quaternion(quaternions)
    relative_position_in_body = rotation_matrix.T @ np.array([relative_position_in_spatial]).T
    return relative_position_in_body.T.squeeze().tolist()


def sim_to_qgc(sim_x, sim_y, sim_z):
    rot_x = np.array([
                        [1, 0,              0             ],
                        [0, np.cos(np.pi), -np.sin(np.pi)],
                        [0, np.sin(np.pi),  np.cos(np.pi)]
                    ])
    
    rot_z = np.array([
                        [ np.cos(np.pi/2), -np.sin(np.pi/2), 0],
                        [ np.sin(np.pi/2),  np.cos(np.pi/2), 0],
                        [ 0,              0,             1]
                    ])

    R = rot_x @ rot_z

    R = np.array([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1]
])
    
    R = np.array([
    [0,  1,  0],
    [1, 0,  0],
    [0,  0, -1]
])
    
    sim_coords = np.array([[sim_x],
                           [sim_y],
                           [sim_z]])
    
    # print(sim_coords.squeeze().tolist())
    qgc_coords = R @ sim_coords
    return qgc_coords.squeeze().tolist()


def sim_quat_to_ned(q0, q1, q2, q3):
    """
    q_sim: [w, x, y, z] in FLU
    returns: quaternion in NED
    """
    q_sim = np.array([q0, q1, q2, q3])
    # FLU → NED frame rotation quaternion
    q_frame = np.array([0.0, 1/np.sqrt(2), 1/np.sqrt(2), 0.0])
    
    q_ned = quat_multiply(q_frame, q_sim)
    
    return q_ned


def get_relative_orientation(q_A, q_B):
    q_A = Quaternion(q_A)
    q_A_inv = q_A.inverse

    q_B = Quaternion(q_B)

    relative_orientation = q_A_inv * q_B
    return relative_orientation.q

    


class SwarmInterface(Node):
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
        self.relative_neigbours = NeighbourRelativePose()
        self.takeoff_height = -5.0


        # below are publishers and subscribers that are used to share position information (they are not related to px4)
        self.vehicle_position_publisher = self.create_publisher(
            String, f'/swarm/multi_vehicle/position', qos_profile
        )

        self.vehicle_position_subscriber = self.create_subscription(
            String, f'/swarm/multi_vehicle/position', self.vehicle_position_callback, qos_profile
        )

        self.relative_pose_to_guidance_publisher = self.create_publisher(
            NeighbourRelativePose, f'/swarm/agent_{self.vehicle_id}/to_guidance', qos_profile
        )

        self.agent_body_velocity_subscriber = self.create_subscription(
            String, f'/swarm/agent_{self.vehicle_id}/from_guidance', self.agent_body_velocity_callback, qos_profile
        )

        self.change_leader_subscriber = self.create_subscription(String,
                                                                 '/swarm/change_leader',
                                                                 self.change_leader_cb,
                                                                 qos_profile)



        self.vehicle_position = String()
        self.neighbour_vehicle_relative_position_info = np.zeros((self.total_vehicles, 3))

        # timer to publish position info
        self.position_timer = self.create_timer(0.08, self.position_timer_callback)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.kp = 0.2
        self.neighbours_last_seen = {}
        self.all_neighbours_relative_position_in_body = []
        self.all_neighbours_relative_pose_body_arr = np.zeros((1000, 2))
        self.agent_spatial_velocity = None
        self.done_with_takeoff = False
        self.leader_id = 2 # (by default)


    def change_leader_cb(self, change_leader):
        new_leader = int(change_leader.data)
        if new_leader <= self.total_vehicles:
            self.leader_id = new_leader
            print(f'leader has been changed to {new_leader} successfully')
        else:
            print(f'vehicle with the id {new_leader} does not exists')


    def agent_body_velocity_callback(self, agent_body_velocity):
        agent_body_velocity = json.loads(agent_body_velocity.data)['body_velocities']
        self.agent_spatial_velocity = convert_body_vel_to_spatial_vel(agent_body_velocity[0], agent_body_velocity[1], agent_body_velocity[2], self.vehicle_odometry.q)


    def vehicle_position_callback(self, vehicle_position):
        '''Callback function for vehicle_position topic subscriber'''

        vehicle_position_info = json.loads(vehicle_position.data)
        if vehicle_position_info['vehicle_id'] != self.vehicle_id:
            

            neighbour_vehicle_position = vehicle_position_info['position']
            neighbour_vehicle_orientation = vehicle_position_info['orientation']

            vehicle_position = sim_to_qgc(self.vehicle_sim_pose.pose.position.x, self.vehicle_sim_pose.pose.position.y, self.vehicle_sim_pose.pose.position.z)
            neighbour_vehicle_position = sim_to_qgc(neighbour_vehicle_position[0], neighbour_vehicle_position[1], neighbour_vehicle_position[2])
            vehicle_quaternion_ned = sim_quat_to_ned(self.vehicle_sim_pose.pose.orientation.w, self.vehicle_sim_pose.pose.orientation.x, self.vehicle_sim_pose.pose.orientation.y, self.vehicle_sim_pose.pose.orientation.z)
            neighbour_vehicle_quaternion_ned = sim_quat_to_ned(neighbour_vehicle_orientation[0], neighbour_vehicle_orientation[1], neighbour_vehicle_orientation[2], neighbour_vehicle_orientation[3])
            
            # print('\n\n')

            neighbour_vehicle_relative_position = [(neighbour_vehicle_position[0] - vehicle_position[0]),
                                                   neighbour_vehicle_position[1] - vehicle_position[1],
                                                   (neighbour_vehicle_position[2] - vehicle_position[2])]
            
            relative_orientation = get_relative_orientation(vehicle_quaternion_ned, neighbour_vehicle_quaternion_ned)

            # rotate relative pose to body frame   
            relative_body_position = get_relative_position_in_body(neighbour_vehicle_relative_position, self.vehicle_odometry.q)

            
            relative_pose_stamped = PoseStamped()
            relative_pose_stamped.header.frame_id = str(vehicle_position_info['vehicle_id'])
            relative_pose_stamped.pose.position.x = relative_body_position[0]
            relative_pose_stamped.pose.position.y = relative_body_position[1]
            relative_pose_stamped.pose.position.z = relative_body_position[2]
            relative_pose_stamped.pose.orientation.w = relative_orientation[0]
            relative_pose_stamped.pose.orientation.x = relative_orientation[1]
            relative_pose_stamped.pose.orientation.y = relative_orientation[2]
            relative_pose_stamped.pose.orientation.z = relative_orientation[3]
            # relative_pose_stamped.pose.orientation.w

        
            # update neighbour or add a new neighbour (does not take into account for checking the old neighbour data and removing them from the array)
            if self.neighbours_last_seen.get(vehicle_position_info['vehicle_id'], False):
                # update
                index = self.neighbours_last_seen[vehicle_position_info['vehicle_id']]['index']
                self.all_neighbours_relative_position_in_body[index] = relative_pose_stamped
            else:
                # add 
                self.neighbours_last_seen[vehicle_position_info['vehicle_id']] = {'last_time_stamp': time.time(),
                                                                             'index': len(self.all_neighbours_relative_position_in_body)}
                self.all_neighbours_relative_position_in_body.append(relative_pose_stamped)

            
                
                

        # publish the relative neighbour position data to the guidance algorithm
        if len(self.all_neighbours_relative_position_in_body) > 0:
            # note: you should check for old neighbour data and remove them
            self.relative_neigbours.neighbour_relative_pose = self.all_neighbours_relative_position_in_body
            self.relative_pose_to_guidance_publisher.publish(self.relative_neigbours)
        
    
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

    def publish_position_setpoint(self, x: float, y: float, z: float, is_leader_follower: bool):
        """Publish the trajectory setpoint."""


        if self.leader_id == self.vehicle_id and self.done_with_takeoff:
            altitude_error = self.takeoff_height - self.vehicle_odometry.position[-1]
            z = float(0.6 * altitude_error)

        msg = TrajectorySetpoint()
        print('feeding velocity -> ', x, ' ' ,y, ' ',z)
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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
        


    def position_timer_callback(self):
        # in real time this should be replaced with code that reads position from ca
        '''
        in real time following should happen :
        1. read pose from camera
        2. convert the pose to body frame of the vehicle
        3. put all the pose in a structured format into an array 
        3. publish the neighbour relative pose to the guidance algo
        '''
        sim_pose = [self.vehicle_sim_pose.pose.position.x,
                    self.vehicle_sim_pose.pose.position.y,
                    self.vehicle_sim_pose.pose.position.z]
        sim_orientation = [self.vehicle_sim_pose.pose.orientation.w,
                           self.vehicle_sim_pose.pose.orientation.x,
                           self.vehicle_sim_pose.pose.orientation.y,
                           self.vehicle_sim_pose.pose.orientation.z]
        vehicle_info = json.dumps({'position':sim_pose,
                                   'orientation':sim_orientation,
                                   'vehicle_id':self.vehicle_id,
                                   'timestamp':time.time()})
        position_data = String()
        position_data.data = vehicle_info
        self.vehicle_position_publisher.publish(position_data)




    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_odometry.position[-1] > self.takeoff_height-2 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.done_with_takeoff:
            print('going up')
            self.publish_position_setpoint(0.0, 0.0, -0.3, False)
            print('\n\n')

        elif self.vehicle_odometry.position[-1] <= self.takeoff_height-2 or self.done_with_takeoff:
            print('\nmoving forward')

            if not self.done_with_takeoff:
                self.done_with_takeoff = True

            if self.agent_spatial_velocity == None:
                self.publish_position_setpoint(0.0, 0.0, 0.0, False)
            else:
                vx, vy, vz = self.agent_spatial_velocity
                self.publish_position_setpoint(vx, vy, vz, True)


        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1



def main(args=None) -> None:
    print('Starting relative vehicle sim control node...')
    rclpy.init(args=args)
    relative_vehicle_sim_control = SwarmInterface()
    rclpy.spin(relative_vehicle_sim_control)
    relative_vehicle_sim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
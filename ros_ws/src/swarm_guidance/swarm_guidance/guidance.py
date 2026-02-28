#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from swarm_custom_msgs.msg import NeighbourRelativePose
# from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,\
#                             VehicleOdometry
from std_msgs.msg import String, Float32MultiArray
import time, json
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from swarm_custom_msgs.msg import NeighbourRelativePose
import numpy as np
import math




class Guidance(Node):
    """guidance (bearing based)"""

    def __init__(self) -> None:
        super().__init__('guidance')

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


        self.change_leader_subscriber = self.create_subscription(String,
                                                                 '/swarm/change_leader',
                                                                 self.change_leader_cb,
                                                                 qos_profile)
        
        self.change_leader_velocity = self.create_subscription(Float32MultiArray,
                                                               '/swarm/change_leader_velocity',
                                                               self.change_leader_velocity_cb,
                                                               qos_profile)

        self.neighbour_pose_subscriber = self.create_subscription(NeighbourRelativePose,
                                                                  f'/swarm/agent_{self.vehicle_id}/to_guidance',
                                                                  self.guidance_cb,
                                                                  qos_profile)
        

        self.agent_body_velocity_publisher = self.create_publisher(String,
                                                                   f'/swarm/agent_{self.vehicle_id}/from_guidance',
                                                                   qos_profile)
        

        self.neighbour_relative_poses = NeighbourRelativePose()
        self.agent_body_velocity = String()
        self.change_leader = String

        self.kpx = 0.2
        self.kpy = 0.02
        self.kpz = 0.2
        self.leader_id = 2 # leader id (default)
        self.leader_velocity = [2.5, 0.0, 0.0]

    
    def change_leader_cb(self, change_leader):
        new_leader = int(change_leader.data)
        if new_leader <= self.total_vehicles:
            self.leader_id = new_leader
            print(f'leader has been changed to {new_leader} successfully')
        else:
            print(f'vehicle with the id {new_leader} does not exists')


    def change_leader_velocity_cb(self, change_leader_velocity):
        leader_new_velocity = change_leader_velocity.data
        self.leader_velocity = [round(vel, 2) for vel in leader_new_velocity]
        print(type(leader_new_velocity))
        print('leader velocity has been set successfully')
        

    
    def guidance_cb(self, neighbour_relative_poses):
        # hard code body velocity for leader and apply propotional constants on relative position for other agents
        if self.vehicle_id == self.leader_id:
            # leader_body_vel 
            agent_velocity = json.dumps({'body_velocities':self.leader_velocity})
            self.agent_body_velocity.data = agent_velocity
            self.agent_body_velocity_publisher.publish(self.agent_body_velocity)
            # heading = np.arctan2(self.leader_velocity[1], self.leader_velocity[1])

        else:
            array_of_relative_poses = neighbour_relative_poses.neighbour_relative_pose
            idx = None
            if self.vehicle_id > self.leader_id:
                idx = self.leader_id - 1
            else:
                idx = self.leader_id - 2
            

            # print('this is the idx : ', idx)

            if array_of_relative_poses[idx].header.frame_id != self.leader_id:
                for neighbor_agent in array_of_relative_poses:
                    if neighbor_agent.header.frame_id == self.leader_id:
                        idx = array_of_relative_poses.index(neighbor_agent)
                        break

            # if self.vehicle_id == 3:
            #     array_of_relative_poses.reverse()

            relative_pose_of_leader = array_of_relative_poses[idx]
            

            # print(array_of_relative_poses)
            # print('got the relative leader pose : ', relative_pose_of_leader, '\n\n')

            vel_x = self.kpx * relative_pose_of_leader.pose.position.x
            vel_y = self.kpy * relative_pose_of_leader.pose.position.y
            vel_z = self.kpz * relative_pose_of_leader.pose.position.z

            # relative_heading = 2 * math.acos(relative_pose_of_leader.pose.orientation.w)
            # print('relative heading of leader : ', relative_heading)
        
            agent_velocity = json.dumps({'body_velocities':[vel_x, vel_y, vel_z]})
            self.agent_body_velocity.data = agent_velocity
            self.agent_body_velocity_publisher.publish(self.agent_body_velocity)
        





def main(args=None) -> None:
    print('Starting guidance node...')
    rclpy.init(args=args)
    relative_vehicle_sim_control = Guidance()
    rclpy.spin(relative_vehicle_sim_control)
    relative_vehicle_sim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)



# ros2 topic pub -1 /swarm/change_leader std_msgs/msg/String "{data: '2'}"
# ros2 topic pub -1 /my_topic std_msgs/msg/Float32MultiArray "{data: [1.1, 2.2, 3.3]}"


# i have position of 2 agents A and B. From them i can find the relative position of B from A. now i need relative heading of agent B from A. how do it do that (keep it short)


# i want how much B is rotated from A's current direction

# can i express this transform in quaternions (yes or no)

# how
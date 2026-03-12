## Multi-Agent Swarm Interface (ROS 2 + PX4)

A ROS 2-based multi-agent swarm interface built on top of PX4, designed for coordinated control of multiple UAVs in simulation.

> **Note:** This implementation is currently tailored for simulation (Pegasus simulator) and assumes network-based communication among agents. Minor modifications are required for real-world deployment.

---

### Overview

This project implements a **multi-agent swarm interface** using:

- **ROS 2** for inter-agent communication and middleware
- **PX4** as the flight control stack
- **Pegasus Simulator** for simulation-based pose feedback

The system enables coordinated swarm behavior by:

1. Receiving vehicle pose from the simulator.
2. Sharing pose information among neighboring agents.
3. Passing neighbor state data to an external guidance module.
4. Receiving velocity commands from the guidance module.
5. Forwarding velocity commands to PX4.

---

### System Architecture

#### Data Flow

```
Pegasus Simulator
        ↓
   Swarm Interface (ROS 2 Node)
        ↓
Publish PoseStamped to Neighbors
        ↓
   Guidance Module
        ↓
Velocity Command
        ↓
   Swarm Interface
        ↓
       PX4
```

---

### Steps to Reproduce

1. **Set up Isaac Sim and PX4**
   - Follow the setup guide: [link]

2. **Place simulation script**
   - Copy `multi_vehicle_sim.py` into the Pegasus Simulator repository at the appropriate location.

3. **Clone this repository**
   ```bash
   git clone <repo-url>
   cd <repo-dir>
   ```

4. **Build the workspace**
   ```bash
   colcon build
   ```

5. **Source the workspace**
   ```bash
   source install/setup.bash
   ```

6. **Launch per-vehicle nodes**
   - Single vehicle communication depends on `interface.py` and `guidance.py`.
   - Run the following in separate terminals for each vehicle (replace `vehicle_id` accordingly):

   ```bash
   # Terminal 1
   ros2 run swarm_guidance interface --ros-args -p vehicle_id:=1 -p total_vehicles:=3

   # Terminal 2
   ros2 run swarm_guidance guidance --ros-args -p vehicle_id:=1 -p total_vehicles:=3
   ```

7. **Repeat step 6 for each of the `n` vehicles**, incrementing `vehicle_id` for each agent.

8. **Modify leader velocity and leader vehicle ID** using the following publisher commands:

   ```bash
   # Change leader velocity
   ros2 topic pub -1 /swarm/change_leader_velocity std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 0.0]}"

   # Change leader vehicle
   ros2 topic pub -1 /swarm/change_leader std_msgs/msg/String "{data: 2}"
   ```

9. **Open QGroundControl (QGC)** to view telemetry data.

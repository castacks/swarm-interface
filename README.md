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
   - Follow the setup guide: [[link](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)]

2. **Place simulation script**
   - Copy `multi_vehicle_px4.py` into the Pegasus Simulator repository inside the examples dir.

3. **Clone this repository**
   ```bash
   git clone git@github.com:castacks/swarm-interface.git
   cd swarm-interface/ros_ws
   ```

4. **Build the workspace**
   ```bash
   colcon build
   ```

5. **Source the workspace**
   ```bash
   source install/setup.bash
   ```

6. **Launch the simulation**
   ```bash
   isaac_run examples/multi_vehicle_sim.py
   ```
   > By default, the simulation runs in **headless mode** for **3 vehicles**. To change this, modify **line 16** (headless flag) and **line 60** (number of vehicles) accordingly.

   Once launched, you should be able to see the simulation running. You can now proceed to run the interface and guidance stack.

7. **Launch per-vehicle nodes**
   - Single vehicle communication depends on `interface.py` and `guidance.py`.
   - Run the following in separate terminals for each vehicle (replace `vehicle_id` accordingly):

   ```bash
   # Terminal 1
   ros2 run swarm_guidance interface --ros-args -p vehicle_id:=1 -p total_vehicles:=3

   # Terminal 2
   ros2 run swarm_guidance guidance --ros-args -p vehicle_id:=1 -p total_vehicles:=3
   ```

8. **Repeat step 7 for each of the `n` vehicles**, incrementing `vehicle_id` for each agent.

9. **Modify leader velocity and leader vehicle ID** using the following publisher commands:

   ```bash
   # Change leader velocity
   ros2 topic pub -1 /swarm/change_leader_velocity std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 0.0]}"

   # Change leader vehicle
   ros2 topic pub -1 /swarm/change_leader std_msgs/msg/String "{data: 2}"
   ```

10. **Open QGroundControl (QGC)** to view telemetry data.

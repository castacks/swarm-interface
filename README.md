# Multi-Agent Swarm Interface (ROS 2 + PX4)

A ROS 2-based multi-agent swarm interface built on top of PX4, designed for coordinated control of multiple UAVs in simulation.  

> ⚠️ **Note:** This implementation is currently tailored for simulation (Pegasus simulator) and assumes network-based communication among agents. Minor modifications are required for real-world deployment.

---

## Overview

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

## System Architecture

### Data Flow

```text
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

# Mobility Challenge Simulator Execution Guide

This document summarizes the execution procedure for **Method 1**, which was used for the final competition environment.

The system runs the `problem3_cpp_launch.py` launch file from the `bisa` package in a ROS 2 Foxy environment, and starts the race by calling the `/start_race` service from a separate terminal.

---

# Project Overview

This repository documents the final deployment workflow for the **2025 KAIST Mobility Challenge** competition environment. The project focused on operating a ROS 2-based autonomous mobility system under realistic competition constraints, including multi-vehicle readiness checks, remote execution, Docker-based CAV bridge operation, deterministic race startup, and reliable recovery from stale ROS/DDS processes.

The final competition setup was executed through **Method 1**, where the main ROS 2 launch file initializes the autonomous driving system and a separate terminal triggers the race through the `/start_race` service. This workflow was used in the final competition run and led to the following result:

- **Runtime:** 97 seconds
- **Collision Count:** 1
- **Human Intervention Count:** 1
- **Award:** Excellence Award

---

# My Role & Contributions

In this project, I contributed to the final competition system by focusing on **deployment reliability, ROS 2 execution architecture, multi-CAV operation, and competition-day procedure stabilization**. My work centered on making the autonomous mobility stack reproducible under field conditions, reducing launch-time uncertainty, and preparing a clear execution flow that could be followed during the final run.

## ROS 2 System Deployment & Execution Architecture

- **Final Launch Workflow Design:** Organized the final competition execution around `problem3_cpp_launch.py`, ensuring that the autonomous driving stack could be launched through a single ROS 2 entry point from the `bisa` package.
- **Two-Terminal Race Operation:** Structured the final workflow into a launch terminal and a race-start terminal, separating system initialization from the `/start_race` trigger so that the vehicle system could be verified before release.
- **Environment Setup Standardization:** Defined the required ROS 2 Foxy setup sequence, workspace sourcing steps, and `ROS_DOMAIN_ID=100` configuration to reduce environment mismatch issues across repeated runs.
- **Package-Level Rebuild Procedure:** Documented both full workspace rebuilds and targeted `bisa` package rebuilds with `colcon build`, allowing faster iteration when only competition code changed.

## Multi-CAV Operation & Competition Readiness

- **CAV Motor Bring-Up Procedure:** Established the Docker-based startup process for enabling CAV1-CAV4 motors and confirming that bridge logs and timeout messages appeared correctly.
- **Readiness Verification Flow:** Treated the `ALL CAVS READY!` message as a critical pre-race checkpoint before calling the race-start service.
- **Simultaneous Start Coordination:** Used the `/start_race` service trigger to release the full system only after all CAV nodes were initialized, helping align the final run with competition timing requirements.
- **Remote Host Operation:** Prepared the remote execution sequence for the competition machine, including SSH access, ROS environment sourcing, workspace setup, and launch command ordering.

## Reliability, Debugging & Recovery

- **Stale Process Cleanup:** Added a repeatable cleanup sequence using `pkill` for previously running ROS 2 and launch processes that could interfere with a fresh run.
- **Fast DDS Shared Memory Recovery:** Included cleanup of `/dev/shm/fastrtps_*` files to resolve communication issues caused by leftover DDS shared memory segments.
- **Environment Variable Reset Procedure:** Documented how to recover from incorrect `AMENT_PREFIX_PATH`, `ROS_PACKAGE_PATH`, or `CMAKE_PREFIX_PATH` contamination by resetting ROS-related environment variables and sourcing the workspace again.
- **Competition Runbook Documentation:** Consolidated the final operating commands into a step-by-step README so that the system could be executed consistently under time pressure.

---

# Technical Highlights

## 1. ROS 2-Based Competition Launch Pipeline

The final system uses a ROS 2 Foxy launch pipeline centered on `ros2 launch bisa problem3_cpp_launch.py`. This launch file acts as the operational entry point for the final competition environment, allowing the required nodes to be initialized together instead of relying on scattered manual node execution.

Key technical decisions:

- Centralized final execution through the `bisa` package launch file.
- Required workspace sourcing through `install/setup.bash` before execution.
- Used `ROS_DOMAIN_ID=100` to isolate the competition communication domain.
- Verified system readiness before triggering vehicle release.

## 2. Service-Based Race Start Mechanism

The race begins through a dedicated ROS 2 service call:

```bash
ros2 service call /start_race std_srvs/srv/Trigger
```

This design separates **system initialization** from **mission start**, which is important for field robotics workflows. Instead of launching and moving immediately, the system can first confirm that all CAV nodes are alive, all required communication channels are established, and the operator is ready to begin the official run.

The expected startup confirmation is:

```text
ALL CAVS READY!
Call service to start:
ros2 service call /start_race std_srvs/srv/Trigger
```

After the service trigger succeeds, the system logs:

```text
RELEASE THE KRAKEN! (Simultaneous Start)
```

## 3. Docker-Based CAV Bridge Operation

The CAV1-CAV4 motor system is brought up through a Docker Compose workflow. This keeps the motor bridge environment separated from the ROS 2 launch environment and makes the startup procedure more repeatable across runs.

The key readiness indicator is the appearance of CAV bridge logs and timeout messages such as:

```text
[INFO]
timeout: 0.5
```

This provides a practical operator-level signal that the CAV bridge layer is active before the main ROS 2 competition launch begins.

## 4. DDS Communication Recovery Strategy

Because ROS 2 Foxy commonly relies on DDS middleware, stale Fast DDS shared memory files can cause communication issues after abnormal shutdowns or repeated launches. The README includes a direct recovery step:

```bash
sudo rm /dev/shm/fastrtps_*
```

This is especially useful in competition settings where repeated test runs, interrupted launches, or crashed processes can leave behind shared memory resources that prevent clean node communication.

## 5. Repeatable Field Deployment Runbook

The final README functions as a field deployment runbook rather than only a code description. It documents the exact command order for:

- Connecting to the remote host.
- Killing stale ROS 2 processes.
- Cleaning DDS shared memory files.
- Sourcing ROS 2 Foxy and the built workspace.
- Verifying `AMENT_PREFIX_PATH`.
- Setting `ROS_DOMAIN_ID`.
- Launching the final system.
- Triggering `/start_race` from a separate terminal.

This reduces operator ambiguity and makes the competition workflow reproducible even under time pressure.

## 6. Final Competition Performance

The final Method 1 deployment was used during the competition and achieved:

- **97-second runtime**
- **1 collision**
- **1 human intervention**
- **Excellence Award**

These results show that the final execution workflow was stable enough for official competition operation and that the documented launch procedure reflects the actual successful deployment path.

---

# 1. Environment

- OS: Ubuntu/Linux
- ROS 2: Foxy
- Remote Host: `bisa@192.168.10.192`
- Workspace: `Mobility_Challenge_Simulator/`
- Package: `bisa`
- Launch File: `problem3_cpp_launch.py`
- ROS Domain ID: `100`

> It is recommended not to include remote access passwords directly in a public README file. Manage sensitive credentials separately in internal documentation.

---

# 2. Prerequisites

## 2.1 Build Workspace

If this is the first execution or the source code has been modified, build the workspace first.

```bash
cd Mobility_Challenge_Simulator/
colcon build
```

To rebuild only the `bisa` package:

```bash
colcon build --packages-select bisa
```

---

# 3. Turning ON CAV1~4 Motors

First, launch the Docker environment to enable the CAV1~4 motors.

## Terminal 1

```bash
./cav_docker.sh
sudo docker compose up
```

After the containers are successfully initialized, press `Ctrl + C`, then run the command again:

```bash
sudo docker compose up
```

If everything is running correctly, logs similar to the following should appear:

```text
[INFO]
timeout: 0.5
```

You should also see CAV bridge logs and messages indicating that the CAV nodes are ready.

---

# 4. Method 1: Final Competition Code Execution

Method 1 uses two terminals:

- **Terminal 1**: Launch the ROS 2 system
- **Terminal 2**: Call the race start service

---

# 4.1 Terminal 1: Launch the ROS 2 System

Connect to the remote machine:

```bash
ssh bisa@192.168.10.192
```

Terminate all previously running ROS 2 processes:

```bash
pkill -9 -f ros2
pkill -9 -f simulator_launch.py
pkill -9 -f problem3_cpp_launch.py
```

Clean Fast DDS shared memory files:

```bash
sudo rm /dev/shm/fastrtps_*
```

Source the ROS 2 Foxy environment:

```bash
source /opt/ros/foxy/setup.bash
```

Move to the workspace and source the built packages:

```bash
cd Mobility_Challenge_Simulator/
source install/setup.bash
```

Check whether the environment path is configured correctly:

```bash
echo $AMENT_PREFIX_PATH
```

If incorrect workspace paths are mixed into the environment, reset the ROS-related environment variables and source the workspace again:

```bash
unset ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source install/setup.bash
```

Set the ROS Domain ID:

```bash
export ROS_DOMAIN_ID=100
```

Launch the final competition system:

```bash
ros2 launch bisa problem3_cpp_launch.py
```

If the system launches successfully, CAV readiness logs will appear, followed by a message similar to:

```text
ALL CAVS READY!
Call service to start:
ros2 service call /start_race std_srvs/srv/Trigger
```

---

# 4.2 Terminal 2: Start the Race

Open a new terminal and connect to the remote machine again:

```bash
ssh bisa@192.168.10.192
```

Terminate any previously running ROS 2 processes:

```bash
pkill -9 -f ros2
```

Source the ROS 2 Foxy environment:

```bash
source /opt/ros/foxy/setup.bash
```

Move to the workspace and source the built packages:

```bash
cd Mobility_Challenge_Simulator/
source install/setup.bash
```

Optionally verify the environment path:

```bash
echo $AMENT_PREFIX_PATH
```

Start the race:

```bash
ros2 service call /start_race std_srvs/srv/Trigger
```

If successful, the following message should appear in the logs:

```text
RELEASE THE KRAKEN! (Simultaneous Start)
```

---

# 5. Full Command Summary

## Terminal 1

```bash
ssh bisa@192.168.10.192

pkill -9 -f ros2
pkill -9 -f simulator_launch.py
pkill -9 -f problem3_cpp_launch.py
sudo rm /dev/shm/fastrtps_*

source /opt/ros/foxy/setup.bash
cd Mobility_Challenge_Simulator/
source install/setup.bash

echo $AMENT_PREFIX_PATH

# Run only if the path configuration is incorrect
unset ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source install/setup.bash

export ROS_DOMAIN_ID=100
ros2 launch bisa problem3_cpp_launch.py
```

## Terminal 2

```bash
ssh bisa@192.168.10.192

pkill -9 -f ros2

source /opt/ros/foxy/setup.bash
cd Mobility_Challenge_Simulator/
source install/setup.bash

echo $AMENT_PREFIX_PATH

ros2 service call /start_race std_srvs/srv/Trigger
```

---

# 6. Troubleshooting

## 6.1 Incorrect `AMENT_PREFIX_PATH`

If multiple workspace paths are mixed into the environment, reset the ROS environment variables:

```bash
unset ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source install/setup.bash
```

Then verify the path again:

```bash
echo $AMENT_PREFIX_PATH
```

---

## 6.2 Existing ROS 2 Processes Causing Issues

Force terminate all related processes:

```bash
pkill -9 -f ros2
pkill -9 -f simulator_launch.py
pkill -9 -f problem3_cpp_launch.py
```

---

## 6.3 Fast DDS Shared Memory Issues

Shared memory files from previous executions may cause communication problems:

```bash
sudo rm /dev/shm/fastrtps_*
```

---

## 6.4 Changes Not Reflected After Code Modification

Rebuild the workspace:

```bash
cd Mobility_Challenge_Simulator/
colcon build
source install/setup.bash
```

To rebuild only the `bisa` package:

```bash
colcon build --packages-select bisa
source install/setup.bash
```

---

# 7. Successful Execution Criteria

The execution is considered successful if the following conditions are satisfied:

1. Docker logs show CAV bridge-related messages
2. `[INFO]` and `timeout: 0.5` logs appear
3. All CAV nodes become ready after running:

```bash
ros2 launch bisa problem3_cpp_launch.py
```

4. After calling `/start_race`, the following message appears:

```text
RELEASE THE KRAKEN!
```

---

# 8. Notes

Method 1 was used during the final competition.

Competition Result:
- Runtime: 97 seconds
- Collision Count: 1
- Human Intervention Count: 1
- Award: Excellence Award

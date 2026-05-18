# Mobility Challenge Simulator Execution Guide

This document summarizes the execution procedure for **Method 1**, which was used for the final competition environment.

The system runs the `problem3_cpp_launch.py` launch file from the `bisa` package in a ROS 2 Foxy environment, and starts the race by calling the `/start_race` service from a separate terminal.

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

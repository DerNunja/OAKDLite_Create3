# OAK-D-Lite Create 3 Object Following

ROS 2 Jazzy nodes for driving an iRobot Create 3 toward an object detected by a Luxonis OAK-D-Lite camera.

## Hardware

- Raspberry Pi 4 running Ubuntu Server 24.04 and ROS 2 Jazzy
- iRobot Create 3 connected to the Pi via USB-C networking
- Luxonis OAK-D-Lite connected to the Pi via USB-A

## Runtime Layout On The Pi

The deployment path used on the robot is:

```text
~/create3_oak/
├── nodes/
│   ├── oak_publisher.py
│   └── drive_to_object.py
└── venv/
```

`oak_publisher.py` runs in the `~/create3_oak/venv` virtual environment because it uses `depthai`.

`drive_to_object.py` runs with the normal ROS Python environment and does not use `depthai`.

## Topics

- Camera detections: `/oak/nn/spatial_detections` (`vision_msgs/Detection3DArray`)
- Robot velocity command: `/cmd_vel` (`geometry_msgs/Twist`)

## Setup On The Pi

Install ROS 2 Jazzy and make sure the shell can source it:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Create the camera virtual environment:

```bash
python3 -m venv --system-site-packages ~/create3_oak/venv
source ~/create3_oak/venv/bin/activate
python3 -m pip install "depthai==3.7.1"
```

Copy the node files to the Pi:

```bash
mkdir -p ~/create3_oak/nodes
cp create3_oak/nodes/*.py ~/create3_oak/nodes/
chmod +x ~/create3_oak/nodes/*.py
```

## Run

Terminal A, start the camera publisher:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/create3_oak/venv/bin/activate
python3 ~/create3_oak/nodes/oak_publisher.py
```

Terminal B, dry-run object following. This does not publish to `/cmd_vel`:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/create3_oak/nodes/drive_to_object.py bottle
```

Live driving with a fixed target class:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/create3_oak/nodes/drive_to_object.py bottle --ros-args -p dry_run:=false
```

Without a target class, the robot follows the nearest detected object:

```bash
python3 ~/create3_oak/nodes/drive_to_object.py --ros-args -p dry_run:=false
```

## Target Selection

The target class can be passed in any of these forms:

```bash
python3 ~/create3_oak/nodes/drive_to_object.py bottle
python3 ~/create3_oak/nodes/drive_to_object.py --target bottle
python3 ~/create3_oak/nodes/drive_to_object.py --target-class bottle
python3 ~/create3_oak/nodes/drive_to_object.py --ros-args -p target_class:=bottle
```

Common model labels include `person`, `bottle`, `chair`, `cup`, `book`, and `cell phone`.

## Motor Enable

If the Create 3 receives commands but does not move, enable motor power:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: false}"
```

## Safety

- `drive_to_object.py` defaults to `dry_run:=true`.
- Live driving requires explicitly passing `--ros-args -p dry_run:=false`.
- Test live driving only with enough free space or with the robot safely lifted.
- Press `Ctrl+C` in the drive node to stop. In live mode it publishes a zero `Twist` on shutdown.

## Camera Reset Note

If the camera publisher reports that the OAK-D-Lite is not reachable or logs `X_LINK_ERROR`, unplug the camera USB cable briefly and plug it back in, then restart `oak_publisher.py`.

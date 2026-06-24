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

## Repository Contents

```text
create3_oak/nodes/oak_publisher.py       # OAK-D-Lite -> /oak/nn/spatial_detections
create3_oak/nodes/drive_to_object.py     # detections -> /cmd_vel
config/cyclonedds.xml                    # Cyclone DDS network configuration
config/netplan/99-usb0.yaml              # static usb0 address for Create 3 USB networking
config/boot/config.txt.append            # Raspberry Pi USB gadget overlay
config/boot/cmdline-token.txt            # Raspberry Pi dwc2/g_ether boot token
scripts/install_pi_config.sh             # idempotent Pi configuration installer
scripts/install_nodes.sh                 # node and camera venv installer
```

## Setup On The Pi

Install ROS 2 Jazzy and make sure the shell can source it:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Install the ROS packages used by these scripts:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-vision-msgs ros-jazzy-irobot-create-msgs
```

Copy this repository to the Pi, then install the Pi-side USB and DDS configuration:

```bash
scripts/install_pi_config.sh
sudo reboot
```

The installer is idempotent and performs these setup steps:

- backs up `/boot/firmware/config.txt` and `/boot/firmware/cmdline.txt` if backups do not exist yet
- adds `dtoverlay=dwc2,dr_mode=peripheral` to `/boot/firmware/config.txt`
- adds `modules-load=dwc2,g_ether` after `rootwait` in `/boot/firmware/cmdline.txt`
- installs `/etc/netplan/99-usb0.yaml` with `192.168.186.3/24`
- installs `~/cyclonedds.xml`
- adds `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI=file://$HOME/cyclonedds.xml` to `~/.bashrc`

After reboot, verify the USB link to the robot:

```bash
ip addr show usb0
ping -c 3 192.168.186.2
```

Install the runtime nodes and the camera virtual environment:

```bash
scripts/install_nodes.sh
```

The node installer copies the scripts to `~/create3_oak/nodes` and creates `~/create3_oak/venv`. It uses `uv` automatically when available, otherwise it falls back to the standard `venv` and `pip` tooling.

Manual setup without `uv`:

```bash
python3 -m venv --system-site-packages ~/create3_oak/venv
source ~/create3_oak/venv/bin/activate
python3 -m pip install "depthai==3.7.1"
```

Optional setup with `uv`:

```bash
uv venv --system-site-packages ~/create3_oak/venv
source ~/create3_oak/venv/bin/activate
uv pip install -r pyproject.toml
```

`--system-site-packages` is required because ROS Python packages such as `rclpy`, `vision_msgs`, and `geometry_msgs` are provided by the ROS installation, not by PyPI.

## Run

Terminal A, start the camera publisher:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/create3_oak/venv/bin/activate
python3 ~/create3_oak/nodes/oak_publisher.py
```

The camera model defaults to `yolov6-nano`. This is the tested model for this project.

To try another DepthAI Hub / Model Zoo detection model, pass its exact public slug with `--model`:

```bash
python3 ~/create3_oak/nodes/oak_publisher.py --model <model-slug>
```

The same value can also be set as a ROS parameter:

```bash
python3 ~/create3_oak/nodes/oak_publisher.py --ros-args -p model:=<model-slug>
```

Use object-detection models for this pipeline. Segmentation, pose, or classification-only models require code changes because their output format differs from spatial detections.

The complete searchable model list with exact slugs is available at <https://models.luxonis.com>. Filter for "Object Detection" and copy the exact slug from the model card. Long-form slugs with a variant are also supported, for example `luxonis/yolov6-nano:r2-coco-512x288`.

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

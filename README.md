# mc_unitree2
Interface between [Unitree robots](https://github.com/unitreerobotics/unitree_ros2/tree/master/robots) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [Go2](https://www.unitree.com/products/go2/) robots.

## 1. Required dependencies

 - [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
 - [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

## 2. Install dependencies

### unitree_sdk2
 - Install the include files
```
$ git clone https://github.com/unitreerobotics/unitree_sdk2
$ mkdir build
$ cmake ..
$ make ; make install
```
## 3. Install this project

### Build instructions

```
$ cd src
$ git clone https://github.com/mc_unitree2
$ mkdir build
$ cd build
$ cmake ..
$ make ; make install
```

## 4. Usage

### Configuration

There is an example in `etc/` which content can be added to mc_rtc's config file to override the default values of this repository.  
The network interface is loopback ("lo") by default for simulation, but can be replace by your ethernet interface (check `ifconfig`) for deployment.

### Building a robot controller

Each robot interface is enabled with a CMake option: `-DGENERATE_GO2_CONTROLLER=ON`, `-DGENERATE_H1_CONTROLLER=ON` or `-DGENERATE_G1_CONTROLLER=ON` (produces `MCControlGo2`, `MCControlH1`, `MCControlG1`).

### G1 specifics

 - Requires [mc_g1](https://github.com/bastien-muraccioli/mc_g1). Use the model of the hardware revision of your robot: `MainRobot: G1_mode_<N>` where `<N>` is the `mode_machine` of the robot (Unitree app: Device → Data → Robot → Machine Type, also printed by `MCControlG1` at startup). On the real robot, `MCControlG1` switches to damping if the reported `mode_machine` does not match the model (`check-mode-machine: false` to bypass).
 - The `g1` entry of the `Unitree` configuration is used for every G1 model; an entry named after the full mc_rtc robot name (e.g. `g1_29dof_mode_15_revo2`) takes precedence.
 - Waist-locked revisions (mode_machine 6, 12, 14, 16) have 27 actuated joints, the waist roll/pitch motors are not used.
 - BrainCo Revo2 hands (`G1_mode_<N>_Revo2`, e.g. G1 EDU Ultimate E) are driven through [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service), which must be running on the robot (PC2): the 6 hand motors (`thumb_metacarpal`, `thumb_proximal`, `index/middle/ring/pinky_proximal`) are sent on `rt/brainco/<left|right>/cmd`, their joint range being mapped to [0, 1] (0: open, 1: closed), and read back from `rt/brainco/<left|right>/state`. The distal finger joints (coupled) and other joints of the model that are not motors follow the mc_rtc command. Hand commands are only sent once the controller runs (`revo2-hands: false` to disable).
 - G1 uses the `unitree_hg` messages (`LowCmd_`/`LowState_` with 35 motor slots, 29 used), ankles are controlled in PR (pitch/roll) mode to match the URDF joints.
 - Gains in the configuration are given as 29 values in motor order (see `etc/mc_rtc_example.yaml`). Motors not present in the model (e.g. wrist pitch/yaw when running the 23dof model on a 29dof robot) are held at their initial position.
 - Joint position/velocity safety limits are taken from the robot module (URDF).
 - On the real robot the Unitree motion control service is released automatically (`release-motion-mode: true`), the robot should be hanging before starting.
 - The G1 has two IMUs: the main one in the pelvis (`rt/lowstate`) feeds the default body sensor of mc_g1 (`Accelerometer`, on `pelvis`), the secondary one in the torso (`rt/secondary_imu`) feeds the `TorsoAccelerometer` body sensor.
 - Gains: `kp`/`kd` (Position/Velocity) are high gains from Unitree xr_teleoperate, `kp_torque`/`kd_torque` are softer gains used for torque control through inverse PD. A commented block in `G1Control::Control()` allows to switch to direct torque commands for testing.

### Running the program

`MCControlG1 -s` (or `--show-sensors`) replaces the status messages by a live dashboard of the sensors, refreshed every 0.5 s (`--sensors-period <s>`): current phase and what Enter does next, both IMUs (orientation, gyro, accelerometer, temperature), every joint (motor id, measured q / commanded q_ref, dq, estimated torque, kp/kd, temperature, near/out of limits, motor error) and the Revo2 hands (normalized state and command). Enter still goes from the initial posture to waiting in the air, waiting on the ground and running the controller. The reason of an emergency damping stays displayed. Values are colored when they need attention (yellow: warning, red: problem): IMU tilt above 10/25 deg, tracking error |q_ref - q| above 0.1/0.3 rad, velocity above 50% of the limit, torque above 60/90% of the URDF effort limit, motor temperature from 60/75 C, joint near/out of limits, motor error codes, hand fingers not following their command. Colors are disabled with `--no-color` or when the output is not a terminal.

```
$ MCControlUnitree --conf <install path/etc/mc_unitree/mc_rtc.yaml> --network <name of network adaptor>
```

```
# Interface specific parameters (mc_unitree)
Unitree:
  network-interface: # Name of network adaptor
```

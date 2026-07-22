# spacecraft_reaction_sim

Free-floating spacecraft arm and reaction-wheel experiments for ROS 2 Jazzy and Gazebo Sim.

## Overview

`spacecraft_reaction_sim` is a zero-gravity, free-floating robot simulation intended for reaction-dynamics and control experiments. The spacecraft base is deliberately not attached to `world`; motion of the arm produces a counter-rotation of the spacecraft.

The package includes a single-axis momentum reference experiment, a six-axis geometric arm, three reaction wheels, manual effort control, an effort-based position hold controller, and a Joint Trajectory Controller (JTC) setup with attitude hold.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim, installed through the ROS Jazzy packages
- `git`, `python3-rosdep`, and `python3-colcon-common-extensions`

GPU acceleration is not required.

### Install ROS packages

Install ROS 2 Jazzy first, then install the simulation dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rqt-joint-trajectory-controller \
  ros-jazzy-rqt-plot \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
```

Optional Foxglove Bridge support:

```bash
sudo apt install -y ros-jazzy-foxglove-bridge
```

Initialize `rosdep` once on a fresh machine:

```bash
sudo rosdep init
rosdep update
```

## Build

```bash
mkdir -p ~/spacecraft_ws/src
cd ~/spacecraft_ws/src
git clone https://github.com/aGan-bot/spacecraft_reaction_sim.git
cd ..

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the final `source install/setup.bash` command in every new terminal before using the package.

## Model and physics

- Gravity is zero.
- `spacecraft_base` has no fixed joint to `world`.
- The six arm joints have position, velocity, and effort limits.
- The final joint is a roll joint whose axis is perpendicular to joint 5.
- Reaction wheels are mounted on body X, Y, and Z axes.
- Every wheel has a bright radial pointer and white tip marker; they are visual-only and do not alter collision geometry or inertia.

For the single-axis reference, ideal angular-momentum conservation gives:

```text
omega_base = -J_wrist / (I_base + J_wrist) * q_dot_joint
```

With the supplied values `I_base,z = 123 kg m^2`, `J_wrist = 0.389 kg m^2`, and `q_dot = 2 rad/s`, the expected base rate is approximately `-0.00630 rad/s`.

## Experiments

Start a launch in one terminal, then wait until its controllers are active before sending commands from another terminal.

### 1. Single-axis momentum reference

```bash
ros2 launch spacecraft_reaction_sim spacecraft_reaction.launch.py
```

This automatically applies the `+1, 0, -1 N m` wrist torque profile and reports measured base angular velocity, predicted angular velocity, and total Z-axis angular momentum.

For manual single-axis torque instead:

```bash
ros2 launch spacecraft_reaction_sim spacecraft_manual_torque.launch.py
```

### 2. Six-axis arm with manual effort control

```bash
ros2 launch spacecraft_reaction_sim spacecraft_arm_manual.launch.py
```

After the controllers have started, command arm torques in joint order 1 through 6:

```bash
ros2 topic pub --rate 100 /arm_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 1.0]}"
```

Stop the publisher with `Ctrl-C`, then send a zero command:

```bash
ros2 topic pub --rate 20 /arm_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0]}"
```

Manual reaction-wheel torques use X, Y, Z order:

```bash
ros2 topic pub --rate 100 /reaction_wheel_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.1, 0, 0]}"
```

### 3. Effort-based joint position hold

```bash
ros2 launch spacecraft_reaction_sim spacecraft_arm_position_hold.launch.py
```

The controller captures the initial arm pose and holds it with a PD effort controller. Send a six-element target in radians:

```bash
ros2 topic pub --once /arm_position_targets \
  std_msgs/msg/Float64MultiArray "{data: [0.2, -0.3, 0.4, 0.0, 0.3, 0.5]}"
```

Targets outside the configured joint limits are clamped.

### 4. Manual arm control with attitude hold

```bash
ros2 launch spacecraft_reaction_sim spacecraft_arm_attitude_hold.launch.py
```

This starts manual arm and wheel effort controllers, then starts attitude hold after ten seconds. The hold controller captures the first spacecraft attitude received from odometry and commands wheel torques to preserve it.

All three wheels are protected by momentum-desaturation loops. The nominal wheel limit is `5000 rpm` (`523.599 rad/s`), equivalent to `33.51 N m s` at the model inertia `J = 0.064 kg m^2`. At `3000 rpm` (`314.159 rad/s`, `20.11 N m s`) a loop overrides its wheel command with a `0.30 N m` braking torque. The three-corner RCS wrench allocator then selects bounded nozzle duties that prioritize the requested unloading torque while allowing the physically coupled translation. It releases below `2500 rpm` (`261.799 rad/s`, `16.76 N m s`).

### 5. JTC with reaction-wheel attitude hold

```bash
ros2 launch spacecraft_reaction_sim spacecraft_arm_trajectory.launch.py
```

This is the recommended integrated experiment. It starts:

- `arm_joint_state_broadcaster`
- `arm_trajectory_controller`
- `reaction_wheel_effort_controller`
- `attitude_hold`

The attitude reference is captured after ten seconds. Send slow trajectories after that point.

Open the trajectory interface in a second terminal:

```bash
rqt --force-discover --standalone JointTrajectoryController
```

Select `/arm_trajectory_controller`, set joint targets within their limits, choose a low velocity, and execute the trajectory.

### 6. Six-axis RCS pulses

```bash
# actuator 0: +X body force at corner A
ros2 launch spacecraft_reaction_sim spacecraft_arm_rcs_pulse.launch.py

# actuator 1: +Y body force at corner A
ros2 launch spacecraft_reaction_sim spacecraft_arm_rcs_pulse.launch.py actuator_index:=1
```

The six nozzles are mounted on three cube corners and use this fixed actuator order: `0=+X`, `1=+Y` at corner A `(-X,-Y,-Z)`; `2=-X`, `3=+Z` at corner B `(-X,-Y,+Z)`; `4=-Y`, `5=-Z` at corner C `(+X,+Y,-Z)`. Red arrows are X jets, green are Y and blue are Z. A bounded wrench allocator converts a requested body-frame `[Fx, Fy, Fz, Tx, Ty, Tz]` into the six duty cycles. Equal opposing jets can cancel force and reinforce torque; unequal duties deliberately combine rotation with translation. Because every jet is one-directional, not every arbitrary six-axis wrench is instantly reachable. `duration_sec:=<seconds>` changes the default 0.5 s pulse.
Gazebo built-in `SpacecraftThrusterModel` applies the force using a 20 Hz PWM
duty-cycle command; the command returns to zero and the one-shot node exits
automatically at the end of the pulse.

## Monitoring and plots

Confirm controller state:

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states
ros2 topic echo /model/spacecraft_arm/odometry
```

The main JTC state topic is:

```text
/arm_trajectory_controller/controller_state
```

In Rqt Plot, open:

```bash
rqt --force-discover --standalone Plot
```

Plot reference, feedback, and error position fields for individual joints, as well as output effort. Also plot `/model/spacecraft_arm/odometry` angular X, Y, and Z to observe spacecraft reaction.

For Foxglove, start the bridge inside the ROS environment:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Connect Foxglove to `ws://localhost:8765`. A useful layout has a 3D panel, a controller-state plot, wheel command plots, and spacecraft odometry plots.

## Current limitations and next steps

- Attitude hold uses a small-angle PD controller and holds the initial attitude; it does not yet accept a world-frame attitude target.
- Three-axis reaction-wheel desaturation, a three-corner six-nozzle RCS layout, and bounded wrench allocation are implemented. Saturation is software-managed because effort-controlled Gazebo joints do not enforce URDF velocity limits.
- JTC controls joint trajectories only. Holding the end effector fixed in the world frame while changing spacecraft attitude requires a future floating-base task-space or inverse-kinematics controller.
- Geometry and inertial values are simplified for control experiments and are not a flight-qualified spacecraft model.

## Test

```bash
colcon test --packages-select spacecraft_reaction_sim
colcon test-result --test-result-base build/spacecraft_reaction_sim --verbose
```

## License

This project is licensed under Apache-2.0; see [`LICENSE`](LICENSE).

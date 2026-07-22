# Spacecraft GNC plan

## Objective

Add one ROS 2 node that owns spacecraft guidance, navigation feedback and control.  It will hold or move the free-floating spacecraft while coordinating reaction wheels and the six three-corner RCS nozzles.

## Authority boundary

`spacecraft_gnc` is the sole publisher of both `/reaction_wheel_effort_controller/commands` and `/spacecraft_arm/command/duty_cycle` in a GNC launch.  Manual `thruster_pulse` and the legacy `attitude_hold` node are diagnostic alternatives and must not run concurrently with it.

## Inputs and outputs

- Input state: `/model/spacecraft_arm/odometry`.
- Input target: `/spacecraft_arm/guidance/target_pose` (`geometry_msgs/PoseStamped`, world frame).
- Output wheel torque: `/reaction_wheel_effort_controller/commands`.
- Output RCS command: `/spacecraft_arm/command/duty_cycle`.

## Control sequence

1. Capture the first odometry pose as a safe default target.
2. Position / velocity PD produces a requested world-frame force.
3. Quaternion attitude / angular-rate PD produces the requested body torque.
4. Convert the requested force into the body frame and prefer reaction wheels for ordinary attitude corrections.
5. When a wheel passes its desaturation threshold, brake it and add the matching external RCS torque request.
6. Send the combined body wrench to the bounded three-corner RCS allocator.

## Operating modes

- `pose_hold`: captured or commanded position and attitude.
- `go_to_pose`: same controller with an updated target pose.
- `wheel_desaturation`: automatic sub-mode while holding the pose.
- `manual_pulse`: diagnostic-only; GNC is inactive.

## Validation order

1. Start in `pose_hold` and confirm zero command at the captured pose.
2. Publish a small position-only target and verify odometry convergence.
3. Publish a small attitude-only target and verify wheel response.
4. Drive a wheel above the configured threshold and verify RCS desaturation while pose error stays bounded.
5. Run JTC and verify GNC counters the spacecraft disturbance without command-topic conflicts.

#!/usr/bin/env python3
"""Send a finite PWM duty-cycle command to a selected Gazebo RCS thruster."""

import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node


def normalized_command(actuator_index, enabled):
    """Return a normalized vector with exactly one selected actuator enabled."""
    if actuator_index < 0:
        raise ValueError("Actuator index must be non-negative.")
    command = [0.0] * (actuator_index + 1)
    if enabled:
        command[actuator_index] = 1.0
    return command


class ThrusterPulse(Node):
    """Fire a selected actuator for a finite duration through SpacecraftThrusterModel."""

    def __init__(self):
        super().__init__("thruster_pulse")
        self.declare_parameter("duration_sec", 0.5)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("actuator_index", 0)
        self._duration = self.get_parameter("duration_sec").value
        command_rate = self.get_parameter("command_rate_hz").value
        self._actuator_index = self.get_parameter("actuator_index").value
        if command_rate <= 0.0:
            raise ValueError("Command rate must be positive.")
        if self._actuator_index < 0:
            raise ValueError("Actuator index must be non-negative.")
        self._started_at = None
        self._complete = False
        self._publisher = self.create_publisher(
            Actuators, "/spacecraft_arm/command/duty_cycle", 10)
        self.create_timer(1.0 / command_rate, self._tick)
        self.get_logger().info(
            "Ready to fire actuator %d through the Gazebo spacecraft thruster model."
            % self._actuator_index)

    def _publish(self, enabled):
        message = Actuators()
        message.normalized = normalized_command(self._actuator_index, enabled)
        self._publisher.publish(message)

    def _tick(self):
        if self._complete:
            return
        now = self.get_clock().now()
        if self._started_at is None:
            self._started_at = now
            self._publish(True)
            self.get_logger().info(
                "Firing RCS actuator %d at full duty cycle for %.3f s."
                % (self._actuator_index, self._duration))
            return
        elapsed = (now - self._started_at).nanoseconds * 1e-9
        if elapsed < self._duration:
            self._publish(True)
            return
        self._publish(False)
        self._complete = True
        self.get_logger().info("RCS pulse complete; actuator %d command set to zero."
            % self._actuator_index)

    
    def complete(self):
        """Whether the zero command has been published after the requested pulse."""
        return self._complete


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterPulse()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

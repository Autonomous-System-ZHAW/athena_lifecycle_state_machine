import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool
from athena_lifecycle_state_machine.athena_lifecycle_state_machine.led_controller import (
    LEDController,
)


class SystemState(Enum):
    INIT = 0
    READY = 1
    MANUAL = 2
    AUTONOMOUS = 3
    EMERGENCY = 4


class StateMachineNode(Node):

    def __init__(self):
        super().__init__("athena_state_machine")

        self.state = SystemState.INIT

        # Publisher
        self.mode_pub = self.create_publisher(String, "/system_mode", 10)

        # Subscriptions
        self.create_subscription(Bool, "/autonomy_toggle", self.autonomy_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)

        # LED Controller
        self.led = LEDController()

        self.timer = self.create_timer(0.2, self.update)

        self.get_logger().info("Athena State Machine started.")
        self.transition_to(SystemState.INIT)

    def autonomy_callback(self, msg):
        if not msg.data:
            return

        if self.state == SystemState.EMERGENCY:
            self.get_logger().warn("Autonomy ignored: system in EMERGENCY")
            return

        if self.state in [SystemState.READY, SystemState.MANUAL]:
            self.transition_to(SystemState.AUTONOMOUS)

        # Toggle zurück
        elif self.state == SystemState.AUTONOMOUS:
            self.transition_to(SystemState.MANUAL)

    def emergency_callback(self, msg):
        if not msg.data:
            return

        if self.state != SystemState.EMERGENCY:
            self.transition_to(SystemState.EMERGENCY)

    def update(self):
        if self.state == SystemState.INIT:
            self.transition_to(SystemState.READY)

        if self.state == SystemState.EMERGENCY:
            self.led.blink_red()

    def transition_to(self, new_state):
        if self.state == new_state:
            return

        self.get_logger().info(f"{self.state.name} → {new_state.name}")

        self.state = new_state

        self.led.set_mode(new_state.name)

        self.publish_mode(new_state.name)

    def publish_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool
from athena_lifecycle_state_machine.led_controller import LEDController
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import Odometry
import time
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class SystemState(Enum):
    INIT = 0
    READY = 1
    MANUAL = 2
    AUTONOMOUS = 3
    EMERGENCY = 4


class StateMachineNode(Node):

    def __init__(self):
        super().__init__("athena_state_machine")

        # --- Timeouts ---
        self.scan_timeout = 0.5
        self.joy_timeout = 0.5
        self.vesc_timeout = 0.5

        self.last_scan_time = None
        self.last_joy_time = None
        self.last_vesc_time = None

        # --- Monitoring Subscriptions ---
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.create_subscription(Odometry, "/ackermann_cmd", self.vesc_callback, 10)

        # --- Lifecycle Clients ---
        self.remote_client = self.create_client(
            ChangeState, "/remote_control/change_state"
        )

        self.ftg_client = self.create_client(
            ChangeState, "/follow_the_gap/change_state"
        )

        # Wait for lifecycle services
        while not self.remote_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for remote_control lifecycle service...")

        while not self.ftg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for follow_the_gap lifecycle service...")

        # --- Mode / Emergency Subscriptions ---
        self.create_subscription(Bool, "/autonomy_toggle", self.autonomy_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)

        # --- Publisher ---
        self.mode_pub = self.create_publisher(String, "/system_mode", 10)

        # --- LED ---
        self.led = LEDController()

        # --- Initial State ---
        self.state = SystemState.INIT
        self.get_logger().info("Athena State Machine started.")

        # --- Timer ---
        self.timer = self.create_timer(0.2, self.update)

    def configure_lifecycle_nodes(self):
        self.get_logger().info("Configuring lifecycle nodes...")

        success_remote = self.change_lifecycle_state(
            self.remote_client, Transition.TRANSITION_CONFIGURE
        )

        success_ftg = self.change_lifecycle_state(
            self.ftg_client, Transition.TRANSITION_CONFIGURE
        )

        return success_remote and success_ftg

    def scan_callback(self, msg):
        self.last_scan_time = time.time()

    def joy_callback(self, msg):
        self.last_joy_time = time.time()

    def vesc_callback(self, msg):
        self.last_vesc_time = time.time()

    def autonomy_callback(self, msg):
        if not msg.data:
            return

        if self.state == SystemState.EMERGENCY:
            self.get_logger().warn("Autonomy ignored: system in EMERGENCY")
            return

        if self.state in [SystemState.READY, SystemState.MANUAL]:
            self.transition_to(SystemState.AUTONOMOUS)

        elif self.state == SystemState.AUTONOMOUS:
            self.transition_to(SystemState.MANUAL)

    def emergency_callback(self, msg):
        if not msg.data:
            return

        if self.state != SystemState.EMERGENCY:
            self.transition_to(SystemState.EMERGENCY)

    def change_lifecycle_state(self, client, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed")
            return False

        if not future.result().success:
            self.get_logger().error("Transition failed")
            return False

        return True

    def update(self):
        now = time.time()
        if self.state == SystemState.INIT:
            if self.system_ready():
                self.get_logger().info("All sensors ready.")
                if self.configure_lifecycle_nodes():
                    self.transition_to(SystemState.READY)
                else:
                    self.transition_to(SystemState.EMERGENCY)
            return

        if self.state in [SystemState.MANUAL, SystemState.AUTONOMOUS]:
            if not self.check_health(now):
                self.transition_to(SystemState.EMERGENCY)
                return

        if self.state == SystemState.EMERGENCY:
            self.led.blink_red()

    def system_ready(self):
        return (
            self.last_scan_time is not None
            and self.last_vesc_time is not None
            and self.last_joy_time is not None
        )

    def check_health(self, now):
        if self.last_scan_time is None or now - self.last_scan_time > self.scan_timeout:
            self.get_logger().error("LiDAR timeout!")
            return False

        if self.last_vesc_time is None or now - self.last_vesc_time > self.vesc_timeout:
            self.get_logger().error("VESC timeout!")
            return False

        if self.state == SystemState.MANUAL:
            if (
                self.last_joy_time is None
                or now - self.last_joy_time > self.joy_timeout
            ):
                self.get_logger().error("Joystick timeout!")
                return False

        return True

    def transition_to(self, new_state):
        if self.state == new_state:
            return

        if new_state == SystemState.MANUAL:
            self.change_lifecycle_state(
                self.ftg_client, Transition.TRANSITION_DEACTIVATE
            )
            self.change_lifecycle_state(
                self.remote_client, Transition.TRANSITION_ACTIVATE
            )

        elif new_state == SystemState.AUTONOMOUS:
            self.change_lifecycle_state(
                self.remote_client, Transition.TRANSITION_DEACTIVATE
            )
            self.change_lifecycle_state(self.ftg_client, Transition.TRANSITION_ACTIVATE)

        elif new_state == SystemState.EMERGENCY:
            self.change_lifecycle_state(
                self.remote_client, Transition.TRANSITION_DEACTIVATE
            )
            self.change_lifecycle_state(
                self.ftg_client, Transition.TRANSITION_DEACTIVATE
            )

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

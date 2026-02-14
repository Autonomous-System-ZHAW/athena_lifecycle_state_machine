import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool
from athena_lifecycle_state_machine.led_controller import LEDController
from sensor_msgs.msg import LaserScan, Joy
import time
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from vesc_msgs.msg import VescStateStamped


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
        self.create_subscription(
            VescStateStamped, "/sensors/core", self.vesc_callback, 10
        )

        # --- Lifecycle Client (nur FTG!) ---
        self.ftg_client = self.create_client(
            ChangeState, "/follow_the_gap/change_state"
        )

        while not self.ftg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for follow_the_gap lifecycle service...")

        # --- Mode / Emergency Subscriptions ---
        self.create_subscription(Bool, "/autonomy_toggle", self.autonomy_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)

        # --- LED ---
        self.led = LEDController()

        # --- Initial State ---
        self.state = SystemState.INIT
        self.get_logger().info("Athena State Machine started.")

        # --- Timer ---
        self.timer = self.create_timer(0.2, self.update)

        # --- Publisher ---
        self.mode_pub = self.create_publisher(String, "/system_mode", 10)

    def configure_lifecycle_nodes(self):
        self.get_logger().info("Configuring FollowTheGap...")
        self.change_lifecycle_state(self.ftg_client, Transition.TRANSITION_CONFIGURE)

    def change_lifecycle_state(self, client, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        future.add_done_callback(self.lifecycle_response_callback)

    def lifecycle_response_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("Lifecycle transition successful")
            else:
                self.get_logger().error("Lifecycle transition failed")
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")

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

    def update(self):
        now = time.time()

        # INIT → READY
        if self.state == SystemState.INIT:
            if not self.system_ready():
                return

            self.get_logger().info("All sensors ready.")
            self.configure_lifecycle_nodes()
            self.transition_to(SystemState.READY)
            return

        # Runtime Health Check
        if self.state in [SystemState.MANUAL, SystemState.AUTONOMOUS]:
            if not self.check_health(now):
                self.transition_to(SystemState.EMERGENCY)
                return

        if self.state == SystemState.EMERGENCY:
            self.led.blink_red()

    def system_ready(self):
        return self.last_scan_time is not None and self.last_vesc_time is not None

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

        old_state = self.state

        # --- Only FTG Lifecycle ---
        if new_state == SystemState.AUTONOMOUS:
            self.change_lifecycle_state(self.ftg_client, Transition.TRANSITION_ACTIVATE)

        elif new_state in [SystemState.MANUAL, SystemState.EMERGENCY]:
            self.change_lifecycle_state(
                self.ftg_client, Transition.TRANSITION_DEACTIVATE
            )

        self.state = new_state

        self.get_logger().info(f"{old_state.name} → {new_state.name}")

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

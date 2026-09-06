import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Joy
import time
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from athena_custom_msgs_pkg.msg import SystemMode, Missions, VescStateStamped

from athena_common.utils import SYSTEM_MODE_NAMES, MISSION_NAMES

MISSION_ORDER = [
    Missions.FTG,
    Missions.GP,
    Missions.MPC,
]


def next_mission(current):
    index = MISSION_ORDER.index(current)
    return MISSION_ORDER[(index + 1) % len(MISSION_ORDER)]


def previous_mission(current):
    index = MISSION_ORDER.index(current)
    return MISSION_ORDER[(index - 1) % len(MISSION_ORDER)]


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

        while rclpy.ok() and not self.ftg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for follow_the_gap lifecycle service...")

        # --- Mode / Emergency Subscriptions ---
        self.create_subscription(Bool, "/emergency_stop", self.emergency_callback, 10)
        self.create_subscription(Bool, "/autonomy_toggle", self.autonomy_callback, 10)
        self.create_subscription(Bool, "/mission_toggle", self.mission_callback, 10)

        self.create_subscription(Bool, "/manuel_toggle", self.manual_callback, 10)
        self.create_subscription(Bool, "/confirm_toggle", self.confirm_callback, 10)
        self.create_subscription(Bool, "/up_toggle", self.up_selection_callback, 10)
        self.create_subscription(Bool, "/down_toggle", self.down_selection_callback, 10)

        # Latest value seen on /emergency_stop, used to gate recovery out of
        # EMERGENCY so we don't leave that state while the hazard is still active.
        self.emergency_stop_active = False

        # --- Initial State ---
        self.state = SystemMode.INIT
        self.mission = None

        self.get_logger().info("Athena State Machine started.")

        # --- Timer ---
        self.timer = self.create_timer(0.2, self.update)

        # --- Publisher ---
        self.mode_pub = self.create_publisher(SystemMode, "/system_mode", 10)
        self.mission_pub = self.create_publisher(Missions, "/mission_mod", 10)
        self.publish_system_mode(self.state)

    def update(self):
        now = time.time()

        # INIT → READY
        if self.state == SystemMode.INIT:
            if not self.system_ready():
                return

            self.get_logger().info("All sensors ready.")
            self.configure_lifecycle_nodes()
            self.transition_to(SystemMode.READY)
            return

        # Runtime Health Check
        if self.state not in (SystemMode.INIT, SystemMode.EMERGENCY):
            if not self.check_health(now):
                self.transition_to(SystemMode.EMERGENCY)
                return

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

        if self.state == SystemMode.EMERGENCY:
            self.get_logger().warn("Autonomy ignored: system in EMERGENCY")
            return

        if self.state == SystemMode.MISSION_SELECT:
            self.get_logger().warn("Autonomy ignored: currently in MISSION_SELECT")
            return

        elif self.state == SystemMode.READY:
            if self.mission is None:
                self.get_logger().error("No mission selected!")
                return

            if self.mission in (Missions.GP, Missions.MPC):
                self.get_logger().error(
                    f"Cannot go AUTONOMOUS: {MISSION_NAMES[self.mission]} controller "
                    "not implemented yet"
                )
                return

            self.transition_to(SystemMode.AUTONOMOUS)

        elif self.state == SystemMode.AUTONOMOUS:
            self.transition_to(SystemMode.READY)

    def manual_callback(self, msg):
        if not msg.data:
            return

        if self.state == SystemMode.EMERGENCY:
            self.get_logger().warn("Manual ignored: system in EMERGENCY")
            return

        if self.state == SystemMode.READY:
            self.transition_to(SystemMode.MANUAL_JOY)

        elif self.state == SystemMode.MANUAL_JOY:
            self.transition_to(SystemMode.READY)

    def mission_callback(self, msg):
        if not msg.data:
            return

        if self.state == SystemMode.EMERGENCY:
            self.get_logger().warn("Mission ignored: system in EMERGENCY")
            return

        elif self.state == SystemMode.AUTONOMOUS:
            self.get_logger().warn("Mission change ignored: currently in AUTONOMOUS")
            return

        if self.state == SystemMode.READY:
            self.transition_to(SystemMode.MISSION_SELECT)

        elif self.state == SystemMode.MISSION_SELECT:
            self.transition_to(SystemMode.READY)

    def emergency_callback(self, msg):
        self.emergency_stop_active = bool(msg.data)

        if self.emergency_stop_active and self.state != SystemMode.EMERGENCY:
            self.transition_to(SystemMode.EMERGENCY)

    def system_ready(self):
        return self.last_scan_time is not None and self.last_vesc_time is not None

    def check_health(self, now):
        if self.last_scan_time is None or now - self.last_scan_time > self.scan_timeout:
            self.get_logger().error("LiDAR timeout!")
            return False

        if self.last_vesc_time is None or now - self.last_vesc_time > self.vesc_timeout:
            self.get_logger().error("VESC timeout!")
            return False

        if self.state == SystemMode.MANUAL_JOY:
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

        if new_state == SystemMode.AUTONOMOUS:
            if self.mission == Missions.FTG:
                self.change_lifecycle_state(
                    self.ftg_client, Transition.TRANSITION_ACTIVATE
                )

            elif self.mission == Missions.GP:
                # ToDo
                self.get_logger().warn("GP not implemented yet")

            elif self.mission == Missions.MPC:
                # ToDo
                self.get_logger().warn("MPC not implemented yet")

        elif new_state == SystemMode.EMERGENCY:
            self.change_lifecycle_state(
                self.ftg_client, Transition.TRANSITION_DEACTIVATE
            )

        elif new_state == SystemMode.MANUAL_JOY:
            # ToDo
            self.get_logger().warn("Remote drive control node not implemented yet")

        elif new_state == SystemMode.MISSION_SELECT:
            if self.mission is None:
                self.mission = Missions.FTG

        self.get_logger().info(
            f"{SYSTEM_MODE_NAMES.get(old_state, old_state)} → "
            f"{SYSTEM_MODE_NAMES.get(new_state, new_state)}"
        )

        self.state = new_state

        self.publish_system_mode(new_state)

        if self.mission is not None:
            self.publish_mission_mode(self.mission)

    def confirm_callback(self, msg):
        if not msg.data:
            return

        if self.state == SystemMode.EMERGENCY:
            # Manual recovery path out of EMERGENCY: only allowed once the
            # hazard has cleared and sensors/VESC are healthy again.
            if self.emergency_stop_active:
                self.get_logger().warn("Cannot reset: /emergency_stop still active")
                return

            if not self.check_health(time.time()):
                self.get_logger().warn("Cannot reset: system not healthy")
                return

            self.get_logger().info("EMERGENCY cleared, returning to READY")
            self.transition_to(SystemMode.READY)
            return

        if self.state != SystemMode.MISSION_SELECT:
            return

        if self.mission is None:
            self.get_logger().warn("No mission selected")
            return

        self.transition_to(SystemMode.READY)

    def up_selection_callback(self, msg):
        if not msg.data:
            return

        if self.state != SystemMode.MISSION_SELECT:
            return

        if self.mission is None:
            self.mission = Missions.FTG
        else:
            self.mission = next_mission(self.mission)

        self.publish_mission_mode(self.mission)
        self.get_logger().info(f"Mission: {MISSION_NAMES[self.mission]}")

    def down_selection_callback(self, msg):
        if not msg.data or self.state != SystemMode.MISSION_SELECT:
            return

        if self.mission is None:
            self.mission = Missions.FTG
        else:
            self.mission = previous_mission(self.mission)

        self.publish_mission_mode(self.mission)
        self.get_logger().info(f"Mission: {MISSION_NAMES[self.mission]}")

    def publish_system_mode(self, mode_value):
        msg = SystemMode()
        msg.mode = mode_value
        self.mode_pub.publish(msg)

    def publish_mission_mode(self, mission_value):
        msg = Missions()
        msg.mission = mission_value
        self.mission_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

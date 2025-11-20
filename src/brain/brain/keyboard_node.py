import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from time import sleep
import sys
import select


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")

        self.toggle_log_pub = self.create_publisher(Bool, "/kb/toggle_log", 10)
        self.set_min_blue_sat_pub = self.create_publisher(Int32, "/kb/set_min_blue_sat", 10)
        self.shutdown_pub = self.create_publisher(Bool, "/kb/shutdown", 10)
        self.enable_prelim_cv_pub = self.create_publisher(Bool, "/kb/enable_prelim_cv", 10)
        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False

        self.get_logger().info(
            "\n"
            "-----------------------------------\n"
            "Keyboard Input Node is running.\n"
            "Press 'l' then Enter to toggle vision node logging.\n"
            "Press 'i' then Enter to enable preliminary CV window.\n"
            "Press 'q' then Enter to quit.\n"
            "-----------------------------------"
        )

        self.main_loop()

    def main_loop(self):
        while rclpy.ok() and not self.shutdown_requested:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.shutdown_requested:
                break

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                try:
                    user_input = input("Enter command: ")
                    if user_input == "l":
                        self.get_logger().info(
                            "User pressed 'l'. Sending LOG toggle signal to grid_vision_node."
                        )

                        log_msg = Bool()
                        log_msg.data = True
                        self.toggle_log_pub.publish(log_msg)

                    elif user_input == "i":
                        self.get_logger().info(
                            "User pressed 'i'. Toggling preliminary CV window."
                        )

                        prelim_msg = Bool()
                        prelim_msg.data = True
                        self.enable_prelim_cv_pub.publish(prelim_msg)
                    
                    elif user_input.startswith("s"):
                        try:
                            _, value_str = user_input.split()
                            value = int(value_str)
                            self.get_logger().info(
                                f"User pressed 's'. Setting minimum blue saturation to {value}."
                            )

                            sat_msg = Int32()
                            sat_msg.data = value
                            self.set_min_blue_sat_pub.publish(sat_msg)
                        except (ValueError, IndexError):
                            self.get_logger().warn("Invalid command format for 's'. Use: s <value>")

                    elif user_input == "q":
                        self.get_logger().info("User pressed 'q'. Shutting down.")

                        shutdown_msg = Bool()
                        shutdown_msg.data = True
                        self.shutdown_pub.publish(shutdown_msg)

                        sleep(0.5)
                        break

                    else:
                        self.get_logger().warn(f"Unknown command: '{user_input}'")

                except EOFError:
                    break

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Shutdown signal received.")
            self.shutdown_requested = True


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    try:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

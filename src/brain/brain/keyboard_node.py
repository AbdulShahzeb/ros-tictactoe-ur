import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")

        self.toggle_log_pub = self.create_publisher(Bool, "/kb/toggle_log", 10)
        self.shutdown_pub = self.create_publisher(Bool, "/kb/shutdown", 10)

        self.get_logger().info(
            "\n"
            "-----------------------------------\n"
            "Keyboard Input Node is running.\n"
            "Press 'l' then Enter to toggle vision node logging.\n"
            "Press 'q' then Enter to quit.\n"
            "-----------------------------------"
        )

        self.main_loop()

    def main_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter command: ")
                if user_input == "l":
                    self.get_logger().info(
                        "User pressed 'l'. Sending LOG toggle signal to grid_vision_node."
                    )

                    log_msg = Bool()
                    log_msg.data = True
                    self.toggle_log_pub.publish(log_msg)

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
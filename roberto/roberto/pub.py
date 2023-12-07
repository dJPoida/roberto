import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__("roberto_talker_test")
        self.counter_ = 0
        self.publisher = self.create_publisher(String, "test_talker", 10)
        self._timer = self.create_timer(0.5, self.publish_hello)
        self.get_logger().info("publishing message")

    def publish_hello(self):
        msg = String()
        self.counter_ += 1
        msg.data = "Flapping Chops: " + str(self.counter_)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
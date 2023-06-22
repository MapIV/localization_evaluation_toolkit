import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import sys
from time import sleep
import threading

finish=False

class MySubscriber(Node):

    msg_init=PoseWithCovarianceStamped()
    is_init=False

    def __init__(self):
        super().__init__('ros_subscriber'+sys.argv[2])
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.listener_callback,
            1)
        self.subscription
        self.data = []

    def listener_callback(self, msg):
        if sys.argv[1]=="only_subscribe":
            return
        self.get_logger().info('Subscribing /initialpose')
        self.msg_init=msg
        thread1 = threading.Thread(target=self.play)
        thread2 = threading.Thread(target=self.init_pub)
        thread1.start()
        thread2.start()
        thread2.join()
        thread1.join()
    
    def play(self):
        command='ros2 bag play '+sys.argv[1]+' --clock 200'
        os.system(command)
        self.get_logger().info("ROSBAG Play Finished!")
        
    def init_pub(self):
        sleep(5)
        self.publisher_.publish(self.msg_init)
        self.get_logger().info('Publishing /initialpose')


def main(args=None):
    rclpy.init(args=args)

    node = MySubscriber()

    rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

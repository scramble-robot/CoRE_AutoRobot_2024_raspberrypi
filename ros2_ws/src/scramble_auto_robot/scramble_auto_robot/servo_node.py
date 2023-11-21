import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import pigpio


def setServoDegree(pi, output_pin, degree):
    min_width = 500
    max_width = 2500
    deg_range = 270
    output = degree / deg_range
    output *= max_width - min_width
    output += min_width
    output = int(output)
    pi.set_servo_pulsewidth(output_pin, output)


class ServoSubscriber(Node):

    def __init__(self, pi):
        super().__init__('servo_node')
        self.declare_parameter('pin', 12)
        self.subscription = self.create_subscription(
            Float64,
            '~/degree',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.pi = pi
        self.output_pin = self.get_parameter('pin').get_parameter_value().integer_value
        self.prev_degree = -1
        
        self.get_logger().info(f"Output Pin: {self.output_pin}")
        
        self.pi.set_mode(self.output_pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.output_pin, 0)

    def listener_callback(self, msg):
        if msg.data < 0 or msg.data > 270:
            return
        if self.prev_degree == msg.data:
            return
        setServoDegree(self.pi, self.output_pin, msg.data)
        self.prev_degree = msg.data

    
def main(args=None):
    rclpy.init(args=args)

    pi = pigpio.pi()

    subscriber = ServoSubscriber(pi)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

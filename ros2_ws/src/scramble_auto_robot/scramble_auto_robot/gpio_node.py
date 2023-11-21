import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from std_msgs.msg import Bool
from datetime import datetime

import pigpio

class GpioInputPublisher:
    def __init__(self, pi:pigpio.pi, clock, publisher: Publisher, pin: int):
        self.pi = pi
        self.clock = clock
        self.publisher = publisher
        self.pin = pin
        self.prev_state = False
        self.prev_pub_clock = self.clock.now()

        self.pi.set_mode(pin, pigpio.INPUT)
        self.publish(force=True)
    
    def publish(self, force=False):
        pin_state = bool(self.pi.read(self.pin))
        duration = self.clock.now() - self.prev_pub_clock
        if force or self.prev_state != pin_state or duration.nanoseconds >= 1e9:
            topic = Bool()
            topic.data = pin_state
            self.publisher.publish(topic)
            self.prev_state = pin_state
            self.prev_pub_clock = self.clock.now()

class GpioNode(Node):

    def __init__(self, pi):
        super().__init__('gpio_node')
        self.pi = pi

        self.declare_parameter('in0', 19)
        self.declare_parameter('in1', 16)
        self.declare_parameter('in2', 26)
        self.declare_parameter('in3', 20)
        
        clock = self.get_clock()
        
        self.input_publishers = []
        for i in range(4):
            pub = self.create_publisher(Bool, f'~/in{i}', 10)
            pin = self.get_parameter(f'in{i}').get_parameter_value().integer_value
            # set GPIO as input        
            self.get_logger().info(f"in{i}: {pin}")

            self.input_publishers.append( GpioInputPublisher(pi, clock, pub, pin) )
        
        self.timer = self.create_timer(0.05, self.timer_callback)
 
    def timer_callback(self):
        for in_pub in self.input_publishers:
            in_pub.publish()

    
def main(args=None):
    rclpy.init(args=args)

    pi = pigpio.pi()

    publisher = GpioNode(pi)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

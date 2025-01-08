# pip install sense-hat
# sudo chmod 777 /dev/fb0
# sudo chmod 777 /dev/input/event1

# ros2 run wheeltec_py_package sense_hat_disp

from sense_hat import SenseHat
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sense_hat_disp')
        # subscribe to voltage: /power_voltage std_msgs/msg/Float32
        self.subscription = self.create_subscription(Float32,'power_voltage', self.voltage_callback, 10)
        self.sense = SenseHat()

    def voltage_callback(self, msg):
        ## for testing: ros2 topic pub /power_voltage std_msgs/msg/Float32 "data: 1.0"
        if msg.data > 21.5:
            self.sense.show_letter("_")
        else:
            self.sense.show_letter("X")
        # self.sense.show_message(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    sense_hat_disp = MinimalPublisher()
    rclpy.spin(sense_hat_disp)
    sense_hat_disp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


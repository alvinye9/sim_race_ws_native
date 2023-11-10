import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        
        # Create publishers for each topic
        self.accelerator_pub = self.create_publisher(Float32, '/joystick/accelerator_cmd', 10)
        self.gear_pub = self.create_publisher(Int8, '/joystick/gear_cmd', 10)
        self.brake_pub = self.create_publisher(Float32, '/joystick/brake_cmd', 10)
        self.steering_pub = self.create_publisher(Float32, '/joystick/kin_control_steering_cmd', 10)
        self.timer = self.create_timer(1, self.publish_messages)

    def publish_messages(self):
        # Publish messages to each topic
        accelerator_msg = Float32()
        accelerator_msg.data = 50.  # Example accelerator command value (adjust as needed)
        self.accelerator_pub.publish(accelerator_msg)
        self.get_logger().info('acc:'+ str(accelerator_msg.data))

        gear_msg = Int8()
        gear_msg.data = 1  # Example gear command value (adjust as needed)
        self.gear_pub.publish(gear_msg)
        self.get_logger().info('gear:' + str(gear_msg.data))

        brake_msg = Float32()
        brake_msg.data = 0.2  # Example brake command value (adjust as needed)
        self.brake_pub.publish(brake_msg)
        self.get_logger().info('brake:' + str(brake_msg.data))

        steering_msg = Float32()
        steering_msg.data = 45. # Example kin_control command value (adjust as needed)
        self.steering_pub.publish(steering_msg)
        self.get_logger().info('steer:' + str(steering_msg.data))
        self.get_logger().info('Published joystick cmds')

def main(args=None):
    print("test1")
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()

    try:
        rclpy.spin(joystick_publisher)
    finally:
        joystick_publisher.destroy_node()
        rclpy.shutdown()

    
    # try:
    #     # Publish messages
    #     joystick_publisher.publish_messages()
    #     rclpy.spin_once(joystick_publisher, timeout_sec=0.1)
    # finally:
    #     joystick_publisher.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()

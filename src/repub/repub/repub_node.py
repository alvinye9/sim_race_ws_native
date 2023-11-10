import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Header
from blackandgold_msgs.msg import BasestationCommand
#from sensor_msgs.msg import Joy
from autonoma_msgs.msg import VehicleInputs



#Node to republish messages from Simulink stack to SIM Race Simulator

class Simulink_to_SIM_Race(Node):
    curr_acc_cmd = 0.0
    curr_brake_cmd = 0.0
    curr_steer_cmd = 0.0
    curr_gear_cmd= 0

    throttle_counter = 0
    brake_counter = 0
    steering_counter = 0
    gear_counter = 0

    STEER_CONSTANT = 13.3333 #200/15

    true_hor_vel = 0.0 #Double 8 m/s
    true_heading =  0.0   #DOuble 8 degrees

    
    def __init__(self):
        super().__init__('repub_node')
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/accelerator_cmd',
            self.accelerator_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/curr_brake_cmd',
            self.curr_brake_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/kin_control_steering_cmd',
            self.steering_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int8,
            '/joystick/gear_cmd',
            self.gear_cmd_callback,
            10
        )

        #create vehicle_input publisher
        self.publisher = self.create_publisher(
            VehicleInputs,
            '/vehicle_inputs',
            10
        )
        self.timer = self.create_timer(1, self.publish_vehicle_inputs)

        #self.subscription = self.create_subscription(BESTVEL, '/novatel_top/bestvel', self.top_vel_callback, 10)


    def accelerator_cmd_callback(self, msg):
        self.curr_acc_cmd = msg.data
        #print(self.curr_acc_cmd)
        #curr_gear_cmd= int(round(msg.data))  # Convert all msg data to int
        #self.publish_vehicle_inputs(throttle_cmd=msg.data, curr_brake_cmd=msg.data, steering_cmd=msg.data, gear_cmd=gear_cmd)
        #self.publish_vehicle_inputs()
        #self.get_logger().info('Republished accelerator cmd based on joystick')

    def curr_brake_cmd_callback(self,msg):
        self.curr_brake_cmd = msg.data
        #print(self.curr_brake_cmd)
        #curr_gear_cmd= int(round(msg.data))  # Convert to int
        #self.publish_vehicle_inputs(throttle_cmd=msg.data, curr_brake_cmd=msg.data, steering_cmd=msg.data, gear_cmd=gear_cmd)
        #self.publish_vehicle_inputs()
        #self.get_logger().info('Republished brake cmd')

    def steering_cmd_callback(self,msg):
        self.curr_steer_cmd = msg.data
        #print(self.curr_steer_cmd)
        #curr_gear_cmd= int(round(msg.data))  # Convert to int
        #self.publish_vehicle_inputs(throttle_cmd=msg.data, curr_brake_cmd=msg.data, steering_cmd=msg.data, gear_cmd=gear_cmd)
        #self.publish_vehicle_inputs()
        #self.get_logger().info('Republished steering cmd:')

    def gear_cmd_callback(self,msg):
        self.curr_gear_cmd= msg.data
        #print(self.curr_gear_cmd)
        # curr_gear_cmd= int(round(msg.data))  # Convert to int
        # self.publish_vehicle_inputs(throttle_cmd=msg.data, curr_brake_cmd=msg.data, kin_control_steering_cmd=msg.data, gear_cmd=gear_cmd)
        #self.publish_vehicle_inputs()
        #self.get_logger().info('Republished gear cmd')

    def top_vel_callback(self,msg):
        return

    def publish_vehicle_inputs(self):
        msg = VehicleInputs()
        msg.header = Header()
        msg.throttle_cmd = self.curr_acc_cmd
        # self.throttle_counter = self.throttle_counter + 1
        # msg.throttle_cmd_count = self.throttle_counter

        msg.brake_cmd = self.curr_brake_cmd
        # self.brake_counter = self.brake_counter + 1
        # msg.brake_cmd_count = self.brake_counter

        msg.steering_cmd = self.curr_steer_cmd
        # self.steering_counter = self.steering_counter + 1
        # msg.steering_cmd_count = self.steering_counter 

        msg.gear_cmd= self.curr_gear_cmd

        self.publisher.publish(msg)

        self.get_logger().info('acc:' + str(self.curr_acc_cmd))
        self.get_logger().info('Republished vehicle cmds')


def main(args=None):
    rclpy.init(args=args)
    node = Simulink_to_SIM_Race()
    print("doo")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Header
from blackandgold_msgs.msg import BasestationCommand
from novatel_oem7_msgs.msg import BESTPOS
from autonoma_msgs.msg import VehicleInputs

from pyproj import CRS, Transformer 

from geometry_msgs.msg import TransformStamped
from math import cos, sin
import tf2_ros
import math

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


    true_easting = 521921.53 #Float32 UTM 32N, origin of Monza 
    true_northing = 5051752.75 #UTM 32N
    true_heading = 0 #Float32 degrees

    
    def __init__(self):
        super().__init__('repub_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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

        self.subscription = self.create_subscription(
            BESTPOS, 
            '/novatel_top/bestgnsspos', 
            self.pos_callback, 
            10)
        self.subscription = self.create_subscription(
            Float32, 
            '/novatel_top/heading2', 
            self.heading_callback, 
            10)   

        #self.subscription = self.create_subscription(BESTVEL, '/novatel_top/bestvel', self.top_vel_callback, 10)

        # #create vehicle_pos publisher
        # self.publisher = self.create_publisher(Float32,'/localization/vehicle_speed', 10)


        # create timer to publish every second
        self.timer1 = self.create_timer(1.0, self.publish_vehicle_inputs)
        self.timer2 = self.create_timer(1.0, self.publish_tf)

        


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

    def pos_callback(self,msg):
        latitude = msg.lat
        longitude = msg.lon
        [self.true_easting, self.true_northing] = self.latlon_to_utm(latitude, longitude)
        # self.get_logger().info('Lat:' + str(latitude))
        # self.get_logger().info('Long:' + str(longitude))
        # self.get_logger().info('Easting:' + str(self.true_easting))
        # self.get_logger().info('Northing:' + str(self.true_northing))

    def heading_callback(self,msg):
        self.true_heading = msg.heading #AKA Yaw (psi)
        self.get_logger().info('Heading:' + str(self.true_heading))

    #callback for timer 1
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

    #callback for timer 2
    def publish_tf(self):
        # Replace these with your actual x, y positions, and yaw angle
        x = self.true_easting
        y = self.true_northing
        yaw = math.radians(self.true_heading)  # Yaw angle in radians

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'vehicle'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0  # Assuming a 2D scenario
        q = tf2_ros.transformations.quaternion_from_euler(0, 0, yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)

# ============== Helper Functions ================

    def latlon_to_utm(self, latitude, longitude):
        # Define UTM projection for Zone 32N
        utm_zone = 32
        utm_band = 'N'  # Northern Hemisphere
        utm_proj = CRS.from_string(f'EPSG:{32600 + utm_zone}')

        # Create a transformer
        transformer = Transformer.from_crs(CRS.from_epsg(4326), utm_proj, always_xy=True)

        # Transform latitude and longitude to UTM coordinates
        utm_easting, utm_northing = transformer.transform(longitude, latitude)

        return utm_easting, utm_northing

def main(args=None):
    rclpy.init(args=args)
    node = Simulink_to_SIM_Race()
    print("doo")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



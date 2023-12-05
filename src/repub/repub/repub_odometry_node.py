import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Int8, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Quaternion
from sensor_msgs.msg import Imu

from blackandgold_msgs.msg import BasestationCommand
from novatel_oem7_msgs.msg import BESTPOS, HEADING2, BESTVEL, INSPVA
from autonoma_msgs.msg import VehicleInputs

from pyproj import CRS, Transformer 

from geometry_msgs.msg import TransformStamped
from math import cos, sin
from tf2_ros import TransformBroadcaster
import math
import numpy as np

#Node to republish messages from Simulink stack to SIM Race Simulator

class Repub_Odom(Node):
    curr_acc_cmd = 0.0
    curr_brake_cmd = 0.0
    curr_steer_cmd = 0.0
    curr_gear_cmd= 0

    throttle_counter = 0
    brake_counter = 0
    steering_counter = 0
    gear_counter = 0

    STEER_CONSTANT = 13.3333 #200/15
    UTM_ORIGIN_NORTHING = 5051752.75
    UTM_ORIGIN_EASTING = 521921.53

# Position
    true_easting = UTM_ORIGIN_EASTING  #Float32 UTM 32N, origin of Monza 
    true_northing = UTM_ORIGIN_NORTHING #UTM 32N
    true_heading = 0.0 #Float32 degrees
    true_local_x = 0.0
    true_local_y = 0.0
    latitude = 0.0
    longitude = 0.0
    height = 0.0

 # Orientation (Quarternion)
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    w = 0.0
    
# Linear Velocity
    v_x = 0.0
    v_y = 0.0
    v_z = 0.0

# Angular Velocity
    v_roll = 0.0
    v_pitch = 0.0
    v_yaw = 0.0

#Linear Acceleration
    a_x = 0.0
    a_y = 0.0
    a_z = 0.0


#Imu data
    imu_data = Imu()
    

    
    def __init__(self):
        super().__init__('repub_odometry_node')


        # self.subscription = self.create_subscription(
        #     BESTVEL, 
        #     '/novatel_top/bestvel', 
        #     self.vel_callback, 
        #     10)

        self.subscription = self.create_subscription(
            INSPVA, 
            '/novatel_top/inspva', 
            self.pva_callback, #pos, vel, attitude
            10)
        
        self.subscription = self.create_subscription(
            Imu, 
            '/novatel_top/rawimux', 
            self.imu_callback, #pos, vel, attitude
            10)

        #PUBLISHERS
        
        self.wheel_odom_publisher = self.create_publisher(
            Odometry,
            '/odometry/wheel_odom', 
            10)
        self.local_filtered_publisher = self.create_publisher(
            Odometry,
            '/odometry/local_filtered', 
            10)
        self.top_odom_filtered_publisher = self.create_publisher(
            Odometry,
            'novatel_top/gps_odom_filtered',
            10
        )
        self.bottom_odom_filtered_publisher = self.create_publisher(
            Odometry,
            'novatel_bottom/gps_odom_filtered',
            10
        )
        self.top_odom_publisher = self.create_publisher(
            Odometry,
            'novatel_top/odom',
            10
        )
        self.bottom_odom_publisher = self.create_publisher(
            Odometry,
            'novatel_bottom/odom',
            10
        )
        
        self.top_ins_twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/novatel_top/ins_twist', 
            10)
        self.bottom_ins_twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/novatel_bottom/ins_twist', 
            10)
        
        self.top_imu_data_filtered_publisher = self.create_publisher(
            Imu, 
            '/novatel_top/imu/data_sw_filtered', 
            10)
        self.bottom_imu_data_filtered_publisher = self.create_publisher(
            Imu, 
            '/novatel_bottom/imu/data_sw_filtered', 
            10)
        self.top_imu_data_publisher = self.create_publisher(
            Imu, 
            '/novatel_top/imu/data', 
            10)
        self.bottom_imu_data_publisher = self.create_publisher(
            Imu, 
            '/novatel_bottom/imu/data', 
            10)
        
        



        # create timer to publish every second
        PUBLISH_RATE = 0.15
        self.timer4 = self.create_timer(PUBLISH_RATE, self.publish_odometry)
        self.timer5 = self.create_timer(PUBLISH_RATE, self.publish_ins_twist)
        self.timer6 = self.create_timer(PUBLISH_RATE, self.publish_imu_data)



    def pva_callback(self, msg):
        # Position (Latitude -> UTM -> Local x y)
        self.latitude  = msg.latitude
        self.longitude = msg.longitude
        self.height = msg.height
        [self.true_easting, self.true_northing] = self.latlon_to_utm(self.latitude, self.longitude)
        self.true_local_x = self.true_easting - self.UTM_ORIGIN_EASTING 
        self.true_local_y = self.true_northing - self.UTM_ORIGIN_NORTHING
        
        # Orientation (Euler -> Quarternion)
        #(yaw is not equivalent to azimuth (I think), thats why true_heading is calculated from azimuth)
        # azimuth is Left-handed rotation around z-axis in degrees clockwise from North
        self.true_heading = -1* self.degrees_to_radians(msg.azimuth) - math.pi/2  #this makes sure that novatel top orients with x in vehicle direction, y to left
        q = self.quaternion_from_euler(msg.roll,msg.pitch,self.true_heading)
        self.roll = q[0]
        self.pitch = q[1]
        self.yaw = q[2]
        self.w = q[3]

        # Linear Velocity
        self.v_x = msg.east_velocity
        self.v_y = msg.north_velocity
        self.v_z = msg.up_velocity
        
    def imu_callback(self, msg):
        #recieve imu data
        self.imu_data = msg

        # angular velocity (if not directly repubing as Imu msg type)
        self.v_roll = msg.angular_velocity.x
        self.v_pitch = msg.angular_velocity.y
        self.v_yaw = msg.angular_velocity.z
        


    #callback for timer 4
    #need to publish (at minimum) x y yaw vx vy vyaw
    def publish_odometry(self):
        # Create an Odometry message
        odometry_msg = Odometry()

        # Fill in the header information
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        #odometry_msg.header.frame_id = 'odom'

        # Fill in the pose information
        odometry_msg.pose.pose.position.x = self.true_local_x # x position WRT local reference (origin)
        odometry_msg.pose.pose.position.y = self.true_local_y  # y position WRT local reference
        odometry_msg.pose.pose.position.z = self.height

        odometry_msg.pose.pose.orientation.x = self.roll #should be orientation in quarternion form
        odometry_msg.pose.pose.orientation.y = self.pitch
        odometry_msg.pose.pose.orientation.z = self.yaw
        odometry_msg.pose.pose.orientation.w = self.w

        # Fill in the twist information
        odometry_msg.twist.twist.linear.x = self.v_x  
        odometry_msg.twist.twist.linear.y = self.v_y  
        odometry_msg.twist.twist.linear.z = self.v_z  

        odometry_msg.twist.twist.angular.x = self.v_roll  
        odometry_msg.twist.twist.angular.y = self.v_pitch  
        odometry_msg.twist.twist.angular.z = self.v_yaw

        # Publish the Odometry message
        self.wheel_odom_publisher.publish(odometry_msg)
        self.local_filtered_publisher.publish(odometry_msg)
        self.top_odom_filtered_publisher.publish(odometry_msg)
        self.bottom_odom_filtered_publisher.publish(odometry_msg)
        self.top_odom_publisher.publish(odometry_msg)
        self.bottom_odom_publisher.publish(odometry_msg)        

    def publish_ins_twist(self):
        ins_twist_msg = TwistWithCovarianceStamped()
        ins_twist_msg.header.stamp = self.get_clock().now().to_msg()
        #ins_twist_msg.header.frame_id = ''
        
        ins_twist_msg.twist.twist.linear.x = self.v_x
        ins_twist_msg.twist.twist.linear.y = self.v_y 
        ins_twist_msg.twist.twist.linear.z = self.v_z

        ins_twist_msg.twist.twist.angular.x = 0.0  # assuming no angular x velocity
        ins_twist_msg.twist.twist.angular.y = 0.0  # assuming no angular y velocity
        ins_twist_msg.twist.twist.angular.z = 0.0
        # publish
        self.top_ins_twist_publisher.publish(ins_twist_msg)
        self.bottom_ins_twist_publisher.publish(ins_twist_msg)

    def publish_imu_data(self):
        imu_msg = self.imu_data
        # imu_msg = Imu()

        # imu_msg.header.stamp = self.get_clock().now().to_msg()
        # #imu_data_msg.header.frame_id = 'base_link'  
        # imu_msg.orientation.x = self.roll
        # imu_msg.orientation.y = self.pitch
        # imu_msg.orientation.z = self.yaw

        # imu_msg.angular_velocity.x = 0.0
        # imu_msg.angular_velocity.y = 0.0
        # imu_msg.angular_velocity.z = 0.0
        self.top_imu_data_filtered_publisher.publish(imu_msg)
        self.top_imu_data_publisher.publish(imu_msg)
        self.bottom_imu_data_filtered_publisher.publish(imu_msg)
        self.bottom_imu_data_publisher.publish(imu_msg)





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

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def degrees_to_radians(self, degrees): #[0,359] to [-pi,pi]
        radians = (degrees - 180) / 180 * math.pi
        return radians

def main(args=None):
    rclpy.init(args=args)
    node = Repub_Odom()
    #print("test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



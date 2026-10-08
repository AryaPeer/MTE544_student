# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below

""" 
Added import statements for Twist (standard message for linear/angular velocity commands),
LaserScan (message type for lidar), Odometry (message type for wheel encoders)
"""
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        self.velocity_=0.0
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        # publishes Twist to the /cmd_vel topic, with 10 being the max queue size
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        """ 
        BEST_EFFORT means that the QoS profile drops stale messages instead of retrying, since only latest readings matter,
        VOLATILE means it only receives messages published after it subscribes, so no stored history
        """
        qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # each subscription pairs a message type with a topic and a callback function

        # IMU subscription
        self.imu_subscription=self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        
        # ENCODER subscription
        self.odom_subscription=self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        
        # LaserScan subscription 
        self.laser_subscription=self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    # takes timestamp from msg header, logs linear_acceleration and sets imu init to true
    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        ts = Time.from_msg(imu_msg.header.stamp).nanoseconds
        self.imu_logger.log_values([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.angular_velocity.z, ts])
        self.imu_initialized = True

    # takes timestamp from msg header, logs position and theta (derived from quarternion) and sets odom init to true
    def odom_callback(self, odom_msg: Odometry):

        # log odom msgs
        ts = Time.from_msg(odom_msg.header.stamp).nanoseconds

        quaternion = odom_msg.pose.pose.orientation
        position = odom_msg.pose.pose.position
        theta = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        self.odom_logger.log_values([position.x, position.y, theta, ts])
        self.odom_initialized = True

    # takes timestamp from msg header, logs ranges and angle_increment and sets laser init to true
    def laser_callback(self, laser_msg: LaserScan):
        # log laser msgs with position msg at that time
        ts = Time.from_msg(laser_msg.header.stamp).nanoseconds
        self.laser_logger.log_values([*laser_msg.ranges, laser_msg.angle_increment, ts])
        self.laser_initialized = True

    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot
    """
    Turtlebot is differential drive, so only linear.x (forward/backward speed) and angular.z (yaw/turn rate) are effective.
    All other values are set to 0.
    """

    # constant linear and angular velocity result in a circular drive path. Radius = v/w = 0.1/0.1 = 1 meter
    def make_circular_twist(self):
        
        msg=Twist()

        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1
        
        return msg

    # constant angular velocity, with radius growing by 0.0005 each 0.1s and increasing linear velocity, results in spiral shape
    def make_spiral_twist(self):
        msg=Twist()

        self.radius_ += 0.0005

        msg.linear.x = min(0.1 + self.radius_, 0.2)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        return msg

    # angular velocity set to zero, linear velocity increases by 0.005 every 0.1s, accelerates in straight line
    def make_acc_line_twist(self):
        msg=Twist()

        self.velocity_ += 0.005

        msg.linear.x = min(0.1 + self.velocity_, 0.2)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")

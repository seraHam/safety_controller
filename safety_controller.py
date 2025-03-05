#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pi = np.pi
cos = np.cos
sin = np.sin

LEFT = 1
RIGHT = -1

WALL_TOPIC = "/wall"

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("safety_topic", "/vesc/low_level/input/safety")

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
		
        # Initialize subscriber to laser scan data
        self.drive_sub = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_cb, 10)
        self.laser_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.control, 10)

        self.control_pub = self.create_publisher(AckermannDriveStamped, self.SAFETY_TOPIC, 10)

        self.car_velocity = 0.

    def least_squares(self, x, y):
        x_bar = np.average(x)
        y_bar = np.average(y)
        m = sum((x - x_bar)*(y-y_bar))/sum((x-x_bar)**2)
        y0 = y_bar - m * x_bar

        return -m, y0
    
    def get_wall(self, msg):

        # get wall in front of robot
        mid_arr = len(msg.ranges)//2
        num_pts = 5
        lower_bnd = mid_arr - num_pts
        upper_bnd = mid_arr + num_pts

        wall_ranges = msg.ranges[lower_bnd: upper_bnd]
        angle_inc = msg.angle_increment
        theta0 = msg.angle_min
        thetas = [theta0 + angle_inc * i for i in range(lower_bnd, upper_bnd)]
            
        x = wall_ranges * cos(thetas)
        y = wall_ranges * sin(thetas)

        return self.least_squares(x, y)
        
    def drive_cb(self, msg):
        self.car_velocity = msg.drive.speed

    def control(self, msg):

        if self.car_velocity:        
            # create a wall line based on lidar data
            m, y0 = self.get_wall(msg)
            x = np.linspace(-2., 2., num=40)
            y = m * x + y0

            # get the reference / closest point to the line
            r = abs(y0) / np.sqrt(m**2 + 1)

            # tune this
            closest_dist = self.car_velocity * 0.5

            if (r < closest_dist):
                # setup control input message
                u = AckermannDriveStamped()
                drive = AckermannDrive()

                u.header.frame_id = "base_link"
                u.header.stamp = self.get_clock().now().to_msg()

                drive.steering_angle = 0.
                drive.steering_angle_velocity = 0.

                drive.speed = 0.
                drive.acceleration = 0.
                drive.jerk = 0.

                u.drive = drive

                # publish control action
                self.control_pub.publish(u)

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
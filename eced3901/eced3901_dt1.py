# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

# http://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from collections import deque

class LaserDataInterface:
    def __init__(self, storage_depth=4, logger=None):
        self.laser_data = deque()
        self.depth = storage_depth
        self.logger = logger

    def get_logger(self):
        return self.logger if self.logger else None

    def get_range_array(self, center_deg, left_offset_deg=-5, right_offset_deg=+5, invalid_data=None):
        """
        Returns an array of lidar data based on specified angles.

        WARNING: THIS FUNCTION IS NOT TESTED AND MAY NOT WORK AS INTENDED. NOT SUGGESTED TO USE AS-IS.
        """
        if not (-180 <= center_deg <= 180):
            raise ValueError(f"Invalid range: {center_deg}")

        if not (-180 <= left_offset_deg <= 0):
            raise ValueError(f"Invalid left offset: {left_offset_deg}")

        if not (0 <= right_offset_deg <= 180):
            raise ValueError(f"Invalid right offset: {right_offset_deg}")

        if not self.laser_data:
            return None

        angleset_deg = (self.anglestep * 180.0) / 3.14159
        offset_i = round(center_deg / angleset_deg)
        offset_pos = round(right_offset_deg / angleset_deg)
        offset_neg = round(left_offset_deg / angleset_deg)

        start = offset_i + offset_neg
        end = offset_i + offset_pos

        if start > 180:
            start -= 360

        if end > 180:
            end -= 360

        if start < -180:
            start += 360

        if end < -180:
            end += 360

        tempdata = self.laser_data[0]
        data = [None if x < self.minrange or x > self.maxrange else x for x in tempdata]

        zero_offset = int(len(data) / 2)
        new_slice = data[zero_offset:] + data[:zero_offset]

        start_index = round(start / angleset_deg) + zero_offset
        end_index = round(end / angleset_deg) + zero_offset

        if end_index >= len(data):
            end_index = len(data) - 1

        if start_index < 0:
            start_index = 0

        if end > start:
            lidar_data = new_slice[start_index:end_index][::-1]
            self.get_logger().info(f'Scan Data: "{str(lidar_data)}"')
            return lidar_data
        else:
            raise NotImplementedError("Function cannot deal with splitting array (e.g., 180/-180)")

    def process_laser_msg(self, msg):
        self.anglestep = msg.angle_increment
        self.minrange = msg.range_min
        self.maxrange = msg.range_max
        ranges = msg.ranges[0:]

        self.laser_data.append(ranges)

        if len(self.laser_data) > self.depth:
            self.laser_data.popleft()


def none_is_infinite(value):
    return float("inf") if value is None else value


def min_ignore_none(data):
    return min(data, key=none_is_infinite)


class NavigateSquare(Node):
    """Simple class designed to navigate a square"""

    def __init__(self):
        super().__init__('navigate_square')

        self.x_vel = -0.2
        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = 1.0

        self.laser_range = None

        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.pub_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.range_callback,
            qos_policy
        )

        self.ldi = LaserDataInterface(logger=self.get_logger())

        self.timer = self.create_timer(0.1, self.timer_callback)

    def control_example_odom(self):
        """Control example using odometry"""
        msg = Twist()

        self.d_now = ((self.x_now - self.x_init) ** 2 + (self.y_now - self.y_init) ** 2) ** 0.5

        if self.d_now < self.d_aim:
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub_vel.publish(msg)
        self.get_logger().info(f"Sent: {str(msg)}")

    def control_example_lidar(self):
        """Control example using LIDAR"""
        msg = Twist()

        laser_ranges = self.ldi.get_range_array(0.0)
        if laser_ranges is None:
            self.get_logger().warning("Invalid range data, skipping...")
            return

        laser_ranges_min = min_ignore_none(laser_ranges)

        if laser_ranges_min and laser_ranges_min > 0.5:
            msg.linear.x = self.x_vel
        elif laser_ranges_min and laser_ranges_min < 0.5:
            msg.angular.z = 1.0

        self.pub_vel.publish(msg)
        self.get_logger().info(f"Sent: {str(msg)}")

    def timer_callback(self):
        """Timer callback for 10Hz control loop"""
        self.control_example_odom()

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y

    def range_callback(self, msg):
        """Callback on 'range' subscription"""
        self.ldi.process_laser_msg(msg)

    def destroy_node(self):
        msg = Twist()
        self.pub_vel.publish(msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    navigate_square = NavigateSquare()

    rclpy.spin(navigate_square)

    navigate_square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

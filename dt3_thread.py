    # Copyright 2016 Open Source Robotics Foundation, Inc.
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
import subprocess
import math
import time
import threading
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


# http://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from collections import deque

class LaserDataInterface(object):

    def __init__(self, storage_depth=4, logger=None):
        self.laser_data = deque() #store lidar data as que
        self.depth = storage_depth #maximum stored data depth
        self.logger = logger

    def get_logger(self):
        if self.logger:
            return self.logger

    def get_range_array(self, center_deg, left_offset_deg=-5, right_offset_deg=+5, invalid_data=None):
        """
        This function should return an array of lidar data. Set center_deg (0 = front), the array starts at left_offset_deg
        and ends at right_offset_deg. You can choose to select specific points within the offset for example.

        WARNING: THIS FUNCTION IS NOT TESTED AND MAY NOT WORK AS INTENDED. NOT SUGGESTED TO USE AS-IS.
        """

        if center_deg < -180 or center_deg > 180:
            raise ValueError("Invalid range: %d"%center_deg)

        if left_offset_deg < -180 or left_offset_deg > 0:
            raise ValueError("Invalid left offset: %d"%left_offset_deg)

        if right_offset_deg > 180 or right_offset_deg < 0:
            raise ValueError("Invalid right offset: %d"%right_offset_deg)

        # No data yet
        if len(self.laser_data) == 0:
            return None

        angleset_deg = (self.anglestep * 180.0) / 3.14159
        offset_i = round(center_deg / angleset_deg)
        offset_pos = round(right_offset_deg / angleset_deg)
        offset_neg = round(left_offset_deg / angleset_deg)

        #calculate start and end indices
        start = offset_i + offset_neg
        end = offset_i + offset_pos
        
        # Remap to -180 to 0 range
        if start > 180:
            start = start - 360

        if end > 180:
            end = end - 360

        # Remap to 180 to 0 range
        if start < -180:
            start = start + 360
        
        if end < -180:
            end = end + 360

        tempdata = self.laser_data[0]
        data = list(map(lambda x: None if x < self.minrange or x > self.maxrange else x, tempdata))
        
        #self.get_logger().info('Scan Data: "%s"' % str(data))
                
        # Index 0 maps to 0, Index len/2 maps to 180, Index len maps to -180
        # Remap this so we have array as expected from -180 to 180
        zero_offset = int(len(data) / 2)
        new_slice = data[zero_offset:] + data[:(zero_offset-1)]

        # Uncomment this to see scan data in console (chatty)
        self.get_logger().info('Scan Data: "%d"' % len(new_slice))
        
        # Normal - we just take a slice
        start_index = round(start / angleset_deg)
        end_index = round(end / angleset_deg)

        start_index += zero_offset
        end_index += zero_offset

        if end_index > len(data):
            end_index = len(data)-1
            raise ValueError("Oops?!")
        
        if start_index < 0:
            start_index = 0
            raise ValueError("Oops?!")

        if end > start:
            lidar_data = new_slice[start_index:end_index]
            lidar_data = lidar_data[::-1]
            #self.get_logger().info('Scan Data: "%d:%d"' % (start_index, end_index))
            self.get_logger().info('Scan Data: "%s"' % str(lidar_data))
            return lidar_data
        else:
            raise NotImplementedError("Function cannot deal with splitting array (typically 180 / -180)")


    def process_laser_msg(self, msg):
        self.anglestep = msg.angle_increment
        self.minrange = msg.range_min
        self.maxrange = msg.range_max
        ranges = msg.ranges[0:]

        # Index 0 is front of robot & counts clockwise from there
        num_points = len(ranges)

        self.laser_data.append(msg.ranges)

        # Get rid of old data
        if len(self.laser_data) > self.depth:
            self.laser_data.popleft()


def noneIsInfinite(value):
    if value is None:
        return float("inf")
    else:
        return value

def min_ignore_None(data):
    return min(data, key=noneIsInfinite)

# Rename variable to avoid conflict with method name
class NavigateSquare(Node):
    """Simple class designed to navigate a square"""

    def __init__(self):
        super().__init__('navigate_square')
        self.prev_yaw = 0.0
        self.x_vel = 0.2
        self.turn_vel = 0.7  # Angular velocity (rad/s)

        # Define route lengths and turns
        self.A = 1.48
        self.BE = 1.1
        self.B = 0.73
        self.C = 0.508
        self.D = 1
        self.E = 0.483
        self.F = 0.7112
        self.G = 0.33
        self.H = 1.11
        self.I = 0.25

        self.J = 1.04

        self.turn90 = 90  # Turn 90 degrees after each side
        self.avoidance_angle = 10
        self.distance_travelled_value = 0.0  # Renamed to avoid conflict
        self.current_step = 0
        self.turning = False
        self.angle_diff = 0.0

        # Robot position tracking
        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = 1.0
        self.roll = None
        self.pitch = None
        self.yaw = None

        # Obstacle detection
        self.laser_range = None
        self.avoiding_obstacle = False
        self.use_advanced_wall = False
        self.use_way_points = True
        self.use_RFID_Mag = False
        self.use_Challange = False

        self.obstacle_distance_threshold = 0.125

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Publish to velocity
        self.pub_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        # Subscribe to lidar
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.sub_lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.range_callback,
            qos_policy
        )

        self.ldi = LaserDataInterface(logger=self.get_logger())
        self.timer = self.create_timer(0.1, self.timer_callback)

    def navigate_advanced_wall(self):
        msg = Twist()
        if self.current_step == 0:  # Step 1: Move forward (A)
            if self.distance_travelled() < self.A:
                msg.linear.x = -0.2
            else:
                self.prepare_turn(90)
                self.current_step += 1

        elif self.current_step == 1:  # Step 2: Turn 90 degrees
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = self.turn_vel  # Rotate counterclockwise at speed

        elif self.current_step == 2:  # Step 3: Move forward (B)
            if self.distance_travelled() < self.B:
                msg.linear.x = -0.2
                self.run_servo()
            else:
                self.prepare_turn(-180)
                self.current_step += 1

        elif self.current_step == 3:  # Step 4: Turn -180 degrees
            if self.turn_complete180():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = self.turn_vel  # Rotate counterclockwise at speed

        elif self.current_step == 4:  # Step 5: Move forward (-B)
            if self.distance_travelled() < self.B:
                msg.linear.x = -0.2
            else:
                self.prepare_turn(-80)
                self.current_step += 1

        elif self.current_step == 5:  # Step 6: Turn -90 degrees
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = -self.turn_vel  # Rotate clockwise at speed

        elif self.current_step == 6:  # Step 7: Move forward (-A)
            if self.distance_travelled() < self.A:
                msg.linear.x = -0.2
            else:
                msg.linear.x = 0.0  # Stop when the route is completed
                self.stop_robot()
                self.pub_vel.publish(msg)
                self.get_logger().info("Route 1 Completed")

        self.pub_vel.publish(msg)

    def navigate_way_points(self):
        msg = Twist()

        if self.current_step == 0:  # Step 1: Move forward (A)
            if self.distance_travelled() < self.A:
                msg.linear.x = -0.2
            else:
                self.prepare_turn(95)
                self.current_step += 1
                self.get_logger().info("Route 0 Completed")

        elif self.current_step == 1:  # Step 2: Turn 90 degrees
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
                self.get_logger().info("Route 1 Completed")
            else:
                msg.angular.z = self.turn_vel  # Rotate counterclockwise at speed

        elif self.current_step == 2:  # Step 3: Move forward (B)
            if self.distance_travelled() < self.BE:
                msg.linear.x = -0.2
            else:
                #self.pause_robot()
                self.prepare_turn(90)
                self.current_step += 1
                self.get_logger().info("Route 2 Completed")

        elif self.current_step == 3:  # Step 2: Turn 90 degrees
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
                self.get_logger().info("Route 3 Completed")
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 4:  # Step 3: Move forward (B)
            if self.distance_travelled() < self.F:
                msg.linear.x = -0.2
            else:
                self.prepare_negturn(180)
                self.current_step += 1
                self.get_logger().info("Route 4 Completed")

        elif self.current_step == 5:  # Step 3: Move forward (B)
            if self.turn_complete180():
                self.current_step += 1
                self.prepare_turn(180)
                self.get_logger().info("Route 3 Completed")
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = self.turn_vel

        elif self.current_step == 6:  # Step 3: Move forward (B)
            if self.turn_complete180():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 7:
            if self.distance_travelled() < self.G:
                msg.linear.x = -0.2
            else:
                self.prepare_turn(90)
                self.current_step += 1

        elif self.current_step == 8:
            if self.turn_complete90():
                self.pub_vel.publish(msg)
                self.current_step +=  1
                self.get_logger().info("Route 1 Completed")
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = self.turn_vel

        elif self.current_step == 9:
            if self.distance_travelled() < self.I:
                msg.linear.x = -0.2
            else:
                self.prepare_negturn(180)
                self.current_step += 1
            
        self.pub_vel.publish(msg)
    

    def navigate_RFID_Mag(self):
        self.get_logger().info("Executing navigate_RFID_Mag")
        msg = Twist()

        if self.current_step == 0:
            if self.distance_travelled() < self.I:
                msg.linear.x = -0.2
            else:
                self.stop_robot()
                self.get_logger().info("Route 1 Completed")

        self.pub_vel.publish(msg)
    
    def navigate_Challange(self):
        msg = Twist()

        if self.current_step == 0:
            if self.distance_travelled() < self.H:
                msg.linear.x = -0.2
            else:
                self.prepare_turn(80)
                self.current_step += 1

        elif self.current_step == 1:
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 2:
            if self.distance_travelled() < self.I:
                msg.linear.x = -0.2
            else:
                self.prepare_negturn(90)
                self.current_step += 1

        elif self.current_step == 3:
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else:
                msg.angular.z = self.turn_vel
        
        elif self.current_step == 4:
            if self.distance_travelled() < (self.A-self.H):
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 5:
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 6:
            if self.distance_travelled() < self.B:
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now
            else: 
                msg.angular.z = self.turn_vel

        elif self.current_step == 7:
            if self.distance_travelled() < self.G:
                msg.linear.x = -0.2
            else:
                self.prepare_negturn(60)
                self.current_step += 1

        elif self.current_step == 8:
            if self.distance_travelled() < self.I:
                msg.linear.x = -0.2
            else:
                self.prepare_negturn(90)
                self.current_step += 1 
        
    
    def pause_robot(self, duration=5.0):
        """Pause the robot in its position for the given duration (seconds)."""
        self.get_logger().info(f"Pausing robot for {duration} seconds")

        # Stop the robot
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)

        # Wait for the specified duration without blocking ROS2
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)  # Keep spinning to handle callbacks

        self.get_logger().info("Resuming movement")

    def prepare_turn(self, angle):
        """Prepare for turn"""
        self.turning = True
        self.prev_yaw = self.yaw
        self.turn_angle = angle
        self.get_logger().info(f"Preparing to turn {angle} degrees")

    def prepare_negturn(self, angle):
        """Prepare for turn"""
        self.turning = True
        self.prev_yaw = self.yaw
        self.turn_angle = -angle
        self.get_logger().info(f"Preparing to turn {-angle} degrees")

    def turn_complete90(self):
        """Check if 90 degree turn is complete"""
        if self.yaw is None:
            self.get_logger().error("Yaw value is None, cannot complete turn.")
            return False  # Prevent further execution if yaw is None

        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw + 170) % 360 - 180

        self.get_logger().info(f"Yaw: {self.yaw}, Target Yaw: {target_yaw}, Angle Diff: {angle_diff}")

        if abs(angle_diff) < 2:  # Allow a tolerance of 5 degrees for turn completion
            self.turning = False
            return True
        return False

    def turn_complete180(self):
        """Check if 180 degree turn is complete"""
        if self.yaw is None:
            self.get_logger().error("Yaw value is None, cannot complete turn.")
            return False

        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw + 180) % 360 - 180

        self.get_logger().info(f"Yaw: {self.yaw}, Target Yaw: {target_yaw}, Angle Diff: {angle_diff}")

        if abs(angle_diff) < 2:  # Tolerance for 180 degree turn
            self.turning = False
            return True
        return False
    
    def run_servo(self):
        thread = threading.Thread(target=self.run_servo)
        thread.daemon = True  # Ends thread when main program exits
        thread.start()

    def turn_complete_90(self):
        """Check if -90 degree turn is complete"""
        if self.yaw is None:
            self.get_logger().error("Yaw value is None, cannot complete turn.")
            return False  # Prevent further execution if yaw is None

        #    Subtract 90 degrees for a -90 turn
        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw + 70) % 360 - 180

        self.get_logger().info(f"Yaw: {self.yaw}, Target Yaw: {target_yaw}, Angle Diff: {angle_diff}")

        if abs(angle_diff) < 2:  # Allow a tolerance of 2 degrees for turn completion
            self.turning = False
            return True
        return False

    def stop_robot(self):
        """Stop the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)
        self.get_logger().info("STOP command")

    def distance_travelled(self):
        """Calculate the distance traveled"""
        return math.sqrt((self.x_now - self.x_init)**2 + (self.y_now - self.y_init)**2)

    def odom_callback(self, msg):
        """Update odometry readings"""
        quaternion = msg.pose.pose.orientation
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        euler_angles = rotation.as_euler('xyz', degrees=True)
        self.yaw = euler_angles[2] if euler_angles[2] >= 0 else euler_angles[2] + 360
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y

        self.get_logger().info(f"Updated Yaw: {self.yaw}")

    def range_callback(self, msg):
        """Callback on 'range' subscription"""
        self.ldi.process_laser_msg(msg)

    def timer_callback(self):
        """Main control loop"""
        if self.use_advanced_wall:
            self.navigate_advanced_wall()
        elif self.use_way_points:
            self.navigate_way_points()
        elif self.use_RFID_Mag:
            self.navigate_RFID_Mag()
        elif self.use_Challange:
            self.navigate_Challange()

def main(args=None):
    rclpy.init(args=args)
    navigate_square = NavigateSquare()
    rclpy.spin(navigate_square)
    navigate_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

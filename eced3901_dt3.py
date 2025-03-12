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
import math
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

class NavigateSquare(Node):
    """Simple class designed to navigate a square"""

    def __init__(self):
        #This calls the initilization function of the base Node class
        super().__init__('navigate_square')

        # We can either create variables here by declaring them
        # as part of the 'self' class, or we could declare them
        # earlier as part of the class itself. I find it easier
        # to declare them here

        # Ensure these are obviously floating point numbers and not
        # integers (python is loosely typed)

        # WARNING: Check for updates, note this is set and will run backwards
        #          on the physical model but correctly in simulation.
        self.x_vel = 0.2 # x velocity
        self.turn_vel = 0.7  # Angular velocity (rad/s)
        
        self.A = 1
        self.B = 8
        self.C =7
        self.D =5
        self.E =6
        self.F =4
        self.G =3
        self.H =2
        self.I =1

        self.turn90 = 90  # Turn 90 degrees after each side
        self.avoidance_angle = 10
        self.distance_travelled = 0.0 #disatance traveled
        self.current_step = 0
        self.turning = False # turn flag
        self.angle_diff = 0.0

        #robot position tracking
        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0. 
        self.d_now = 0.0
        self.d_aim = 1.0
        self.roll = None
        self.pitch = None
        self.yaw = None

        self.laser_range = None
        self.avoiding_obstacle = False
        self.use_route_1 = True

        self.obstacle_distance_threshold = 0.125 #meters

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry, #Matches the import at the top (nav_msgs.msg import Odometry)
            'odom', #odom topic
            self.odom_callback, #Call this function
            10) #Queue size 10

        # Publish to velocity
        self.pub_vel = self.create_publisher(
            Twist, #Expects the twist message format
            'cmd_vel',
            10) #Queue size 10

        # Subscribe to lidar - this requires a policy, otherwise you won't see any data
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

    def navigate_route_1(self):
        msg = Twist()

        if self.avoiding_obstacle:
            self.avaoid_obstacle()
            return

        if self.current_step == 0: #Step 1. A
            if self.distance_traveled() < self.A:
                msg.linear.x = 0.2
            else:self.prepare_turn()

        elif self.current_step == 1: #Step 2. Turn 90
            if self.turn_complete90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now  # Reset position tracking
            else:
                msg.angular.z = 0.5  # Rotate counterclockwise at speed 0.5 rad/s

        elif self.current_step == 2: #Step 3. B
            if self.distance_traveled() < self.B:
                msg.linear.x = 0.2
            else:
                self.prepare_turn()

        elif self.current_step == 3: #Step 4. Turn -180
            if self.turn_complete_180():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now  # Reset position tracking
            else:
                msg.angular.z = 0.5  # Rotate counterclockwise at speed 0.5 rad/s

        elif self.current_step == 4: #Step 5. -B
            if self.distance_traveled() < self.B:
                msg.linear.x = 0.2
            else:self.prepare_turn()

        elif self.current_step == 5: #Step 6. Turn -90
            if self.turn_complete_90():
                self.current_step += 1
                self.x_init, self.y_init = self.x_now, self.y_now  # Reset position tracking
            else:
                msg.angular.z = 0.5  # Rotate counterclockwise at speed 0.5 rad/s
        
        elif self.current_step == 6: #Step 7. -A
            if self.distance_traveled() < self.A:
                msg.linear.x = 0.2
            else:self.linear.x = 0.0

        self.pub_vel.publish(msg)

    def prepare_turn(self): #Prepare for turn
        self.turning = True
        self.prev_yaw = self.yaw
        
    def turn_complete90(self): #Complete 90 turn
        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw + 90) % 360 - 180
        if abs(angle_diff) < 2:
            self.turning = False
            return True
        return False

    def turn_complete_90(self): #Complete 90 turn
        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw - 90) % 360 - 180
        if abs(angle_diff) < 2:
            self.turning = False
            return True
        return False

    def turn_complete180(self):  #Complete 90 turn
        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw + 180) % 360 - 180
        if abs(angle_diff) < 2:
            self.turning = False
            return True
        return False
 
    def turn_complete_180(self):  #Complete 90 turn
        target_yaw = (self.prev_yaw + self.turn_angle) % 360
        angle_diff = (target_yaw - self.yaw - 180) % 360 - 180
        if abs(angle_diff) < 2:
            self.turning = False
            return True
        return False
    
        # Calculate distance traveled
    def distance_travelled (self):
        return math.sqrt((self.x_now - self.x_init)**2 + (self.y_now - self.y_init)**2)


    def control_example_lidar(self):
        print("Hello")
        """ Control example using LIDAR"""
        msg = Twist()

        # Fetch LIDAR data: range in front of the robot
        laser_ranges = self.ldi.get_range_array(0.0, left_offset_deg=-90, right_offset_deg=10)
        if laser_ranges is None or len(laser_ranges) ==0:
            self.get_logger().warning("Invalid range data, skipping control loop...")
            return

        # Minimum distance to obstacle in the forward direction
        laser_ranges_min = min_ignore_None(laser_ranges)

        # State machine logic for navigating around the box
        if self.state == "before_box":
            if laser_ranges_min and laser_ranges_min > 0.5:
                # Continue moving forward
                msg.linear.x = self.x_vel
                msg.angular.z = 0.0
            else:
                # Edge of box detected
                self.state = "along_box"

        elif self.state == "along_box":
            if laser_ranges_min and laser_ranges_min > 0.3:
                # Maintain close distance while moving along the box edge
                msg.linear.x = self.x_vel
                msg.angular.z = 0.1  # Slight adjustment to follow the edge
            else:
                # Detected corner or obstacle, prepare to turn
                self.state = "corner"
                self.turn_start_time = self.get_clock().now()

        elif self.state == "corner":
            if (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9 < self.turn_duration:
                # Perform the 90° turn
                msg.linear.x = 0.0
                msg.angular.z = self.turn_vel
            else:
                # Completed the turn
                self.loops += 1
                if self.loops >= 4:
                    # Stop after completing the desired number of loops
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.state = "stopped"
                else:
                    self.state = "before_box"

        elif self.state == "stopped":
            # Robot is stopped
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish the velocity command
        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))

    def timer_callback(self):
        """Main control loop"""
        if self.obstacle_detected():
            self.avoiding_obstacle = True
            self.prepare_obstacle_avoidance()
            return

        if self.use_route_1:
            self.navigate_route_1()
        else:
            self.navigate_route_2()

     def obstacle_detected(self):
        """Check if an obstacle is within the threshold distance"""
        laser_ranges = self.ldi.get_range_array(0.0, left_offset_deg=-30, right_offset_deg=30)
        if laser_ranges:
            return min_ignore_None(laser_ranges) < self.obstacle_distance_threshold
        return False

    def prepare_obstacle_avoidance(self):
        """Determine which direction to turn and start avoidance"""
        left_distance = self.ldi.get_range(-30)  # Left side
        right_distance = self.ldi.get_range(30)  # Right side

        if right_distance is None:
            right_distance = 0  # Treat None as 0 distance
        if left_distance is None:
            left_distance = 0

        if right_distance > left_distance:
            self.avoidance_turn_direction = -1  # Turn right
        else:
            self.avoidance_turn_direction = 1  # Turn left

        self.prev_yaw = self.yaw
        self.avoiding_obstacle = True

    def avoid_obstacle(self):
        """Rotate slightly away from the obstacle and resume motion"""
        msg = Twist()
        target_yaw = (self.prev_yaw + self.avoidance_turn_direction * self.avoidance_angle) % 360
        angle_diff = (target_yaw - self.yaw + 180) % 360 - 180

        if abs(angle_diff) < 2:
            self.avoiding_obstacle = False  # Resume normal navigation
        else:
            msg.angular.z = 0.3 * self.avoidance_turn_direction  # Rotate away

        self.pub_vel.publish(msg)

    def odom_callback(self, msg):
        """Update odometry readings"""
        quaternion = msg.pose.pose.orientation
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        euler_angles = rotation.as_euler('xyz', degrees=True)
        self.yaw = euler_angles[2] if euler_angles[2] >= 0 else euler_angles[2] + 360
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y
        
        #self.get_logger().info('Msg Data: "%s"' % msg)        
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y


    def range_callback(self, msg):
        """Callback on 'range' subscription"""
        #self.get_logger().info('Scan Data: "%s"' % msg)
        #self.get_logger().info('Scan Data: "%s"' % msg.angle_increment)

        anglestep = msg.angle_increment
        self.minrange = msg.range_min
        maxrange = msg.range_max
        ranges = msg.ranges[1:]

        self.laser_range = max(ranges[0:5])
       # self.get_logger().info('Scan Data: "%d"' % len(ranges))

        self.ldi.process_laser_msg(msg)

    def destroy_node(self):
        msg = Twist()
        self.pub_vel.publish(msg)
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)

    navigate_square = NavigateSquare()

    rclpy.spin(navigate_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

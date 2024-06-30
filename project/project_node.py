# used to interact also with services
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist, Pose
import time
import math
from asebaros_msgs.msg import Event

import numpy as np
from sensor_msgs.msg import Range


# to handle requests in callbacks functions (requests to service SetPen and callback in timer) using multiple threads
from rclpy.executors import MultiThreadedExecutor


class project(Node):
    def __init__(self):
        super().__init__('project')

        self.angular = 0.0
        self.linear = 0.0
        self.sensors = {}
        # there are 9 states:
        # STATE 0 -> searches for the line
        # STATE 1 -> follow the line
        # STATE 2 -> hit a wall, direct thymio orthogonal to the wall (in front)
        # STATE 3 -> rotate thymio parallel to wall
        # STATE 4 -> move along the first edge of the wall
        # STATE 5 -> rotate with respect to first corner of wall
        # STATE 6 -> move along  the second edge of the wall
        # STATE 7 -> rotate with respect to second corner of the wall and move along third edge of the wall
        #             go to state 8 if the line is detected again
        # STATE 8 -> 'catch' the line to continue the path

        # states 2,3,4,5,6,7 are in move_callback. 
        # states 0,1,7,8 are in ground_callback

        #For scene 1,2,3,4 we have only state 0,1. For scene 5 we have all states.
        
        self.state = 0
        # threshold such that thymio doesn't hit walls
        self.threshold = 0.03

        # timer used to move ahead the thymio avoiding it hits the wall when rotating with respect to corners
        self.start_timer = 0
        self.started_timer = False

        # different distance and threshold used to align the thymio orthogonal to the wall
        self.t = 0.0008
        self.md = 0.05

        self.direction_rotation = 0


        self.vel_publisher = self.create_publisher(Twist, '/thymio/cmd_vel', 10)
        self.create_subscription(Event, 'thymio/aseba/events/ground', self.ground_callback,10)

        # used for scene 5
        self.create_subscription(Range, '/thymio/proximity/center_left', self.prox_callback,10)
        self.create_subscription(Range, '/thymio/proximity/center', self.prox_callback,10)
        self.create_subscription(Range, '/thymio/proximity/right', self.prox_callback,10)
        self.create_subscription(Range, '/thymio/proximity/center_right', self.prox_callback,10)


    def start_moving(self):
        # Create and immediately start a timer that call periodically  move_callback()
        self.timer = self.create_timer(0.1, self.move_callback)




    def move_callback(self):
        msg = Twist()
        msg.angular.z = self.angular
        msg.linear.x = self.linear
        self.vel_publisher.publish(msg)

        # orientate thymio orthogonal in front of the wall
        if self.state == 2:
            diff_center = self.sensors['center_left'] - self.sensors['center_right']
            # case in which plane is already orthogonal to thymio orientation -> don't care about center left and right sensors
            if (diff_center == -2):
                self.angular = 0.0
                self.state = 3
            #if at least one sensor is activated
            if self.sensors['center_left'] != -1 or self.sensors['center_right'] != -1:
                # if same distance left and right from object, thymio is orthogonal to wall
                if np.isclose(0, diff_center, atol = self.t):
                    self.angular = 0.0
                    self.state = 3
                else:
                    # adjust orientation if thymio not orthogonal
                    if not np.isclose(0, diff_center, atol= self.t):
                        if self.sensors['center_right'] == -1:
                            self.angular = 0.1
                        elif self.sensors['center_left'] == -1:
                            self.angular = -0.1
                        elif diff_center < 0:
                            self.angular = 0.1
                        else:
                            self.angular = -0.1

        # rotate thymio parallel to the wall
        if self.state == 3:
            if self.started_timer == False:
                self.started_timer = True
                self.start_timer = time.perf_counter()
            if time.perf_counter() - self.start_timer <= 4.8:
                self.angular = 0.4
            else:
                self.angular = 0.0
                self.linear = 0.1
                self.started_timer = False
                self.state = 4

        # move thymio along first edge of the wall. When right sensor doesn't detect the wall anymore,
        # continue to move it to avoid a collision when rotate to move along the next edge
        if self.state == 4:
            if self.sensors['right'] == -1:
                if self.started_timer == False:
                    self.start_timer = time.perf_counter()
                    self.started_timer = True
                if time.perf_counter() - self.start_timer >= 1.7:
                    self.started_timer = False
                    self.linear = 0.0
                    self.angular = 0.0
                    self.state = 5
        # rotate thymio with respect to first corner until right sensor detect the wall
        if self.state == 5:
            if (self.sensors['right'] == -1):
                self.angular = -0.2
                self.linear = 0.0
            else:
                self.angular = 0.0
                self.linear = 0.0
                self.state = 6

        # move thymio along second edge. When right sensor doesn't detect the wall anymore,
        # continue to move it to avoid a collision when rotate to move along the next edge
        if self.state == 6:
            if self.sensors['right'] != -1:
                self.linear = 0.1
                if self.sensors['right'] <= self.threshold - 0.015:
                    self.angular = 0.2
                elif self.sensors['right'] >= self.threshold + 0.01:
                    self.angular = -0.4
                else:
                    self.angular = 0.0
            else:
                self.angular = 0.0
                if self.started_timer == False:
                    self.started_timer = True
                    self.start_timer = time.perf_counter()
                if time.perf_counter() - self.start_timer >= 2:
                    self.started_timer = False
                    self.state = 7
                    self.linear = 0.0
                    self.angular = 0.0

        # rotate thymio with respect to second corner and move along third edge
        if self.state == 7:
            if self.sensors['right'] == -1:
                self.angular = -0.2
                self.linear = 0.0
            else:
                if self.sensors['right'] < self.threshold:
                    self.angular = 0.2
                    self.linear = 0.1
                elif self.sensors['right'] >= self.threshold + 0.03:
                    self.angular = -0.2
                    self.linear = 0.1
                else:
                    self.angular = 0.0
                    self.linear = 0.1




    def ground_callback(self, msg):
        # seach for lines at the beginning -> search for black colour
        if self.state == 0:
            self.linear = 0.2
            if msg.data[0] != 1023:
                self.state = 1
                self.angular = 4.0
            if msg.data[1] != 1023:
                self.state = 1
                self.angular = -4.0

        # line found and follow it
        if self.state == 1:
            self.linear = 0.2
            # found grey -> end of line
            if ((msg.data[0] >= 1010 and msg.data[0] <= 1013) or (msg.data[1] >= 1010 and msg.data[1] <= 1013)):
                self.linear = 0.0
                self.angular = 0.0
            if msg.data[0] != 1023 and msg.data[1] != 1023:
                self.angular = 0.0
            elif msg.data[0] != 1023:
                self.direction_rotation = 'clockwise'
                self.angular = 4.0
            elif msg.data[1] != 1023:
                self.direction_rotation = 'counterclockwise'
                self.angular = -4.0
            else:
                self.linear = 0.0
                if self.direction_rotation == 'clockwise':
                    self.angular = 0.4
                else:
                    self.angular = -0.4
            

        # Thymio detect the line when moving along third edge
        if self.state == 7:
            if msg.data[0] != 1023 and msg.data[1] != 1023:
                self.state = 8

        # rotate thymio such that left sensor doesn't detect the line anymore (used as orientation parameter) and return to state 0
        if self.state == 8:
            self.linear = 0.03
            self.angular = 0.8

            if msg.data[0] == 1023:
                self.angular = -0.6
                self.state = 0
            


    def prox_callback(self,msg):
        #save proximity sensor updated values and names
        topic_name = msg.header.frame_id.split('_')
        if topic_name[2] != 'link':
            self.sensors[topic_name[1] + '_' + topic_name[2]] = msg.range
        else:
            self.sensors[topic_name[1]] = msg.range

        # Sensor detect a wall
        if self.state == 1 and msg.range >= 0 and msg.range <= self.md:
            self.state = 2
            self.angular = 0.0  
            self.linear = 0.0 
      
  

def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of your node class
    node = project()
    done = node.start_moving()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Shut down with CTRL-C')
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        rclpy.shutdown()


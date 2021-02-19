import numpy as np

import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from graph import Graph


class TennisCollectorNavigationMulti(Node):

    def __init__(self):
        
        super().__init__('navigation_multi')
        
        self.is_ball_collected = False
        self.is_ball_on_court = False
        self.pose_ball_x = 0.0
        self.pose_ball_y = 0.0
        self.pose_rob_x = 0.0
        self.pose_rob_y = 0.0
        self.list = []

        self.state = 0
        self.y_wp = 0.0
        self.y_wp = 0.0
        self.move = False

        qos = QoSProfile(depth=10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.navigation_multi)

        # PUBLISHER INITIALIZATION
        #-------------------------------------------------------------------
        self.wp_pub = self.create_publisher(Pose, 'balls_pose', qos)

        self.move_pub = self.create_publisher(Bool, 'move', qos)
        #-------------------------------------------------------------------


        # PUBLISHER SUBSCIPTER INITIALIZATION
        #-------------------------------------------------------------------
        self.ball_sub = self.create_subscription(
            Pose,
            'ball',
            self.ball_callback,
            qos)
        
        self.rob_sub = self.create_subscription(
            Pose,
            'robot',
            self.rob_callback,
            qos)

        self.is_ball_collected_sub = self.create_subscription(
            Bool,
            'ibc',
            self.ibc_callback,
            qos)

        self.is_ball_on_court_sub = self.create_subscription(
            Bool,
            'iboc',
            self.iboc_callback,
            qos)
        #-------------------------------------------------------------------


    def ball_callback(self, msg):
        
        self.pose_ball_x = msg.pose.pose.position.x
        self.pose_ball_y = msg.pose.pose.position.y

    def rob_callback(self, msg):

        self.pose_rob_x = msg.pose.pose.position.x
        self.pose_rob_y = msg.pose.pose.position.y

    def ibc_callback(self, msg):

        self.is_ball_collected = msg.bool.data

    def iboc_callback(self, msg):

        self.is_ball_on_court = msg.bool.data


    def navigation_multi(self):
        pose = Pose()
        bool = Bool()

        # STATE 1: GO TO THE OTHER SIDE OF THE COURT BYPASSING THE NET
        #-------------------------------------------------------------------
        if self.state == 1:

            if self.is_ball_on_court == True:
                if np.sign(self.pose_rob.y) != np.sign(self.pose_ball_y):
                    self.x_wp = 6.5*np.sign(self.poserob.x)
                    self.y_wp = 0.0
                else:
                    self.state = 2
            else :
                self.state = 3
        #-------------------------------------------------------------------


        # STATE 2: GO THE NEXT BALL TO COLLECT IT
        #-------------------------------------------------------------------
        if self.state == 2:

            if self.is_ball_on_court == True:
                if np.sign(self.pose_rob.y) != np.sign(self.pose_ball_y):
                    state = 1
                else:
                    self.x_wp = self.pose_ball_x
                    self.y_wp = self.pose_ball_y
            else:
                self.state = 3
        #-------------------------------------------------------------------


        # STATE 3: BRING THE BALL INTO THE COLLECTING AREAS
        #-------------------------------------------------------------------
        if self.state == 3:

            if self.is_ball_collected == True:
                self.x_wp = 7.0*np.sign(self.pose_rob.y)
                self.y_wp = 14.0*np.sign(self.pose_rob.y)
            else:
                self.state = 2
        #-------------------------------------------------------------------


        # STATE 0: IDLE
        #-------------------------------------------------------------------
        else: # state == 0

            if self.is_ball_on_court == True:
                self.state = 2
                self.move == True
            else:
                if 6.0 < abs(self.pose_rob.x) < 7.0 and abs(self.pose_rob.y) < .5:
                    # stop robot
                    self.move = False
                else:
                    self.state = 1
                    self.move = True
        #-------------------------------------------------------------------


        # PUBLISHING WAYPONT (POSE) AND MOVING COMMAND (BOOL)
        #-------------------------------------------------------------------
        pose.pose.position.x = self.x_wp
        pose.pose.position.y = self.y_wp
        self.wp_pub.publish(pose)
        self.get_logger().info("Publishing: {}, {}, {}".format(pose.x, pose.y))

        bool.data = self.move
        self.move_pub.publish(bool)
        #-------------------------------------------------------------------

    def dijkstra(self):
        
        n = len(self.list)

        self.list.append(self.ball_sub)
        g = Graph(n)

        g.graph = []
        for j in range (n):
            line = []
            for i in range(n):
                dist = np.sqrt((self.list[i].pose.x-self.list[j].pose.x)**2+(self.list[i].pose.y-self.list[j].pose.y)**2)
                line.append(dist)
            g.graph.append(line)

        """
        g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
                [4, 0, 8, 0, 0, 0, 0, 11, 0],
                [0, 8, 0, 7, 0, 4, 0, 0, 2],
                [0, 0, 7, 0, 9, 14, 0, 0, 0],
                [0, 0, 0, 9, 0, 10, 0, 0, 0],
                [0, 0, 4, 14, 10, 0, 2, 0, 0],
                [0, 0, 0, 0, 0, 2, 0, 1, 6],
                [8, 11, 0, 0, 0, 0, 1, 0, 7],
                [0, 0, 2, 0, 0, 0, 6, 7, 0]]
        """

        distances g.dijkstra(0)

    def main(args=None):
        rclpy.init(args=args)
        navigation = TennisCollectorNavigationMulti()
        rclpy.spin(navigation)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
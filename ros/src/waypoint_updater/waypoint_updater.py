#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import numpy as np
import math
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
POSE_LOG_PERIOD = 100
SMALL_VELOCITY = 1.
MAX_DECEL = 0.5


class WaypointUpdater(object):
    
    def __init__(self):

        self.node_name = 'waypoint_updater'
        rospy.init_node(self.node_name)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /obstacle_waypoint 

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.server_rate = rospy.get_param('/server_rate', 50.) # 50 Hz by default
        rospy.loginfo('server_rate={0}'.format(self.server_rate))

        self.pose = None
        self.waypoints = None
        self.wp_tree = None
        self.pose_counter = 0
        self.tl_stopline_idx = -1

        rospy.loginfo('Starting {}'.format(self.node_name))
        self.loop()

    def loop(self):

        rate = rospy.Rate(self.server_rate)
        while not rospy.is_shutdown():
            
            if self.wp_tree and self.pose:
                #self.loginfo_pose_xy()

                closest_idx = self.find_closest()
                lane = self.generate_lane(closest_idx)                
                self.publish_final_waypoints(lane)

            rate.sleep()

    def loginfo_pose_xy(self):

        if self.pose_counter % POSE_LOG_PERIOD == 0:

            p = self.pose.pose.position
            t = self.pose.header.stamp

            rospy.loginfo('t={}: ({:.3f}, {:.3f})'.format(t, p.x, p.y))

            self.pose_counter = 0

    def find_closest(self):
        
        p = self.pose.pose.position
        xy = np.array([p.x, p.y])

        closest_idx = self.wp_tree.query(xy, 1)[1]

        target = self.waypoints_xy[closest_idx]
        prev = self.waypoints_xy[closest_idx - 1]

        val = np.dot(target - prev, xy - target)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_xy)

        return closest_idx

    def publish_final_waypoints(self, lane):
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self, closest_idx):

        lane = Lane()
        lane.header = self.waypoints.header

        waypoints_normal = self.waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        
        if self.tl_stopline_idx == -1 or self.tl_stopline_idx >= farthest_idx:
            lane.waypoints = waypoints_normal
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints_normal, closest_idx)
        
        return lane

    def decelerate_waypoints(self, waypoints_normal, closest_idx):
        
        waypoints_decel = []

        for i, wp in enumerate(waypoints_normal):

            new_wp = Waypoint()
            new_wp.pose = wp.pose

            stop_idx = max(self.tl_stopline_idx - closest_idx - 2, 0)

            dist = self.distance(waypoints_normal, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < SMALL_VELOCITY:
                vel = 0.
    
            vx = wp.twist.twist.linear.x
            new_wp.twist.twist.linear.x = min(vel, vx)

            waypoints_decel.append(new_wp)

        return waypoints_decel

    def pose_cb(self, msg):

        self.pose = msg
        self.pose_counter += 1

    def waypoints_cb(self, waypoints):
        
        self.waypoints = waypoints

        positions = (wp.pose.pose.position for wp in self.waypoints.waypoints)
        self.waypoints_xy = np.array([[p.x, p.y] for p in positions])
        self.wp_tree = KDTree(self.waypoints_xy)

        rospy.loginfo("Got {} base waypoints".format(len(self.waypoints_xy)))

    def traffic_cb(self, msg):
        self.tl_stopline_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

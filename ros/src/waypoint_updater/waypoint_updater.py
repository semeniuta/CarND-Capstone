#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
UPDATER_RATE = 15 # 50
POSE_LOG_PERIOD = 100


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.wp_tree = None
        self.pose_counter = 0

        rospy.loginfo("Starting WaypointUpdater")
        #rospy.spin()
        self.loop()

    def loop(self):

        rate = rospy.Rate(UPDATER_RATE)
        while not rospy.is_shutdown():
            
            if self.wp_tree and self.pose:
                self.loginfo_pose_xy()

                closest_idx = self.find_closest()
                self.publish_final_waypoints(closest_idx)

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

    def publish_final_waypoints(self, closest_idx):
        
        lane = Lane()
        lane.header = self.waypoints.header
        lane.waypoints = self.waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]

        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement

        self.pose = msg
        self.pose_counter += 1

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        
        self.waypoints = waypoints

        positions = (wp.pose.pose.position for wp in self.waypoints.waypoints)
        self.waypoints_xy = np.array([[p.x, p.y] for p in positions])
        self.wp_tree = KDTree(self.waypoints_xy)

        rospy.loginfo("Got {} base waypoints".format(len(self.waypoints_xy)))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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

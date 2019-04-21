#!/usr/bin/env python

import rospy
from styx_msgs.msg import TrafficLightArray
from sensor_msgs.msg import Image
from styx_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped
import yaml
import numpy as np
from scipy.spatial import KDTree
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    
    def __init__(self):

        rospy.init_node('image_saver')
        
        self.sub_tl = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.sub_im = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.sub_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub_wp = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()

        self.pose = None
        self.waypoints = None
        self.waypoints_xy = None
        self.wp_tree = None
        self.image = None
        self.lights = None

        rospy.loginfo('CWD: {}'.format(os.getcwd()))

        rospy.spin()

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        self.image = msg

        light_wp, state, idx_diff = self.process_traffic_lights()

        if idx_diff < 30 and idx_diff > 5:
            im = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            ts = rospy.get_rostime()
            fname = '{0}{1}_{2}.jpg'.format(ts.secs, ts.nsecs, state)
            cv2.imwrite(fname, im)
            rospy.loginfo('Saved image to {}'.format(fname))

            rospy.sleep(0.25)

            # https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        
        self.waypoints = waypoints

        positions = (wp.pose.pose.position for wp in self.waypoints.waypoints)
        self.waypoints_xy = np.array([[p.x, p.y] for p in positions])
        self.wp_tree = KDTree(self.waypoints_xy)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        # Find the closest visible traffic light (if one exists)

        closest_light = None
        idx_closest_to_light = None

        if self.pose:
            
            car_p = self.pose.pose.position
            idx_closest_to_car = self.get_closest_waypoint(car_p.x, car_p.y)

            diff = len(self.waypoints_xy)    

            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                idx_closest_to_light_cand = self.get_closest_waypoint(line[0], line[1])

                d = idx_closest_to_light_cand - idx_closest_to_car

                if d >= 0 and d < diff:
                    diff = d
                    idx_closest_to_light = idx_closest_to_light_cand
                    closest_light = light
        
        if idx_closest_to_light:
            
            state = self.get_light_state(closest_light)
            
            rospy.loginfo('Light state: {}'.format(state))
            rospy.loginfo('diff={}'.format(diff))
            
            return idx_closest_to_light, state, diff
        
        return -1, TrafficLight.UNKNOWN, -1

    def get_closest_waypoint(self, x, y):
        
        closest_idx = self.wp_tree.query((x, y), 1)[1]
        return closest_idx

    def get_light_state(self, light):
        return light.state


if __name__ == '__main__':
    try:
        ImageSaver()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start ImageSaver')
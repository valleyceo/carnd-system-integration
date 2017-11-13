#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from copy import deepcopy

import math

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

# Waypoints Updater
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        #self.obstacle_waypoints_sub = rospy.Subscriber("/obstacle_waypoint", message_type, obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.waypoints = None
        self.car_position_x = 0.
        self.car_position_y = 0.
        #rospy.logwarn('Waypoint Updater Init, waypoint length: %d', len(self.waypoints))
        #rospy.loginfo('this is an loginfo')
        #rospy.logwarn('this is an logwarn') # works
        #rospy.logerr('this is logerr') # works
        #rospy.logfatal('this is logfatal') # works
        
        # Handle ROS srv requests
        rospy.spin()
        #rospy.loop()

    def pose_cb(self, PoseStamped):
        # TODO: Implement
        self.current_pose = PoseStamped.pose
        self.car_position_x = PoseStamped.pose.position.x
        self.car_position_y = PoseStamped.pose.position.y
        self.publish_final_waypoint()
        return
        
    def waypoints_cb(self, Lane):
        # TODO: Implement
        # init waypoints
        if self.waypoints is None:
            self.waypoints = Lane.waypoints
            self.publish_final_waypoint()
            rospy.logwarn('Waypoint Updater Init, waypoint length: %d', len(self.waypoints))
        return

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

    def get_closest_waypoint(self):
        wp_len = len(self.waypoints)
        min_dist = float('inf')
        min_idx = -1
        
        dl = lambda a: ((self.car_position_x-a.x)**2 + (self.car_position_y-a.y)**2)
        for i in range(wp_len):
            dist = dl(self.waypoints[i].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def publish_final_waypoint(self):
        
        wp_len = len(self.waypoints)
        # find closest waypoint
        min_wp_idx = self.get_closest_waypoint()

        # get final waypoints
        final_waypoints = []

        # another method
        max_idx = min(min_wp_idx+LOOKAHEAD_WPS, wp_len)
        final_waypoints = deepcopy(self.waypoints[min_wp_idx:])
        #for i in range(LOOKAHEAD_WPS):
        #    next_wp_idx = (min_wp_idx + i) % LOOKAHEAD_WPS
        #    final_waypoints += [self.waypoints[next_wp_idx]]

        lane = Lane()
        lane.waypoints = final_waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()

        rospy.loginfo('published final waypoints')
        rospy.loginfo('closest waypoint idx: %d', min_wp_idx)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

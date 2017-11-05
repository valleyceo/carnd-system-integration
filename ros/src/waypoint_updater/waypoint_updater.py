#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint_sub = rospy.Subscriber("/traffic_waypoint", Waypoint, traffic_cb)
        #self.obstacle_waypoints_sub = rospy.Subscriber("/obstacle_waypoint", message_type, obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_pose_z = None

        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_z = []
        rospy.loginfo('GoToPositionRequest Received, waypoint length: %d', len(waypoints_x))

        rospy.spin()
        rospy.loginfo('spin completed, waypoint length: %d', len(waypoints_x))

    def pose_cb(self, PoseStamped):
        # TODO: Implement
        self.current_pose_x = PoseStamped.pose.position.x
        self.current_pose_y = PoseStamped.pose.position.y
        self.current_pose_z = PoseStamped.pose.position.z
        return

    def waypoints_cb(self, Lane):
        # TODO: Implement
        # init waypionts
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_z = []

        # append waypoints        
        for i in range(LOOKAHEAD_WPS):
            self.waypoints_x += waypoints.pose.position.x
            self.waypoints_y += waypoints.pose.position.y
            self.waypoints_z += waypoints.pose.position.z
        return

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        return msg.data

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

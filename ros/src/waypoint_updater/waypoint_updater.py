#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import deepcopy

import math

DEBUG_MODE = False
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
DELAY_IDX = 30

#######################
# Debugging Functions #
#######################
def print_waypoint_state():
    #rospy.logwarn('car pos x %f, posy: %f', self.current_x, self.current_y)
    #rospy.logwarn('closest waypoint idx: %d, distance to it: %f', self.closest_waypoint_idx, closest_dist)
    #rospy.logwarn('car posx %f, posy: %f, waypoint: %f, %f', self.current_x, self.current_y, closest_wp_pos.x, closest_wp_pos.y)
    #rospy.logwarn('new wp idx: %d, to: %d', new_wp_begin, new_wp_end)
    return

###########################
# Waypoints Updater Class #
###########################
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        # variables
        self.current_x = None
        self.current_y = None
        self.waypoints = None
        self.closest_waypoint_idx = None
        self.traffic_light_idx = None

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        
        # add publisher
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        #self.obstacle_waypoints_sub = rospy.Subscriber("/obstacle_waypoint", message_type, obstacle_cb)
                
        # Handle ROS srv requests
        rospy.spin()

    # current pose callback
    def pose_cb(self, PoseStamped):
        # TODO: Implement
        # get car's current x and y point
        self.current_x = PoseStamped.pose.position.x
        self.current_y = PoseStamped.pose.position.y
        
        # update final waypoint
        self.publish_final_waypoint()
        return

    # base waypoints callback
    def waypoints_cb(self, Lane):
        # TODO: Implement
        # initialize base waypoints
        if self.waypoints is None:
            self.waypoints = Lane.waypoints

        return

    # traffic waypoints callback
    def traffic_cb(self, msg):
        self.traffic_light_idx = msg

    # obstacle waypoints callback
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # get waypoint velocity
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # set waypoint velocity
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # compute path distance between two waypoints
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # returns the closest waypoint index to the car position
    def get_closest_waypoint(self):
        min_dist = float('inf')
        min_idx = -1

        for i in range(len(self.waypoints)):
            dist = (self.current_x - self.waypoints[i].pose.pose.position.x)**2 \
                    + (self.current_y - self.waypoints[i].pose.pose.position.y)**2
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    # returns the new closest waypoint by starting from the previous waypoint
    def get_next_closest_waypoint(self, last_idx):

        min_buffer = 10 # starts n waypoints behind
        min_dist = float('inf')

        # look a little back
        min_idx = max(0, last_idx - min_buffer)

        # search for the shortest distance
        for i in range(min_idx, len(self.waypoints)):
            dist = (self.current_x - self.waypoints[i].pose.pose.position.x)**2 \
                    + (self.current_y - self.waypoints[i].pose.pose.position.y)**2
            if dist < min_dist:
                min_dist = dist
            else:
                return i

        return -1

    # get distance of two points
    def get_dist(self, p1_x, p1_y, p2_x, p2_y):
        return math.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)

    # compute and publish next waypoint
    def publish_final_waypoint(self):
        
        ######################
        # find next waypoint #
        ######################
        
        # find closest waypoint
        if self.closest_waypoint_idx == None:
            self.closest_waypoint_idx = self.get_closest_waypoint()
        # if closest waypoint is already found, propagate waypoint to find the next closest
        else:
            self.closest_waypoint_idx = self.get_next_closest_waypoint(self.closest_waypoint_idx)
        
        # get coordinate for closest waypoint (added delay)
        closest_wp_pos = self.waypoints[self.closest_waypoint_idx].pose.pose.position
        closest_dist = self.get_dist(self.current_x, self.current_y, closest_wp_pos.x, closest_wp_pos.y)
        
        # get final waypoints
        final_waypoints = []

        # get new waypoints range
        new_wp_begin = self.closest_waypoint_idx + DELAY_IDX
        new_wp_end = self.closest_waypoint_idx + DELAY_IDX + LOOKAHEAD_WPS

        #########################
        # process traffic light #
        #########################
        
        # traffic data (-1 if traffic light is red)
        stop_idx = int(self.traffic_light_idx.data)

        # clip waypoint end if traffic light is red and in range
        if (stop_idx > 0):
            #rospy.logwarn('Traffic light is red, idx: %d', stop_idx)

            # compute distance
            dist = self.distance(self.waypoints, self.closest_waypoint_idx, int(self.traffic_light_idx.data))
            rospy.logwarn('tl distance: %f', dist)

            # if traffic is within waypoint end, clip waypoint end to traffic idx
            if stop_idx < new_wp_end:
                #rospy.logwarn('BREAK! at wp idx: %f, current idx: %f', stop_idx, self.closest_waypoint_idx)
                new_wp_end = stop_idx
            else:
                stop_idx = -1
        else:
            #rospy.logwarn('No traffic light signal, idx: %d', stop_idx)
        
        # append to final waypoints
        if new_wp_end < len(self.waypoints):
            final_waypoints = deepcopy(self.waypoints[new_wp_begin:new_wp_end])
        else:
            final_waypoints = deepcopy(self.waypoints[new_wp_begin:])
            final_waypoints.append(deepcopy(self.waypoints[:len(self.waypoints)-new_wp_end]))

        # create waypoint message template
        lane = Lane()
        lane.waypoints = final_waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()

        # publish /final_waypoints topic
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

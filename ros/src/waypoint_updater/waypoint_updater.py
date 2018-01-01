#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float64
from copy import deepcopy

import math

# Constants
ONE_MPH = 0.44704
CREEP_VELOCITY = 1.5
CREEP_RANGE = 30

# Target velocity in meters per second.
TARGET_VELOCITY = ONE_MPH * 24.0

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

# If it is not the highway, then it is the churchlot.  Highway is the
# default
HIGHWAY = True

if HIGHWAY:
    LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
    BRAKING_RANGE = 14.0 # This is in meters
else:
    # My version of churchlot has only 55 way points
    LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number
    BRAKING_RANGE = 15.0 # This _must_ be smaller than lookahead

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
        #self.manual_mode = True
        #self.current_x = None
        #self.current_y = None
        #self.waypoints = None
        #self.closest_waypoint_idx = None
        #self.traffic_light_idx = None
        self.waypoints = []
        self.x_ave = 0.0
        self.y_ave = 0.0
        self.cos_rotate = 0.0
        self.sin_rotate = 0.0
        self.phi = []
        self.current_velocity = 0.0
        self.red_light = Int32(-1)
        self.lap_count = 0
        self.lap_toggle = False

        max_velocity = float(rospy.get_param("/waypoint_loader/velocity")) / 3.6
        self.target_velocity = min(TARGET_VELOCITY, max_velocity)
        rospy.loginfo("target velocity %f" % self.target_velocity)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        
        # add publisher
        self.cte_publisher = rospy.Publisher('cross_track_error', Float64, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        #self.obstacle_waypoints_sub = rospy.Subscriber("/obstacle_waypoint", message_type, obstacle_cb)
        
        # Handle ROS srv requests
        rospy.spin()

    def lap(self, rho):
        # Ugly but robust
        if rho < 0.2 and self.lap_toggle:
            self.lap_count += 1
            rospy.logwarn("Completed lap %d" % self.lap_count)
            self.lap_toggle = False
        elif rho > 6.0:
            self.lap_toggle = True
        return

    def get_index(self, x, y):
        rho = self.get_angle(x, y)
        # special case of wrap around when past last waypoint
        if not self.phi or rho > self.phi[-1]:
            return 0
        
        self.lap(rho)
        
        idx = 0
        while rho > self.phi[idx]:
            idx += 1
        return idx

    def get_angle(self, x, y):
        # First center
        xc = x - self.x_ave
        yc = y - self.y_ave
        
        # and now rotate
        xr = xc * self.cos_rotate - yc * self.sin_rotate
        yr = yc * self.cos_rotate + xc * self.sin_rotate
        
        # rho now starts at 0 and goes to 2pi for the track waypoints
        rho = math.pi - math.atan2(xr, yr)
        return rho

    def get_cte(self, idx, x0, y0):
        x1 = self.waypoints[idx].pose.pose.position.x
        y1 = self.waypoints[idx].pose.pose.position.y
        x2 = self.waypoints[idx-1].pose.pose.position.x
        y2 = self.waypoints[idx-1].pose.pose.position.y
        cte = -((x2 - x1)*(y1 - y0) - (x1 - x0)*(y2 - y1)) /   \
                 math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        self.cte_publisher.publish(Float64(cte))
        return cte
    
    # current pose callback
    def pose_cb(self, PoseStamped):
        # pose_cb might be called before waypoints_cb
        if self.waypoints == []:
            return None

        idx = self.get_index(msg.pose.position.x, msg.pose.position.y)
        cte = self.get_cte(idx, msg.pose.position.x, msg.pose.position.y)
        #rospy.logwarn("position: %d   cte: %f  stop_line %d" % \
        #              (idx, cte, self.red_light.data))
        self.publish(idx)

    # waypoints callback
    def waypoints_cb(self, lane):
        wp = lane.waypoints
        x_tot = 0.0
        y_tot = 0.0
        for p in wp:
            x_tot += p.pose.pose.position.x
            y_tot += p.pose.pose.position.y

        # We use the average values to recenter the waypoints
        self.x_ave = x_tot / len(wp)
        self.y_ave = y_tot / len(wp)
        
        # The very first waypoint determines the angle we need to rotate
        # all waypoints by
        xc = wp[0].pose.pose.position.x - self.x_ave
        yc = wp[0].pose.pose.position.y - self.y_ave
        rot_angle = math.atan2(xc, yc) + math.pi
        self.cos_rotate = math.cos(rot_angle)
        self.sin_rotate = math.sin(rot_angle)

        for p in wp:
            rho = self.get_angle(p.pose.pose.position.x, p.pose.pose.position.y)
            self.phi.append(rho)
                
        #self.waypoints.extend(wp)
        self.waypoints = wp
        # make wrap around easier by extending waypoints
        self.waypoints.extend(wp[0:LOOKAHEAD_WPS])

    # car velocity callback
    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        return

    # traffic waypoints callback
    def traffic_cb(self, msg):
        self.red_light = msg
        a = msg.data
        outstr = "Red light msg : " + str(a)
        rospy.loginfo(outstr)
        return

    # obstacle waypoints callback
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # get waypoint velocity
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # set waypoint velocity
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        if not (0 <= waypoint <= len(waypoints)):
            rospy.logwarn("WTF: %d %d" % (waypoint, len(waypoints)))
        waypoints[waypoint].twist.twist.linear.x = velocity

    # compute path distance between two waypoints
    def distance(self, waypoints, wp1, wp2):
        dist = 0.0
        dl = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        if wp1 > wp2:
            for i in range(wp1, len(self.waypoints)):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
            for i in range(0, wp2+1):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
        else:
            for i in range(wp1, wp2+1):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
        return dist

    # compute and publish next waypoint
    def publish(self, idx):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = []
        lane.waypoints.extend(self.waypoints[idx:idx+LOOKAHEAD_WPS])

        # Dead nuts simple braking strategy.  This brakes at max value
        dist = self.distance(self.waypoints, idx, self.red_light.data)
        if self.red_light.data != -1 and dist < BRAKING_RANGE:
            for i in range(LOOKAHEAD_WPS):
                self.set_waypoint_velocity(lane.waypoints, i, 0.0)
        else:
            for i in range(LOOKAHEAD_WPS):
                self.set_waypoint_velocity(lane.waypoints, i, self.target_velocity)
        
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

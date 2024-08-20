#!/usr/bin/env python

import rospy
import numpy as np
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from  std_msgs.msg import Bool

class MoveWhenPrompt():
    def __init__(self, pose_file) -> None:
        self.poses = np.loadtxt(pose_file) # in format of N x (x, y, z, qw, qx, qy, qz)
        self.current_waypoint = None
        self.prompter = rospy.Subscriber("/next_waypoint",Bool, callback=self.prompter_cb)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print("Number of way points:", self.poses.shape[0])
    
    def prompter_cb(self, msg : Bool):
        print(msg)
        if msg.data == True:
            if self.current_waypoint == self.poses.shape[0]:
                print("Reached the end of the way points")
                return
            if self.current_waypoint is None:
                self.current_waypoint = 0
            else:
                self.current_waypoint += 1
            print("Go to way point number", self.current_waypoint)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.poses[self.current_waypoint,0]
            goal.target_pose.pose.position.y = self.poses[self.current_waypoint,1]
            goal.target_pose.pose.position.z = self.poses[self.current_waypoint,2]
            goal.target_pose.pose.orientation.w = self.poses[self.current_waypoint,3]
            goal.target_pose.pose.orientation.x = self.poses[self.current_waypoint,4]
            goal.target_pose.pose.orientation.y = self.poses[self.current_waypoint,5]
            goal.target_pose.pose.orientation.z = self.poses[self.current_waypoint,6]
            
            print("waiting for server")
            self.client.wait_for_server()
            
            print("sending goal")
            self.client.send_goal(goal)
            
            print("waiting for result")
            # wait = self.client.wait_for_result()
            wait = True
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                if self.client.get_result():
                    rospy.loginfo("Goal execution done!")
                # return self.client.get_result()
            

if __name__ == '__main__':
    pose_file = '/root/ws/data/waypoints.txt'
    rospy.init_node('movebase_client')
    move_when_prompt = MoveWhenPrompt(pose_file)
    print("Initialized action server")
    try:
        rospy.spin()
    except:
        print("spin failed")
    
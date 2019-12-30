#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 29 13:13:14 2019

@author: shay
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from collections import defaultdict 
from collections import deque


"""
Finds a shortest path, or none if no such path exists in unweighted graph
"""
def bfs(graph, start, goal):
    if start == goal:
        return [start]
    visited = {start}
    queue = deque([(start, [])])

    while queue:
        current, path = queue.popleft()
        visited.add(current)
        for neighbor in graph[current]:
            if neighbor == goal:
                return path + [current, neighbor]
            if neighbor in visited:
                continue
            queue.append((neighbor, path + [current]))
            visited.add(neighbor)   
    return None  # no path found. not strictly needed


"""
Transforms coord string in the form of <sign x><coord x>_<sign y><coord y>
 to dictionary with entries x: float(x) y:float(y)
"""
def parse_coords(coords_str):
    x_str = coords_str.split('_')[0]
    y_str = coords_str.split('_')[1]
    if 'minus' in x_str:
        x_str = x_str.split('minus')[1]
        x = -1*float(x_str)
    else:
        x = float(x_str)
    
    if 'minus' in y_str:
        y_str = y_str.split('minus')[1]
        y = -1*float(y_str)
    else:
        y = float(y_str)
        
    coords = {'x': x, 'y':y}
    return coords

def parse_path(path):
    return list(map(parse_coords, path))
    


class Navigator():
    def __init__(self):
        self.goal_sent = False
    	rospy.on_shutdown(self.shutdown)
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("Wait for the action server to come up")
    	# Wait up to 5 seconds for move_base server
    	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        rospy.loginfo('Next checkpoint is:'+str(pos['x'])+', '+str(pos['y']))
        self.goal_sent = True
    	goal = MoveBaseGoal()
    	goal.target_pose.header.frame_id = 'map'
    	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
    
    	# Move
        self.move_base.send_goal(goal)
    
    	# Wait up to 60 seconds for turtlebot to get to checkpoint
    	success = self.move_base.wait_for_result(rospy.Duration(60)) 
        result = False

        if success and self.move_base.get_state() == GoalStatus.SUCCEEDED:
            # succeeded getting to checkpont
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        
        return result
    """
    Follow the shortest path found. Quaternions do not have to change between checkpoints
    as turtlebot can face towards exit throughout the journey.
    """
    
    def follow_path(self, path, quat):
        for pos in path:
            result_checkpoint = self.goto(pos, quat)
            if result_checkpoint == False:
                rospy.loginfo('Failed getting to checkpoint ' +str(pos['x'])+' '+str(pos['y']))
                
        
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('maze_navigator', anonymous=False)
        navigator = Navigator()
        graph = {
                '2.27_0.03': set(['2.27_1.44', '2.33_minus1.87']),  
                '2.33_minus1.87': set(['4.19_minus1.87']),
                '4.19_minus1.87': set([]),   
                '2.27_1.44': set(['5.85_1.44']),     
                '5.85_1.44': set(['7.52_3.21', '5.85_minus0.739']),             
                '7.52_3.21': set([]),
                '5.85_minus0.739': set(['8.46_minus0.739']),
                '8.46_minus0.739': set([])
                }

        
        path = bfs(graph, '2.27_0.03', '8.46_minus0.739')
        
        if path is not None:
            rospy.loginfo('Path exists, turtlebot will go through the following coordinates:')
            rospy.loginfo(path)
            #turn each node name to float coordinates 
            path = parse_path(path)
            #
            navigator.follow_path(path, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000})
        else:
            rospy.loginfo("No path was found for the following coordinates")


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
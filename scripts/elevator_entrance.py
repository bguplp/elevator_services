#!/usr/bin/env python 

import rospy
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import tf.transformations
import tf2_ros
from sensor_msgs.msg import LaserScan
import numpy as np
from elevator_services.srv import elevator_entrance, elevator_entranceRequest #, ser_message, ser_messageRequest
import shlex
from psutil import Popen

global z, error_max, num
num = -10

def wrong_floor_num_func():
    rospy.logerr('Invalid floor number!\nplease insert "0" for first floor and "1" for second floor')
    new_floor_num = input('\nwaiting for your respond choose <"0","1">: ')
    return int(new_floor_num), False

def get_scan(msg):
    global z
    z = np.array(msg.ranges)

def first_floor_func():
    global z, error_max, num

    rospy.Subscriber('/scan', LaserScan, get_scan)

    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("elevator_entrance/poses/first_floor/outside_elevator/x")
    input_pose.pose.position.y = rospy.get_param("elevator_entrance/poses/first_floor/outside_elevator/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/first_floor/outside_elevator/Y")))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached initial position for elevator entrance")
        # ser_messageResponse(True)

    else:
        rospy.loginfo("The robot failed to reache the initial position for elevator entrance")
        # ser_messageResponse(False)

    z = np.array(rospy.wait_for_message("/scan", LaserScan).ranges)
        
    # rospy.sleep(0.5)
    temp_z = z
    error = np.absolute(z - temp_z)
    error[~np.isfinite(error)] = 0
    error_max = np.sort(error)[num] 
    # flag = True
    while (error_max < 1.5):
        
        error = np.absolute(z - temp_z)
        temp_z = z
        error[~np.isfinite(error)] = 0
        error_max = np.sort(error)[num]                 

        #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
        rospy.sleep(0.5)
        # if (flag):
        #     flag = False
        #     error_max = 0
        #     rospy.sleep(1)

    rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("elevator_entrance/poses/first_floor/inside_elevator_first_try/x")
    input_pose.pose.position.y = rospy.get_param("elevator_entrance/poses/first_floor/inside_elevator_first_try/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/first_floor/inside_elevator_first_try/Y")
))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have entered to the elevator")
        # ser_messageResponse(True)
        try_1 = True
    else:
        rospy.loginfo("The robot failed to enter to the elevator")
        # ser_messageResponse(False)
        try_1 = False
    
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("poses/elevator_entrance/first_floor/inside_elevator_second_try/x")
    input_pose.pose.position.y = rospy.get_param("poses/elevator_entrance/first_floor/inside_elevator_second_try/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/first_floor/inside_elevator_second_try/Y")))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("the robot adjusted his position for exit elevator service")
        # ser_messageResponse(True)
        try_2 = True
    else:
        rospy.loginfo("The robot failed to adjust his position for exit elevator service")
        # ser_messageResponse(False)
        try_2 = False
        
    # start
    if rospy.get_param("elevator_entrance/gazebo"):
        node_process = Popen(shlex.split('rosrun elevator_services inside_elevator.py'))
        rospy.logwarn("node \"/inside_elevator\" is running")
    rospy.loginfo("you can call exit elevator service now!!")
    if try_1 is True or try_2 is True:
        return True, True
    else:
        return False, True

def second_floor_func():
    global z, error_max, num

    rospy.Subscriber('/scan', LaserScan, get_scan)

    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("elevator_entrance/poses/second_floor/outside_elevator/x")
    input_pose.pose.position.y = rospy.get_param("elevator_entrance/poses/second_floor/outside_elevator/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/second_floor/outside_elevator/Y")))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached initial position for elevator entrance")
        # ser_messageResponse(True)

    else:
        rospy.loginfo("The robot failed to reache the initial position for elevator entrance")
        # ser_messageResponse(False)

    z = np.array(rospy.wait_for_message("/scan", LaserScan).ranges)
        
    # rospy.sleep(0.5)
    temp_z = z
    error = np.absolute(z - temp_z)
    error[~np.isfinite(error)] = 0
    error_max = np.sort(error)[num] 
    # flag = True
    while (error_max < 1.5):
        
        error = np.absolute(z - temp_z)
        temp_z = z
        error[~np.isfinite(error)] = 0
        error_max = np.sort(error)[num]            

        #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
        rospy.sleep(0.5)
        # if (flag):
        #     flag = False
        #     error_max = 0
        #     rospy.sleep(1)

    rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_first_try/x")
    input_pose.pose.position.y = rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_first_try/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_first_try/Y")))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have entered to the elevator")
        # ser_messageResponse(True)
        try_1 = True
    else:
        rospy.loginfo("The robot failed to enter to the elevator")
        # ser_messageResponse(False)
        try_1 = False
    
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_second_try/x")
    input_pose.pose.position.y = rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_second_try/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("elevator_entrance/poses/second_floor/inside_elevator_second_try/Y")))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("the robot adjusted his position for exit elevator service")
        # ser_messageResponse(True)
        try_2 = True
    else:
        rospy.loginfo("The robot failed to adjust his position for exit elevator service")
        # ser_messageResponse(False)
        try_2 = False
        
    # start
    if rospy.get_param("elevator_entrance/gazebo"):
        node_process = Popen(shlex.split('rosrun elevator_services inside_elevator.py'))
        rospy.logwarn("node \"/inside_elevator\" is running")
    rospy.loginfo("you can call exit elevator service now!!")
    if try_1 is True or try_2 is True:
        return True, True
    else:
        return False, True

def switch_func(current_floor, flag = True):
    global status
    switcher = {
        0: first_floor_func,
        1: second_floor_func
    } 
    switch_to = switcher.get(current_floor, wrong_floor_num_func)
    enter, flag = switch_to()
    if not flag:
        switch_func(current_floor = enter)
    else:
        status = enter         

def elevator_entrance_func(req):
    global status
    switch_func(current_floor = int(req.current_floor))
    return status

rospy.init_node('elevator_entrance', anonymous=True)
rospy.Service("/elevator_entrance", elevator_entrance, elevator_entrance_func)
rospy.loginfo("elevator entrance service is waiting for request...")

rospy.spin()

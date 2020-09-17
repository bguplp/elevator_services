#!/usr/bin/env python 

import rospy
import rosnode
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, PoseWithCovarianceStamped
import tf.transformations
import tf2_ros
from sensor_msgs.msg import LaserScan
import numpy as np
from elevator_services.srv import exit_elevator, exit_elevatorResponse
from std_msgs.msg import String
from elevator_services.srv import switch_map, switch_mapResponse

global z, error_max, num, node_name
num = -10

def init_pose_check(x, y):
    init_pose_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped, queue_size=2)
    amcl = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
    pose = amcl.pose.pose.position
    R = np.sqrt((pose.x - x)**2 + (pose.y - y)**2)
    rospy.loginfo("distance from init pose \"R\": "+ str(R))

    if R > 2:
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = rospy.Time.now()
        init_pose.pose.pose.position.x = rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/x")
        init_pose.pose.pose.position.y = rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/y")
        init_pose.pose.pose.position.z = 0
        init_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/Y")))
        init_pose.pose.covariance =[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.25] #0.25
        for ii in xrange(3):
            init_pose_pub.publish(init_pose) 

def get_scan(msg):
    global z
    z = np.array(msg.ranges)     

def wrong_floor_num_func():
    rospy.logerr('Invalid floor number!\nplease insert "0" for first floor and "1" for second floor')
    new_floor_num = input('\nwaiting for your respond choose <"0","1">: ')
    return int(new_floor_num), False

def first_floor_func():
    global z, error_max, num, node_name
    initialpose = rospy.Publisher("initialpose",PoseWithCovarianceStamped, queue_size=2)

    switch_srv = rospy.ServiceProxy('/switch_map', switch_map)
    switch_srv(rospy.get_param("exit_elevator/maps/first_floor"))
    
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.position.x = rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/x")
    init_pose.pose.pose.position.y = rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/y")
    init_pose.pose.pose.position.z = 0
    init_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/Y")))
    init_pose.pose.covariance =[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.25] #0.25
    initialpose.publish(init_pose)

    rospy.Subscriber('/scan', LaserScan, get_scan)

    # input_pose = PoseStamped()
    # input_pose.header.stamp = rospy.Time.now()
    # input_pose.header.frame_id = 'map'
    # input_pose.pose.position.x = 6.5 
    # input_pose.pose.position.y = 6.3
    # input_pose.pose.position.z = 0
    # #input_pose.pose.orientation.x = 
    # #input_pose.pose.orientation.y =
    # #input_pose.pose.orientation.z =
    # #input_pose.pose.orientation.w =

    # #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    # #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    # #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    # input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,-1.57))

    # #define a client for to send goal requests to the move_base server through a SimpleActionClient
    # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # #wait for the action server to come up
    # while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base action server to come up")
    # '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    # goal = MoveBaseGoal()
    # #set up the frame parameters
    # goal.target_pose.header.frame_id = "/map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # # moving towards the goal*/
    # goal.target_pose = input_pose

    # rospy.loginfo("Sending goal location ...")
    # ac.send_goal(goal)	
    # ac.wait_for_result(rospy.Duration(60))

    # if(ac.get_state() ==  GoalStatus.SUCCEEDED):
    #     rospy.loginfo("You have reached initial position for elevator entrance")
        # exit_elevatorResponse(True)
    
    # else:
    #     rospy.loginfo("The robot failed to the initial position for elevator entrance")
        # exit_elevatorResponse(False)


    z = np.array(rospy.wait_for_message("/scan", LaserScan).ranges)
    
    
    # rospy.sleep(0.5)
    temp_z = z
    error = np.absolute(z - temp_z)
    error[~np.isfinite(error)] = 0
    error_max = np.sort(error)[num]
    # flag = True
    while (error_max < 0.15):
        error = np.absolute(z - temp_z)
        temp_z = z
        error[~np.isfinite(error)] = 0
        error_max = np.sort(error)[num]

        if rospy.get_param("exit_elevator/gazebo"):
            rospy.logwarn("node \"/inside_elevator\" is alive")                

        #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
        rospy.sleep(0.5)
        # if (flag):
        #     flag = False
        #     error_max = 0
        #     rospy.sleep(1)

    if rospy.get_param("exit_elevator/gazebo"):
        success, fail = rosnode.kill_nodes([node_name.data])
        rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
        if fail != []:
            rospy.logerr("node \"/inside_elevator\" is still alive!!")
        elif success != []:
            rospy.loginfo("node \"/inside_elevator\" died!!")
    else:
        rospy.sleep(1)

    rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("exit_elevator/poses/first_floor/outside_elevator/x")
    input_pose.pose.position.y = rospy.get_param("exit_elevator/poses/first_floor/outside_elevator/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("exit_elevator/poses/first_floor/outside_elevator/Y")))

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

    start_time = rospy.Time.now().secs
    while (rospy.Time.now().secs - start_time) < 5:
        init_pose_check(rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/x"), rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/y"))
	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You got out of the elevator")
        # exit_elevatorResponse(True)
        return  True, True

    else:
        rospy.loginfo("The robot failed to get out of the elevator")
        # exit_elevatorResponse(False)
        return  False, True

def second_floor_func():
    global z, error_max, num, node_name
    initialpose = rospy.Publisher("initialpose",PoseWithCovarianceStamped, queue_size=2)

    switch_srv = rospy.ServiceProxy('/switch_map', switch_map)
    switch_srv(rospy.get_param("exit_elevator/maps/second_floor"))

    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.position.x = rospy.get_param("exit_elevator/poses/second_floor/inside_elevator/x")
    init_pose.pose.pose.position.y = rospy.get_param("exit_elevator/poses/second_floor/inside_elevator/y")
    init_pose.pose.pose.position.z = 0
    init_pose.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("exit_elevator/poses/second_floor/inside_elevator/Y")))
    init_pose.pose.covariance =[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.25] #0.25
    initialpose.publish(init_pose)

    rospy.Subscriber('/scan', LaserScan, get_scan)

    # input_pose = PoseStamped()
    # input_pose.header.stamp = rospy.Time.now()
    # input_pose.header.frame_id = 'map'
    # input_pose.pose.position.x = 6.5 
    # input_pose.pose.position.y = 6.3
    # input_pose.pose.position.z = 0
    # #input_pose.pose.orientation.x = 
    # #input_pose.pose.orientation.y =
    # #input_pose.pose.orientation.z =
    # #input_pose.pose.orientation.w =

    # #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    # #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    # #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    # input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,-1.57))

    # #define a client for to send goal requests to the move_base server through a SimpleActionClient
    # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # #wait for the action server to come up
    # while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base action server to come up")
    # '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    # goal = MoveBaseGoal()
    # #set up the frame parameters
    # goal.target_pose.header.frame_id = "/map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # # moving towards the goal*/
    # goal.target_pose = input_pose

    # rospy.loginfo("Sending goal location ...")
    # ac.send_goal(goal)	
    # ac.wait_for_result(rospy.Duration(60))

    # if(ac.get_state() ==  GoalStatus.SUCCEEDED):
    #     rospy.loginfo("You have reached initial position for elevator entrance")
        # exit_elevatorResponse(True)
    
    # else:
    #     rospy.loginfo("The robot failed to the initial position for elevator entrance")
        # exit_elevatorResponse(False)


    z = np.array(rospy.wait_for_message("/scan", LaserScan).ranges)
    
    # rospy.sleep(0.5)
    temp_z = z
    error = np.absolute(z - temp_z)
    error[~np.isfinite(error)] = 0
    error_max = np.sort(error)[num]
    # flag = True
    while (error_max < 0.15):
        error = np.absolute(z - temp_z)
        temp_z = z
        error[~np.isfinite(error)] = 0
        error_max = np.sort(error)[num]

        if rospy.get_param("exit_elevator/gazebo"):
            rospy.logwarn("node \"/inside_elevator\" is alive")                

        #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
        rospy.sleep(0.5)
        # if (flag):
        #     flag = False
        #     error_max = 0
        #     rospy.sleep(1)

    if rospy.get_param("exit_elevator/gazebo"):
        success, fail = rosnode.kill_nodes([node_name.data])
        rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
        if fail != []:
            rospy.logerr("node \"/inside_elevator\" is still alive!!")
        elif success != []:
            rospy.loginfo("node \"/inside_elevator\" died!!")
    else:
        rospy.sleep(1)

    rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = rospy.get_param("exit_elevator/poses/second_floor/outside_elevator/x")
    input_pose.pose.position.y = rospy.get_param("exit_elevator/poses/second_floor/outside_elevator/y")
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,rospy.get_param("exit_elevator/poses/second_floor/outside_elevator/Y")))

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

    start_time = rospy.Time.now().secs
    while (rospy.Time.now().secs - start_time) < 5:
        init_pose_check(rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/x"), rospy.get_param("exit_elevator/poses/first_floor/inside_elevator/y"))
	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You got out of the elevator")
        # exit_elevatorResponse(True)
        return  True, True

    else:
        rospy.loginfo("The robot failed to get out of the elevator")
        # exit_elevatorResponse(False)
        return  False, True
    

def switch_func(floor_num, flag = True):
    global status
    switcher = {
        0: first_floor_func,
        1: second_floor_func
    } 
    switch_to = switcher.get(floor_num, wrong_floor_num_func)
    new_floor, flag = switch_to()
    if not flag:
        switch_func(floor_num = new_floor)
    else:
        status = new_floor

def exit_elevator_func(req):
    global status
    rospy.logwarn("waiting for switch map service...")
    rospy.wait_for_service('/switch_map')
    switch_func(floor_num = int(req.floor_num))
    return status

rospy.init_node('exit_elevator', anonymous=True)
rospy.Service("/exit_elevator", exit_elevator, exit_elevator_func)
if rospy.get_param("exit_elevator/gazebo"):
    node_name = String()
    node_name = rospy.wait_for_message("/inside_elevator_node_name", String)
rospy.loginfo("exit elevator service is waiting for request...")

rospy.spin()

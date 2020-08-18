#!/usr/bin/env python 

import rospy
import rosnode
import subprocess, shlex
from subprocess import Popen, PIPE, call
from elevator_services.srv import switch_map, switch_mapResponse

def kill_node(taype_name="/map_server"):
    success, fail = rosnode.kill_nodes([taype_name])
    rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
    if fail != []:
        rospy.logwarn("node "+taype_name+" is still alive")
        return False
    elif success != []:
        rospy.loginfo("node "+taype_name+" died")
        return True
    else:
        rospy.logwarn("something went wrong, with killing the node: "+taype_name)
        return False

def run_node(taype_name="map_server", pkg_name="map_server", arg=[]):
    node_process = Popen(shlex.split('rosrun '+pkg_name+' '+taype_name+' '+arg))   # arg=req.map_name
    rospy.loginfo("new node ["+taype_name+"] have been launched!") 
    rospy.sleep(2)
    return True

def switch_map_func(req):
    kill_node("/map_server")
    run_node("map_server", "map_server", req.map_name)
    return True

    # success, fail = rosnode.kill_nodes(["map_server"])
    # rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
    # if fail != []:
    #     rospy.logwarn("node \"map_server\" is still alive")
    # elif success != []:
    #     rospy.loginfo("node \"map_server\" died")

    # command_line = "roslaunch elevator_services map_server.launch map:="+req.map_name
    # args = shlex.split(command_line)
    # p = subprocess.Popen(args, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    # node_process = Popen(shlex.split('rosrun map_server map_server '+req.map_name))
    # rospy.loginfo("new map service node have been launched, with new map!") 
    # rospy.sleep(2)
    # p.terminate()
    # return True

def kill_map_func(req):
    kill_node("/map_server")
    return True

def run_map_func(req):
    run_node("map_server", "map_server", req.map_name)
    return True

rospy.init_node("switch_map_service", anonymous=True, disable_signals=True)
rospy.Service("/switch_map", switch_map, switch_map_func)
rospy.Service("/kill_map",switch_map, kill_map_func)
rospy.Service("/run__map", switch_map, run_map_func)
rospy.loginfo("Ready to switch map...")
rospy.spin()

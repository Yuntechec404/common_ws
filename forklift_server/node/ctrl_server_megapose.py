#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS_server', forklift_server.msg.PBVSMegaposeAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSMegaposeGoal(command=msg[1], layer_dist=msg[2])
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap_server', forklift_server.msg.TopologyMapAction)
    client.wait_for_server()
    goal = forklift_server.msg.TopologyMapGoal(goal=msg[1])
    # print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')
    rospy.logwarn(rospy.get_name() + "start")
    rospy.logwarn("your command list:\n")
    command = rospy.get_param(rospy.get_name() + "/command") 
    for i in command:
        print(i)

    for msg in command:
        rospy.sleep(1)
        if(msg[0] == 'PBVS' or msg[0] == 'odom'):
            rospy.logwarn(f"send {msg[0]}: {msg[1]}, {msg[2]}")
            result = PBVS_client(msg)
            print("PBVS_client result ", result)

        elif(msg[0] == 'TopologyMap'):
            rospy.logwarn(f"send {msg[0]}: {msg[1]}")
            result = TopologyMap_client(msg)
            print("TopologyMap result ", result)
        else:
            print("error command: ", msg)
            
    rospy.signal_shutdown("finish command list")
  
    

#! /usr/bin/env python

import rospy
import time

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the chasi actions, including the
import rxt_skills_chasi.msg


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# client request implementations of CHASI action server functions
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_request_WaitForUserInput(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForUserInput', rxt_skills_chasi.msg.WaitForUserInputAction) # Creates SimpleActionClient with WaitForUserInputAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.WaitForUserInputGoal(inputContent=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action


def chasi_request_MoveToLocation(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('MoveToLocation', rxt_skills_chasi.msg.MoveToLocationAction) # Creates SimpleActionClient with MoveToLocationAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.MoveToLocationGoal(location=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (MoveToLocationResult) of executing the action
    
    
def chasi_request_WaitForExternalEvent(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('WaitForExternalEvent', rxt_skills_chasi.msg.WaitForExternalEventAction) # Creates SimpleActionClient WaitForExternalEventAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.WaitForExternalEventGoal(inputText=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForExternalEventResult) of executing the action
    
    
def chasi_request_GraphicalUserInteraction(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GraphicalUserInteraction', rxt_skills_chasi.msg.GraphicalUserInteractionAction) # Creates SimpleActionClient with GraphicalUserInteractionAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.GraphicalUserInteractionGoal(outputMessage=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GraphicalUserInteractionResult) of executing the action
    
    
def chasi_request_GetData(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('GetData', rxt_skills_chasi.msg.GetDataAction) # Creates SimpleActionClient with GetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.GetDataGoal(inputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (GetDataResult) of executing the action


def chasi_request_SetData(msgBytes):
    
    rospy.init_node('chasi_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient('SetData', rxt_skills_chasi.msg.SetDataAction) # Creates SimpleActionClient with SetDataAction action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    goal = rxt_skills_chasi.msg.SetDataGoal(outputData=msgBytes) # Creates a goal to send to the action server
    client.send_goal(goal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (SetDataResult) of executing the action
	
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:	
        
        # request MoveToLocation
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: MoveToLocation')
        result = chasi_request_MoveToLocation(b'parkingslot')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')

        # request WaitForUserInput
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: WaitForUserInput')
        result = chasi_request_WaitForUserInput(b'rviz')
        if result:
            print("Result was:", ', '.join([str(n) for n in result.returnMessage.decode("utf-8")]))
        print ('----------------------------------')
        
        # request WaitForExternalEvent
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: WaitForExternalEvent')
        result = chasi_request_WaitForExternalEvent(b'rviz')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request GraphicalUserInteraction
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: GraphicalUserInteraction')
        result = chasi_request_GraphicalUserInteraction(b'rviz')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
        
        # request GetData
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: GetData')
        result = chasi_request_GetData(b'position')
        if result:
            print("Result was:", ', '.join([str(n) for n in result.data.decode("utf-8")]))
        print ('----------------------------------')
        
        # request SetData
		print ('----------------------------------')
		print ('INVOKING RXT_SKILL: SetData')
        result = chasi_request_SetData(b'position')
        if result:
            print("Result was: " + str(result.isOK))
        print ('----------------------------------')
          

        # shutdown node
        #print ('----------------------------------')
        #print ('All requests done: Now trying to shutdown everything...')
        #print ('----------------------------------')
        #rospy.signal_shutdown("Finished with success!")
        #rospy.spin()
             
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

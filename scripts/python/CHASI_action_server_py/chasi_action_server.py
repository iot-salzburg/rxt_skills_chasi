#!/usr/bin/env python
import os, sys, time
import rospy
import actionlib
import rxt_skills_chasi.msg

import tf.transformations
from geometry_msgs.msg import PoseStamped

# self registration
from self_registration import *

# Global variable
sent_message = None
received_message = None

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# class for listening position
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class Position_Listener(object):

    def __init__(self):
        self.flag = True
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.listener_callback)

    def listener_callback(self, msg):

        rospy.loginfo("Received at goal message!")
        rospy.loginfo("Timestamp: " + str(msg.header.stamp))
        rospy.loginfo("frame_id: " + str(msg.header.frame_id))

        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rospy.loginfo("Euler Angles: %s"%str(euler))  

        # terminate listening...(only first package read...)
        self.flag = False
        self.positionSTR = "x=" + str(msg.pose.position.x) + ", y=" + str(msg.pose.position.y) + ", z="  + str(msg.pose.position.z)
        self.orientationSTR = "x=" + str(msg.pose.orientation.x) + ", y=" + str(msg.pose.orientation.y) + ", z="  + str(msg.pose.orientation.z) + ", w=" + str(msg.pose.orientation.w)
        print(positionSTR)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: move to location
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
def chasi_moveTo(position):
    
    # Tutorials:
    # https://www.oreilly.com/library/view/ros-programming-building/9781788627436/192de5c9-e5bd-40b3-a75a-2990bdfa7caf.xhtml
    # https://answers.ros.org/question/306582/unable-to-publish-posestamped-message/



    # set command to move to position
    if position == "patient room":
        artiparking_position = "position: { x: 7.41178993832, y: 3.43726404144, z: -0.007645 }"
        artiparking_orientation = "orientation: { x: 0, y: 0, z: -0.223993065432, w: 0.974590738022 }"

        command = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id:  \"/map\"}, pose: { " + artiparking_position + ", " + artiparking_orientation + " } }'"
        os.system(command)

    elif position == "camera turn":
        command = "rosrun arti_docking navigate.sh"
        os.system(command)

    elif position == "parking slot":
        command = "rosrun arti_docking arti_parking.sh"
        os.system(command)

    else:
        rospy.loginfo('Found invalid input slot argument for skill MoveToLocation')



    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: listen for input
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_listen_for_input(input):
    
    try:   
        if(input.decode("utf-8") == 'rviz'):
            rospy.loginfo('Trying to listen from topic: /move_base_simple/goal')
            list = Position_Listener()

            while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
                rospy.sleep(1) 
            
        else:
            return b'NOT OK' 

    except rospy.ROSInterruptException:
        pass

    return b'OK' 
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: waitexternal
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_waitexternal(input):
    
    try:   
        if(input.decode("utf-8") == 'rviz'):
            rospy.loginfo('Trying to listen from topic: /move_base_simple/goal')
            list = Position_Listener()

            while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
                rospy.sleep(1) 
            
        else:
            return False 

    except rospy.ROSInterruptException:
        pass
    
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: UI interaction
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_graphicalInteraction(input):

    try:   
        if(input.decode("utf-8") == 'rviz'):
            rospy.loginfo('Trying to listen from topic: /move_base_simple/goal')
            list = Position_Listener()

            while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
                rospy.sleep(1) 
            
        else:
            return False 

    except rospy.ROSInterruptException:
        pass

    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: write a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_write_setting(setting, value):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: read a setting
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_read_setting(setting):

    try:   
        if(setting.decode("utf-8") == 'position'):
            rospy.loginfo('Trying to listen from topic: /move_base_simple/goal')
            list = Position_Listener()

            while list.flag: # sleep to block ActionEnd until we received "Stopped"-message
                rospy.sleep(1) 
            
            returnSTR = "position: [" + list.positionSTR + "] orientation: [" + list.orientationSTR + "]"
            return returnSTR.encode('utf-8')

    except rospy.ROSInterruptException:
        pass

    return b'NO DATA FOUND'


			
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# WaitForUserInput
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class WaitForUserInput(object):
    
    _feedback = rxt_skills_chasi.msg.WaitForUserInputFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.WaitForUserInputResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.WaitForUserInputAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating WaitForUserInput with input content %s with seeds %i, %i' % (self._action_name, goal.inputContent, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = True
        returnMsg = chasi_listen_for_input(goal.inputContent)
          
        if success:
            self._result.returnMessage = returnMsg
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# MoveToLocation
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class MoveToLocation(object):
       
    _feedback = rxt_skills_chasi.msg.MoveToLocationFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.MoveToLocationResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.MoveToLocationAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating MoveToLocation sequence with location %s with seeds %i, %i' % (self._action_name, goal.location, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = chasi_moveTo(goal.location)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# WaitForExternalEvent
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class WaitForExternalEvent(object):
    
    _feedback = rxt_skills_chasi.msg.WaitForExternalEventFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.WaitForExternalEventResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.WaitForExternalEventAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating WaitForExternalEvent sequence with inputText %s with seeds %i, %i' % (self._action_name, goal.inputText, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = chasi_waitexternal(goal.inputText)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# GraphicalUserInteraction
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class GraphicalUserInteraction(object):
    
    _feedback = rxt_skills_chasi.msg.GraphicalUserInteractionFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.GraphicalUserInteractionResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.GraphicalUserInteractionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating GraphicalUserInteraction sequence with outputMessage %s with seeds %i, %i' % (self._action_name, goal.outputMessage, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = chasi_graphicalInteraction(goal.outputMessage)
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# GetData
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class GetData(object):
    
    _feedback = rxt_skills_chasi.msg.GetDataFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.GetDataResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.GetDataAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating GetData sequence with inputData %s with seeds %i, %i' % (self._action_name, goal.inputData, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = True
        robotName = chasi_read_setting(goal.inputData);
          
        if success:
            self._result.data = robotName
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# SetData
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class SetData(object):
    
    _feedback = rxt_skills_chasi.msg.SetDataFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.SetDataResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.SetDataAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        
        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating SetData sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.outputData, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # start executing the action
        #success = fibonacci_example(self, success)
        success = chasi_write_setting('robotName', goal.outputData)
        while success != True:
            rospy.sleep(0.1)
            pass
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# SendMessage
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class SendMessage(object):
    
    _feedback = rxt_skills_chasi.msg.SendMessageFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.SendMessageResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.SendMessageAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        global sent_message

        # start executing the action
        sent_message = str(goal.messageContent)

         # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating SendMessage sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.messageContent, self._feedback.sequence[0], self._feedback.sequence[1]))

        self._result.isOK = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# OnMessageReceive
#------------------------------------------------------------------------------------------------------------------------------------------------------------
class OnMessageReceive(object):
   
    _feedback = rxt_skills_chasi.msg.OnMessageReceiveFeedback() #create feedback message
    _result = rxt_skills_chasi.msg.OnMessageReceiveResult() #create result message

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rxt_skills_chasi.msg.OnMessageReceiveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
      
    def execute_cb(self, goal):
        global received_message, sent_message
        # start executing the action
        received_message = str(goal.messageContent)
        
        while sent_message != received_message:
            rospy.sleep(0.1)
            print("arti while",sent_message)
            if sent_message == received_message:
                break
            if rospy.is_shutdown() == True:
                break
            else:
                pass
        print("exited successfully")
        rospy.sleep(0.2)

        # append the seeds to give user feedback
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
        
        rospy.loginfo('%s: Executing, creating OnMessageReceive sequence with outputData %s with seeds %i, %i' % (self._action_name, goal.messageContent, self._feedback.sequence[0], self._feedback.sequence[1]))


        self._result.isOK = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------	 
if __name__ == '__main__':
    
    # self registration
    registration_file = loadRegistrationFile()
    uploadAAS(registration_file)
    
    # init server
    rospy.init_node('chasi')
    server1 = WaitForUserInput('WaitForUserInput')
    server2 = MoveToLocation('MoveToLocation')
    server3 = WaitForExternalEvent('WaitForExternalEvent')
    server4 = GraphicalUserInteraction('GraphicalUserInteraction')
    server5 = GetData('GetData')
    server6 = SetData('SetData')
    server7 = SendMessage('SendMessage')
    server8 = OnMessageReceive('OnMessageReceive')
    rospy.spin()
	
	
	

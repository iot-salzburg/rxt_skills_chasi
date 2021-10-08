#!/usr/bin/env python
import os, sys, time
import rospy
import actionlib
import rxt_skills_chasi.msg


# TODO import required CHASI libs here


#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_speak(sentence):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: listen
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_listen():
    
    print ('TODO: NOT YET IMPLEMENTED!')
    ret=b'TODO'
    return ret

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: move head
#------------------------------------------------------------------------------------------------------------------------------------------------------------ 
def chasi_movehead(position):
 
    print ('TODO: NOT YET IMPLEMENTED!')
    return True
    
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_facedetection(inputFace):
    
    print ('TODO: NOT YET IMPLEMENTED!')
    return True

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# helper function: speak
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def chasi_facialexpression(inputFace):

    print ('TODO: NOT YET IMPLEMENTED!')
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
    
    print ('TODO: NOT YET IMPLEMENTED!')
    ret=b'TODO'
    return ret
			
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
        returnMsg = chasi_listen()
          
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
        success = chasi_movehead(goal.location)
          
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
        success = chasi_facedetection(goal.inputText)
          
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
        success = chasi_facialexpression(goal.outputMessage)
          
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
          
        if success:
            self._result.isOK = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------	 
if __name__ == '__main__':
    
    rospy.init_node('chasi')
    server1 = WaitForUserInput('WaitForUserInput')
    server2 = MoveToLocation('MoveToLocation')
    server3 = WaitForExternalEvent('WaitForExternalEvent')
    server4 = GraphicalUserInteraction('GraphicalUserInteraction')
    server5 = GetData('GetData')
    server6 = SetData('SetData')
    rospy.spin()
	
	
	

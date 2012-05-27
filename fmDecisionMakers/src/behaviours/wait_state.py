import rospy
from smach import State

class WaitState(State):
    """
        Simple wait state which sleeps for \"duration\" seconds and the succeeds,
        currently it cannot be preempted. so long sleeps should be avoided. 
    """
    def __init__(self,duration):
        State.__init__(self,outcomes=['succeeded','aborted','preempted'])
        self.timer = duration
        
    def execute(self,userdata):
        rospy.sleep(self.timer)
        return "succeeded"
        


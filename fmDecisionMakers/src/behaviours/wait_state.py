import rospy
from smach import State

class WaitState(State):
    def __init__(self,duration):
        State.__init__(self,outcomes=['succeeded','aborted','preempted'])
        self.timer = duration
        
    def execute(self,userdata):
        rospy.sleep(self.timer)
        return "succeeded"
        


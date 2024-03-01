#!/usr/bin/env python
import sys
import rospy
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
import moveit_commander
import actionlib
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction, TtsGoal
from geometry_msgs.msg import PoseStamped

class arm_move:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.ac = actionlib.SimpleActionClient('/tts' , TtsAction)
        self.ac.wait_for_server()
        self.tts_goal = TtsActionGoal()
        self.tts_goal.goal.rawtext.text = 'Hi,_I_am_up'
        self.tts_goal.goal.rawtext.lang_id = 'en_GB'
        self.ac.send_goal(self.tts_goal.goal)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.reference_frame = 'arm_1_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.reference_frame
        self.object1_move_1 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object1_move_2 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object1_move_3 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object2_move_1 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object2_move_2 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object2_move_3 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object3_move_1 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object3_move_2 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]
        self.object3_move_3 = [0.27 , 0.30 , -1.47 , 1.63 , -0.70 , -0.11 , 0.94]

    def goto_obj1(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object1_move_1 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object1_move_2 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object1_move_3 , wait=True)

    def goto_obj2(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object2_move_1 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object2_move_2 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object2_move_3 , wait=True)

    def goto_obj3(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object3_move_1 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object3_move_2 , wait=True)
        rospy.sleep(5)
        self.arm.go(self.object3_move_3 , wait=True)

    def speak(self, params):
        """
        A function to speaking 
        
        """
        self.goal.goal.rawtext.text = params # 'Please_take_the_bottle_from_my_hand'
        self.ac.send_goal(self.goal.goal  )

arm = arm_move()

def speak(tosay):
    """Function to make the tiago speak arbitrary words

    Args:
        tosay (string): A string to say using the speaker
    """
    arm.speak(tosay)
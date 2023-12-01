#! /usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String, Bool, Int32
from idmind_tabletop_msgs.msg import TTSAction, TTSActionFeedback, TTSActionGoal, TTSActionResult, TTSGoal
from idmind_tabletop_msgs.msg import RoutineAction, RoutineActionFeedback, RoutineActionGoal, RoutineActionResult, RoutineGoal

class HRIHaruActionBridge():
    def __init__(self):
        # init
        rospy.init_node('hri_haru_action_bridge')

        # ros handles
        # haru
        self.ros_ac_tts = actionlib.SimpleActionClient('/idmind_tabletop/action_tts', TTSAction)
        self.ros_ac_routine = actionlib.SimpleActionClient('/idmind_tabletop/action_routine', RoutineAction)
        # external
        self.ros_sub_tts_goal = rospy.Subscriber('/hri_haru_action_bridge/tts/goal', String, callback=self.ros_cb_tts_goal)
        self.ros_sub_tts_cancel = rospy.Subscriber('/hri_haru_action_bridge/tts/cancel', String, callback=self.ros_cb_tts_cancel)
        self.ros_sub_routine_goal = rospy.Subscriber('/hri_haru_action_bridge/routine/goal', Int32, callback=self.ros_cb_routine_goal)

        # action server waits
        self.ros_ac_tts.wait_for_server()
        self.ros_ac_routine.wait_for_server()

        # spin
        rospy.spin()

    # haru interaction

    def tts_goal(self, say):
        print('Say:', say)
        goal = TTSGoal()
        goal.message = str(say)
        self.ros_ac_tts.send_goal(goal)

    def tts_cancel(self):
        print('Cancelled.')
        self.ros_ac_tts.cancel_goal()
        print(self.ros_ac_tts.get_goal_status_text())

    def routine_goal(self, number):
        print('Routine:', number)
        goal = RoutineGoal()
        goal.id = number
        self.ros_ac_routine.send_goal(goal)
        print(self.ros_ac_routine.get_goal_status_text())

    # callbacks
    def ros_cb_tts_goal(self, msg):
        self.tts_goal(msg.data)
    
    def ros_cb_tts_cancel(self, msg):
        self.tts_cancel()

    def ros_cb_routine_goal(self, msg):
        self.routine_goal(msg.data)

if __name__ == '__main__':
    hhab = HRIHaruActionBridge()
#! /usr/bin/env python3
import rospy
import actionlib
from flask import Flask, request
from flask_cors import CORS
from std_msgs.msg import String, Bool, Int32
from idmind_tabletop_msgs.msg import TTSAction, TTSActionFeedback, TTSActionGoal, TTSActionResult, TTSGoal
from idmind_tabletop_msgs.msg import RoutineAction, RoutineActionFeedback, RoutineActionGoal, RoutineActionResult, RoutineGoal

class HRIHaruActionBridge():
    def __init__(self):
        # init
        rospy.init_node('hri_haru_action_bridge', disable_signals=True)

        # ros handles
        self.ros_ac_tts = actionlib.SimpleActionClient('/idmind_tabletop/action_tts', TTSAction)
        self.ros_ac_routine = actionlib.SimpleActionClient('/idmind_tabletop/action_routine', RoutineAction)

        # print('Waiting for ROS action servers...')
        # action server waits
        # self.ros_ac_tts.wait_for_server()
        # self.ros_ac_routine.wait_for_server()

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
    def zenoh_cb_tts_goal(self, msg):
        self.tts_goal(msg.payload.decode('utf-8'))
    
    def zenoh_cb_tts_cancel(self, msg):
        self.tts_cancel()

    def zenoh_cb_routine_goal(self, msg):
        number = None
        try:
            number = int(msg.payload.decode('utf-8'))
        except ValueError:
            print('Invalid message received. Expected integer.')
            return
        self.routine_goal(msg.payload.decode('utf-8'))

if __name__ == '__main__':
    hhab = HRIHaruActionBridge()

    app = Flask(__name__)
    CORS(app)

    print('Starting Flask...')
    @app.route('/hri_haru_action_bridge/tts/goal', methods = ['POST'])
    def tts_goal_handler():
        command = request.get_json()
        print('[/hri_haru_action_bridge/tts/goal]', command)
        hhab.tts_goal(command['data'])
        return 'OK'
    
    @app.route('/hri_haru_action_bridge/tts/cancel', methods = ['POST'])
    def tts_cancel_handler():
        command = request.get_json()
        print('/hri_haru_action_bridge/tts/cancel', command)
        hhab.tts_cancel()
        return 'OK'
    
    @app.route('/hri_haru_action_bridge/routine/goal', methods = ['POST'])
    def routine_goal():
        command = request.get_json()
        print('/hri_haru_action_bridge/routine/goal', command)
        routine = None
        try:
            routine = int(command['data'])
        except ValueError:
            print('Invalid message received, expected integer.')
        hhab.routine_goal(routine)
        return 'OK'

    app.run(host='0.0.0.0', port = 5000)
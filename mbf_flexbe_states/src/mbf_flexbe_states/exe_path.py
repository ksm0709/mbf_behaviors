#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathGoal, ExePathResult)


class ExePathActionState(EventState):

    def __init__(self, controller):
        super(ExePathActionState, self).__init__(output_keys = ['outcome','final_pose','dist_to_goal','angle_to_goal'],
                                                input_keys = ['path'],
                                                outcomes = ['succeeded','failed','aborted'])

        self._controller = controller

        # Create the action client when building the behavior.
        self._topic = '/move_base_flex/exe_path'
        self._client = ProxyActionClient({self._topic: ExePathAction}) # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            # Set output keys
            userdata.outcome = {'origin': 'exe_path', 'outcome': result.outcome}
            userdata.final_pose = result.final_pose
            userdata.dist_to_goal = result.dist_to_goal
            userdata.angle_to_goal = result.angle_to_goal

            # Send result log
            Logger.loginfo(result.message)

            # Based on the result, decide which outcome to trigger.
            if result.outcome == ExePathResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == ExePathResult.FAILURE:
                return 'failed'
            elif result.outcome == ExePathResult.BLOCKED_PATH:
                return 'failed'
            elif result.outcome == ExePathResult.COLLISION:
                return 'failed'
            elif result.outcome == ExePathResult.CANCELED:
                return 'aborted'
            elif result.outcome == ExePathResult.INTERNAL_ERROR:
                return 'aborted'
            elif result.outcome == ExePathResult.INVALID_PATH:
                return 'aborted'
            elif result.outcome == ExePathResult.INVALID_PLUGIN:
                return 'aborted'
            elif result.outcome == ExePathResult.MISSED_GOAL:
                return 'failed'
            elif result.outcome == ExePathResult.MISSED_PATH:
                return 'failed'
            elif result.outcome == ExePathResult.NO_VALID_CMD:
                return 'failed'
            elif result.outcome == ExePathResult.NOT_INITIALIZED:
                return 'aborted'
            elif result.outcome == ExePathResult.OSCILLATION:
                return 'failed'
            elif result.outcome == ExePathResult.PAT_EXCEEDED:
                return 'failed'
            elif result.outcome == ExePathResult.ROBOT_STUCK:
                return 'failed'
            elif result.outcome == ExePathResult.TF_ERROR:
                return 'aborted'
            else:
                return 'aborted'

            # If the action has not yet finished, no outcome will be returned and the state stays active.


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = ExePathGoal()
        goal.path = userdata.path
        goal.controller = self._controller #'dwa_controller'

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
        Logger.loginfo('Cancelled active action goal.')

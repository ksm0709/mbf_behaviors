#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathGoal
from mbf_msgs.msg import GetPathFeedback
from mbf_msgs.msg import GetPathResult

class GetPathActionState(EventState):

    def __init__(self, planner, tolerance):
        super(GetPathActionState, self).__init__(output_keys = ['outcome','path'],
                                                input_keys = ['target_pose'],
                                                outcomes = ['succeeded','failed','aborted'])
        
        self._planner = planner
        self._tolerance = tolerance
        
        # Create the action client when building the behavior.
        self._topic = '/move_base_flex/get_path'
        self._client = ProxyActionClient({self._topic: GetPathAction}) # pass required clients as dict (topic: type)
        
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
            userdata.outcome = {'origin': 'get_path',
                                'outcome': result.outcome}
            userdata.path = result.path

            # Send result log
            Logger.loginfo(result.message)

            # Based on the result, decide which outcome to trigger.
            if result.outcome == GetPathResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == GetPathResult.FAILURE:
                return 'failed'
            elif result.outcome == GetPathResult.NO_PATH_FOUND:
                return 'failed'
            elif result.outcome == GetPathResult.INVALID_GOAL:
                return 'failed'
            elif result.outcome == GetPathResult.INVALID_PLUGIN:
                return 'aborted'
            elif result.outcome == GetPathResult.INVALID_START:
                return 'aborted'
            elif result.outcome == GetPathResult.EMPTY_PATH:
                return 'aborted'
            elif result.outcome == GetPathResult.INTERNAL_ERROR:
                return 'aborted'
            elif result.outcome == GetPathResult.NOT_INITIALIZED:
                return 'aborted'
            elif result.outcome == GetPathResult.PAT_EXCEEDED:
                return 'failed'
            elif result.outcome == GetPathResult.STOPPED:
                return 'aborted'
            else:
                return 'aborted'
            # If the action has not yet finished, no outcome will be returned and the state stays active.


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = GetPathGoal()
        goal.use_start_pose = False
        goal.tolerance = self._tolerance #0.2
        goal.target_pose = userdata.target_pose
        goal.planner = self._planner#'global_planner'

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send goal:\n%s' % str(e))
            self._error = True

        if not self._error :
            Logger.loginfo('Sending goal succeeded')


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
        Logger.loginfo('Cancelled active action goal.')


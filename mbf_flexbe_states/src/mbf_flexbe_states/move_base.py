#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from mbf_msgs.msg import MoveBaseAction
from mbf_msgs.msg import MoveBaseGoal
from mbf_msgs.msg import MoveBaseFeedback
from mbf_msgs.msg import MoveBaseResult

class MoveBaseActionState(EventState):

    def __init__(self, planner, tolerance = 0.2, controller, recovery_behaviors = []):
        super(MoveBaseActionState, self).__init__(output_keys = ['outcome','final_pose','dist_to_goal','angle_to_goal'],
                                                input_keys = ['target_pose'],
                                                outcomes = ['succeeded','failed','aborted'])

        self._planner = planner
        self._tolerance = tolerance
        self._controller = controller
        self._recovery_behaviors = recovery_behaviors

        # Create the action client when building the behavior.
        self._topic = '/move_base_flex/move_base'
        self._client = ProxyActionClient({self._topic: MoveBaseAction}) # pass required clients as dict (topic: type)

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
            userdata.outcome = {'origin': 'move_base',
                    'outcome': result.outcome}
            userdata.final_pose = result.final_pose
            userdata.dist_to_goal = result.dist_to_goal
            userdata.angle_to_goal = result.angle_to_goal

            # Send result log
            Logger.loginfo(result.message)

            # Based on the result, decide which outcome to trigger.
            if result.outcome == MoveBaseResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == MoveBaseResult.FAILURE:
                return 'failed'
            elif result.outcome == MoveBaseResult.CANCELED:
                return 'aborted'
            elif result.outcome == MoveBaseResult.COLLISION:
                return 'failed'
            elif result.outcome == MoveBaseResult.OSCILLATION:
                return 'failed'
            elif result.outcome == MoveBaseResult.START_BLOCKED:
                return 'failed'
            elif result.outcome == MoveBaseResult.GOAL_BLOCKED:
                return 'failed'
            elif result.outcome == MoveBaseResult.TF_ERROR:
                return 'aborted'
            elif result.outcome == MoveBaseResult.INTERNAL_ERROR:
                return 'aborted'
            elif result.outcome == MoveBaseResult.PLAN_FAILURE:
                return 'failed'
            elif result.outcome == MoveBaseResult.CTRL_FAILURE:
                return 'failed'
            else:
                return 'aborted'

            # If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = MoveBaseGoal()
        goal.tolerance = self._tolerance
        goal.target_pose = userdata.target_pose
        goal.planner = self._planner
        goal.controller = self._controller
        goal.recovery_behaviors = self._recovery_behaviors

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


#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryGoal
from mbf_msgs.msg import RecoveryFeedback
from mbf_msgs.msg import RecoveryResult

class RecoveryActionState(EventState):

    def __init__(self, behavior):
        super(RecoveryActionState, self).__init__(output_keys = ['outcome'],
                                                    outcomes = ['done'])

        self._behavior = behavior

        # Create the action client when building the behavior.
        self._topic = '/move_base_flex/recovery'
        self._client = ProxyActionClient({self._topic: RecoveryAction}) # pass required clients as dict (topic: type)

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
            userdata.outcome = {'origin': behavior,
                    'outcome': result.outcome}

            # Send result log
            Logger.loginfo(result.message)

            # Based on the result, decide which outcome to trigger.
            if result.outcome == RecoveryResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == RecoveryResult.FAILURE:
                return 'failed'
            elif result.outcome == RecoveryResult.CANCELED:
                return 'aborted'
            elif result.outcome == RecoveryResult.PAT_EXCEEDED:
                return 'failed'
            elif result.outcome == RecoveryResult.INVALID_NAME:
                return 'aborted'
            elif result.outcome == RecoveryResult.INTERNAL_ERROR:
                return 'aborted'
            else:
                return 'aborted'

            # If the action has not yet finished, no outcome will be returned and the state stays active.


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = RecoveryGoal()
        goal.behavior = self._behavior

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



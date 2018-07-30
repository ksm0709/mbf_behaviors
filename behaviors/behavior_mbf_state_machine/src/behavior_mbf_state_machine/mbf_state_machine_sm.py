#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_mbf_state_machine')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.subscriber_state import SubscriberState
from mbf_flexbe_states.exe_path import ExePathActionState
from mbf_flexbe_states.recovery import RecoveryActionState
from mbf_flexbe_states.get_path import GetPathActionState
from mbf_flexbe_states.recovery_manage import RecoveryManageState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 26 2018
@author: taehokang
'''
class mbfstatemachineSM(Behavior):
	'''
	move base flex state macine
	'''


	def __init__(self):
		super(mbfstatemachineSM, self).__init__()
		self.name = 'mbf state machine'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:113 y:469
		_state_machine = OperatableStateMachine(outcomes=['terminate'])
		_state_machine.userdata.outcome = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:253 y:27
			OperatableStateMachine.add('WAIT_FOR_GOAL',
										SubscriberState(topic="/move_base_simple/goal", blocking=True, clear=False),
										transitions={'received': 'CLEAR_BEFORE_PLANNING', 'unavailable': 'WAIT_FOR_GOAL'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'target_pose'})

			# x:467 y:164
			OperatableStateMachine.add('EXE_PATH',
										ExePathActionState(controller="dwa_controller"),
										transitions={'succeeded': 'WAIT_FOR_GOAL', 'failed': 'RECOVERY_MANAGE', 'aborted': 'WAIT_FOR_GOAL'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'path': 'path', 'outcome': 'outcome', 'final_pose': 'final_pose', 'dist_to_goal': 'dist_to_goal', 'angle_to_goal': 'angle_to_goal'})

			# x:1047 y:109
			OperatableStateMachine.add('CLEAR_COSTMAP',
										RecoveryActionState(behavior="clear_costmap"),
										transitions={'done': 'RECOVERY_MANAGE'},
										autonomy={'done': Autonomy.Off},
										remapping={'outcome': 'outcome'})

			# x:1049 y:378
			OperatableStateMachine.add('ROTATE_RECOVERY',
										RecoveryActionState(behavior="rotate_recovery"),
										transitions={'done': 'RECOVERY_MANAGE'},
										autonomy={'done': Autonomy.Off},
										remapping={'outcome': 'outcome'})

			# x:251 y:317
			OperatableStateMachine.add('GET_PATH',
										GetPathActionState(planner="global_planner", tolerance=0.2),
										transitions={'succeeded': 'EXE_PATH', 'failed': 'RECOVERY_MANAGE', 'aborted': 'WAIT_FOR_GOAL'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'target_pose': 'target_pose', 'outcome': 'outcome', 'path': 'path'})

			# x:851 y:173
			OperatableStateMachine.add('RECOVERY_MANAGE',
										RecoveryManageState(),
										transitions={'case_a': 'CLEAR_COSTMAP', 'case_b': 'ROTATE_RECOVERY', 'succeeded': 'GET_PATH', 'aborted': 'WAIT_FOR_GOAL'},
										autonomy={'case_a': Autonomy.Off, 'case_b': Autonomy.Off, 'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'outcome': 'outcome'})

			# x:43 y:173
			OperatableStateMachine.add('CLEAR_BEFORE_PLANNING',
										RecoveryActionState(behavior="clear_costmap"),
										transitions={'done': 'WAIT_CLEAR'},
										autonomy={'done': Autonomy.Off},
										remapping={'outcome': 'outcome'})

			# x:99 y:280
			OperatableStateMachine.add('WAIT_CLEAR',
										WaitState(wait_time=1.0),
										transitions={'done': 'GET_PATH'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

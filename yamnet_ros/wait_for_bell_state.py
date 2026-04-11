#!/usr/bin/env python3
"""
WaitForBellState — reusable SMACH state for yamnet_ros.

Activates the yamnet_ros LifecycleNode, waits until a bell is detected
(or timeout), then deactivates the node before returning.

Outcomes:
    'detected'  — bell was heard within timeout
    'timeout'   — no bell before timeout_sec elapsed
    'error'     — action server not available or lifecycle failed

Usage in a SMACH state machine:
    smach.StateMachine.add(
        'WAIT_FOR_BELL',
        WaitForBellState(node, timeout_sec=30.0),
        transitions={
            'detected': 'DO_SOMETHING',
            'timeout':  'ABORT',
            'error':    'ABORT',
        }
    )
"""

import time
import smach
import rclpy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from sobits_interfaces.action import ListenForSound


# Lifecycle transition IDs
_ACTIVATE   = Transition.TRANSITION_ACTIVATE
_DEACTIVATE = Transition.TRANSITION_DEACTIVATE


class WaitForBellState(smach.State):

    def __init__(self, node: rclpy.node.Node, timeout_sec: float = 30.0,
                 threshold: float = 0.0):
        """
        Args:
            node:        your rclpy Node instance
            timeout_sec: how long to listen (0 = no timeout)
            threshold:   YAMNet score threshold (0 = use node's config default)
        """
        super().__init__(outcomes=['detected', 'timeout', 'error'])
        self._node        = node
        self._timeout_sec = timeout_sec
        self._threshold   = threshold

        # Lifecycle services
        self._change_state = node.create_client(
            ChangeState, '/yamnet_ros/change_state'
        )
        # Action client
        self._action_client = ActionClient(
            node, ListenForSound, '/yamnet_ros/listen_for_sound'
        )

    # ------------------------------------------------------------------ #

    def execute(self, userdata):
        # 1. Activate the node
        if not self._set_lifecycle(_ACTIVATE):
            self._node.get_logger().error('WaitForBellState: failed to activate yamnet_ros')
            return 'error'

        self._node.get_logger().info(
            f'WaitForBellState: listening for bell  (timeout={self._timeout_sec}s)'
        )

        # 2. Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error('WaitForBellState: action server not available')
            self._set_lifecycle(_DEACTIVATE)
            return 'error'

        # 3. Send goal and wait for result
        goal = ListenForSound.Goal()
        goal.timeout_sec = float(self._timeout_sec)
        goal.threshold   = float(self._threshold)

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self._node, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self._node.get_logger().error('WaitForBellState: goal rejected')
            self._set_lifecycle(_DEACTIVATE)
            return 'error'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result

        # 4. Deactivate before returning (node sits idle until next call)
        self._set_lifecycle(_DEACTIVATE)

        if result.detected:
            self._node.get_logger().info(
                f'WaitForBellState: bell detected!  '
                f'label="{result.label}"  score={result.score:.3f}  '
                f'elapsed={result.elapsed_time:.1f}s'
            )
            return 'detected'
        else:
            self._node.get_logger().info(
                f'WaitForBellState: timed out after {result.elapsed_time:.1f}s'
            )
            return 'timeout'

    # ------------------------------------------------------------------ #

    def _set_lifecycle(self, transition_id: int) -> bool:
        if not self._change_state.wait_for_service(timeout_sec=3.0):
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._change_state.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result() is not None and future.result().success

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._node.get_logger().debug(
            f'  listening... top="{fb.current_top_label}" ({fb.current_top_score:.3f})'
            f'  candidate={fb.candidate_detected}'
        )

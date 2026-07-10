#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class LifecycleManager(Node):

    def __init__(self):
        super().__init__('lifecycle_manager')

        self.node_name = '/screw_inspection'

        self.client = self.create_client(
            ChangeState,
            f'{self.node_name}/change_state'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for lifecycle service...")

        self.step = 0

        # start sequence
        self.timer = self.create_timer(2.0, self.run_sequence)

    # -------------------------
    # NON-BLOCKING REQUEST
    # -------------------------
    def send_request(self, transition_id, callback):

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = self.client.call_async(req)
        future.add_done_callback(callback)

    # -------------------------
    # RESPONSE HANDLER
    # -------------------------
    def response_callback(self, future):

        try:
            result = future.result()

            if result.success:
                self.get_logger().info("Transition SUCCESS")
            else:
                self.get_logger().error("Transition FAILED")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    # -------------------------
    # SEQUENCE CONTROL
    # -------------------------
    def run_sequence(self):

        if self.step == 0:
            self.get_logger().info("CONFIGURING NODE")

            self.send_request(
                Transition.TRANSITION_CONFIGURE,
                self.response_callback
            )

            self.step += 1

        elif self.step == 1:
            self.get_logger().info("ACTIVATING NODE")

            self.send_request(
                Transition.TRANSITION_ACTIVATE,
                self.response_callback
            )

            self.step += 1

        elif self.step == 2:
            self.get_logger().info("NODE ACTIVE - inspection running")
            self.step += 1

        # elif self.step == 3:
        #     self.get_logger().info("DEACTIVATING NODE")

        #     self.send_request(
        #         Transition.TRANSITION_DEACTIVATE,
        #         self.response_callback
        #     )

        #     self.step += 1

        else:
            self.get_logger().info("DONE")
            self.timer.cancel()


def main():

    rclpy.init()

    node = LifecycleManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
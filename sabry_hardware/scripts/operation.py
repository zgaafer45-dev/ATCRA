#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sabry_hardware.action import ChangeTool
from moveit_msgs.action import MoveGroup

from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume
)

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # <-- FIXED: This registers PointStamped with tf2

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Int32


class Operation(Node):

    def __init__(self):

        super().__init__("operation_manager")

        # Tool changer action
        self.tool_action_client = ActionClient(self, ChangeTool, "change_tool")
        while not self.tool_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for change_tool action...")

        # MoveIt action
        self.move_action_client = ActionClient(self, MoveGroup, "/move_action")
        while not self.move_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for MoveIt action server...")

        self.get_logger().info("Operation Manager ready")

        self.tool_name = None
        self.move_sequence = []
        self.current_move = 0
        self.tools = ["gripper", "screwdriver", "gripper"]
        self.current_tool_index = 0

        self.max_move_retries = 3
        self.current_retry = 0
        self.last_goal_pose = None

        self.workpiece_position = None
        self.last_received_position = None
        self.operation_running = False

        # Named staging / final placement positions for the gripper
        self.staging_position = (0.2, -0.200, 0.4)
        self.final_position   = (0.2, -0.1, 0.4)

        # Hole-count state from the inspection node — used to decide
        # whether the part needs drilling or can go straight to placement
        self.target_holes = None
        self.actual_holes = None

        self.workpiece_sub = self.create_subscription(
            Point,
            "workpiece_coordinates",
            self.workpiece_callback,
            10
        )

        self.target_holes_sub = self.create_subscription(
            Int32,
            "target_holes_count",
            self.target_holes_callback,
            10
        )

        self.actual_holes_sub = self.create_subscription(
            Int32,
            "actual_holes_count",
            self.actual_holes_callback,
            10
        )

        self.drill_pub = self.create_publisher(
            Float64MultiArray,
            "/drill_controller/commands",
            10
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.operation_sequence()

    def workpiece_callback(self, msg):
        

        point_cam = PointStamped()

        point_cam.header.frame_id = "camera_link"
        point_cam.header.stamp = self.get_clock().now().to_msg()

        point_cam.point.x = msg.x
        point_cam.point.y = msg.y
        point_cam.point.z = msg.z

        try:

            point_base = self.tf_buffer.transform(
                point_cam,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = point_base.point.x
            y = point_base.point.y
            z = point_base.point.z

        except Exception as e:

            self.get_logger().warn(
                f"Transform failed: {e}"
            )
            return

        new_position = (
            round(x, 4),
            round(y, 4),
            round(z, 4)
        )

        # Ignore updates while processing
        if self.operation_running:
            return

        if new_position == self.last_received_position:
            return

        self.last_received_position = new_position

        self.workpiece_position = new_position

        self.get_logger().info(
            f"New workpiece detected: {x:.3f}, {y:.3f}, {z:.3f}"
        )

        self.current_tool_index = 0
        self.tool_name = None
        self.current_move = 0

        # Decide whether this part needs drilling based on the latest
        # hole-count readout from the inspection node.
        if self.target_holes is not None and self.actual_holes is not None \
                and self.target_holes == self.actual_holes:
            self.get_logger().info(
                f"Holes match ({self.actual_holes}/{self.target_holes}) — "
                f"gripper will move workpiece directly to final position."
            )
            self.tools = ["gripper"]
        else:
            self.get_logger().info(
                f"Holes mismatch or unknown (actual={self.actual_holes}, "
                f"target={self.target_holes}) — running full drill sequence."
            )
            self.tools = ["gripper", "screwdriver", "gripper"]

        # Consume the readout so a stale value can't leak into the next part
        self.target_holes = None
        self.actual_holes = None

        self.operation_sequence()

    # -------------------------------------------------
    # HOLE-COUNT CALLBACKS
    # -------------------------------------------------

    def target_holes_callback(self, msg):
        self.target_holes = msg.data
        self.get_logger().info(f"Target holes received: {self.target_holes}")

    def actual_holes_callback(self, msg):
        self.actual_holes = msg.data
        self.get_logger().info(f"Actual holes received: {self.actual_holes}")

    # -------------------------------------------------
    # OPERATION SEQUENCE
    # -------------------------------------------------

    def operation_sequence(self):

        if self.current_tool_index >= len(self.tools):
            self.get_logger().info("All tools processed")

            self.operation_running = False
            self.current_tool_index = 0
            self.tool_name = None

            return

        tool = self.tools[self.current_tool_index]

        if tool == "gripper" and self.workpiece_position is None:
            self.get_logger().warn(
                "Waiting for workpiece coordinates..."
            )
            self.operation_running = False
            return

        self.operation_running = True

        self.attach_tool(tool)

    # -------------------------------------------------
    # TOOL ATTACH
    # -------------------------------------------------

    def attach_tool(self, tool):

        goal_msg = ChangeTool.Goal()
        goal_msg.tool_name = tool

        self.tool_name = tool
        self.get_logger().info(f"Attaching tool: {tool}")

        future = self.tool_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.tool_feedback_cb
        )

        future.add_done_callback(self.attach_goal_response_cb)

    def attach_goal_response_cb(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Attach goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.attach_result_cb)

    def attach_result_cb(self, future):

        result = future.result().result

        if not result.success:
            self.get_logger().error("Tool attach failed - retrying")
            self.attach_tool(self.tool_name)
            return

        self.get_logger().info(f"{self.tool_name} attached successfully")

        # Load motion sequence for the tool
        self.load_tool_sequence()

        self.execute_next_move()

    # -------------------------------------------------
    # TOOL DETACH
    # -------------------------------------------------

    def detach_tool(self):

        goal_msg = ChangeTool.Goal()
        goal_msg.tool_name = "none"

        self.get_logger().info("Detaching tool")

        future = self.tool_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.tool_feedback_cb
        )

        future.add_done_callback(self.detach_goal_response_cb)

    def detach_goal_response_cb(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Detach goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.detach_result_cb)

    def detach_result_cb(self, future):

        result = future.result().result

        if result.success:
            self.get_logger().info(f"{self.tool_name} detached successfully")
        else:
            self.get_logger().error("Detach failed - retrying")
            self.detach_tool()
            return

        # Move to next tool
        self.current_tool_index += 1

        self.operation_sequence()

    # -------------------------------------------------
    # TOOL FEEDBACK
    # -------------------------------------------------

    def tool_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[Tool] {feedback.current_state}")

    # -------------------------------------------------
    # TOOL MOTION SEQUENCES
    # -------------------------------------------------

    def load_tool_sequence(self):

        if self.tool_name == "gripper" and self.current_tool_index == 0:

            x, y, z = self.workpiece_position

            if len(self.tools) == 1:
                # Holes already match — pick up and place directly
                # at the final position, no drilling needed.
                self.move_sequence = [
                    (x, y, z + 0.175),
                    self.final_position,
                ]
            else:
                # Holes mismatch — drop the part at the staging
                # position so the screwdriver can drill it.
                self.move_sequence = [
                    (x, y, z + 0.175),
                    self.staging_position,
                ]

        elif self.tool_name == "screwdriver":
            self.move_sequence = [
                self.staging_position,
                # (0.281, -0.110, 0.332),
                # (0.346, -0.136, 0.288),
                # (0.296, -0.231, 0.295),
                # (0.218, -0.181, 0.301),
            ]

        elif self.tool_name == "gripper" and self.current_tool_index == 2:

            self.move_sequence = [
                self.staging_position,
                self.final_position,
            ]


        self.current_move = 0

    # -------------------------------------------------
    # EXECUTE MOVES
    # -------------------------------------------------
    def execute_next_move(self):

        if self.current_move >= len(self.move_sequence):
            self.get_logger().info("All moves complete")
            self.detach_tool()
            return

        x, y, z = self.move_sequence[self.current_move]

        self.get_logger().info(
            f"Executing move {self.current_move+1}/{len(self.move_sequence)}"
        )

        self.move_tool(x, y, z)

        self.current_move += 1

    def set_drill_velocity(self, velocity):

        msg = Float64MultiArray()
        msg.data = [velocity]

        self.drill_pub.publish(msg)

        self.get_logger().info(
            f"Drill velocity command: {velocity}"
        )

    def stop_drill(self):

        self.set_drill_velocity(1.57)

        self.drill_timer.cancel()

        self.execute_next_move()

    def control_gripper(self, pose_name):

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = "gripper"
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.8

        # Named target (open / close)
        goal_msg.request.goal_constraints.append(
            self.create_named_target_constraints(pose_name)
        )

        self.get_logger().info(f"Gripper action: {pose_name}")

        future = self.move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.gripper_goal_response_cb)

    def gripper_goal_response_cb(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_cb)


    def gripper_result_cb(self, future):

        self.get_logger().info("Gripper action completed")

        # Continue motion sequence after gripper action
        self.execute_next_move()

    # -------------------------------------------------
    # MOVE TOOL USING MOVEIT
    # -------------------------------------------------
    def move_tool(self, x, y, z):

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = "arm"
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.9

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        # Save pose for retry
        self.last_goal_pose = pose


        goal_msg.request.goal_constraints.append(
            self.create_constraints(pose)
        )

        self.get_logger().info(f"Moving to {x:.2f}, {y:.2f}, {z:.2f}")

        future = self.move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.move_goal_response_cb)

    def create_constraints(self, pose):

        constraint = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = "tool_mount_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(pose.pose)

        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        constraint.position_constraints.append(pos_constraint)

        # Orientation constraint (important!)
        ori = OrientationConstraint()
        ori.header.frame_id = pose.header.frame_id
        ori.link_name = "tool_mount_link"

        ori.orientation = pose.pose.orientation

        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1

        ori.weight = 1.0

        constraint.orientation_constraints.append(ori)

        return constraint
    
    def create_named_target_constraints(self, name):

        # The raw MoveGroup action has no way to resolve a named SRDF group
        # state from Constraints.name alone — that lookup normally happens
        # inside MoveGroupInterface.set_named_target(). Since we talk to the
        # action server directly, we build the joint constraint ourselves
        # using the joint values from the "gripper" group states in sabry.srdf.
        gripper_joint_positions = {
            "open": 0.0,
            "closed": 0.9,
        }

        constraint = Constraints()
        constraint.name = name

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "left_tool_joint"
        joint_constraint.position = gripper_joint_positions[name]
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0

        constraint.joint_constraints.append(joint_constraint)

        return constraint

    # -------------------------------------------------
    # MOVE CALLBACKS
    # -------------------------------------------------
    def move_goal_response_cb(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            self.retry_move()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_cb)

    def move_result_cb(self, future):

        result = future.result().result
        status = future.result().status

        if status != 4:   # 4 = SUCCEEDED
            self.get_logger().warn("Move failed")
            self.retry_move()
            return

        # Success → reset retry counter
        self.current_retry = 0

        self.get_logger().info("Move completed")
        if self.tool_name == "gripper":

            # First gripper: pick
            if self.current_tool_index == 0:

                if self.current_move == 1:
                    self.control_gripper("closed")
                    return

                elif self.current_move == 2:
                    self.control_gripper("open")
                    return

            # Second gripper: release
            elif self.current_tool_index == 2:

                if self.current_move == 1:
                    self.control_gripper("closed")
                    return

                elif self.current_move == 2:
                    self.control_gripper("open")
                    return
            
        if self.tool_name == "screwdriver":

            # Finished moving to screw position
            if self.current_move == 1:

                self.set_drill_velocity(3.14)

                # stop after 5 seconds
                self.drill_timer = self.create_timer(
                        5.0,
                        self.stop_drill
                    )
                return
        self.execute_next_move()

    def retry_move(self):

        if self.current_retry >= self.max_move_retries:
            self.get_logger().error("Move failed after retries. Skipping move.")
            self.current_retry = 0
            self.execute_next_move()
            return

        self.current_retry += 1

        self.get_logger().warn(
            f"Retrying motion planning ({self.current_retry}/{self.max_move_retries})"
        )

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.8

        goal_msg.request.goal_constraints.append(
            self.create_constraints(self.last_goal_pose)
        )

        future = self.move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.move_goal_response_cb)

# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = Operation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
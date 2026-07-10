#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, AllowedCollisionEntry
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, CollisionObject, AttachedCollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point, Pose
from sabry_hardware.srv import ChangeTool, LinearMotor
from sabry_hardware.action import ChangeTool
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
import trimesh
from shape_msgs.msg import Mesh, MeshTriangle
import os
from ament_index_python.packages import get_package_share_directory
# from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from moveit_msgs.srv import GetCartesianPath
import time

class ToolChangeManager(Node):

    def __init__(self):
        super().__init__("tool_change_manager")

        # Action server
        self._action_server = ActionServer(
            self,
            ChangeTool,
            "change_tool",
            self.execute_callback
        )

        # Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_scene_client = self.create_client(GetPlanningScene, "get_planning_scene")
        self.tool_client = self.create_client(LinearMotor, "tool_changer/set_state")
        # self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        # self.spawn_client = self.create_client(SpawnEntity, "/world/empty/create")
        # self.delete_client = self.create_client(DeleteEntity, "/world/empty/remove")

        # Wait for dependencies
        self.move_client.wait_for_server()
        self.scene_client.wait_for_service()
        self.tool_client.wait_for_service()
        # self.cartesian_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_scene_client.wait_for_service()

    

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.tools = {
        "gripper": {
            "mount_frame": "left_tool",
            "mesh": "Gripper.STL",
            # "relative_pose": [0.0, -0.04, 0.01]
        },

        "screwdriver": {
            "mount_frame": "mid_tool",
            "mesh": "Gripper.STL",
            # "relative_pose": [0.0, -0.04, 0.01]
        },

        "camera": {
            "mount_frame": "right_tool",
            "mesh": "Gripper.STL",
            # "relative_pose": [0.0, -0.04, 0.01]
        }
    }
        # These mirror your SRDF exactly

   
        self.NAMED_STATES = {
            
            "gripper_dock": {
                "waist_joint":       -1.9802505693127666,
                "upperarm_joint":     0.6609561877302522,
                "lowerarm_joint":    -0.616450291804397,
                "wrist_pitch_joint":  0.07668104068637063,
                "wrist_yaw_joint":    0.31910072880368845

            },
          
            "gripper_approach": {
                "waist_joint":       -1.978679772985972,
                "upperarm_joint":     0.6763150851478027,
                "lowerarm_joint":    -0.8836602002847289,
                "wrist_pitch_joint":  -0.1784162827851205,
                "wrist_yaw_joint":    0.31360294165990654,
                   
            },
            "gripper_lift": {
                "waist_joint":        -2.2668336324902354,
                "upperarm_joint":     0.01884955592153848,
                "lowerarm_joint":      0.3038618227722122,
                "wrist_pitch_joint":  0.36771469345642727,
                "wrist_yaw_joint":    0.6859253043415968,

            },
 
            # ── SCREWDRIVER ───────────────────────────────────────────────────────────
            "screwdriver_dock": {
                "waist_joint":        -1.6460200175558526,
                "upperarm_joint":      0.5290092962794813,
                "lowerarm_joint":     -0.39828413530510587,
                "wrist_pitch_joint":   0.14298609896963557,
                "wrist_yaw_joint":     -0.00022907446432425364,

            },
            "screwdriver_lift": {
                "waist_joint":        -1.6489870772842428,
                "upperarm_joint":      0.018151424220740887,
                "lowerarm_joint":      0.25918139392115735,
                "wrist_pitch_joint":   0.2890934284182546,
                "wrist_yaw_joint":    -0.0026725354171163694,

            },

            "screwdriver_approach": {
                "waist_joint":        -1.64602001755585267,
                "upperarm_joint":      0.525475318388725,
                "lowerarm_joint":     -0.6572769461936134,
                "wrist_pitch_joint":  -0.11946802663916199,
                "wrist_yaw_joint":     -0.00022907446432425364,
          
            },

            # ── CAMERA ───────────────────────────────────────────────────────────────
            "camera_dock": {
                "waist_joint":        -1.2777281295314393,
                "upperarm_joint":      0.6280531557329712,
                "lowerarm_joint":     -0.4850790537077438,
                "wrist_pitch_joint":   0.14312729548081135,
                "wrist_yaw_joint":     1.2777839001441273,
            
            },
            "camera_lift": {
                "waist_joint":         -1.1015969424780463,
                "upperarm_joint":      0.06421635532830548,
                "lowerarm_joint":      0.2734665962382407,
                "wrist_pitch_joint":   0.3378270075511886,
                "wrist_yaw_joint":     1.101730894814258,
             
            },
            "camera_approach": {
                "waist_joint":        -1.2776741588914893,
                "upperarm_joint":      0.6377796782586148,
                "lowerarm_joint":     -0.8290646213875537,
                "wrist_pitch_joint":  -0.19101631527870563,
                "wrist_yaw_joint":    1.2776537130087449,

              
            },
        }

        # State
        self.state = "IDLE"

        self.current_tool = None
        self.target_tool = None

        self.goal_handle = None
        self._result = None
        self.goal_active = False

        self.lift_waypoints = []
        self.current_waypoint = 0

        self.get_logger().info("ToolChangeManager ready")
        self.spawn_tools_in_rack()

    def allow_collision(self, object_name, links):

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        ps = future.result().scene
        acm = ps.allowed_collision_matrix
        ps.is_diff = True

        # Ensure names exist
        for name in [object_name] + links:
            if name not in acm.entry_names:
                acm.entry_names.append(name)

        size = len(acm.entry_names)

        # Resize rows
        while len(acm.entry_values) < size:
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * size
            acm.entry_values.append(entry)

        for row in acm.entry_values:
            while len(row.enabled) < size:
                row.enabled.append(False)

        # Enable collisions
        indices = [acm.entry_names.index(n) for n in [object_name] + links]

        for i in indices:
            for j in indices:
                acm.entry_values[i].enabled[j] = True
                acm.entry_values[j].enabled[i] = True

        req = ApplyPlanningScene.Request()
        req.scene = ps

        self.scene_client.call_async(req)
        time.sleep(3.0)  # Allow some time for the scene to update

    def spawn_tools_in_rack(self):

        ps = PlanningScene()
        ps.is_diff = True

        pkg_path = get_package_share_directory("sabry")

        for name, tool in self.tools.items():

            co = CollisionObject()
            co.id = name
            co.header.frame_id = tool["mount_frame"]

            mesh_path = os.path.join(pkg_path, "meshes", tool["mesh"])
            mesh = self.create_mesh(mesh_path)

            co.meshes.append(mesh)

            pose = Pose()
            pose.position.x = -0.045
            pose.position.y = -0.017
            pose.position.z = -0.027
            pose.orientation.w = 1.0
            co.mesh_poses.append(pose)

            co.operation = CollisionObject.ADD

            ps.world.collision_objects.append(co)
        
        req = ApplyPlanningScene.Request()
        req.scene = ps

        self.scene_client.call_async(req)

        for name in self.tools:
            self.allow_collision(
                name,
                ["wrist_yaw_Link", "wrist_pitch_Link", "tool_mount_link", 
                 "rack_link", "lowerarm_Link"]
            )

    def create_mesh(self, mesh_path):

        try:
            mesh = trimesh.load(mesh_path)

            mesh_msg = Mesh()

            # Add triangles
            for face in mesh.faces:
                triangle = MeshTriangle()
                triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                mesh_msg.triangles.append(triangle)

            # Add vertices
            for vertex in mesh.vertices:
                point = Point()
                point.x = float(vertex[0])
                point.y = float(vertex[1])
                point.z = float(vertex[2])
                mesh_msg.vertices.append(point)

            return mesh_msg

        except Exception as e:
            self.get_logger().error(f"Failed to load mesh: {e}")
            return None

    
    def execute_callback(self, goal_handle):

        if self.state != "IDLE":
            result = ChangeTool.Result()
            result.success = False
            result.message = "Busy"
            goal_handle.abort()
            return result
    
        self.goal_handle = goal_handle
        self.goal_active = True
        self._result = None
    
        tool_name = goal_handle.request.tool_name
        if tool_name in self.tools:
            if self.current_tool == tool_name:
                result = ChangeTool.Result()
                result.success = True
                result.message = f"{tool_name} already attached"
                goal_handle.succeed()
                return result
            self.target_tool = tool_name
            self.start_attach_sequence()

        elif tool_name == "none":
            if self.current_tool is None:
                result = ChangeTool.Result()
                result.success = False
                result.message = "No tool attached"
                goal_handle.abort()
                return result
            self.start_detach_sequence()

        else:
            result = ChangeTool.Result()
            result.success = False
            result.message = "Unsupported tool"
            goal_handle.abort()
            return result
        
        # BLOCK HERE using rclpy spinning
        while self._result is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self._result
    

    def publish_feedback(self, text):
        
        feedback = ChangeTool.Feedback()
        feedback.current_state = text
        self.goal_handle.publish_feedback(feedback)
    

    def generate_lift_waypoints(self, tool_name, steps=10, reverse=False):

        if reverse:
            start = self.NAMED_STATES[f"{tool_name}_lift"]
            end   = self.NAMED_STATES[f"{tool_name}_dock"]
        else:
            start = self.NAMED_STATES[f"{tool_name}_dock"]
            end   = self.NAMED_STATES[f"{tool_name}_lift"]

        self.lift_waypoints = []

        for i in range(1, steps + 1):

            alpha = i / steps

            joints = {}

            for joint in start.keys():
                joints[joint] = (
                    start[joint]
                    + alpha * (end[joint] - start[joint])
                )

            self.lift_waypoints.append(joints)

        self.current_waypoint = 0

    def create_joint_goal(self, joints):

        goal = MoveGroup.Goal()

        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"

        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 2

        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4

        goal.request.start_state = self.get_current_robot_state()

        constraints = Constraints()

        for name, position in joints.items():

            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        return goal

    def execute_next_lift_waypoint(self):

        if self.current_waypoint >= len(self.lift_waypoints):

            self.lift_waypoints = []
            self.current_waypoint = 0

            if self.state == "MOVE_LIFT":
                self.finish_success()
            else:
                self.state = "DETACH_MOVE_DOCK"
                goal = self.create_named_goal(f"{self.current_tool}_dock")
                future = self.move_client.send_goal_async(goal)
                future.add_done_callback(self.goal_response_cb)
            return

        goal = self.create_joint_goal(
            self.lift_waypoints[self.current_waypoint]
        )

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)
    # ==========================================================
    # STATE MACHINE START
    # ==========================================================
    def start_attach_sequence(self):
        self.get_logger().info(f"Starting pickup of {self.target_tool}")
        self.publish_feedback("Moving to approach")
        self.state = "MOVE_APPROACH"

        goal = self.create_named_goal(f"{self.target_tool}_approach")
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def start_detach_sequence(self):
        self.get_logger().info(f"Starting detach sequence of {self.current_tool}")
        self.publish_feedback("Starting detach")
        self.state = "DETACH_MOVE_APPROACH"
        self._allow_docking_collisions(self.current_tool)
        time.sleep(0.3)
        goal = self.create_named_goal(f"{self.current_tool}_lift")
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

        # pose = self.get_transform('base_link', self.tools[self.current_tool]["mount_frame"])
        # if pose is None:
        #     self.abort("Dock transform not available")
        #     return

        # # ── Same hardcoded orientation as attach ─────────────────────────────────
        # q = self.DOCK_QUATERNIONS[self.current_tool]
        # pose.pose.orientation.x = q["x"]
        # pose.pose.orientation.y = q["y"]
        # pose.pose.orientation.z = q["z"]
        # pose.pose.orientation.w = q["w"]
        # # ─────────────────────────────────────────────────────────────────────────

        # self.send_move(self.offset_pose(pose, dy=-0.10))
    # ==========================================================
    # MOVE HANDLING
    # ==========================================================
    def send_move(self, pose: PoseStamped):
        self.get_logger().info(f"Sending move to: "f"x={pose.pose.position.x:.4f}, "f"y={pose.pose.position.y:.4f}, " f"z={pose.pose.position.z:.4f}")
        goal = self.create_goal(pose)
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
    
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.abort("Move rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_cb)

    def move_result_cb(self, future):
        if not self.goal_active:
            return
        result = future.result().result
        if result.error_code.val != 1:
            self.abort(f"Move failed {result.error_code.val}")
            return

        # Advance state
        if self.state == "MOVE_APPROACH":
            # for name in self.tools:
            #     self.allow_collision(
            #         name,
            #         ["wrist_yaw_Link", "wrist_pitch_Link", "tool_mount_link", 
            #         "rack_link", "lowerarm_Link"]
            #     )
            time.sleep(2.0) 
            self.state = "MOVE_DOCK"
            self.publish_feedback("Docking")
            goal = self.create_named_goal(f"{self.target_tool}_dock")   # ← exact joint target
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_cb)

        elif self.state == "MOVE_DOCK":
            self.publish_feedback("Unlocking tool")
            self.state = "UNLOCK"
            self.send_tool_command(2)

        elif self.state == "MOVE_LIFT":
            self.current_waypoint += 1
            self.execute_next_lift_waypoint()
                    
        elif self.state == "DETACH_MOVE_APPROACH":

            if not self.lift_waypoints:
                self.generate_lift_waypoints(
                    self.current_tool,
                    steps=7,
                    reverse=True
                )

            self.current_waypoint += 1
            self.execute_next_lift_waypoint()
            # pose = self.get_transform('base_link', self.tools[self.current_tool]["mount_frame"])
            # if pose is None:
            #     self.abort("Dock transform not available")
            #     return
            # self.send_linear_move(dy=0.08)
            # self.send_move(self.offset_pose(pose, dz=0.02))

        elif self.state == "DETACH_MOVE_DOCK":
            self.state = "DETACH_UNLOCK"
            self.publish_feedback("Unlocking tool")
            self.send_tool_command(2)

        elif self.state == "DETACH_MOVE_LIFT":
            self.finish_detach_success()
            

    
    # ==========================================================
    # TOOL ACTUATOR
    # ==========================================================
    def send_tool_command(self, cmd):
        req = LinearMotor.Request()
        req.command = cmd
        future = self.tool_client.call_async(req)
        future.add_done_callback(self.tool_result_cb)

    def tool_result_cb(self, future):
        if not self.goal_active:
            return
        result = future.result()
        if result is None or not result.success:
            self.abort("Tool command failed")
            return

        if self.state == "UNLOCK":
            self.state = "ATTACH"
            self.attach_tool()
            # self.delete_tool_from_dock("gripper")

        elif self.state == "LOCK":
            self.state = "MOVE_LIFT"
            # ── Use named joint state ──────────
            self.generate_lift_waypoints(self.target_tool, steps=10)

            self.execute_next_lift_waypoint()
    # ──────────────────────────────────────────────────────────────────────
        elif self.state == "DETACH_UNLOCK":
            self.state = "DETACH_REMOVE"
            self.detach_tool()
            # self.delete_attached_tool("gripper")

        elif self.state == "DETACH_LOCK":
            self.state = "DETACH_MOVE_LIFT"
            goal = self.create_named_goal(f"{self.target_tool}_approach")
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_cb)
            # pose = self.get_transform('base_link', self.tools[self.current_tool]["mount_frame"])
            # if pose is None:
            #     self.abort("Dock transform not available")
            #     return
            # self.send_move(self.offset_pose(pose, dz=0.1))

    # ==========================================================
    # PLANNING SCENE
    # ==========================================================
    def attach_tool(self):

        tool = self.tools[self.target_tool]

        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True

        remove = CollisionObject()
        remove.id = self.target_tool
        remove.header.frame_id = "world"
        remove.operation = CollisionObject.REMOVE

        ps.world.collision_objects.append(remove)

        pkg_path = get_package_share_directory("sabry")
        mesh_path = os.path.join(pkg_path, "meshes", tool["mesh"])

        mesh = self.create_mesh(mesh_path)

        co = CollisionObject()
        co.id = self.target_tool
        co.header.frame_id = "tool_mount_link"

        co.meshes.append(mesh)
    
        pose = Pose()
        pose.position.x = 0.017
        pose.position.y = -0.045
        pose.position.z = -0.027
        pose.orientation.z = 1.0

        co.mesh_poses.append(pose)

        co.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = "tool_mount_link"
        aco.object = co
        aco.touch_links = [
            "tool_mount_link",
            "wrist_yaw_Link",
            "wrist_pitch_Link", 
            "lowerarm_Link",
            "rack_link"
        ]

        ps.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request()
        req.scene = ps

        future = self.scene_client.call_async(req)
        future.add_done_callback(self.attach_done_cb)

    def attach_done_cb(self, future):
        if not self.goal_active:
            return
        time.sleep(0.5)
        self._allow_docking_collisions(self.target_tool)
        time.sleep(0.3)
        self.state = "LOCK"
        self.publish_feedback("Locking tool")
        self.send_tool_command(1)

    def detach_tool(self):
        tool = self.tools[self.current_tool]

        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True

        aco = AttachedCollisionObject()
        aco.object.id = self.current_tool
        aco.object.operation = CollisionObject.REMOVE

        ps.robot_state.attached_collision_objects.append(aco)

        pkg_path = get_package_share_directory("sabry")
        mesh_path = os.path.join(pkg_path, "meshes", tool["mesh"])

        mesh = self.create_mesh(mesh_path)

        co = CollisionObject()
        co.id = self.current_tool
        co.header.frame_id = tool["mount_frame"]
        
        pose = Pose()
        pose.position.x = -0.045
        pose.position.y = -0.017
        pose.position.z = -0.027
        pose.orientation.w = 1.0

        co.meshes.append(mesh)
        co.mesh_poses.append(pose)
        co.operation = CollisionObject.ADD

        ps.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request()
        req.scene = ps

        future = self.scene_client.call_async(req)
        future.add_done_callback(self.detach_done_cb)

    def detach_done_cb(self, future):
        if not self.goal_active:
            return
        time.sleep(0.5)
        self.state = "DETACH_LOCK"
        self.publish_feedback("Unlocking tool")
        self.send_tool_command(2)
        # self.allow_collision(
        #     self.current_tool,
        #     ["wrist_yaw_Link", "wrist_pitch_Link", "tool_mount_link", "lowerarm_Link", "rack_link"]
        # )

    
    def _allow_docking_collisions(self, tool_name):
        """Allow tool to touch robot links and rack during dock/undock."""
        self.allow_collision(tool_name, [
            "tool_mount_link", "wrist_yaw_Link", "wrist_pitch_Link",
            "lowerarm_Link", "rack_link",
            "left_tool", "mid_tool", "right_tool",
        ])

    def _restrict_free_motion_collisions(self, tool_name):
        """
        After lifting away from rack, disallow tool colliding with rack/other tools.
        Tool still allowed to touch its own robot mount links.
        We do this by removing rack/other-tools from the ACM allow list.
        """
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        ps  = future.result().scene
        acm = ps.allowed_collision_matrix
        ps.is_diff = True

        # Links to DISALLOW collision with (rack + other tools)
        rack_and_tools = [
            "rack_link", "left_tool", "mid_tool", "right_tool",
            "gripper", "screwdriver", "camera", "lowerarm_Link",
        ]
        # Remove current tool from that list — don't disallow with itself
        rack_and_tools = [x for x in rack_and_tools if x != tool_name]

        if tool_name not in acm.entry_names:
            return  # nothing to restrict

        ti = acm.entry_names.index(tool_name)

        for name in rack_and_tools:
            if name in acm.entry_names:
                ni = acm.entry_names.index(name)
                acm.entry_values[ti].enabled[ni] = False
                acm.entry_values[ni].enabled[ti] = False

        req2       = ApplyPlanningScene.Request()
        req2.scene = ps
        self.scene_client.call_async(req2)
        self.get_logger().info(f"Restricted {tool_name} collisions for free motion")



    def create_named_goal(self, state_name: str):
        """Move to a named state defined in the SRDF"""

        # Use GetRobotStateFromWarehouse or just hardcode from SRDF
        # We read named states via the robot model
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.8
        goal.request.max_acceleration_scaling_factor = 0.8
        goal.request.start_state = self.get_current_robot_state()

        # Named states from SRDF are referenced directly
        goal.request.goal_constraints.append(
            self.named_state_to_constraints(state_name)
        )
        return goal

    def named_state_to_constraints(self, state_name: str):
        """Convert SRDF named state to joint constraints"""
        from moveit_msgs.msg import Constraints, JointConstraint

        

        joints = self.NAMED_STATES[state_name]
        constraints = Constraints()

        for joint_name, position in joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        return constraints
       
    def finish_success(self):

        if not self.goal_active:
            return

        self.state = "IDLE"
        self.current_tool = self.target_tool
        self._restrict_free_motion_collisions(self.current_tool)

        self._result = ChangeTool.Result()
        self._result.success = True
        self._result.message = f"{self.current_tool} attached"

        self.goal_handle.succeed()
        self.goal_active = False 
    
    def finish_detach_success(self):

        if not self.goal_active:
            return

        self.state = "IDLE"
        prev       = self.current_tool
        self.current_tool = None
        self._restrict_free_motion_collisions(prev)

        self._result = ChangeTool.Result()
        self._result.success = True
        self._result.message = "Tool detached"

        self.goal_handle.succeed()
        self.goal_active = False 

    def abort(self, message):
        if not self.goal_active:
            return

        self.get_logger().error(message)
        self.state = "IDLE"

        self._result = ChangeTool.Result()
        self._result.success = False
        self._result.message = message

        self.goal_handle.abort()
        self.goal_active = False

    # ==========================================================
    # HELPERS
    # ==========================================================
    def get_dock_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.193
        pose.pose.position.y = -0.287
        pose.pose.position.z = 0.238
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose
    
    def get_transform(self, target_frame, source_frame, timeout=2.0):
        """Get transform between frames with error handling"""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, now, rclpy.duration.Duration(seconds=timeout))
           
            t = transform.transform.translation
            r = transform.transform.rotation

            self.get_logger().info(
                f"TF {source_frame} -> {target_frame} | "
                f"Translation: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f} | "
                f"Rotation (quat): x={r.x:.4f}, y={r.y:.4f}, z={r.z:.4f}, w={r.w:.4f}"
            )
            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x 
            pose.pose.position.y = transform.transform.translation.y 
            # pose.pose.position.z = transform.transform.translation.z + 0.018
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            # pose.pose.orientation.x = 1.0
            # pose.pose.orientation.y = 0.0
            # pose.pose.orientation.z = 0.0
            # pose.pose.orientation.w = 0.0

            return pose
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {str(e)}")
            return None

    def offset_pose(self, base, dx=0, dy=0, dz=0):
        pose = PoseStamped()
        pose.header.frame_id = base.header.frame_id
        pose.pose.position.x = base.pose.position.x + dx
        pose.pose.position.y = base.pose.position.y + dy
        pose.pose.position.z = base.pose.position.z + dz
        pose.pose.orientation = base.pose.orientation
        # pose.pose.orientation.x = 1.0
        # pose.pose.orientation.y = 0.0
        # pose.pose.orientation.z = 0.0
        # pose.pose.orientation.w = 0.0
        return pose

    def relative_pose(self, rel_pos):
        pose = PoseStamped()
        pose.header.frame_id = "tool_mount_link"
        pose.pose.position.x = rel_pos[0]
        pose.pose.position.y = rel_pos[1]
        pose.pose.position.z = rel_pos[2]
        pose.pose.orientation.w = 1.0
        
        return pose

    def create_goal(self, pose):
        pos = PositionConstraint()
        pos.header.frame_id = pose.header.frame_id
        pos.link_name = "tool_mount_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        pos.constraint_region.primitives.append(primitive)
        pos.constraint_region.primitive_poses.append(pose.pose)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = pose.header.frame_id
        ori.link_name = "tool_mount_link"
        ori.orientation = pose.pose.orientation

        # For a 5-DOF robot: one rotation axis is uncontrollable by kinematics.
        # Tighten the 2 axes that matter for tool alignment (approach direction).
        # Relax the axis your robot structurally cannot control (usually Z — tune this).
        ori.absolute_x_axis_tolerance = 0.09   # tight — must align for docking
        ori.absolute_y_axis_tolerance = 0.09   # tight — must align for docking  
        ori.absolute_z_axis_tolerance = 3.14   # relaxed — 5-DOF can't control this
        ori.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)

        goal = MoveGroup.Goal()
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = 0.0
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.5
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 25
        goal.request.allowed_planning_time = 20.0
        goal.request.max_velocity_scaling_factor = 0.8
        goal.request.max_acceleration_scaling_factor = 0.9
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = False
        goal.request.start_state = self.get_current_robot_state()

        return goal

    def joint_state_cb(self, msg):
        self.current_joint_state = msg

    def get_current_robot_state(self):
        state = RobotState()
        if self.current_joint_state is not None:
            state.joint_state = self.current_joint_state
            state.is_diff = False
        else:
            state.is_diff = True
        return state

def main(args=None):
    rclpy.init(args=args)
    node = ToolChangeManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("Terminating Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
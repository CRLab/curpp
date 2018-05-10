import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import graspit_interface.msg

import moveit_commander
import tf
import pyquaternion
import numpy as np
import deprecated
import trollius as asyncio

import grasping_controller
import graspit_moveit_message_utils

import config


@deprecated.deprecated("This function is purely for debugging purposes. Use grid_sample_plugin instead")
def construct_graspit_grasp(position, orientation):
    graspit_grasp_msg = graspit_interface.msg.Grasp()
    pose = geometry_msgs.msg.Pose()
    pose.position = position
    pose.orientation = orientation
    graspit_grasp_msg.pose = pose
    return graspit_grasp_msg


@deprecated.deprecated("This function is purely for debugging purposes. Use grid_sample_plugin instead")
def plan_grasps(x=0, y=0, z=0.080):
    position = geometry_msgs.msg.Point(x, y, z)

    normal_orientation = pyquaternion.Quaternion(x=0, y=0, z=0, w=1)
    rotated_orientation = pyquaternion.Quaternion(axis=[1, 0, 0], angle=np.pi/2)
    rotated_orientation = rotated_orientation * normal_orientation

    geom_orient = geometry_msgs.msg.Quaternion(
        x=normal_orientation[0], y=normal_orientation[1], z=normal_orientation[2], w=normal_orientation[3]
    )
    geom_orient_rot = geometry_msgs.msg.Quaternion(
        x=rotated_orientation[0], y=rotated_orientation[1], z=rotated_orientation[2], w=rotated_orientation[3]
    )

    grasps = list()
    grasps.append(construct_graspit_grasp(position, geom_orient))
    grasps.append(construct_graspit_grasp(position, geom_orient_rot))

    return grasps


class CURPPManager:

    def __init__(self, config):
        # type: (config.Config) -> ()

        self.config = config

        # Initialize ros service interfaces
        self.grasping_controller = grasping_controller.MoveitPickPlaceInterface(
            arm_name=self.config.arm_move_group_name,
            gripper_name=self.config.gripper_move_group_name,
            grasp_approach_tran_frame=self.config.grasp_approach_frame,
            analyzer_planner_id=self.config.analyzer_planner_id,
            execution_planner_id=self.config.executor_planner_id,
            allowed_analyzing_time=self.config.allowed_analyzing_time,
            allowed_execution_time=self.config.allowed_execution_time
        )

        self.scene = moveit_commander.PlanningSceneInterface()
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # If we need to broadcast approach_tran - start publishing
        if self.config.broadcast_approach_tran:
            asyncio.ensure_future(self._publish_approach_tran())

    @asyncio.coroutine
    def _publish_approach_tran(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransformMessage(self.config.approach_tran)
            rate.sleep()

    def graspit_grasp_to_moveit_grasp(self, object_name, graspit_grasp):
        # type: (str, graspit_interface.msg.Grasp) -> moveit_msgs.msg.Grasp

        grasp_approach_tran_frame = self.config.grasp_approach_frame

        moveit_grasp_msg = graspit_moveit_message_utils.graspit_grasp_to_moveit_grasp(
            object_name=object_name,
            graspit_grasp_msg=graspit_grasp,
            listener=self.tf_listener,
            grasp_tran_frame_name=grasp_approach_tran_frame,
            end_effector_link=self.grasping_controller.get_end_effector_link(),

            pre_grasp_goal_point_effort=self.config.pre_grasp_goal_point_effort,
            pre_grasp_goal_point_positions=self.config.pre_grasp_goal_point_positions,
            pre_grasp_goal_point_time_from_start_secs=self.config.pre_grasp_goal_point_time_from_start_secs,
            pre_grasp_joint_names=self.config.pre_grasp_joint_names,

            grasp_goal_point_effort=self.config.grasp_goal_point_effort,
            grasp_goal_point_positions=self.config.grasp_goal_point_positions,
            grasp_goal_point_time_from_start_secs=self.config.grasp_goal_point_time_from_start_secs,

            grasp_posture_joint_names=self.config.grasp_posture_joint_names,

            pre_grasp_approach_min_distance=self.config.pre_grasp_approach_min_distance,
            pre_grasp_approach_desired_distance=self.config.pre_grasp_approach_desired_distance,
            pre_grasp_approach_direction=self.config.pre_grasp_approach_direction,

            post_grasp_retreat_min_distance=self.config.post_grasp_retreat_min_distance,
            post_grasp_retreat_desired_distance=self.config.post_grasp_retreat_desired_distance,
            post_grasp_retreat_direction=self.config.post_grasp_retreat_direction,

            max_contact_force=self.config.max_contact_force
        )

        return moveit_grasp_msg

    def analyze_grasp_reachability(self, object_name, graspit_grasp):
        # type: (str, graspit_interface.msg.Grasp) -> (moveit_msgs.msg.PickupResult, bool)
        """
        @return: Whether the grasp is expected to succeed
        """
        # Convert graspit grasp to moveit grasp
        rospy.loginfo("Analyzing grasp for object: {}".format(object_name))

        object_ids = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_objects_with_ids(object_ids)

        moveit_grasp_msg = self.graspit_grasp_to_moveit_grasp(object_name, graspit_grasp)
        success, pick_result = self.grasping_controller.analyze_moveit_grasp(object_name, moveit_grasp_msg)

        rospy.loginfo("Able to execute grasp with grasp id {} after analysis: {}".format(moveit_grasp_msg.id, success))

        return pick_result, success

    def execute_grasp(self, object_name, graspit_grasp, place_position):
        # type: (str, graspit_interface.msg.Grasp) -> bool
        rospy.loginfo("Executing grasp goal")

        object_ids = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_objects_with_ids(object_ids)

        # Acquire block position for place
        objects = self.scene.get_object_poses([object_name])
        if object_name not in objects:
            rospy.logerr("Object {} not in planning scene. Execute grasp failed".format(object_name))
            return False

        object_pose_stamped = geometry_msgs.msg.PoseStamped()
        object_pose_stamped.pose = objects[object_name]
        object_pose_stamped.header.frame_id = self.grasping_controller.get_planning_frame()

        rospy.loginfo("Object {} in planning scene. Pose: {}".format(object_name, object_pose_stamped.pose))

        # Shift block pose to place location in param server
        object_pose_stamped.pose.position = place_position

        # Convert graspit grasp to moveit grasp
        moveit_grasp_msg = self.graspit_grasp_to_moveit_grasp(object_name, graspit_grasp)

        # Execute pick on block
        success, pick_result = self.grasping_controller.execute_moveit_grasp(object_name, moveit_grasp_msg)
        # type: pick_result -> moveit_msgs.msg.PickupResult

        if not success:
            error_code = graspit_moveit_message_utils.moveit_error_code_to_string(pick_result.error_code)
            rospy.logerr("Failed to execute pick. Reason: {}".format(error_code))
            return False
        else:
            rospy.loginfo("Successfully executed pick")

        rospy.loginfo("Placing block as position ({}, {}, {})"
                      .format(object_pose_stamped.pose.position.x,
                              object_pose_stamped.pose.position.y,
                              object_pose_stamped.pose.position.z))
        # Execute place on block
        success, place_result = self.grasping_controller.place(object_name, pick_result, object_pose_stamped)

        if not success:
            error_code = graspit_moveit_message_utils.moveit_error_code_to_string(place_result.error_code)
            rospy.logerr("Failed to execute place. Reason: {}".format(error_code))
            return False
        else:
            rospy.loginfo("Successfully executed place")

        # Home arm and open hand
        if not self.grasping_controller.can_home_arm():
            rospy.loginfo("No home goal specified. Finishing trajectory.")
            return True

        success = self.grasping_controller.home_arm()
        if not success:
            rospy.logerr("Failed to home arm")
            return False
        else:
            rospy.loginfo("Successfully homed arm")

        success = self.grasping_controller.open_hand()
        if not success:
            rospy.logerr("Failed to open hand")
            return False
        else:
            rospy.loginfo("Successfully opened hand")

        return True

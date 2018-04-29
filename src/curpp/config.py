import yaml
import os
import geometry_msgs.msg


class Config:

    def __init__(self, config_filename):
        # Load in parameters from config file
        config_dict = self._load_yaml_file(config_filename)

        self._approach_tran = self._load_approach_tran(config_dict)

        self._world_frame = config_dict["world_frame"]

        self._arm_move_group_name = config_dict["arm_move_group_name"]
        self._gripper_move_group_name = config_dict["gripper_move_group_name"]

        self._analyzer_planner_id = config_dict["analyzer_planner_id"]
        self._executor_planner_id = config_dict["executor_planner_id"]

        self._allowed_analyzing_time = config_dict["allowed_analyzing_time"]
        self._allowed_execution_time = config_dict["allowed_execution_time"]

        self._pre_grasp_approach_direction = self._load_direction(config_dict["pre_grasp_approach_direction"])
        self._post_grasp_retreat_direction = self._load_direction(config_dict["post_grasp_retreat_direction"])

        # TODO: add checks that these are floats/ints
        self._pre_grasp_goal_point_time_from_start_secs = config_dict["pre_grasp_goal_point_time_from_start_secs"]
        self._pre_grasp_approach_min_distance = config_dict["pre_grasp_approach_min_distance"]
        self._pre_grasp_approach_desired_distance = config_dict["pre_grasp_approach_desired_distance"]
        self._grasp_goal_point_time_from_start_secs = config_dict["grasp_goal_point_time_from_start_secs"]
        self._post_grasp_retreat_min_distance = config_dict["post_grasp_retreat_min_distance"]
        self._post_grasp_retreat_desired_distance = config_dict["post_grasp_retreat_desired_distance"]
        self._max_contact_force = config_dict["max_contact_force"]

        # TODO: Make sure all of these are the same length
        self._pre_grasp_joint_names = tuple(config_dict["pre_grasp_joint_names"])
        self._grasp_posture_joint_names = tuple(config_dict["grasp_posture_joint_names"])
        self._pre_grasp_goal_point_effort = tuple(config_dict["pre_grasp_goal_point_effort"])
        self._pre_grasp_goal_point_positions = tuple(config_dict["pre_grasp_goal_point_positions"])
        self._grasp_goal_point_effort = tuple(config_dict["grasp_goal_point_effort"])
        self._grasp_goal_point_positions = tuple(config_dict["grasp_goal_point_positions"])

        # TODO: Affirm parameters are correct types

    @staticmethod
    def _load_direction(direction_dict):
        # type: (dict) -> geometry_msgs.msg.Vector3Stamped

        # TODO: add type checking
        direction = geometry_msgs.msg.Vector3Stamped()
        direction.header.frame_id = direction_dict["frame_id"]
        direction.vector.x = direction_dict["x"]
        direction.vector.y = direction_dict["y"]
        direction.vector.z = direction_dict["z"]

        return direction

    @staticmethod
    def _load_approach_tran(config_dict):
        approach_tran_dict = config_dict.get("approach_tran", None)
        if approach_tran_dict is None:
            return None

        approach_tran = geometry_msgs.msg.TransformStamped()
        approach_tran.transform.translation.x = approach_tran_dict["translation"]["x"]
        approach_tran.transform.translation.y = approach_tran_dict["translation"]["y"]
        approach_tran.transform.translation.z = approach_tran_dict["translation"]["z"]

        approach_tran.transform.rotation.x = approach_tran_dict["rotation"]["x"]
        approach_tran.transform.rotation.y = approach_tran_dict["rotation"]["y"]
        approach_tran.transform.rotation.z = approach_tran_dict["rotation"]["z"]
        approach_tran.transform.rotation.w = approach_tran_dict["rotation"]["w"]

        approach_tran.child_frame_id = approach_tran_dict["child_frame_id"]
        approach_tran.header.frame_id = approach_tran_dict["parent_frame_id"]

        return approach_tran

    @staticmethod
    def _load_yaml_file(config_filename):
        if isinstance(config_filename, str):
            if not os.path.isfile(config_filename):
                raise ValueError("config_filename '{}' must exist".format(config_filename))
            config_dict = yaml.load(open(config_filename, 'r'))
        elif isinstance(config_filename, file):
            config_dict = yaml.load(config_filename)
        else:
            raise ValueError("config_filename must be a filename or a file handle")

        return config_dict

    # Approach direction from GraspIt! differing from URDF
    approach_tran = property(lambda self: self._approach_tran) # type: geometry_msgs.msg.TransformStamped
    broadcast_approach_tran = property(lambda self: self.approach_tran is None)

    # Defines the frame that a grasp will be placed in from GraspIt! TODO: Double check this
    world_frame = property(lambda self: self._world_frame)

    # Defines the name of the arm and gripper move groups
    arm_move_group_name = property(lambda self: self._arm_move_group_name)
    gripper_move_group_name = property(lambda self: self._gripper_move_group_name)

    # Planner ids to use to determine optimal trajectories
    analyzer_planner_id = property(lambda self: self._analyzer_planner_id)
    executor_planner_id = property(lambda self: self._executor_planner_id)

    # Execution and analyzing time - execution should be longer than analyzer
    allowed_analyzing_time = property(lambda self: self._allowed_analyzing_time)
    allowed_execution_time = property(lambda self: self._allowed_execution_time)

    # type: geometry_msgs.msg.Vector3Stamped
    pre_grasp_approach_direction = property(lambda self: self._pre_grasp_approach_direction)

    # type: geometry_msgs.msg.Vector3Stamped
    post_grasp_retreat_direction = property(lambda self: self._post_grasp_retreat_direction)

    # Gives the fie
    grasp_approach_frame = property(lambda self: self._pre_grasp_approach_direction.header.frame_id)

    pre_grasp_goal_point_time_from_start_secs = property(lambda self: self._pre_grasp_goal_point_time_from_start_secs)
    pre_grasp_approach_min_distance = property(lambda self: self._pre_grasp_approach_min_distance)
    pre_grasp_approach_desired_distance = property(lambda self: self._pre_grasp_approach_desired_distance)
    grasp_goal_point_time_from_start_secs = property(lambda self: self._grasp_goal_point_time_from_start_secs)
    post_grasp_retreat_min_distance = property(lambda self: self._post_grasp_retreat_min_distance)
    post_grasp_retreat_desired_distance = property(lambda self: self._post_grasp_retreat_desired_distance)
    max_contact_force = property(lambda self: self._max_contact_force)
    pre_grasp_goal_point_effort = property(lambda self: self._pre_grasp_goal_point_effort)
    pre_grasp_goal_point_positions = property(lambda self: self._pre_grasp_goal_point_positions)
    pre_grasp_joint_names = property(lambda self: self._pre_grasp_joint_names)
    grasp_goal_point_effort = property(lambda self: self._grasp_goal_point_effort)
    grasp_goal_point_positions = property(lambda self: self._grasp_goal_point_positions)
    grasp_posture_joint_names = property(lambda self: self._grasp_posture_joint_names)

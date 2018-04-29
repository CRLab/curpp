import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

import promise

import world_manager
import constants


def capture_grasp_marker(color=None):
    grasp_promise = promise.Promise()

    # Create subscriber to /move_group/display_grasp_markers
    def listen_for_grasp_markers(markers):
        # type: (visualization_msgs.msg.MarkerArray) -> ()
        for marker in markers.markers:
            if color is not None:
                marker.color = color
            else:
                marker.color = constants.GRASP_MARKER_COLOR

        grasp_marker_subscriber.unregister()
        grasp_promise.resolve(markers)

    grasp_marker_subscriber = rospy.Subscriber(
        '/move_group/display_grasp_markers',
        visualization_msgs.msg.MarkerArray,
        listen_for_grasp_markers
    )

    return grasp_promise


def create_block_marker(object_pose_stamped, is_highlighted=False, color=None, edge_length=0.1, duration=0):
    # type: (geometry_msgs.msg.PoseStamped, bool, std_msgs.msg.ColorRGBA, float, int) -> visualization_msgs.msg.Marker
    marker = visualization_msgs.msg.Marker()

    marker.header = object_pose_stamped.header
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD

    # Show block until it is deleted if duration == 0, otherwise in seconds
    marker.lifetime = rospy.Duration(duration)

    if is_highlighted:
        # Add scaling factor based on the size of the cube * 1.1 for highlighted
        marker.scale.x = edge_length * 1.0
        marker.scale.y = edge_length * 1.0
        marker.scale.z = edge_length * 1.0
        marker.color = constants.HIGHLIGHTED_BLOCK_COLOR

        marker.id = -1  # "highlighted"
    else:
        # Add scaling factor based on the size of the cube
        marker.scale.x = edge_length * 1.0
        marker.scale.y = edge_length * 1.0
        marker.scale.z = edge_length * 1.0
        marker.color = constants.NORMAL_BLOCK_COLOR

        # Unsure if you need an id or not
        marker.id = object_pose_stamped.__hash__()

    if color is not None:
        marker.color = color

    marker.pose = object_pose_stamped.pose

    return marker


def create_block_position_marker(object_pose_stamped, is_highlighted=False, color=None, edge_length=0.1, duration=0):
    # type: (geometry_msgs.msg.PoseStamped, bool, std_msgs.msg.ColorRGBA, float, int) -> visualization_msgs.msg.Marker
    marker = visualization_msgs.msg.Marker()

    marker.header = object_pose_stamped.header
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.lifetime = rospy.Duration(duration)  # Show block until it is deleted

    # Add scaling factor based on the size of the cube * 1.1 for highlighted
    if is_highlighted:
        marker.scale.x = edge_length * 1.5
        marker.scale.y = edge_length * 1.5
        marker.scale.z = edge_length * 1.5
        marker.color = constants.HIGHLIGHTED_BLOCK_COLOR
        marker.id = -1  # "highlighted"
    else:
        # Add scaling factor based on the size of the cube
        marker.scale.x = edge_length * 1.3
        marker.scale.y = edge_length * 1.3
        marker.scale.z = edge_length * 1.3
        marker.color = constants.NORMAL_BLOCK_COLOR
        marker.id = object_pose_stamped.__hash__()

    if color is not None:
        marker.color = color

    marker.pose.position = object_pose_stamped.position

    return marker


def generate_delete_all_marker_array():
    delete_all_array = visualization_msgs.msg.MarkerArray()
    delete_all_marker = visualization_msgs.msg.Marker()
    delete_all_marker.action = visualization_msgs.msg.Marker.DELETEALL
    delete_all_array.markers.append(delete_all_marker)
    return delete_all_array


def run_recognition():
    rospy.loginfo("Running recognition")

    world_manager.world_manager_client.clear_objects()

    detected_blocks = block_recognition.find_blocks()
    # type: detected_blocks -> typing.List[block_recognition_msgs.msg.DetectedBlock]

    if len(detected_blocks) == 0:
        rospy.loginfo("Detected no blocks. No work done.")
        return []

    rospy.loginfo("Detected {} blocks".format(len(detected_blocks)))

    for detected_block in detected_blocks:
        # Add all blocks to the scene
        world_manager.world_manager_client.add_box(detected_block.unique_block_name,
                                          detected_block.pose_stamped,
                                          detected_block.edge_length,
                                          detected_block.edge_length,
                                          detected_block.edge_length)

    # Return all detected blocks
    return detected_blocks


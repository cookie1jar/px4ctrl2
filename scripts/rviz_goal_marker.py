"""Editable 3D goal marker: dragging edits; a menu action submits the goal."""
from copy import deepcopy
import math
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


def make_goal_marker(pose):
    marker = InteractiveMarker()
    marker.header.frame_id = 'global'
    marker.name = 'flight_target'
    marker.description = '3D target: drag axes; right-click -> Fly to target'
    marker.scale = 0.8
    marker.pose = deepcopy(pose)
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.pose.orientation.w = 1.
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
    sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.2, 0.7, 1., 1.
    menu = InteractiveMarkerControl()
    menu.name = 'target_menu'
    menu.interaction_mode = InteractiveMarkerControl.MENU
    menu.always_visible = True
    menu.markers.append(sphere)
    marker.controls.append(menu)
    for name, axis in [('x', 'x'), ('y', 'z'), ('z', 'y')]:
        control = InteractiveMarkerControl()
        control.name = 'move_'+name
        control.orientation.w = math.sqrt(0.5)
        setattr(control.orientation, axis, math.sqrt(0.5))
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
    heading = InteractiveMarkerControl()
    heading.name = 'rotate_yaw'
    heading.orientation.w = heading.orientation.y = math.sqrt(0.5)
    heading.orientation_mode = InteractiveMarkerControl.FIXED
    heading.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    marker.controls.append(heading)
    return marker

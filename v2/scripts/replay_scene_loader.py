#!/usr/bin/env python3
"""Load the isolated replay's static map once, after TF and MoveIt are ready."""
import time
import rospy
import tf2_ros
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPlanningSceneRequest


def main():
    rospy.init_node('v2_replay_scene_loader')
    rospy.wait_for_service('/apply_planning_scene', timeout=60)
    rospy.wait_for_service('/get_planning_scene', timeout=60)
    m = rospy.wait_for_message('/octomap_binary', Octomap, timeout=60)
    if not m.data or not m.id or m.header.frame_id != 'odom':
        raise RuntimeError('STATIC_OCTOMAP_INVALID')
    buf = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(buf)
    end = time.monotonic() + 60
    while not rospy.is_shutdown():
        try:
            transform = buf.lookup_transform('world', m.header.frame_id, rospy.Time(0), rospy.Duration(.5))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if time.monotonic() > end: raise RuntimeError('WORLD_ODOM_TF_TIMEOUT')
            time.sleep(.1)
    scene = PlanningScene(); scene.is_diff = True
    scene.world.octomap.header.frame_id = 'world'
    scene.world.octomap.octomap = m
    scene.world.octomap.octomap.header.frame_id = 'world'
    scene.world.octomap.origin.position.x = transform.transform.translation.x
    scene.world.octomap.origin.position.y = transform.transform.translation.y
    scene.world.octomap.origin.position.z = transform.transform.translation.z
    scene.world.octomap.origin.orientation = transform.transform.rotation
    if not rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)(scene).success:
        raise RuntimeError('APPLY_PLANNING_SCENE_FAILED')
    req = GetPlanningSceneRequest(); req.components.components = PlanningSceneComponents.OCTOMAP
    seen = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)(req).scene.world.octomap
    if not seen.octomap.data or seen.octomap.id != m.id or abs(seen.octomap.resolution - m.resolution) > 1e-9:
        raise RuntimeError('SCENE_MAP_READBACK_FAILED')
    rospy.loginfo('REPLAY_SCENE_READY frame=%s id=%s resolution=%.6f bytes=%d origin=%s',
                  seen.header.frame_id, seen.octomap.id, seen.octomap.resolution,
                  len(seen.octomap.data), seen.origin)
    rospy.loginfo('GAZEBO_ENVIRONMENT_READY: scene is loaded; no CSV trajectory has been started. Run view_v2.sh --gazebo-csv to align and replay.')

if __name__ == '__main__':
    main()

"""Inject the repository OctoMap into this run's isolated MoveIt scene once.

No Gazebo or depth sensor needed. The caller must first verify a dedicated master.
"""
import argparse
import io
import json
from hashlib import sha256


def main():
    import rospy
    from octomap_msgs.msg import Octomap
    from moveit_msgs.msg import PlanningScene,PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene,ApplyPlanningSceneRequest,GetPlanningScene,GetPlanningSceneRequest
    parser=argparse.ArgumentParser();parser.add_argument('--timeout',type=float,default=30);args=parser.parse_args()
    rospy.init_node('v2_static_scene_loader',anonymous=True,disable_signals=True)
    octomap=rospy.wait_for_message('/octomap_binary',Octomap,timeout=args.timeout)
    if not octomap.id or not octomap.data:raise RuntimeError('STATIC_OCTOMAP_EMPTY')
    if octomap.header.frame_id!='odom':raise RuntimeError('MAP_FRAME_MISMATCH '+octomap.header.frame_id)
    rospy.wait_for_service('/apply_planning_scene',timeout=args.timeout)
    rospy.wait_for_service('/get_planning_scene',timeout=args.timeout)
    apply=rospy.ServiceProxy('/apply_planning_scene',ApplyPlanningScene)
    get=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
    diff=PlanningScene();diff.is_diff=True
    diff.world.octomap.header=octomap.header
    diff.world.octomap.octomap=octomap
    diff.world.octomap.origin.orientation.w=1.
    req=ApplyPlanningSceneRequest();req.scene=diff
    response=apply(req)
    if not response.success:raise RuntimeError('APPLY_PLANNING_SCENE_FAILED')
    query=GetPlanningSceneRequest();query.components.components=1023
    scene=get(query).scene
    seen=scene.world.octomap.octomap
    if not seen.id or not seen.data:raise RuntimeError('SCENE_MAP_NOT_VISIBLE')
    raw=io.BytesIO();scene.serialize(raw)
    print(json.dumps(dict(status='MAP_VISIBLE',frame=scene.world.octomap.header.frame_id,
                          octomap_id=seen.id,octomap_bytes=len(seen.data),scene_sha256=sha256(raw.getvalue()).hexdigest()),sort_keys=True))

if __name__=='__main__':main()

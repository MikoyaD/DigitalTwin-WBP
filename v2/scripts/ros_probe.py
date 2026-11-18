"""Read-only live ROS scene/model/TF probe; writes immutable snapshot on success."""
import argparse
import io
import json
from hashlib import sha256
from pathlib import Path


def probe(output,timeout=10):
    import rospy
    import tf2_ros
    from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
    rospy.init_node('v2_scene_probe',anonymous=True,disable_signals=True)
    rospy.wait_for_service('/get_planning_scene',timeout=timeout)
    rospy.wait_for_service('/check_state_validity',timeout=timeout)
    rospy.wait_for_service('/plan_kinematic_path',timeout=timeout)
    scene_service=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
    request=GetPlanningSceneRequest();request.components.components=1023
    scene=scene_service(request).scene
    if scene.is_diff:raise RuntimeError('SCENE_INCOMPLETE')
    octomap=scene.world.octomap.octomap
    if not octomap.id or not octomap.data:raise RuntimeError('STATIC_OBSTACLE_MISSING')
    # MoveIt may transform the injected odom map into its world planning frame.
    if scene.world.octomap.header.frame_id not in ('odom','world'):
        raise RuntimeError('STATIC_OBSTACLE_FRAME '+scene.world.octomap.header.frame_id)
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(5));listener=tf2_ros.TransformListener(tf_buffer)
    relation=tf_buffer.lookup_transform('world','odom',rospy.Time(0),rospy.Duration(timeout))
    urdf=rospy.get_param('/robot_description');srdf=rospy.get_param('/robot_description_semantic')
    if not urdf or not srdf:raise RuntimeError('ROBOT_MODEL_MISSING')
    adapters=rospy.get_param('/move_group/planning_pipelines/ompl/request_adapters',None)
    if not adapters or 'AddTimeOptimalParameterization' not in adapters:raise RuntimeError('TOTG_STAGE_UNVERIFIED')
    if rospy.get_param('/move_group/allow_trajectory_execution',True):raise RuntimeError('TRAJECTORY_EXECUTION_ENABLED')
    wire=io.BytesIO();scene.serialize(wire);blob=wire.getvalue()
    snapshot=output/'scene_snapshot.bin'
    with snapshot.open('xb') as f:f.write(blob)
    result=dict(status='PASS',scene_sha256=sha256(blob).hexdigest(),scene_snapshot=str(snapshot),
                urdf_sha256=sha256(urdf.encode()).hexdigest(),srdf_sha256=sha256(srdf.encode()).hexdigest(),
                scene_name=scene.name,robot_model_name=scene.robot_model_name,
                octomap_id=octomap.id,octomap_data_bytes=len(octomap.data),octomap_frame=scene.world.octomap.header.frame_id,
                allowed_collision_matrix_entries=len(scene.allowed_collision_matrix.entry_names),
                fixed_transforms_count=len(scene.fixed_frame_transforms),world_to_odom=dict(frame=relation.header.frame_id,child=relation.child_frame_id),
                request_adapters=adapters)
    with (output/'robot_description.urdf').open('x') as f:f.write(urdf)
    with (output/'robot_description.srdf').open('x') as f:f.write(srdf)
    with (output/'scene_probe.json').open('x') as f:json.dump(result,f,indent=2)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--timeout',type=float,default=10);a=parser.parse_args()
    result=probe(Path(a.output),a.timeout);print(json.dumps(result,sort_keys=True))

if __name__=='__main__':main()

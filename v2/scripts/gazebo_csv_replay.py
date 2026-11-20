"""Single-CSV Gazebo replay; run only against an isolated, prepared simulation."""
import csv, math, os, subprocess, sys, time
from pathlib import Path
import numpy as np
from ..adapters.trajectory_viewer import read_csv

def run(csv_path, output, with_rviz=True):
    import rospy, actionlib, tf, tf2_ros
    from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState
    from gazebo_msgs.msg import ModelState
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
    from actionlib_msgs.msg import GoalStatus
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11511': raise RuntimeError('ISOLATED_MASTER_11511_REQUIRED')
    if not output: raise ValueError('--replay-output is required')
    print('GAZEBO_REPLAY: connecting to simulation; aligning start state before trajectory motion',flush=True)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    times, positions, _ = read_csv(csv_path); t=np.asarray(times); q=np.asarray(positions); t=t-t[0]
    rospy.init_node('v2_gazebo_csv_replay',disable_signals=True)
    if not rospy.get_param('/use_sim_time',False):raise RuntimeError('SIM_TIME_REQUIRED')
    for svc in ('/gazebo/get_model_state','/gazebo/get_link_state','/gazebo/set_model_state','/get_planning_scene'):rospy.wait_for_service(svc,timeout=20)
    model=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState); link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState); setmodel=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState); scene=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
    req=GetPlanningSceneRequest(); req.components.components=PlanningSceneComponents.OCTOMAP
    def octomap():
        m=scene(req).scene.world.octomap
        if not m.octomap.data:raise RuntimeError('EMPTY_PLANNING_SCENE_OCTOMAP')
        return (m.header.frame_id,m.octomap.id,m.octomap.resolution,tuple(m.octomap.data))
    before=octomap(); names=['joint%d'%i for i in range(1,7)]
    def state():
        s=rospy.wait_for_message('/arm_controller/state',JointTrajectoryControllerState,timeout=10)
        if list(s.joint_names)!=names:raise RuntimeError('JOINT_ORDER_MISMATCH')
        return s
    client=actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(15)):raise RuntimeError('ARM_ACTION_UNAVAILABLE')
    def posematrix(p):
        a=tf.transformations.quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);a[:3,3]=[p.position.x,p.position.y,p.position.z];return a
    def poses():
        m=model('robot','world');l=link('robot::car_link','world')
        if not m.success or not l.success:raise RuntimeError('GAZEBO_POSE_UNAVAILABLE')
        return posematrix(m.pose),posematrix(l.link_state.pose)
    m,l=poses(); offset=np.linalg.inv(m).dot(l); rospy.sleep(.15);m2,l2=poses();delta=np.max(np.abs(np.linalg.inv(m2).dot(l2)-offset))
    if delta>.01:raise RuntimeError('MODEL_TO_CAR_NOT_STABLE %.6f'%delta)
    z=l2[2,3]
    def setbase(ref):
        c=tf.transformations.euler_matrix(0,0,float(ref[2]));c[:3,3]=[float(ref[0]),float(ref[1]),float(z)]; m=c.dot(np.linalg.inv(offset)); quat=tf.transformations.quaternion_from_matrix(m)
        s=ModelState();s.model_name='robot';s.reference_frame='world';s.pose.position.x,s.pose.position.y,s.pose.position.z=m[:3,3];s.pose.orientation.x,s.pose.orientation.y,s.pose.orientation.z,s.pose.orientation.w=quat
        if not setmodel(s).success:raise RuntimeError('SET_MODEL_STATE_FAILED')
    # Align arm in a separate motion, before the replay clock starts.
    setbase(q[0]); goal=FollowJointTrajectoryGoal();goal.trajectory.joint_names=names
    p=JointTrajectoryPoint();p.positions=q[0,3:].tolist();p.time_from_start=rospy.Duration(2);goal.trajectory.points=[p];client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(8)) or client.get_state()!=GoalStatus.SUCCEEDED:raise RuntimeError('INITIAL_ARM_ALIGNMENT_FAILED')
    setbase(q[0]); rospy.sleep(.1); s=state();_,actual=poses(); arm_error=np.max(np.abs(np.asarray(s.actual.positions)-q[0,3:])); xy_error=np.max(np.abs(actual[:2,3]-q[0,:2])); yaw=tf.transformations.euler_from_matrix(actual)[2];yaw_error=abs(math.atan2(math.sin(yaw-q[0,2]),math.cos(yaw-q[0,2])))
    if max(arm_error,xy_error,yaw_error)>.01:raise RuntimeError('INITIAL_ALIGNMENT_FAILED %.6g %.6g %.6g'%(xy_error,yaw_error,arm_error))
    # Keep all source event changes; action and base share the same simulated start time.
    with open(csv_path,newline='') as f: raw=list(csv.DictReader(f))
    event=[t[i] for i in range(1,len(t)) if any(raw[i].get(k)!=raw[i-1].get(k) for k in ('chart_index','direction','mode_index'))]
    grid=np.unique(np.r_[np.arange(0,t[-1],.02),t[-1],event]); values=np.stack([np.interp(grid,t,q[:,i]) for i in range(9)],axis=1)
    g=FollowJointTrajectoryGoal();g.trajectory.joint_names=names;t0=rospy.Time.now()+rospy.Duration(1.5);g.trajectory.header.stamp=t0
    for tt,v in zip(grid,values):
        p=JointTrajectoryPoint();p.positions=v[3:].tolist();p.time_from_start=rospy.Duration(float(tt));g.trajectory.points.append(p)
    rviz=None
    if with_rviz:
        rviz=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parent/'replay_rviz.py')])
    tfbuf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tfbuf);client.send_goal(g);records=[];rate=rospy.Rate(50)
    print('GAZEBO_REPLAY: trajectory scheduled at simulation time %.3f for %.3f seconds'%(t0.to_sec(),t[-1]),flush=True)
    try:
        while not rospy.is_shutdown():
            now=rospy.Time.now();elapsed=(now-t0).to_sec()
            if elapsed<0:rate.sleep();continue
            if elapsed>t[-1]:break
            if client.get_state() in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.LOST):raise RuntimeError('ARM_ACTION_FAILED '+str(client.get_state()))
            ref=np.array([np.interp(elapsed,t,q[:,i]) for i in range(9)]);setbase(ref); _,p=poses();s=state();actual_yaw=tf.transformations.euler_from_matrix(p)[2]
            tr=tfbuf.lookup_transform('world','car_link',rospy.Time(0),rospy.Duration(.2));records.append([elapsed,now.to_sec(),s.header.stamp.to_sec(),*ref,*p[:2,3],actual_yaw,*s.actual.positions,tr.header.stamp.to_sec(),tr.transform.translation.x,tr.transform.translation.y])
            rate.sleep()
        if not client.wait_for_result(rospy.Duration(12)) or client.get_state()!=GoalStatus.SUCCEEDED:raise RuntimeError('ARM_ACTION_DID_NOT_FINISH')
        after=octomap();
        if before!=after:raise RuntimeError('OCTOMAP_CHANGED')
        print('REPLAY_COMPLETED_SAMPLES',len(records),'T',float(t[-1]),'ARM_INITIAL_ERROR',float(arm_error),'XY_INITIAL_ERROR',float(xy_error),'YAW_INITIAL_ERROR',float(yaw_error),'MAP_BYTES',len(before[3]),flush=True)
    finally:
        if records:
            with (output/'state.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['elapsed','sim','arm_stamp']+['ref_%d'%i for i in range(9)]+['x','y','yaw']+names+['tf_stamp','tf_x','tf_y']);w.writerows(records)
        if client.get_state() not in (GoalStatus.SUCCEEDED,GoalStatus.ABORTED,GoalStatus.REJECTED):client.cancel_goal()
        if rviz is not None:rviz.terminate();rviz.wait(timeout=10)

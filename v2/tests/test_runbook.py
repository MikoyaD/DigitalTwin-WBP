import json,os,subprocess,tempfile,unittest,io
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
from hashlib import sha256
from v2.scripts import experiment
from v2.scripts.planning_request import build
from v2.tests.support import PACKAGE,SCHEMA
from v2.adapters.moveit_input import capture
from v2.scripts.validate_g import audit

def args(**k):return S(config=str(experiment.DEFAULT),stage='all',run_id=k.get('run_id'),input=k.get('input'),headless=False,dry_run=k.get('dry_run',False))
class RunbookTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        patcher=patch.object(experiment,'RUNS',Path(temp.name));patcher.start();self.addCleanup(patcher.stop);self.folder=Path(temp.name)
    def test_clean_environment(self):
        env=dict(os.environ,ROS_DISTRO='humble',AMENT_PREFIX_PATH='/tmp/ros2')
        r=subprocess.run([str(PACKAGE/'v2/scripts/run_experiment.sh'),'--help'],cwd=str(PACKAGE),env=env,capture_output=True,text=True,timeout=8)
        self.assertEqual(r.returncode,0,r.stderr);self.assertIn('ROS_DISTRO=noetic',r.stdout);self.assertNotIn('/tmp/ros2',r.stdout)
    def test_run_files_and_hashes(self):
        r=experiment.Run(args(dry_run=True))
        with patch.object(r,'inventory'):r.command('dry',['python3','-V'],'commands.log',0);r.finish()
        m=experiment.read(r.root/'manifest.json');self.assertEqual(m['status'],'DRY_RUN')
        self.assertTrue(all((r.root/n).is_file() for n in experiment.ARTIFACTS))
        self.assertEqual(m['artifacts']['environment.json']['sha256'],experiment.digest(r.root/'environment.json'))
    def test_resume_creates_child_and_immutable_config(self):
        p=experiment.Run(args(dry_run=True))
        with patch.object(p,'inventory'):p.finish()
        c=experiment.Run(args(run_id=p.id,dry_run=True));self.assertEqual(c.manifest['parent_run_id'],p.id)
        self.assertNotEqual(c.root,p.root)
        with patch.object(c,'inventory'):c.finish()
        bad=self.folder/'changed.yaml';bad.write_text(experiment.DEFAULT.read_text()+'\n#changed\n')
        a=args(run_id=p.id);a.config=str(bad)
        with self.assertRaisesRegex(ValueError,'CONFIG_CHANGED'):experiment.Run(a)
    def test_all_stage_uses_own_preflight_before_parent(self):
        parent=experiment.Run(args(dry_run=True))
        with patch.object(parent,'inventory'):parent.finish()
        child=experiment.Run(args(run_id=parent.id,dry_run=True))
        folder=child.root/'preflight';folder.mkdir()
        experiment.put(folder/'scene_probe.json',dict(scene_sha256='a'*64))
        self.assertEqual(child.parent_scene()['scene_sha256'],'a'*64)
    def test_failure_exit_and_log(self):
        r=experiment.Run(args())
        with self.assertRaisesRegex(RuntimeError,'exit 7'):r.command('fail',['python3','-B','-c','raise SystemExit(7)'],'planning.log',3)
        self.assertIn('exit=7',(r.root/'planning.log').read_text());self.assertEqual(r.manifest['commands'][-1]['exit_code'],7)
    def test_stop_unowned_has_no_kill(self):
        r=experiment.Run(args(dry_run=True))
        with patch.object(r,'inventory'):r.finish()
        with patch.object(experiment.os,'killpg') as kill:experiment.stop(S(run_id=r.id));kill.assert_not_called()
    def test_request_has_nine_variables(self):
        c=experiment.yaml.safe_load(experiment.DEFAULT.read_text());p=build(c,c['tasks'][0],PACKAGE)
        req=p['motion_plan_request'];self.assertEqual(len(req['goal_constraints'][0]['joint_constraints']),9)
        self.assertEqual(req['planner_id'],'RRTstar');self.assertEqual(req['group_name'],'whole')
    def test_ros_message_roundtrip_and_g_repeat(self):
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint,MultiDOFJointTrajectoryPoint
        from geometry_msgs.msg import Transform
        from genpy import Duration
        msg=RobotTrajectory();msg.joint_trajectory.joint_names=list(SCHEMA.joint_names[3:]);msg.multi_dof_joint_trajectory.joint_names=['world_car'];msg.multi_dof_joint_trajectory.header.frame_id='world'
        for i in (0,1):
            j=JointTrajectoryPoint();j.positions=[.1*i]*6;j.time_from_start=Duration(i);msg.joint_trajectory.points.append(j)
            m=MultiDOFJointTrajectoryPoint();tr=Transform();tr.translation.x=i;tr.rotation.w=1;m.transforms=[tr];m.time_from_start=Duration(i);msg.multi_dof_joint_trajectory.points.append(m)
        buf=io.BytesIO();msg.serialize(buf);wire=buf.getvalue();msg2=RobotTrajectory().deserialize(wire)
        goal=[1.,0.,0.]+[.1]*6
        meta=dict(trial_id='ros_message_test',planner_id='RRTstar',input_stage='post_TOTG',frame_id='world',requested_start=[0.]*9,requested_goal=goal,request={'test':True},seed_status='unknown')
        self.assertEqual(capture(msg2,meta).parent_path_hash,sha256(wire).hexdigest())
        q=lambda value,unit:dict(value=value,unit=unit,kind='research_limit',source='explicit synthetic test')
        limits=dict(xy_speed=q(.5,'m/s'),xy_acceleration=q(.5,'m/s^2'),tangential_acceleration=q(.5,'m/s^2'),wheel_velocity=q([8.333333333333334]*2,'rad/s'),wheel_acceleration=q([8.333333333333334]*2,'rad/s^2'),spin_jerk=q(2.,'rad/s^3'))
        inp=self.folder/'input.json';inp.write_text(json.dumps(dict(robot_trajectory_hex=wire.hex(),metadata=meta,research_limits=limits,velocity_scaling=1.,acceleration_scaling=1.)))
        out=[]
        for i in (0,1):
            target=self.folder/('g%d'%i)
            r=subprocess.run(['python3','-B','-m','v2.adapters.run_g',str(inp),'--output-dir',str(target)],cwd=str(PACKAGE),capture_output=True,text=True,timeout=35)
            self.assertEqual(r.returncode,2,r.stdout+r.stderr)
            self.assertEqual(audit(target,inp,False)['numerical_status'],'PASS')
            out.append(target)
        self.assertEqual(experiment.digest(out[0]/'trajectory.csv'),experiment.digest(out[1]/'trajectory.csv'))

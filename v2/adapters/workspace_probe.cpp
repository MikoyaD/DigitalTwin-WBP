// Read-only OMPL context inspection; never invokes solve or executes trajectories.
#include <ros/ros.h>
#include <ros/serialization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <fstream>
#include <iterator>
#include <iomanip>
int main(int argc,char**argv){
 ros::init(argc,argv,"v2_workspace_context_probe",ros::init_options::AnonymousName);ros::AsyncSpinner spinner(1);spinner.start();
 if(argc<3)return 2;
 std::ifstream in(argv[1],std::ios::binary);std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),{});
 moveit_msgs::GetMotionPlanRequest message;ros::serialization::IStream stream(bytes.data(),bytes.size());ros::serialization::deserialize(stream,message);
 auto req=message.motion_plan_request;auto&w=req.workspace_parameters;
 if(w.min_corner.x==0&&w.max_corner.x==0&&w.min_corner.y==0&&w.max_corner.y==0&&w.min_corner.z==0&&w.max_corner.z==0){
 double extent=10;ros::param::param("/move_group/planning_pipelines/ompl/default_workspace_bounds",extent,10.);if(argc>3)extent=std::stod(argv[3]);
 w.min_corner.x=w.min_corner.y=w.min_corner.z=-extent/2;w.max_corner.x=w.max_corner.y=w.max_corner.z=extent/2;
 }
 robot_model_loader::RobotModelLoader loader("robot_description");auto model=loader.getModel();
 planning_scene::PlanningScenePtr scene(new planning_scene::PlanningScene(model));
 ompl_interface::OMPLInterface planner(model,ros::NodeHandle("/move_group/planning_pipelines/ompl"));moveit_msgs::MoveItErrorCodes error;
 auto context=planner.getPlanningContext(scene,req,error);if(!context)return 3;
 auto space=context->getOMPLStateSpace();const auto&joints=space->getJointModelGroup()->getActiveJointModels();const auto&bounds=space->getJointsBounds();
 moveit::core::RobotState start(model);start.setToDefaultValues();moveit::core::robotStateMsgToRobotState(req.start_state,start);
 std::ofstream out(argv[2]);out<<std::setprecision(17)<<"{\"model_frame\":\""<<model->getModelFrame()<<"\",\"requested_workspace_frame\":\""<<req.workspace_parameters.header.frame_id<<"\",\"space_name\":\""<<space->getName()<<"\",\"parameterization\":\""<<space->getParameterizationType()<<"\",\"variables\":[";bool first=true;
 for(size_t j=0;j<joints.size();++j)for(size_t v=0;v<joints[j]->getVariableCount();++v){
 auto name=joints[j]->getVariableNames()[v];auto b=(*bounds[j])[v];double value=start.getVariablePosition(name);double goal=value;for(const auto& c:req.goal_constraints)for(const auto& g:c.joint_constraints)if(g.joint_name==name)goal=g.position;
 if(!first)out<<",";first=false;
 out<<"{\"name\":\""<<name<<"\",\"bounded\":"<<(b.position_bounded_?"true":"false")<<",\"lower\":"<<b.min_position_<<",\"upper\":"<<b.max_position_<<",\"goal\":"<<goal<<",\"goal_inside\":"<<(!b.position_bounded_||(goal>=b.min_position_&&goal<=b.max_position_)?"true":"false")<<",\"start\":"<<value<<",\"start_inside\":"<<(!b.position_bounded_||(value>=b.min_position_&&value<=b.max_position_)?"true":"false")<<"}";
 }
 out<<"],\"solve_called\":false}";return 0;
}

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
// for Object detection
#include <ros/ros.h>
#include <time.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometric_shapes/shape_operations.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/scoped_ptr.hpp>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/ompl_interface/ompl_interface.h>


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>



namespace pr2_matlab_bridge_ns{

class PR2MatlabBridgeClass
{
private:
  //ros::NodeHandle n;
  bool ready_;


  // Moveit
  boost::shared_ptr<move_group_interface::MoveGroup> group_;
  // boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::Publisher planning_scene_diff_publisher_;
  ros::ServiceClient client_get_scene_;
                                                                                                                        

  // The trajectory variables                                                                                                                                                     
  double    circle_phase_;      // Phase along the circle                                                                                                                         
  ros::Time last_time_;         // Time of the last servo cycle   
  ros::Time time_step;          // estimated Time to reach the next controll point
  double dtime;

  // Publisher for the GripperPose, ObjectPointcloud
  ros::Publisher Gripper_Pos_pub;  
  ros::Publisher Torso_Pos_pub;
  ros::Publisher Marker_Pos_pub;
  ros::Publisher ObjPointCloud_pub;
  ros::Publisher TrajectoryPublisher;
  ros::Publisher Test_Res_pub;

  // Subscriber for the desired Trajectory, gripperstate, requests
  ros::Subscriber Precomp_traj_sub;      
  ros::Subscriber OCGripper_sub;
  ros::Subscriber PR2Request_sub;
  std::map<std::string, ros::Subscriber> pose_subscribers_;
  
  // action client for Gripper
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
  GripperClient* gripper_client_;     

  //ros::ServiceClient executeKnownTrajectoryServiceClient; 
  //ros::ServiceClient cartesianPathClient; 

  // tf listener
  tf::TransformListener listener;
  tf::StampedTransform transformTorso_;
  tf::StampedTransform transformMarker_;

  // Object detection
  //ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoalConstPtr objectgoal;

  bool stop_motion;  

public:
  PR2MatlabBridgeClass(ros::NodeHandle n);
  bool start();
  void update();
  void stopping();
  
  void DesiredPoseCallback(const geometry_msgs::PoseStamped msg);
  void PrecompTrajCallback(const geometry_msgs::PoseArray msg);
  void ChangeGripperStateCallback(const std_msgs::String msg);
  void RequestsCallback(const std_msgs::String msg);
  void allow_obj_collision(const std::string);

  //void callObjectHandling(const ros::MessageEvent<geometry_msgs::PoseStamped const>& event, const std::string& object_name);
  void callObjectHandling(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& object_name);
};
}

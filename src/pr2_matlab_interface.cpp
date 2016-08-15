// This Version executes the trajectory with the ee_imped_action:
// http://wiki.ros.org/ee_cart_imped/Tutorials/Writing%20a%20Stiffness%20Controller%20%28C%2B%2B%29

#include "/home/twelsche/catkin_ws/src/pr2_matlab_interface/include/pr2_matlab_interface/pr2_matlab_interface.h"
#include "/home/twelsche/catkin_ws/src/pr2_matlab_interface/include/pr2_matlab_interface/iterative_time_parameterization_dmp.h"



using namespace pr2_matlab_bridge_ns;


/// Class initialization in non-realtime                                                                                                                                     
PR2MatlabBridgeClass::PR2MatlabBridgeClass(ros::NodeHandle n)
{

  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
  group_.reset(new moveit::planning_interface::MoveGroup ("right_arm"));

  
  // Client for gripper controller
  gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  // open the Gripper on start:  
  std_msgs::String String_msg;
  String_msg.data = "open";
  ChangeGripperStateCallback(String_msg);



  // Start subscriber for the gripperState 
  OCGripper_sub = n.subscribe("/PRCommunicator/OCGripper", 1, &PR2MatlabBridgeClass::ChangeGripperStateCallback,this);
  // Start subscriber for the Trajectory
  // Precomp_traj_sub = n.subscribe("/PRCommunicator/Precomp_Trajectory", 1, &PR2MatlabBridgeClass::PrecompTrajCallback,this);   
  // Start subscriber for the Requests
  PR2Request_sub = n.subscribe("/PRCommunicator/Request", 1, &PR2MatlabBridgeClass::RequestsCallback,this);

  // Subscriber for the object tracking
  std::vector<std::string> model_names;
  //model_names.push_back("bowl");
  model_names.push_back("muesli2");
  model_names.push_back("KallaxDrawer2");
  model_names.push_back("Kallax_Tuer");

  for (std::vector<std::string>::iterator it = model_names.begin(); it != model_names.end();it++) {
    //pose_subscribers_[*it] = n.subscribe<geometry_msgs::PoseStamped>("/simtrack/" + *it, 50,boost::bind(&PR2MatlabBridgeClass::callObjectHandling,this,_1,*it));
    pose_subscribers_[*it] = n.subscribe<geometry_msgs::PoseStamped>("/PRCommunicator/" + *it, 50,boost::bind(&PR2MatlabBridgeClass::callObjectHandling,this,_1,*it));
  }



  // Start publisher for the Griper Pose as geometry_msgs/PoseStamped.msg
  Gripper_Pos_pub = n.advertise<geometry_msgs::PoseStamped>("Gripper_Pose", 1,false);
  // Start publisher for Robot Torso_lift_link Pose as geometry_msgs/PoseStamped
  Torso_Pos_pub = n.advertise<geometry_msgs::PoseStamped>("Base_Pose", 1,false);
  // Publisher for robotTrajectory
  TrajectoryPublisher = n.advertise<moveit_msgs::DisplayTrajectory>("RobotTrajectory",1,false);
  // Start moveit stuff
  planning_scene_diff_publisher_ = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1,true);
  client_get_scene_ = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::PlanningScene currentScene;
  moveit_msgs::GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;


  ready_ = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Controller startup in realtime                                                                                                                                                
bool PR2MatlabBridgeClass::start()
{
  if (!ready_) {
    return false;
  }                                                                                                                                    
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Controller update loop in realtime                                                                                                                                            
void PR2MatlabBridgeClass::update()
{
  // Find and publish new Gripper Pose every xt seconds
  
    tf::StampedTransform transform;
    listener.lookupTransform("/odom_combined", "/r_wrist_roll_link", ros::Time(0), transform);

    // group_->getCurrentPose and pose known to tf differ in sign in quaternion for certain poses.  
    geometry_msgs::PoseStamped gripper_position = group_->getCurrentPose();
    gripper_position.pose.orientation.x  = transform.getRotation().x();
    gripper_position.pose.orientation.y  = transform.getRotation().y();
    gripper_position.pose.orientation.z  = transform.getRotation().z();
    gripper_position.pose.orientation.w  = transform.getRotation().w();
    
    gripper_position.header.stamp = ros::Time::now();

    //Publish gripper pose
    Gripper_Pos_pub.publish(gripper_position);

    //get the current transform for robot base
    try
    {
    listener.lookupTransform("odom_combined", "torso_lift_link",
                              ros::Time(0), transformTorso_);
    }
    catch (tf::TransformException ex)
    {
    ROS_ERROR("%s",ex.what());
    }
    geometry_msgs::PoseStamped torso_position;
    torso_position.pose.position.x  = transformTorso_.getOrigin().x();
    torso_position.pose.position.y  = transformTorso_.getOrigin().y();
    torso_position.pose.position.z  = transformTorso_.getOrigin().z();
    torso_position.pose.orientation.x  = transformTorso_.getRotation().x();
    torso_position.pose.orientation.y  = transformTorso_.getRotation().y();
    torso_position.pose.orientation.z  = transformTorso_.getRotation().z();
    torso_position.pose.orientation.w  = transformTorso_.getRotation().w();
    Torso_Pos_pub.publish(torso_position);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Controller stopping in realtime                                                                                                                                               
void PR2MatlabBridgeClass::stopping()
{
  // TODO 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback function to set desired Pose acording to received Pose message'geometry_msgs/'
void PR2MatlabBridgeClass::DesiredPoseCallback(const geometry_msgs::PoseStamped msg)
{
  // traj_nextgoal_.trajectory[0].pose = msg.pose;
  // traj_nextgoal_.trajectory[0].time_from_start = ros::Duration(msg.header.stamp.toSec());
  // traj_nextgoal_.trajectory[0].header.stamp = ros::Time(0);
  // traj_nextgoal_.header.stamp = ros::Time::now();
  // // traj_client_->cancelAllGoals();   
  // traj_client_->sendGoal(traj_nextgoal_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback function to execute precomputed trajectory
void PR2MatlabBridgeClass::PrecompTrajCallback(const geometry_msgs::PoseArray msg)
{
  unsigned int traj_length = msg.poses.size();
  ROS_INFO("Trajectory msg with length %u received",traj_length);
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> torsoPoints;
  tf::Transform offset;
  offset.setIdentity();
  tf::Vector3 translationOff(0.05,0.0,0.0);
  offset.setOrigin(translationOff);
  tf::Transform gripperOffset;
  gripperOffset.setIdentity();
  tf::Vector3 gripperTranslationOff(-0.18,0.0,0.0);
  gripperOffset.setOrigin(gripperTranslationOff);
  for (unsigned int j = 0 ; j < traj_length ; j++)
  {
    if (j % 2)
    {
      tf::Transform torsoTransform;
      tf::poseMsgToTF(msg.poses[j],torsoTransform);      
      torsoTransform = torsoTransform*offset;
      geometry_msgs::Pose poseMsg;
      tf::poseTFToMsg(torsoTransform,poseMsg);
      torsoPoints.push_back(poseMsg); 
    }      
    else
    {
      tf::Transform gripperTransform;
      tf::poseMsgToTF(msg.poses[j],gripperTransform);
      // gripperTransform = gripperTransform*gripperOffset;
      tf::Transform torsoTransform;
      tf::poseMsgToTF(msg.poses[j+1],torsoTransform);
      // torsoTransform = torsoTransform*offset;
      torsoTransform = transformTorso_.inverse()*torsoTransform;  
      gripperTransform = torsoTransform.inverse()*gripperTransform;
      geometry_msgs::Pose poseMsg;
      tf::poseTFToMsg(gripperTransform,poseMsg);
      waypoints.push_back(poseMsg);
    }
  }
  ROS_INFO_STREAM("Startpoint after Transform:");
  ROS_INFO_STREAM(waypoints[0]);
  // stop any running motion before staring the new
  group_->stop(); 
  group_->setPlanningTime(30);
  //compute joint trajectory
  moveit_msgs::RobotTrajectory trajectory_msg;
  moveit_msgs::RobotTrajectory fullBodyTraj_msg;
  // double fraction = group_->computeCartesianPath(waypoints,1.9, 15.5, trajectory_msg,true); // jump_threshold  // eef_step
  double fraction = group_->computeCartesianPath(waypoints,20.0, 0.0, trajectory_msg,false); 
  ROS_INFO_STREAM(fraction);




  // add torso poses to Trajectory
  fullBodyTraj_msg.joint_trajectory.joint_names = trajectory_msg.joint_trajectory.joint_names;
  fullBodyTraj_msg.joint_trajectory.joint_names.push_back("torso_lift_joint");
  fullBodyTraj_msg.joint_trajectory.header = msg.header;
  fullBodyTraj_msg.multi_dof_joint_trajectory.header.frame_id = "odom_combined";
  fullBodyTraj_msg.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
  traj_length = trajectory_msg.joint_trajectory.points.size()-1;

  for (int i = 0; i < traj_length;i++)
  {
    // Torso_lift_link trajectory
    trajectory_msgs::JointTrajectoryPoint jointPoint;
    jointPoint = trajectory_msg.joint_trajectory.points[i];
    int closestRelative;
    closestRelative = i;
    // add torso position
    jointPoint.positions.push_back(torsoPoints[closestRelative].position.z-0.8);
    fullBodyTraj_msg.joint_trajectory.points.push_back(jointPoint);
    // Base Trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint;
    geometry_msgs::Transform transform;
    transform.translation.x = torsoPoints[closestRelative].position.x;//+ offset.getOrigin().getX()
    transform.translation.y = torsoPoints[closestRelative].position.y;//+ offset.getOrigin().getY()
    transform.translation.z = 0; 
    transform.rotation = torsoPoints[closestRelative].orientation; 
    basePoint.transforms.push_back(transform);
    fullBodyTraj_msg.multi_dof_joint_trajectory.points.push_back(basePoint); 
  }
  // Build Display Trajectory
  moveit_msgs::DisplayTrajectory displayTrajectory;
  displayTrajectory.model_id = "pr2";
  displayTrajectory.trajectory.push_back(fullBodyTraj_msg);
  // Set startstate
  moveit_msgs::RobotState start_state;
  int m_num_joints = 8;
  start_state.joint_state.name.resize(m_num_joints);
  start_state.joint_state.position.resize(m_num_joints);
  start_state.joint_state.velocity.resize(m_num_joints); 
  start_state.joint_state.name = fullBodyTraj_msg.joint_trajectory.joint_names;
  for (int j = 0 ; j < m_num_joints; j++)
  {
      start_state.joint_state.position[j] = fullBodyTraj_msg.joint_trajectory.points[0].positions[j];  
  }
  start_state.multi_dof_joint_state.header.frame_id = "odom_combined";
  start_state.multi_dof_joint_state.joint_names.push_back("world_joint");
  geometry_msgs::Transform startTransform;
  startTransform.translation.x = 0;
  startTransform.translation.y = 0;
  startTransform.translation.z = 0;
  startTransform.rotation.x = 0;
  startTransform.rotation.y = 0;
  startTransform.rotation.z = 0;
  startTransform.rotation.w = 1;
  start_state.multi_dof_joint_state.transforms.push_back(startTransform);
  displayTrajectory.trajectory_start = start_state; //set robot start state
  ROS_INFO_STREAM("Publish fullBodyTraj_msg:");
  TrajectoryPublisher.publish(displayTrajectory);



  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group_->getCurrentState()->getRobotModel(), "right_arm");
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group_->getCurrentState(), trajectory_msg);
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing_Tim::IterativeParabolicTimeParameterization_Tim iptp;
  // Fourth compute computeTimeStamps
  // this lib does not actually work properly when angles wrap around, so we need to unwind the path first
  rt.unwind();
  const int num_points = rt.getWayPointCount();
  std::vector<double> time_diff(num_points-1, 0.0);
  for (int i = 0 ; i < num_points-1 ; ++i)
  {
    time_diff[i] = msg.header.stamp.toSec()/num_points;
  }  

  bool success = iptp.computeTimeStamps(rt,time_diff);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  // rt.getRobotTrajectoryMsg(fullBodyTraj_msg);
  // Finally plan and execute the trajectory
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory_msg;
  ROS_INFO_STREAM(trajectory_msg.joint_trajectory.header.frame_id);
  ROS_INFO("Executing cartesian path plan (%.2f%% acheived)",fraction * 100.0);   
  // group_->asyncExecute(plan);
  waypoints.clear();  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback function to open/close the gripper acording to received message
void PR2MatlabBridgeClass::ChangeGripperStateCallback(const std_msgs::String msg)
{
  // display the received message  
  ROS_INFO_STREAM("Open/close gripper message received:");
  ROS_INFO_STREAM(msg);

  pr2_controllers_msgs::Pr2GripperCommandGoal stCommand;
  if (msg.data == "open"){
    stCommand.command.position = 0.08;
    stCommand.command.max_effort = -1.0;  // Do not limit effort (negative)
  } 
  else if (msg.data == "close"){
    stCommand.command.position = 0.0;
    stCommand.command.max_effort = -1.0;  // Close gently
  }
  else {
    ROS_INFO_STREAM("Gripper Command not recongnized");
  }

  gripper_client_->sendGoal(stCommand);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened/closed!");
  else
    ROS_INFO("The gripper failed to open/close.");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback function to serve the Requests from Matlab controller acording to received message
void PR2MatlabBridgeClass::RequestsCallback(const std_msgs::String msg)
{
  // display the received message  
  ROS_INFO_STREAM("Request message received:");
  ROS_INFO_STREAM(msg);
  // TODO: get request from matlab which object to allow!
  if (msg.data == "ObjectGrasp"){
    //allow_obj_collision("bowl");
    //allow_obj_collision("muesli2");
    //allow_obj_collision("KallaxDrawer2");
  }
  if (msg.data == "stop motion"){
    group_->stop();
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
void PR2MatlabBridgeClass::callObjectHandling(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& object_name)
{
  // checl if object is already in known collision_objects
  moveit_msgs::CollisionObject collision_object;
  bool object_in_world = false;
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  ROS_INFO_STREAM("looking for object in world scene");
  if (client_get_scene_.call(srv))
  {
      for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
      {
          if (srv.response.scene.world.collision_objects[i].id == object_name)
          {
            object_in_world = true;
            collision_object = srv.response.scene.world.collision_objects[i];
          }              
          ROS_INFO_STREAM(srv.response.scene.world.collision_objects[i].id);
      }
  }
  
  if (object_in_world)
  {
    // Object known -> update pose
    geometry_msgs::Pose obj_pose = msg->pose;
    //collision_object.primitive_poses.push_back(obj_pose);
    collision_object.mesh_poses.push_back(obj_pose);    
    collision_object.operation = collision_object.MOVE;
    ROS_INFO_STREAM("Object already in scene. Updating pose");
  } 
  else
  { 
    ROS_INFO_STREAM("Object not yet in scene. Adding collision_object");   
    // Object unknown -> add to planning scene
    collision_object.header.frame_id = group_->getPlanningFrame();
    /* The id of the object is used to identify it. */
    collision_object.id = object_name;  
    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose obj_pose = msg->pose;
    
   
    
    //shapes::Mesh* obj_shape = shapes::createMeshFromResource("file:///home/twelsche/code/PR2/SimTrak/src/simtrack/data/object_models/" + object_name +"_Simplified/"+object_name+".obj");
    shapes::Mesh* obj_shape = shapes::createMeshFromResource("file:///home/twelsche/code/PR2/SimTrak/src/simtrack/data/object_models/" + object_name +"/"+object_name+".obj");
    if (!obj_shape)
    {
      // assimp can't load mesh off kallaxDrawer, insert box instead
      ROS_INFO_STREAM("Assimp cannot import Mesh. Build box instead"); 
      /* Define a box to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.4;
      primitive.dimensions[1] = 0.4;
      primitive.dimensions[2] = 0.1;
      collision_object.primitive_poses.push_back(obj_pose);
      collision_object.primitives.push_back(primitive);
    }
    else
    {
      shape_msgs::Mesh obj_mesh;    
      shapes::ShapeMsg obj_mesh_msg = obj_mesh;
      shapes::constructMsgFromShape(obj_shape,obj_mesh_msg);
      obj_mesh = boost::get<shape_msgs::Mesh>(obj_mesh_msg);
      collision_object.meshes.push_back(obj_mesh);
      collision_object.mesh_poses.push_back(obj_pose);
    }
      
      
    collision_object.operation = collision_object.ADD;    
  }
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  moveit_msgs::PlanningScene planning_scene_add;
  planning_scene_add.world.collision_objects.push_back(collision_object);
  planning_scene_add.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_add);
  ROS_INFO_STREAM("Object added.");   
  allow_obj_collision(object_name);
}


void PR2MatlabBridgeClass::allow_obj_collision(const std::string object_name)
{
  // allow collisions for specified object
  moveit_msgs::PlanningScene currentScene;
  moveit_msgs::PlanningScene newSceneDiff;
  moveit_msgs::GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;
  if(!client_get_scene_.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    ROS_INFO_STREAM("Allowing Collisions");  
    currentScene = scene_srv.response.scene;
    moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;
    // TODO check if collision is already allowed
    currentACM.entry_names.push_back(object_name);
    moveit_msgs::AllowedCollisionEntry entry;
    entry.enabled.resize(currentACM.entry_names.size());
    for(int i = 0; i < entry.enabled.size(); i++)
        entry.enabled[i] = true;       
    //add new row to allowed collsion matrix
    currentACM.entry_values.push_back(entry);
    for(int i = 0; i < currentACM.entry_values.size(); i++)
    {
        //extend the last column of the matrix
        currentACM.entry_values[i].enabled.push_back(true);
    }
    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher_.publish(newSceneDiff);
  }   

  if(!client_get_scene_.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    ROS_INFO_STREAM("Modified scene!");
    currentScene = scene_srv.response.scene;
    moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;
  }

}

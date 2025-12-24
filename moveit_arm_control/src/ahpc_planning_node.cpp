#include "ahpc_planning_node.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ahpc_planning");

static const std::string PLANNING_GROUP = "jaka_arm";
static const double PLANNING_TIME_S = 30.0;
static const double PLANNING_ATTEMPTS = 5.0;

robot_driver_status::robot_driver_status()
{
	this->ts_.sec = 0;
	this->ts_.nanosec = 0;
	this->connect_status_ = false;                 /* 连接状态 */
	this->enable_status_ = false;                 /* 使能状态 */
	this->is_robot_stop_ = true;                  /* 机器人是否停止 */
	this->in_collision_ = false;                   /* 处在碰撞状态 */
	this->in_emergency_ = false;                   /* 处在急停状态 */
	this->is_free_drive_ = false;                  /* 是否处在自由模式 */
	this->errcode_ = 0;                         /* 错误码 */
	this->collision_level_ = 0;                 /* 碰撞等级 */
	this->tcp_pose_.clear();        /* 工具TCP姿态 */
	this->joint_pose_.clear();      /* 关节姿态 */
	this->tool_id_ = 0;                         /* 工具编号 */
	this->tool_tcp_.clear();        /* 工具TCP */
	this->mass_ = 0;                         /* 负载重量 */
	this->center_of_mass_.clear();  /* 负载重心 */
	this->force_control_enable_ = false;           /* 是否处在力控模式 */
	this->torque_adjust_enable_ = false; /* 力控调整 */
	this->torque_.clear();
	this->act_torque_.clear();
	this->torque_extract_adjust_enable_ = false; /* 力控调整 */
	this->admit_extract_enable_ = false;
	this->admit_extract_seq_ = 0;
	return;
}
AHPCPlanning::AHPCPlanning(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ahpc_planning_node", options) }
{
	robot_driver_status_sub_ = node_->create_subscription<shared_message::msg::RobotDriverStatus>(
      "/jaka_zu5_driver/robot_states", 10, std::bind(&AHPCPlanning::robotDriverStatusHandle, this, std::placeholders::_1));
	move_to_location_pos_srv_ = node_->create_service<shared_message::srv::MoveToLocationPosition>("move_to_location_position", std::bind(&AHPCPlanning::moveToLocationPosition, this, std::placeholders::_1,std::placeholders::_2));
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    move_group_interface_->setPlanningTime(PLANNING_TIME_S);
    move_group_interface_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr AHPCPlanning::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}
AHPCPlanning::~AHPCPlanning()
{
    
}
void AHPCPlanning::moveToLocationPosition(const std::shared_ptr<shared_message::srv::MoveToLocationPosition::Request> request,
                                          std::shared_ptr<shared_message::srv::MoveToLocationPosition::Response> response)
{
	RCLCPP_INFO(LOGGER,"input loc_type = %d",request->loc_type);
	if(request->pose.size() == 6)
		RCLCPP_INFO(LOGGER,"input pose: x = %f,y = %f, z =%f,yaw = %f,pitch =%f,roll =%f",request->pose[0],request->pose[1],request->pose[2]
		            ,request->pose[3],request->pose[4],request->pose[5]);
	const moveit::core::JointModelGroup* joint_model_group = move_group_interface_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
	// Now, we call the planner to compute the plan and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// When done with the path constraint be sure to clear it.
	move_group_interface_->clearPoseTargets();
	move_group_interface_->clearPathConstraints();
	response->result = false;
	moveit::core::RobotStatePtr cur_state = move_group_interface_->getCurrentState();
	if(!cur_state)
	{
		return;
	}
	if(request->loc_type == 0)// Cartesian Paths 
	{
		// Raw pointers are frequently used to refer to the planning group for improved performance.
		// moveit::core::RobotState start_state(*cur_state);
		// geometry_msgs::msg::Pose start_pose2;//目标位置2
		// start_pose2.position.x = 0.0;
		// start_pose2.position.y = 0.0;
		// start_pose2.position.z = -0.013;
		// start_pose2.orientation.w = 1.0; //表示无旋转，默认朝向
		// 2. 绕z轴旋转90度
		// orientation.x = 0.0;
		// orientation.y = 0.0;
		// orientation.z = 0.7071;  // sin(45°) = 0.7071
		// orientation.w = 0.7071;  // cos(45°) = 0.7071
		//move_group_interface_->setStartStateToCurrentState();//设置当前状态为开始位置（表示1）
		//Purpose: It computes and sets the joint positions (i.e., the robot's configuration) in start_state by solving inverse kinematics (IK) 
		//for the specified joint group to reach a target end-effector pose.
		// bool ik_success = start_state.setFromIK(joint_model_group, start_pose2);
        // if(ik_success)
		// {
		// 	RCLCPP_ERROR(LOGGER,"IK fail");
		// 	return;
		// }
        
		move_group_interface_->setStartStateToCurrentState();//设置当前状态为开始位置（表示1）
		// Cartesian Paths
  		geometry_msgs::msg::Pose start_pose = move_group_interface_->getCurrentPose().pose;
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(start_pose);

		geometry_msgs::msg::Pose target_pose2 = start_pose;
		target_pose2.position.z -= 0.013;  // 往后退13mm
		waypoints.push_back(target_pose2);
		
		geometry_msgs::msg::Pose target_pose3 = target_pose2;
        //-0.082, 0.014, -0.250定位位置
		target_pose3.position.x -= 0.082;
		target_pose3.position.y += 0.014;
		target_pose3.position.z -= 0.250;
		waypoints.push_back(target_pose3);  

		// We want the Cartesian path to be interpolated at a resolution of 1 cm
		// which is why we will specify 0.01 as the max step in Cartesian
		// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
		// Warning - disabling the jump threshold while operating real hardware can cause
		// large unpredictable motions of redundant joints and could be a safety issue
		moveit_msgs::msg::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.005;
		double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		RCLCPP_INFO(LOGGER, "plan 0 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
        if(fraction < 1.0)
		{
			return;
		}
		// You can execute a trajectory like this.
		move_group_interface_->execute(trajectory);
		response->result = true;
	}
	else if(request->loc_type == 1) //send pose goal
	{
		// Planning to a Pose goal
		geometry_msgs::msg::Pose target_pose1;
        
		// Eigen::Quaterniond q = Eigen::AngleAxisd(request->pose[5], Eigen::Vector3d::UnitZ())*
		// 					   Eigen::AngleAxisd(request->pose[4], Eigen::Vector3d::UnitY())*
		// 					   Eigen::AngleAxisd(request->pose[3], Eigen::Vector3d::UnitX());
		//q.normalize();
		
		//另外一种表达方式
	    tf2::Quaternion q;
        q.setRPY(request->pose[3], request->pose[4], request->pose[5]);
        q = q.normalize();
	
		target_pose1.position.x = request->pose[0];
		target_pose1.position.y = request->pose[1];
		target_pose1.position.z = request->pose[2];
		target_pose1.orientation.w = q.w();
		target_pose1.orientation.x = q.x();
		target_pose1.orientation.y = q.y();
		target_pose1.orientation.z = q.z();
		move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
		//RRT 对 精确姿态目标 非常不友好。
		move_group_interface_->setGoalPositionTolerance(0.01);
		move_group_interface_->setGoalOrientationTolerance(0.15);

		move_group_interface_->setStartStateToCurrentState(); //设置当前状态为开始位置（表示2）
		move_group_interface_->setPoseTarget(target_pose1);

		bool success = (move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		RCLCPP_INFO(LOGGER, "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
		if(!success)
		{
			response->result = false;
			return;
		}
		// Moving to a pose goal
  		move_group_interface_->move();
		response->result = true;
	}
	else if(request->loc_type == 2) //space joint goal
	{
		// Planning to a joint-space goal
		moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);

		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

		// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
		joint_group_positions[0] = request->pose[0];  // radians
		joint_group_positions[1] = request->pose[1];  // radians
		joint_group_positions[2] = request->pose[2];  // radians
		joint_group_positions[3] = request->pose[3];  // radians
		joint_group_positions[4] = request->pose[4];  // radians
		joint_group_positions[5] = request->pose[5];  // radians

        move_group_interface_->setStartStateToCurrentState();
		move_group_interface_->setJointValueTarget(joint_group_positions);
		
		move_group_interface_->setMaxVelocityScalingFactor(0.05);
		move_group_interface_->setMaxAccelerationScalingFactor(0.05);

		bool success = (move_group_interface_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		RCLCPP_INFO(LOGGER, "Plan 2 (joint space goal) %s", success ? "SUCCEEDED" : "FAILED");
		if(!success)
		{
			response->result = false;
			return;
		}
		// Moving to a joint-space goal
  		move_group_interface_->move();
		response->result = true;

	}
}
void AHPCPlanning::robotDriverStatusHandle(const shared_message::msg::RobotDriverStatus::ConstSharedPtr driver_status)
{
    this->driver_status_.ts_ = driver_status->header.stamp;
	this->driver_status_.connect_status_ = driver_status->connect_status;
	this->driver_status_.enable_status_ = driver_status->enable;
	this->driver_status_.is_robot_stop_ = driver_status->is_robot_stop;
	this->driver_status_.is_free_drive_ = driver_status->free_drive;
	this->driver_status_.force_control_enable_ = driver_status->force_mode;
	this->driver_status_.in_collision_ = driver_status->collision;
	this->driver_status_.in_emergency_ = driver_status->emergency;
	this->driver_status_.torque_adjust_enable_ = driver_status->force_freedrive;
	this->driver_status_.torque_extract_adjust_enable_ =
	    driver_status->torque_extract_adjust;
	this->driver_status_.admit_extract_enable_ =
	    driver_status->admit_extract;
	this->driver_status_.admit_extract_success_ =
	    driver_status->admit_extract_success;
	this->driver_status_.in_pos_ = driver_status->in_pos;

	this->driver_status_.tcp_pose_.resize(6);
	this->driver_status_.tcp_pose_.at(0) = driver_status->tcp_pose.x;
	this->driver_status_.tcp_pose_.at(1) = driver_status->tcp_pose.y;
	this->driver_status_.tcp_pose_.at(2) = driver_status->tcp_pose.z;
	this->driver_status_.tcp_pose_.at(3) = driver_status->tcp_pose.yaw;
	this->driver_status_.tcp_pose_.at(4) = driver_status->tcp_pose.pitch;
	this->driver_status_.tcp_pose_.at(5) = driver_status->tcp_pose.roll;

	this->driver_status_.joint_base_position_.resize(
	    driver_status->joint_base_position.size());

	for (size_t i = 0; i < this->driver_status_.joint_base_position_.size(); i++)
	{
		this->driver_status_.joint_base_position_.at(i).resize(3);
		this->driver_status_.joint_base_position_.at(i).at(0) =
		    driver_status->joint_base_position.at(i).x;
		this->driver_status_.joint_base_position_.at(i).at(1) =
		    driver_status->joint_base_position.at(i).y;
		this->driver_status_.joint_base_position_.at(i).at(2) =
		    driver_status->joint_base_position.at(i).z;
	}

	this->driver_status_.joint_pose_ = driver_status->joint_pose.pose;

	this->driver_status_.tool_id_ = driver_status->tool_id;
	this->driver_status_.tool_tcp_.resize(6);
	this->driver_status_.tool_tcp_.at(0) = driver_status->tool_tcp.x;
	this->driver_status_.tool_tcp_.at(1) = driver_status->tool_tcp.y;
	this->driver_status_.tool_tcp_.at(2) = driver_status->tool_tcp.z;
	this->driver_status_.tool_tcp_.at(3) = driver_status->tool_tcp.yaw;
	this->driver_status_.tool_tcp_.at(4) = driver_status->tool_tcp.pitch;
	this->driver_status_.tool_tcp_.at(5) = driver_status->tool_tcp.roll;

	this->driver_status_.mass_ = driver_status->mass;
	this->driver_status_.center_of_mass_.resize(3);
	this->driver_status_.center_of_mass_.at(0) = driver_status->cog.x;
	this->driver_status_.center_of_mass_.at(1) = driver_status->cog.y;
	this->driver_status_.center_of_mass_.at(2) = driver_status->cog.z;
	this->driver_status_.errcode_ = driver_status->errcode;
	this->driver_status_.collision_level_ = driver_status->collision_level;

	this->driver_status_.force_adjust_seq_ = driver_status->force_adjust_seq;
	this->driver_status_.force_control_seq_ = driver_status->force_control_seq;
	this->driver_status_.force_extract_adjust_seq_ =
	    driver_status->force_extract_adjust_seq;
	this->driver_status_.admit_extract_seq_ = driver_status->admit_extract_seq;

	this->driver_status_.torque_.resize(driver_status->torque.size());

	for (size_t i = 0; i < this->driver_status_.torque_.size(); i++)
	{
		this->driver_status_.torque_.at(i) = driver_status->torque.at(i);
	}

	this->driver_status_.act_torque_.resize(driver_status->act_torque.size());

	for (size_t i = 0; i < this->driver_status_.act_torque_.size(); i++)
	{
		this->driver_status_.act_torque_.at(i) = driver_status->act_torque.at(i);
	}

	this->driver_status_.j_torque_.resize(driver_status->j_torque.size());

	for (size_t i = 0; i < this->driver_status_.j_torque_.size(); i++)
	{
		this->driver_status_.j_torque_.at(i) = driver_status->j_torque.at(i);
	}

	this->driver_status_.j_current_.resize(driver_status->j_current.size());

	for (size_t i = 0; i < this->driver_status_.j_current_.size(); i++)
	{
		this->driver_status_.j_current_.at(i) = driver_status->j_current.at(i);
	}

	static uint32_t print_data_cnt = 0;

	if (1)
	{
		if (0 == (print_data_cnt % 500))
		{
        std::cout<< " act_torque " << vec2str(this->driver_status_.act_torque_) <<
        " raw_torque " << vec2str(this->driver_status_.torque_) <<
        " tcp " << vec2str(this->driver_status_.tcp_pose_) <<
        " joint " << vec2str(this->driver_status_.joint_pose_) <<
        " jtorque " << vec2str(this->driver_status_.j_torque_) <<
        " jcurrent " << vec2str(this->driver_status_.j_current_)<<std::endl;
		}
	}

	print_data_cnt ++;

	return;
}
void AHPCPlanning::doTask()
{

}
int main(int argc, char** argv)
{
  
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto planning_node = std::make_shared<AHPCPlanning>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &planning_node]() {
    executor.add_node(planning_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(planning_node->getNodeBaseInterface());
  });

  planning_node->doTask();
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}